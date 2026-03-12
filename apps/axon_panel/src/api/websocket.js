/**
 * WebSocket client for AxonPanel real-time updates
 *
 * Provides automatic reconnection with exponential backoff and event-based message handling.
 * Supports bidirectional RPC commands with concurrent request handling.
 */

const BASE_URL = import.meta.env.VITE_API_BASE_URL || ''
const WS_URL = import.meta.env.VITE_WS_URL || ''

// Get WebSocket URL based on current page URL or env variable
function getWebSocketUrl() {
  if (WS_URL) {
    return WS_URL
  }

  // If BASE_URL is set, derive WebSocket URL from it
  if (BASE_URL) {
    const url = new URL(BASE_URL)
    const wsProtocol = url.protocol === 'https:' ? 'wss:' : 'ws:'
    // WebSocket server runs on port 8081 (separate from HTTP on 8080)
    const wsPort = url.port === '8080' ? '8081' : url.port
    return `${wsProtocol}//${url.hostname}:${wsPort}/ws`
  }

  const wsProtocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:'
  const hostname = window.location.hostname

  if (import.meta.env.DEV) {
    // Dev mode: WebSocket server is always on port 8081
    return `${wsProtocol}//${hostname}:8081/ws`
  }

  // Production: panel is served on port N+2, WebSocket is on port N+1 (N=8080 default)
  const port = parseInt(window.location.port, 10) || 8082
  return `${wsProtocol}//${hostname}:${port - 1}/ws`
}

/**
 * WebSocket client with auto-reconnect, event handling, and RPC support
 */
export class WebSocketClient {
  constructor(options = {}) {
    this.url = options.url || getWebSocketUrl()
    this.reconnectInterval = options.reconnectInterval || 1000
    this.maxReconnectInterval = options.maxReconnectInterval || 30000
    this.reconnectMultiplier = options.reconnectMultiplier || 1.5
    this.pingInterval = options.pingInterval || 30000
    this.rpcTimeout = options.rpcTimeout || 5000

    this.ws = null
    this.reconnectTimer = null
    this.pingTimer = null
    this.currentReconnectInterval = this.reconnectInterval
    this.isConnecting = false
    this.shouldReconnect = true

    // Pending RPC requests for concurrent request support
    this.pendingRequests = new Map()
    this.requestCounter = 0

    // Event handlers
    this.handlers = {
      connected: [],
      state: [],
      stats: [],
      config: [],
      log: [],
      error: [],
      disconnected: [],
      raw: [],
      rpc_response: []
    }
  }

  /**
   * Connect to WebSocket server
   */
  connect() {
    if (this.ws && (this.ws.readyState === WebSocket.OPEN || this.ws.readyState === WebSocket.CONNECTING)) {
      return
    }

    if (this.isConnecting) {
      return
    }

    this.isConnecting = true
    this.shouldReconnect = true

    try {
      if (import.meta.env.DEV) {
        console.log('[WebSocket] Connecting to', this.url)
      }

      this.ws = new WebSocket(this.url)

      this.ws.onopen = () => {
        if (import.meta.env.DEV) {
          console.log('[WebSocket] Connected to', this.url)
        }
        this.isConnecting = false
        this.currentReconnectInterval = this.reconnectInterval

        // Start ping/pong to keep connection alive
        this.startPing()

        // Notify connected handlers
        this.handlers.connected.forEach(h => h())

        // Request initial state
        this.send({ action: 'get_state' })
      }

      this.ws.onmessage = (event) => {
        try {
          const message = JSON.parse(event.data)

          if (import.meta.env.DEV) {
            console.log('[WebSocket] Message:', message.type, message.data)
          }

          // Call raw handler with full message
          this.handlers.raw.forEach(h => h(message))

          // Handle RPC response for request correlation
          if (message.type === 'rpc_response' && message.request_id) {
            const pending = this.pendingRequests.get(message.request_id)
            if (pending) {
              clearTimeout(pending.timeoutId)
              this.pendingRequests.delete(message.request_id)

              // Extract response data matching HTTP RPC format
              // Include success, message, and data fields
              const response = {
                success: message.success !== undefined ? message.success : true,
                message: message.message || '',
                data: message.data !== undefined ? message.data : {}
              }

              if (message.success === false) {
                pending.reject(new Error(message.message || 'Command failed'))
              } else {
                pending.resolve(response)
              }
            }
            // Also call rpc_response handlers
            if (this.handlers.rpc_response) {
              this.handlers.rpc_response.forEach(h => h(message.data, message))
            }
            return
          }

          // Call type-specific handlers
          if (message.type && this.handlers[message.type]) {
            this.handlers[message.type].forEach(h => h(message.data, message))
          }
        } catch (e) {
          if (import.meta.env.DEV) {
            console.error('[WebSocket] Failed to parse message:', event.data)
          }
        }
      }

      this.ws.onclose = (event) => {
        if (import.meta.env.DEV) {
          console.log('[WebSocket] Closed:', event.code, event.reason)
        }

        this.isConnecting = false
        this.stopPing()

        // Reject all pending requests
        this.pendingRequests.forEach((pending) => {
          clearTimeout(pending.timeoutId)
          pending.reject(new Error('WebSocket disconnected'))
        })
        this.pendingRequests.clear()

        // Notify handlers
        this.handlers.disconnected.forEach(h => h({ code: event.code, reason: event.reason }))

        // Attempt reconnection
        if (this.shouldReconnect) {
          this.scheduleReconnect()
        }
      }

      this.ws.onerror = (error) => {
        if (import.meta.env.DEV) {
          console.error('[WebSocket] Error:', error)
        }

        this.isConnecting = false
      }

    } catch (error) {
      if (import.meta.env.DEV) {
        console.error('[WebSocket] Connection failed:', error)
      }

      this.isConnecting = false

      if (this.shouldReconnect) {
        this.scheduleReconnect()
      }
    }
  }

  /**
   * Disconnect from WebSocket server
   */
  disconnect() {
    this.shouldReconnect = false
    this.stopPing()

    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer)
      this.reconnectTimer = null
    }

    // Reject all pending requests
    this.pendingRequests.forEach((pending) => {
      clearTimeout(pending.timeoutId)
      pending.reject(new Error('WebSocket disconnected'))
    })
    this.pendingRequests.clear()

    if (this.ws) {
      this.ws.close(1000, 'Client disconnect')
      this.ws = null
    }
  }

  /**
   * Schedule reconnection with exponential backoff
   */
  scheduleReconnect() {
    if (this.reconnectTimer) {
      return
    }

    if (import.meta.env.DEV) {
      console.log('[WebSocket] Reconnecting in', this.currentReconnectInterval, 'ms')
    }

    this.reconnectTimer = setTimeout(() => {
      this.reconnectTimer = null
      this.connect()
    }, this.currentReconnectInterval)

    // Increase interval for next attempt
    this.currentReconnectInterval = Math.min(
      this.currentReconnectInterval * this.reconnectMultiplier,
      this.maxReconnectInterval
    )
  }

  /**
   * Start ping interval to keep connection alive
   */
  startPing() {
    this.stopPing()

    this.pingTimer = setInterval(() => {
      if (this.ws && this.ws.readyState === WebSocket.OPEN) {
        this.ws.send(JSON.stringify({ action: 'ping' }))
      }
    }, this.pingInterval)
  }

  /**
   * Stop ping interval
   */
  stopPing() {
    if (this.pingTimer) {
      clearInterval(this.pingTimer)
      this.pingTimer = null
    }
  }

  /**
   * Send message to server
   * @param {object} message Message to send
   */
  send(message) {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(message))
    }
  }

  /**
   * Send RPC command and return Promise
   * @param {string} action RPC action to execute
   * @param {object} params Parameters for the RPC command
   * @returns {Promise} Promise that resolves with response data
   */
  sendCommand(action, params = {}) {
    return new Promise((resolve, reject) => {
      if (!this.isConnected()) {
        reject(new Error('WebSocket not connected'))
        return
      }

      // Generate unique request ID for correlation
      const requestId = `req_${Date.now()}_${++this.requestCounter}`

      // Store pending request with timeout
      const timeoutId = setTimeout(() => {
        this.pendingRequests.delete(requestId)
        reject(new Error('Command timeout'))
      }, this.rpcTimeout)

      this.pendingRequests.set(requestId, { resolve, reject, timeoutId })

      // Send command with request ID
      this.send({ action, request_id: requestId, ...params })
    })
  }

  /**
   * Subscribe to events
   * @param {string[]} events Event types to subscribe to
   */
  subscribe(events) {
    this.send({ action: 'subscribe', events })
  }

  /**
   * Unsubscribe from events
   * @param {string[]} events Event types to unsubscribe from
   */
  unsubscribe(events) {
    this.send({ action: 'unsubscribe', events })
  }

  /**
   * Set stats interval
   * @param {number} intervalMs Interval in milliseconds
   */
  setStatsInterval(intervalMs) {
    this.send({ action: 'set_stats_interval', interval_ms: intervalMs })
  }

  /**
   * Request current state (fire and forget)
   */
  requestState() {
    this.send({ action: 'get_state' })
  }

  /**
   * Get current state (RPC method with response)
   * @returns {Promise<object>} State information
   */
  async getState() {
    return this.sendCommand('get_state')
  }

  /**
   * Begin recording
   * @param {string} taskId Task ID for the recording
   * @returns {Promise<object>} Response data
   */
  async begin(taskId) {
    return this.sendCommand('begin', { task_id: taskId })
  }

  /**
   * Finish recording
   * @param {string} taskId Task ID for the recording
   * @returns {Promise<object>} Response data
   */
  async finish(taskId) {
    return this.sendCommand('finish', { task_id: taskId })
  }

  /**
   * Pause recording
   * @returns {Promise<object>} Response data
   */
  async pause() {
    return this.sendCommand('pause')
  }

  /**
   * Resume recording
   * @returns {Promise<object>} Response data
   */
  async resume() {
    return this.sendCommand('resume')
  }

  /**
   * Cancel recording
   * @returns {Promise<object>} Response data
   */
  async cancel() {
    return this.sendCommand('cancel')
  }

  /**
   * Clear configuration
   * @returns {Promise<object>} Response data
   */
  async clear() {
    return this.sendCommand('clear')
  }

  /**
   * Set task configuration
   * @param {object} configData Task configuration data
   * @returns {Promise<object>} Response data
   */
  async config(configData) {
    return this.sendCommand('config', { task_config: configData })
  }

  /**
   * Quit recorder
   * @returns {Promise<object>} Response data
   */
  async quit() {
    return this.sendCommand('quit')
  }

  /**
   * Register event handler
   * @param {string} eventType Event type (connected, state, stats, config, log, error, disconnected, raw, rpc_response)
   * @param {function} handler Handler function
   * @returns {function} Unsubscribe function
   */
  on(eventType, handler) {
    if (!this.handlers[eventType]) {
      console.warn(`Unknown event type: ${eventType}`)
      return () => {}
    }

    this.handlers[eventType].push(handler)

    // Return unsubscribe function
    return () => {
      const index = this.handlers[eventType].indexOf(handler)
      if (index !== -1) {
        this.handlers[eventType].splice(index, 1)
      }
    }
  }

  /**
   * Check if connected
   */
  isConnected() {
    return this.ws && this.ws.readyState === WebSocket.OPEN
  }

  /**
   * Check if connecting
   */
  isConnecting_() {
    return this.isConnecting
  }
}

// Singleton instance
let wsClient = null

/**
 * Get WebSocket client singleton
 */
export function getWebSocketClient(options = {}) {
  if (!wsClient) {
    wsClient = new WebSocketClient(options)
  }
  return wsClient
}

export default WebSocketClient
