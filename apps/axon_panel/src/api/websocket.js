/**
 * WebSocket client for AxonPanel real-time updates
 * 
 * Provides automatic reconnection with exponential backoff and event-based message handling.
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
    return `${wsProtocol}//${url.hostname}:${wsPort}`
  }

  // Otherwise, construct from current page location
  const wsProtocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:'
  const host = window.location.host

  // Development fallback: use port 8081 for WebSocket (separate from HTTP on 8080)
  let wsHost = host
  if (import.meta.env.DEV) {
    const port = host.split(':')[1]
    if (port && port !== '8081') {
      wsHost = host.replace(/:\d+$/, ':8081')
    }
  }

  return `${wsProtocol}//${wsHost}`
}

/**
 * WebSocket client with auto-reconnect and event handling
 */
export class WebSocketClient {
  constructor(options = {}) {
    this.url = options.url || getWebSocketUrl()
    this.reconnectInterval = options.reconnectInterval || 1000
    this.maxReconnectInterval = options.maxReconnectInterval || 30000
    this.reconnectMultiplier = options.reconnectMultiplier || 1.5
    this.pingInterval = options.pingInterval || 30000
    
    this.ws = null
    this.reconnectTimer = null
    this.pingTimer = null
    this.currentReconnectInterval = this.reconnectInterval
    this.isConnecting = false
    this.shouldReconnect = true
    
    // Event handlers
    this.handlers = {
      connected: [],
      state: [],
      stats: [],
      config: [],
      log: [],
      error: [],
      disconnected: [],
      raw: []
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
   * Request current state
   */
  requestState() {
    this.send({ action: 'get_state' })
  }

  /**
   * Register event handler
   * @param {string} eventType Event type (connected, state, stats, config, log, error, disconnected, raw)
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
