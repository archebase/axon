/**
 * Unit tests for WebSocket Client RPC methods
 */

import { describe, it, expect, vi, beforeEach, afterEach, jest } from 'vitest'

// Mock WebSocket class
class MockWebSocket {
  constructor(url) {
    this.url = url
    this.readyState = MockWebSocket.CONNECTING
    this.sentMessages = []

    // Simulate async connection
    setTimeout(() => {
      this.readyState = MockWebSocket.OPEN
      if (this.onopen) {
        this.onopen({ type: 'open' })
      }
    }, 0)
  }

  send(data) {
    this.sentMessages.push(data)
    if (this.onmessage && this.readyState === MockWebSocket.OPEN) {
      // Echo back for ping messages
      const parsed = JSON.parse(data)
      if (parsed.action === 'get_state') {
        this.onmessage({
          data: JSON.stringify({
            type: 'rpc_response',
            request_id: parsed.request_id || null,
            success: true,
            data: { state: 'idle' }
          })
        })
      }
    }
  }

  close(code, reason) {
    this.readyState = MockWebSocket.CLOSED
    if (this.onclose) {
      this.onclose({ code, reason, type: 'close' })
    }
  }

  // Simulate receiving a message from server
  simulateMessage(message) {
    if (this.onmessage) {
      this.onmessage({ data: JSON.stringify(message) })
    }
  }
}

MockWebSocket.CONNECTING = 0
MockWebSocket.OPEN = 1
MockWebSocket.CLOSING = 2
MockWebSocket.CLOSED = 3

// Set global WebSocket mock
global.WebSocket = MockWebSocket

// Import after setting mock
import { WebSocketClient } from '@/api/websocket'

describe('WebSocketClient RPC Methods', () => {
  let client

  beforeEach(() => {
    client = new WebSocketClient()
    // Use a mock sender to avoid actual WebSocket connection
    client.send = vi.fn()
  })

  afterEach(() => {
    if (client) {
      client.disconnect()
    }
    vi.clearAllMocks()
  })

  describe('sendCommand', () => {
    it('should send command with request_id and return Promise', async () => {
      const mockWs = {
        readyState: WebSocket.OPEN,
        send: vi.fn()
      }
      client.ws = mockWs

      const promise = client.sendCommand('get_state')

      // Verify request_id is generated
      const sentData = JSON.parse(mockWs.send.mock.calls[0][0])
      expect(sentData.action).toBe('get_state')
      expect(sentData.request_id).toMatch(/^req_\d+_/)

      // Simulate response
      mockWs.onmessage({
        data: JSON.stringify({
          type: 'rpc_response',
          request_id: sentData.request_id,
          success: true,
          data: { state: 'recording' }
        })
      })

      const result = await promise
      expect(result.state).toBe('recording')
    })

    it('should reject when WebSocket is not connected', async () => {
      client.ws = null

      await expect(client.sendCommand('get_state')).rejects.toThrow('WebSocket not connected')
    })

    it('should timeout if no response received', async () => {
      vi.useFakeTimers()

      const mockWs = {
        readyState: WebSocket.OPEN,
        send: vi.fn(),
        onmessage: null
      }
      client.ws = mockWs
      client.rpcTimeout = 100

      const promise = client.sendCommand('get_state')

      // Fast-forward past timeout
      vi.advanceTimersByTime(150)

      await expect(promise).rejects.toThrow('Command timeout')

      vi.useRealTimers()
    })

    it('should reject on error response', async () => {
      const mockWs = {
        readyState: WebSocket.OPEN,
        send: vi.fn(),
        onmessage: null
      }
      client.ws = mockWs

      const promise = client.sendCommand('begin', { task_id: 'test_task' })

      const sentData = JSON.parse(mockWs.send.mock.calls[0][0])

      // Simulate error response
      mockWs.onmessage({
        data: JSON.stringify({
          type: 'rpc_response',
          request_id: sentData.request_id,
          success: false,
          message: 'Task not found'
        })
      })

      await expect(promise).rejects.toThrow('Task not found')
    })

    it('should handle concurrent requests correctly', async () => {
      const mockWs = {
        readyState: WebSocket.OPEN,
        send: vi.fn(),
        onmessage: null
      }
      client.ws = mockWs

      // Send multiple concurrent requests
      const promise1 = client.sendCommand('get_state')
      const promise2 = client.sendCommand('get_stats')
      const promise3 = client.sendCommand('pause')

      // Get all request IDs
      const reqId1 = JSON.parse(mockWs.send.mock.calls[0][0]).request_id
      const reqId2 = JSON.parse(mockWs.send.mock.calls[1][0]).request_id
      const reqId3 = JSON.parse(mockWs.send.mock.calls[2][0]).request_id

      expect(reqId1).not.toBe(reqId2)
      expect(reqId2).not.toBe(reqId3)

      // Respond in different order
      mockWs.onmessage({
        data: JSON.stringify({
          type: 'rpc_response',
          request_id: reqId2,
          success: true,
          data: { messages: 100 }
        })
      })

      mockWs.onmessage({
        data: JSON.stringify({
          type: 'rpc_response',
          request_id: reqId3,
          success: true,
          data: { state: 'paused' }
        })
      })

      mockWs.onmessage({
        data: JSON.stringify({
          type: 'rpc_response',
          request_id: reqId1,
          success: true,
          data: { state: 'recording' }
        })
      })

      const [result1, result2, result3] = await Promise.all([promise1, promise2, promise3])

      expect(result1.state).toBe('recording')
      expect(result2.messages).toBe(100)
      expect(result3.state).toBe('paused')
    })
  })

  describe('begin', () => {
    it('should send begin command with task_id', async () => {
      const mockWs = {
        readyState: WebSocket.OPEN,
        send: vi.fn(),
        onmessage: null
      }
      client.ws = mockWs

      const promise = client.begin('task_123')
      const sentData = JSON.parse(mockWs.send.mock.calls[0][0])

      expect(sentData.action).toBe('begin')
      expect(sentData.task_id).toBe('task_123')
      expect(sentData.request_id).toBeDefined()

      // Simulate success response
      mockWs.onmessage({
        data: JSON.stringify({
          type: 'rpc_response',
          request_id: sentData.request_id,
          success: true,
          data: { state: 'recording' }
        })
      })

      await promise
    })

    it('should return response data on success', async () => {
      const mockWs = {
        readyState: WebSocket.OPEN,
        send: vi.fn(),
        onmessage: null
      }
      client.ws = mockWs

      const promise = client.begin('task_456')
      const sentData = JSON.parse(mockWs.send.mock.calls[0][0])

      mockWs.onmessage({
        data: JSON.stringify({
          type: 'rpc_response',
          request_id: sentData.request_id,
          success: true,
          data: { state: 'recording', task_id: 'task_456' }
        })
      })

      const result = await promise
      expect(result.state).toBe('recording')
      expect(result.task_id).toBe('task_456')
    })
  })

  describe('finish', () => {
    it('should send finish command with task_id', async () => {
      const mockWs = {
        readyState: WebSocket.OPEN,
        send: vi.fn(),
        onmessage: null
      }
      client.ws = mockWs

      const promise = client.finish('task_789')
      const sentData = JSON.parse(mockWs.send.mock.calls[0][0])

      expect(sentData.action).toBe('finish')
      expect(sentData.task_id).toBe('task_789')

      mockWs.onmessage({
        data: JSON.stringify({
          type: 'rpc_response',
          request_id: sentData.request_id,
          success: true,
          data: { state: 'idle' }
        })
      })

      await promise
    })
  })

  describe('pause', () => {
    it('should send pause command', async () => {
      const mockWs = {
        readyState: WebSocket.OPEN,
        send: vi.fn(),
        onmessage: null
      }
      client.ws = mockWs

      const promise = client.pause()
      const sentData = JSON.parse(mockWs.send.mock.calls[0][0])

      expect(sentData.action).toBe('pause')

      mockWs.onmessage({
        data: JSON.stringify({
          type: 'rpc_response',
          request_id: sentData.request_id,
          success: true,
          data: { state: 'paused' }
        })
      })

      await promise
    })
  })

  describe('resume', () => {
    it('should send resume command', async () => {
      const mockWs = {
        readyState: WebSocket.OPEN,
        send: vi.fn(),
        onmessage: null
      }
      client.ws = mockWs

      const promise = client.resume()
      const sentData = JSON.parse(mockWs.send.mock.calls[0][0])

      expect(sentData.action).toBe('resume')

      mockWs.onmessage({
        data: JSON.stringify({
          type: 'rpc_response',
          request_id: sentData.request_id,
          success: true,
          data: { state: 'recording' }
        })
      })

      await promise
    })
  })

  describe('cancel', () => {
    it('should send cancel command', async () => {
      const mockWs = {
        readyState: WebSocket.OPEN,
        send: vi.fn(),
        onmessage: null
      }
      client.ws = mockWs

      const promise = client.cancel()
      const sentData = JSON.parse(mockWs.send.mock.calls[0][0])

      expect(sentData.action).toBe('cancel')

      mockWs.onmessage({
        data: JSON.stringify({
          type: 'rpc_response',
          request_id: sentData.request_id,
          success: true,
          data: { state: 'idle' }
        })
      })

      await promise
    })
  })

  describe('clear', () => {
    it('should send clear command', async () => {
      const mockWs = {
        readyState: WebSocket.OPEN,
        send: vi.fn(),
        onmessage: null
      }
      client.ws = mockWs

      const promise = client.clear()
      const sentData = JSON.parse(mockWs.send.mock.calls[0][0])

      expect(sentData.action).toBe('clear')

      mockWs.onmessage({
        data: JSON.stringify({
          type: 'rpc_response',
          request_id: sentData.request_id,
          success: true,
          data: { state: 'idle' }
        })
      })

      await promise
    })
  })

  describe('config', () => {
    it('should send config command with task_config', async () => {
      const mockWs = {
        readyState: WebSocket.OPEN,
        send: vi.fn(),
        onmessage: null
      }
      client.ws = mockWs

      const configData = {
        task_id: 'new_task',
        device_id: 'robot_01',
        topics: ['/camera', '/lidar']
      }

      const promise = client.config(configData)
      const sentData = JSON.parse(mockWs.send.mock.calls[0][0])

      expect(sentData.action).toBe('config')
      expect(sentData.task_config).toEqual(configData)

      mockWs.onmessage({
        data: JSON.stringify({
          type: 'rpc_response',
          request_id: sentData.request_id,
          success: true,
          data: { state: 'ready', task_id: 'new_task' }
        })
      })

      const result = await promise
      expect(result.state).toBe('ready')
      expect(result.task_id).toBe('new_task')
    })
  })

  describe('quit', () => {
    it('should send quit command', async () => {
      const mockWs = {
        readyState: WebSocket.OPEN,
        send: vi.fn(),
        onmessage: null
      }
      client.ws = mockWs

      const promise = client.quit()
      const sentData = JSON.parse(mockWs.send.mock.calls[0][0])

      expect(sentData.action).toBe('quit')

      mockWs.onmessage({
        data: JSON.stringify({
          type: 'rpc_response',
          request_id: sentData.request_id,
          success: true,
          message: 'Program quitting'
        })
      })

      await promise
    })
  })

  describe('getState', () => {
    it('should send get_state command and return state', async () => {
      const mockWs = {
        readyState: WebSocket.OPEN,
        send: vi.fn(),
        onmessage: null
      }
      client.ws = mockWs

      const promise = client.getState()
      const sentData = JSON.parse(mockWs.send.mock.calls[0][0])

      expect(sentData.action).toBe('get_state')

      mockWs.onmessage({
        data: JSON.stringify({
          type: 'rpc_response',
          request_id: sentData.request_id,
          success: true,
          data: {
            state: 'recording',
            task_config: { task_id: 'test_task' }
          }
        })
      })

      const result = await promise
      expect(result.state).toBe('recording')
      expect(result.task_config.task_id).toBe('test_task')
    })
  })

  describe('Pending Requests Cleanup', () => {
    it('should clear pending requests on disconnect', async () => {
      const mockWs = {
        readyState: WebSocket.OPEN,
        send: vi.fn(),
        close: vi.fn()
      }
      client.ws = mockWs

      // Start a command
      const promise = client.sendCommand('get_state')
      const sentData = JSON.parse(mockWs.send.mock.calls[0][0])

      // Disconnect before response
      mockWs.readyState = WebSocket.CLOSED
      mockWs.onclose({ type: 'close' })

      await expect(promise).rejects.toThrow('WebSocket disconnected')
    })

    it('should handle multiple pending requests with some timeouts', async () => {
      vi.useFakeTimers()

      const mockWs = {
        readyState: WebSocket.OPEN,
        send: vi.fn(),
        onmessage: null
      }
      client.ws = mockWs
      client.rpcTimeout = 100

      // Start three requests
      const promise1 = client.sendCommand('get_state')
      const promise2 = client.sendCommand('get_stats')
      const promise3 = client.sendCommand('pause')

      const reqId1 = JSON.parse(mockWs.send.mock.calls[0][0]).request_id
      const reqId2 = JSON.parse(mockWs.send.mock.calls[1][0]).request_id
      const reqId3 = JSON.parse(mockWs.send.mock.calls[2][0]).request_id

      // Respond to first request
      mockWs.onmessage({
        data: JSON.stringify({
          type: 'rpc_response',
          request_id: reqId1,
          success: true,
          data: { state: 'idle' }
        })
      })

      // Timeout second request
      vi.advanceTimersByTime(150)

      // Respond to third request
      mockWs.onmessage({
        data: JSON.stringify({
          type: 'rpc_response',
          request_id: reqId3,
          success: true,
          data: { state: 'paused' }
        })
      })

      const result1 = await promise1
      expect(result1.state).toBe('idle')

      await expect(promise2).rejects.toThrow('Command timeout')

      const result3 = await promise3
      expect(result3.state).toBe('paused')

      vi.useRealTimers()
    })
  })

  describe('Response Handler Integration', () => {
    it('should handle rpc_response event type alongside other events', () => {
      const mockWs = {
        readyState: WebSocket.OPEN,
        send: vi.fn(),
        onmessage: null
      }
      client.ws = mockWs

      // Register handler for rpc_response
      const rpcHandler = vi.fn()
      client.on('rpc_response', rpcHandler)

      // Send command
      const promise = client.begin('task_test')
      const sentData = JSON.parse(mockWs.send.mock.calls[0][0])

      // Send RPC response
      mockWs.onmessage({
        data: JSON.stringify({
          type: 'rpc_response',
          request_id: sentData.request_id,
          success: true,
          data: { state: 'recording' }
        })
      })

      await promise

      // Verify rpc_response handler was called
      expect(rpcHandler).toHaveBeenCalled()
    })

    it('should not treat other event types as RPC responses', () => {
      const mockWs = {
        readyState: WebSocket.OPEN,
        send: vi.fn(),
        onmessage: null
      }
      client.ws = mockWs

      // Register handlers
      const stateHandler = vi.fn()
      const rpcHandler = vi.fn()

      client.on('state', stateHandler)
      client.on('rpc_response', rpcHandler)

      // Send state update (not RPC response)
      mockWs.onmessage({
        data: JSON.stringify({
          type: 'state',
          data: { current: 'recording', task_id: 'task_123' }
        })
      })

      // State handler should be called, RPC handler should not
      expect(stateHandler).toHaveBeenCalled()
      expect(rpcHandler).not.toHaveBeenCalled()
    })
  })

  describe('Request ID Generation', () => {
    it('should generate unique request IDs', () => {
      const mockWs = {
        readyState: WebSocket.OPEN,
        send: vi.fn()
      }
      client.ws = mockWs

      client.sendCommand('get_state')
      client.sendCommand('get_stats')
      client.sendCommand('pause')

      const ids = mockWs.send.mock.calls.map(call => JSON.parse(call[0]).request_id)

      expect(new Set(ids).size).toBe(3)
      expect(ids[0]).toMatch(/^req_\d+_/)
      expect(ids[1]).toMatch(/^req_\d+_/)
      expect(ids[2]).toMatch(/^req_\d+_/)
    })

    it('should increment request counter', () => {
      const mockWs = {
        readyState: WebSocket.OPEN,
        send: vi.fn()
      }
      client.ws = mockWs

      client.requestCounter = 0

      client.sendCommand('get_state')
      const id1 = JSON.parse(mockWs.send.mock.calls[0][0]).request_id

      client.sendCommand('get_stats')
      const id2 = JSON.parse(mockWs.send.mock.calls[1][0]).request_id

      // Extract counter from request ID (format: req_<timestamp>_<counter>)
      const counter1 = parseInt(id1.split('_')[2])
      const counter2 = parseInt(id2.split('_')[2])

      expect(counter2).toBe(counter1 + 1)
    })
  })
})
