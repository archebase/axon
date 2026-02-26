# WebSocket Real-Time Push Design Document

**Date:** 2026-02-24
**Status:** Design
**Target Version:** 0.3.0

## Overview

This document describes the design for WebSocket-based real-time push notifications in Axon Recorder. The goal is to replace the current HTTP polling mechanism in AxonPanel with an efficient, event-driven WebSocket connection that provides instant state updates and statistics streaming.

## Motivation

### Current Limitations

1. **Polling Overhead**: AxonPanel polls `/rpc/state` and `/rpc/stats` every 1 second, creating unnecessary HTTP requests even when no state changes occur
2. **Latency**: Up to 1-second delay before UI reflects state changes
3. **Resource Waste**: Server handles 2 requests/second per connected client regardless of activity
4. **Scalability Concerns**: Polling doesn't scale well with multiple monitoring clients

### Benefits of WebSocket

1. **Instant Updates**: State changes pushed immediately to all connected clients
2. **Reduced Overhead**: Single persistent connection vs. continuous polling
3. **Bandwidth Efficiency**: Data only sent when changes occur
4. **Foundation for Future Features**: Real-time logs, alerts, and per-topic statistics

## Architecture

### System Overview

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        Axon Recorder (C++)                              │
├─────────────────────────────────────────────────────────────────────────┤
│  ┌────────────────┐     ┌─────────────────────┐     ┌────────────────┐  │
│  │  HTTP Server   │     │  WebSocket Server   │     │  StateManager  │  │
│  │  (Boost.Beast) │     │  (Boost.Beast)      │     │                │  │
│  │                │     │                     │     │  - State       │  │
│  │  /rpc/*        │     │  ws://host:8081/ws  │◄────┤  - Callbacks   │  │
│  │                │     │                     │     │                │  │
│  └────────────────┘     └──────────┬──────────┘     └───────┬────────┘  │
│                                    │                        │           │
│                                    ▼                        │           │
│                         ┌────────────────────┐              │           │
│                         │  Connection Manager │◄────────────┘           │
│                         │  - Client tracking  │                          │
│                         │  - Broadcast        │                          │
│                         │  - Event routing    │                          │
│                         └────────────────────┘                          │
└─────────────────────────────────────────────────────────────────────────┘
                                     │
                         WebSocket   │
                                     ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                        AxonPanel (Vue 3)                                │
├─────────────────────────────────────────────────────────────────────────┤
│  ┌────────────────────────────────────────────────────────────────────┐ │
│  │  WebSocket Client (api/websocket.js)                               │ │
│  │  - Auto-reconnect with exponential backoff                         │ │
│  │  - Event dispatching                                               │ │
│  │  - Connection state management                                     │ │
│  └──────────────────────────────┬─────────────────────────────────────┘ │
│                                 │                                        │
│                                 ▼                                        │
│  ┌────────────────────────────────────────────────────────────────────┐ │
│  │  Vue Components                                                    │ │
│  │  - Reactive state updates via event bus                            │ │
│  │  - Real-time statistics charts                                     │ │
│  │  - Instant state machine animation                                 │ │
│  └────────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────┘
```

### Protocol Design

#### Connection Endpoint

```
ws://localhost:8081/ws
```

**Query Parameters:**
| Parameter | Type | Description |
|-----------|------|-------------|
| `subscribe` | string | Comma-separated event types (default: all) |
| `stats_interval` | int | Stats push interval in ms (default: 1000, min: 100) |

**Example:**
```
ws://localhost:8081/ws?subscribe=state,stats&stats_interval=500
```

#### Message Format

All messages are JSON text frames with the following structure:

```typescript
interface WebSocketMessage {
  type: string;        // Event type
  timestamp: string;   // ISO 8601 timestamp
  data: any;           // Event-specific payload
}
```

#### Event Types

| Event | Description | Trigger |
|-------|-------------|---------|
| `state` | State machine transition | On state change |
| `stats` | Recording statistics | Periodic (configurable) |
| `config` | Task configuration update | On config set/clear |
| `log` | Activity log entry | On recorder log event |
| `error` | Error notification | On error condition |
| `connected` | Connection acknowledgment | On WebSocket connect |

### Event Payloads

#### 1. `state` - State Transition

Sent when the recorder state changes.

```json
{
  "type": "state",
  "timestamp": "2026-02-24T10:30:00.123Z",
  "data": {
    "previous": "ready",
    "current": "recording",
    "task_id": "task_123"
  }
}
```

**Fields:**
| Field | Type | Description |
|-------|------|-------------|
| `previous` | string | Previous state (idle/ready/recording/paused) |
| `current` | string | New state |
| `task_id` | string | Current task ID (if applicable) |

#### 2. `stats` - Recording Statistics

Sent periodically during recording or on-demand.

```json
{
  "type": "stats",
  "timestamp": "2026-02-24T10:30:01.000Z",
  "data": {
    "messages_received": 1523456,
    "messages_written": 1523450,
    "messages_dropped": 6,
    "bytes_written": 2147483648,
    "duration_ms": 45000,
    "topics": {
      "/camera/image": {
        "messages": 1500,
        "bytes": 1073741824
      },
      "/lidar/scan": {
        "messages": 450,
        "bytes": 524288000
      }
    }
  }
}
```

**Fields:**
| Field | Type | Description |
|-------|------|-------------|
| `messages_received` | uint64 | Total messages received from middleware |
| `messages_written` | uint64 | Messages successfully written to MCAP |
| `messages_dropped` | uint64 | Messages dropped due to queue overflow |
| `bytes_written` | uint64 | Total bytes written to MCAP file |
| `duration_ms` | uint64 | Recording duration in milliseconds |
| `topics` | object | Per-topic statistics (optional, if enabled) |

#### 3. `config` - Task Configuration

Sent when task configuration is set or cleared.

```json
{
  "type": "config",
  "timestamp": "2026-02-24T10:29:00.000Z",
  "data": {
    "action": "set",
    "task_config": {
      "task_id": "task_123",
      "device_id": "robot_01",
      "scene": "warehouse_navigation",
      "topics": ["/camera/image", "/lidar/scan"]
    }
  }
}
```

**Config Clear Event:**
```json
{
  "type": "config",
  "timestamp": "2026-02-24T10:35:00.000Z",
  "data": {
    "action": "clear"
  }
}
```

#### 4. `log` - Activity Log

Sent for important recorder events that should appear in the UI log.

```json
{
  "type": "log",
  "timestamp": "2026-02-24T10:30:00.500Z",
  "data": {
    "level": "info",
    "message": "Recording started",
    "details": {
      "task_id": "task_123",
      "output_file": "/data/recordings/task_123.mcap"
    }
  }
}
```

**Log Levels:**
- `debug` - Debug information
- `info` - Normal operation events
- `warning` - Warning conditions
- `error` - Error conditions

#### 5. `error` - Error Notification

Sent when an error occurs that may require user attention.

```json
{
  "type": "error",
  "timestamp": "2026-02-24T10:31:00.000Z",
  "data": {
    "code": "DISK_FULL",
    "message": "Disk space critically low",
    "severity": "critical",
    "details": {
      "available_mb": 100,
      "required_mb": 1024
    }
  }
}
```

**Error Codes:**
| Code | Severity | Description |
|------|----------|-------------|
| `DISK_FULL` | critical | Insufficient disk space |
| `WRITE_ERROR` | error | MCAP write failure |
| `UPLOAD_FAILED` | warning | S3 upload failure |
| `TOPIC_ERROR` | warning | Topic subscription issue |
| `PLUGIN_ERROR` | error | Middleware plugin error |

#### 6. `connected` - Connection Acknowledgment

Sent immediately after WebSocket connection is established.

```json
{
  "type": "connected",
  "timestamp": "2026-02-24T10:28:59.000Z",
  "data": {
    "version": "0.3.0",
    "state": "idle",
    "client_id": "conn_abc123"
  }
}
```

### Client-to-Server Messages

Clients can send control messages to the server:

#### Subscribe to Events

```json
{
  "action": "subscribe",
  "events": ["state", "stats"]
}
```

#### Unsubscribe from Events

```json
{
  "action": "unsubscribe",
  "events": ["log"]
}
```

#### Set Stats Interval

```json
{
  "action": "set_stats_interval",
  "interval_ms": 500
}
```

#### Request Current State

```json
{
  "action": "get_state"
}
```

**Response:**
```json
{
  "type": "state",
  "timestamp": "2026-02-24T10:30:00.000Z",
  "data": {
    "current": "recording",
    "task_id": "task_123"
  }
}
```

## Backend Implementation

### Component Design

#### 1. WebSocketServer Class

**File:** `apps/axon_recorder/websocket_server.hpp`

```cpp
namespace axon {
namespace recorder {

class WebSocketServer {
public:
  using MessageHandler = std::function<void(const std::string&)>;

  WebSocketServer(boost::asio::io_context& ioc, uint16_t port);
  ~WebSocketServer();

  // Start accepting connections
  void start();

  // Stop all connections
  void stop();

  // Broadcast to all connected clients
  void broadcast(const std::string& type, const nlohmann::json& data);

  // Broadcast to specific event subscribers
  void broadcast_to_subscribers(const std::string& event_type,
                                 const nlohmann::json& data);

  // Get connection count
  size_t connection_count() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace recorder
}  // namespace axon
```

#### 2. WebSocketSession Class

**File:** `apps/axon_recorder/websocket_session.hpp`

```cpp
namespace axon {
namespace recorder {

class WebSocketSession : public std::enable_shared_from_this<WebSocketSession> {
public:
  WebSocketSession(tcp::socket&& socket);

  // Start the session
  void run();

  // Send a message
  void send(const std::string& message);

  // Close the connection
  void close();

  // Get subscribed events
  const std::set<std::string>& subscribed_events() const;

private:
  void on_accept(beast::error_code ec);
  void do_read();
  void on_read(beast::error_code ec, std::size_t bytes_transferred);
  void on_write(beast::error_code ec, std::size_t bytes_transferred);
  void handle_client_message(const std::string& message);

  beast::websocket::stream<beast::tcp_stream> ws_;
  beast::flat_buffer buffer_;
  std::set<std::string> subscribed_events_;
  std::queue<std::string> write_queue_;
  std::atomic<bool> writing_{false};
};

}  // namespace recorder
}  // namespace axon
```

#### 3. EventBroadcaster Class

**File:** `apps/axon_recorder/event_broadcaster.hpp`

```cpp
namespace axon {
namespace recorder {

class EventBroadcaster {
public:
  EventBroadcaster(WebSocketServer& ws_server, StateManager& state_manager);

  // Start periodic stats broadcast
  void start_stats_broadcast(std::chrono::milliseconds interval);

  // Stop stats broadcast
  void stop_stats_broadcast();

  // Broadcast state change
  void broadcast_state_change(RecorderState from, RecorderState to,
                               const std::string& task_id);

  // Broadcast config change
  void broadcast_config_change(const TaskConfig* config);

  // Broadcast log event
  void broadcast_log(const std::string& level, const std::string& message,
                     const nlohmann::json& details = {});

  // Broadcast error
  void broadcast_error(const std::string& code, const std::string& message,
                       const std::string& severity,
                       const nlohmann::json& details = {});

private:
  void on_state_transition(RecorderState from, RecorderState to);

  WebSocketServer& ws_server_;
  StateManager& state_manager_;
  std::atomic<bool> stats_running_{false};
  std::thread stats_thread_;
};

}  // namespace recorder
}  // namespace axon
```

### Integration Points

#### 1. HttpServer Integration

The WebSocket server runs alongside the HTTP server on the same port using Boost.Beast's ability to upgrade HTTP connections to WebSocket.

```cpp
// In http_server.cpp
void HttpServer::handle_request(...) {
  // Check for WebSocket upgrade
  if (is_websocket_upgrade_request(request)) {
    upgrade_to_websocket(std::move(socket), request);
    return;
  }

  // Handle normal HTTP request
  // ...
}
```

#### 2. StateManager Integration

Register a callback to broadcast state changes:

```cpp
// In axon_recorder.cpp
state_manager_.register_transition_callback(
  [this](RecorderState from, RecorderState to) {
    event_broadcaster_->broadcast_state_change(from, to, current_task_id_);
  }
);
```

#### 3. Recorder Integration

Hook into recording events:

```cpp
// In recorder.cpp
void Recorder::on_recording_started() {
  event_broadcaster_->broadcast_log("info", "Recording started", {
    {"task_id", task_id_},
    {"output_file", output_path_}
  });
}

void Recorder::on_recording_finished() {
  event_broadcaster_->broadcast_log("info", "Recording finished", {
    {"task_id", task_id_},
    {"messages_written", stats_.messages_written},
    {"bytes_written", stats_.bytes_written}
  });
}
```

### Threading Model

```
┌─────────────────────────────────────────────────────────────────┐
│                        Thread Architecture                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  │
│  │  HTTP Thread    │  │  WebSocket      │  │  Stats Thread   │  │
│  │  (existing)     │  │  Accept Thread  │  │  (optional)     │  │
│  └────────┬────────┘  └────────┬────────┘  └────────┬────────┘  │
│           │                    │                     │           │
│           ▼                    ▼                     ▼           │
│  ┌─────────────────────────────────────────────────────────────┐│
│  │              io_context (shared)                             ││
│  │              - HTTP handlers                                 ││
│  │              - WebSocket sessions (one strand per session)   ││
│  └─────────────────────────────────────────────────────────────┘│
│                                                                  │
│  ┌─────────────────────────────────────────────────────────────┐│
│  │              Broadcast Queue (thread-safe)                   ││
│  │              - State transitions                             ││
│  │              - Stats updates                                 ││
│  │              - Log events                                    ││
│  └─────────────────────────────────────────────────────────────┘│
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

**Key Points:**
- WebSocket sessions run on the same `io_context` as HTTP server
- Each session uses a strand for thread-safe async operations
- Broadcasting is thread-safe via `post()` to `io_context`
- Stats broadcast runs on a dedicated timer

### Heartbeat / Ping-Pong Mechanism

The WebSocket connection uses a heartbeat mechanism to detect stale or disconnected clients:

**Server-Initiated Ping:**
- Server sends WebSocket Ping frame every `ping_interval_ms` (default: 30000ms)
- Client must respond with Pong frame within `ping_timeout_ms` (default: 10000ms)
- If Pong not received, connection is closed with code 1000 (normal close) or 1006 (abnormal close)

**Implementation in WebSocketSession:**
```cpp
void WebSocketSession::start_ping_timer() {
  ping_timer_.expires_after(std::chrono::milliseconds(ping_interval_ms_));
  ping_timer_.async_wait([self = shared_from_this()](beast::error_code ec) {
    if (!ec) {
      self->ws_.async_ping([self](beast::error_code ec) {
        if (ec) {
          // Pong not received, close connection
          self->close(1006, "Ping timeout");
        }
      });
    }
  });
}
```

**Client Handling:**
- Browser WebSocket API automatically responds to Ping with Pong
- Client should detect `onclose` event and trigger reconnection
- `wsClient` already implements exponential backoff reconnection (see line 623-637)

### Thread Safety Guarantees

The EventBroadcaster provides the following thread safety guarantees:

| Component | Thread Safety | Notes |
|-----------|---------------|-------|
| WebSocketServer | Thread-safe | Uses `strand` for all operations |
| WebSocketSession | Thread-safe | Single strand per session |
| EventBroadcaster | Thread-safe | Uses `post()` to io_context |
| Broadcast Queue | Lock-free | SPSC queue for message passing |
| Write Queue | Thread-safe | Atomic flag for async writes |

**Critical Rules:**
1. All public methods of EventBroadcaster are thread-safe
2. Callbacks from StateManager happen on the StateManager thread - must use `post()` to broadcast
3. Stats thread uses timer with handler bound to strand - no external synchronization needed
4. WebSocketSession write queue uses atomic flag to prevent concurrent writes

### Rate Limiting

To protect against malicious clients or excessive message floods:

**Per-Client Limits:**
| Limit | Default | Configurable |
|-------|---------|--------------|
| Max messages/sec | 100 | Yes |
| Max connection rate | 10/sec | Yes |
| Max concurrent connections | 100 | Yes |

**Implementation:**
```cpp
class RateLimiter {
public:
  RateLimiter(size_t max_per_second, size_t max_connections)
    : max_per_second_(max_per_second), max_connections_(max_connections) {}

  bool allow_message(const std::string& client_id) {
    auto now = std::chrono::steady_clock::now();
    auto& client = clients_[client_id];
    
    // Check per-second limit
    if (client.message_count >= max_per_second_) {
      return false;
    }
    
    // Reset counter every second
    if (now - client.last_reset > 1s) {
      client.message_count = 0;
      client.last_reset = now;
    }
    
    client.message_count++;
    return true;
  }

private:
  size_t max_per_second_;
  size_t max_connections_;
  std::unordered_map<std::string, ClientState> clients_;
};
```

**Violations:**
- Exceeding message rate: Close connection with code 1008 (policy violation)
- Exceeding connection rate: HTTP 429 Too Many Requests for new connections

## Frontend Implementation

### WebSocket Client Module

**File:** `apps/axon_panel/src/api/websocket.js`

```javascript
/**
 * WebSocket client for Axon Recorder real-time updates
 */
class AxonWebSocket {
  constructor(url = 'ws://localhost:8081/ws') {
    this.url = url
    this.ws = null
    this.reconnectAttempts = 0
    this.maxReconnectAttempts = 10
    this.reconnectDelay = 1000
    this.listeners = new Map()
    this.connectionState = 'disconnected'
  }

  // Connect to WebSocket server
  connect() {
    if (this.ws?.readyState === WebSocket.OPEN) {
      return
    }

    this.connectionState = 'connecting'
    this.ws = new WebSocket(this.url)

    this.ws.onopen = () => {
      console.log('[WebSocket] Connected')
      this.connectionState = 'connected'
      this.reconnectAttempts = 0
      this.emit('connection', { state: 'connected' })
    }

    this.ws.onmessage = (event) => {
      try {
        const message = JSON.parse(event.data)
        this.emit(message.type, message.data)
      } catch (e) {
        console.error('[WebSocket] Parse error:', e)
      }
    }

    this.ws.onclose = (event) => {
      console.log('[WebSocket] Disconnected:', event.code, event.reason)
      this.connectionState = 'disconnected'
      this.emit('connection', { state: 'disconnected' })
      this.scheduleReconnect()
    }

    this.ws.onerror = (error) => {
      console.error('[WebSocket] Error:', error)
      this.emit('error', error)
    }
  }

  // Disconnect from server
  disconnect() {
    if (this.ws) {
      this.ws.close(1000, 'Client disconnect')
      this.ws = null
    }
    this.connectionState = 'disconnected'
  }

  // Schedule reconnection with exponential backoff
  scheduleReconnect() {
    if (this.reconnectAttempts >= this.maxReconnectAttempts) {
      console.error('[WebSocket] Max reconnect attempts reached')
      return
    }

    const delay = Math.min(
      this.reconnectDelay * Math.pow(2, this.reconnectAttempts),
      30000
    )
    this.reconnectAttempts++

    console.log(`[WebSocket] Reconnecting in ${delay}ms (attempt ${this.reconnectAttempts})`)
    setTimeout(() => this.connect(), delay)
  }

  // Send message to server
  send(action, data = {}) {
    if (this.ws?.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify({ action, ...data }))
    }
  }

  // Subscribe to events
  subscribe(events) {
    this.send('subscribe', { events })
  }

  // Unsubscribe from events
  unsubscribe(events) {
    this.send('unsubscribe', { events })
  }

  // Set statistics interval
  setStatsInterval(intervalMs) {
    this.send('set_stats_interval', { interval_ms: intervalMs })
  }

  // Request current state
  requestState() {
    this.send('get_state')
  }

  // Event emitter methods
  on(event, callback) {
    if (!this.listeners.has(event)) {
      this.listeners.set(event, [])
    }
    this.listeners.get(event).push(callback)
  }

  off(event, callback) {
    if (this.listeners.has(event)) {
      const callbacks = this.listeners.get(event)
      const index = callbacks.indexOf(callback)
      if (index > -1) {
        callbacks.splice(index, 1)
      }
    }
  }

  emit(event, data) {
    if (this.listeners.has(event)) {
      for (const callback of this.listeners.get(event)) {
        callback(data)
      }
    }
  }
}

export const wsClient = new AxonWebSocket(
  import.meta.env.VITE_WS_URL || 'ws://localhost:8081/ws'
)
```

### Vue Composable for WebSocket

**File:** `apps/axon_panel/src/composables/useWebSocket.js`

```javascript
import { ref, onMounted, onUnmounted } from 'vue'
import { wsClient } from '@/api/websocket'

export function useWebSocket() {
  const connectionState = ref('disconnected')
  const currentState = ref('idle')
  const stats = ref(null)
  const logs = ref([])

  // Event handlers
  const handlers = {
    connection: (data) => {
      connectionState.value = data.state
    },

    connected: (data) => {
      currentState.value = data.state
      connectionState.value = 'connected'
    },

    state: (data) => {
      currentState.value = data.current
    },

    stats: (data) => {
      stats.value = data
    },

    log: (data) => {
      logs.value.push({
        timestamp: new Date().toISOString(),
        level: data.level,
        message: data.message,
        details: data.details
      })
      // Keep only last 100 logs
      if (logs.value.length > 100) {
        logs.value.shift()
      }
    },

    error: (data) => {
      logs.value.push({
        timestamp: new Date().toISOString(),
        level: 'error',
        message: data.message,
        details: { code: data.code, severity: data.severity }
      })
    }
  }

  onMounted(() => {
    // Register handlers
    for (const [event, handler] of Object.entries(handlers)) {
      wsClient.on(event, handler)
    }

    // Connect
    wsClient.connect()
  })

  onUnmounted(() => {
    // Unregister handlers
    for (const [event, handler] of Object.entries(handlers)) {
      wsClient.off(event, handler)
    }
  })

  return {
    connectionState,
    currentState,
    stats,
    logs,
    subscribe: (events) => wsClient.subscribe(events),
    unsubscribe: (events) => wsClient.unsubscribe(events),
    setStatsInterval: (ms) => wsClient.setStatsInterval(ms),
    requestState: () => wsClient.requestState()
  }
}
```

### Updated App.vue

**Key changes to integrate WebSocket:**

```vue
<script setup>
import { ref, onMounted, onUnmounted, computed } from 'vue'
import { useWebSocket } from '@/composables/useWebSocket'
import { rpcApi } from '@/api/rpc'

// WebSocket for real-time updates
const {
  connectionState: wsConnectionState,
  currentState,
  stats,
  subscribe,
  setStatsInterval
} = useWebSocket()

// Fallback to HTTP polling when WebSocket is disconnected
const httpPollingActive = computed(() => wsConnectionState.value !== 'connected')

// Existing RPC methods for commands (still use HTTP)
const handleCommand = async (command) => {
  try {
    switch (command) {
      case 'begin':
        await rpcApi.begin(currentTaskId.value)
        break
      case 'finish':
        await rpcApi.finish(currentTaskId.value)
        break
      // ... other commands
    }
    // State will be updated via WebSocket
  } catch (error) {
    console.error('Command failed:', error)
  }
}

onMounted(() => {
  // Subscribe to all events
  subscribe(['state', 'stats', 'config', 'log', 'error'])

  // Set stats interval to 500ms for responsive UI
  setStatsInterval(500)
})
</script>
```

## Migration Strategy

### Phase 1: Backend Implementation

1. Add WebSocket server alongside HTTP server
2. Implement basic state change broadcasting
3. Add stats periodic push
4. Test with wscat or similar tools

### Phase 2: Frontend Integration

1. Create WebSocket client module
2. Add Vue composable for state management
3. Update App.vue to use WebSocket
4. Keep HTTP polling as fallback

### Phase 3: Deprecation

1. Remove HTTP polling code
2. Simplify state management
3. Add advanced features (per-topic stats, real-time logs)

### Backward Compatibility

- HTTP RPC API remains unchanged
- Clients can still poll `/rpc/state` and `/rpc/stats`
- WebSocket is opt-in via `/ws` endpoint

## Testing Strategy

### Unit Tests

1. **WebSocketServer**
   - Connection accept/reject
   - Message broadcasting
   - Subscription filtering

2. **EventBroadcaster**
   - State change events
   - Stats periodic push
   - Error broadcasting

### Integration Tests

1. **End-to-End Flow**
   - Connect → receive `connected` event
   - State change → receive `state` event
   - Recording start → receive `stats` events

2. **Multiple Clients**
   - Broadcast to all clients
   - Subscription filtering

### Performance Tests

1. **Connection Scalability**
   - 100 concurrent connections
   - Message throughput

2. **Latency Measurement**
   - Time from state change to client receipt
   - Target: < 10ms

## Configuration

### Server-Side

```yaml
# recorder_config.yaml
websocket:
  enabled: true
  port: 8081          # HTTP port + 1 (separate from HTTP RPC on 8080)
  max_connections: 100
  stats_interval_ms: 1000
  ping_interval_ms: 30000
  ping_timeout_ms: 10000
```

### Client-Side

```javascript
// Environment variables
VITE_WS_URL=ws://localhost:8081/ws
VITE_WS_RECONNECT_ATTEMPTS=10
VITE_WS_RECONNECT_DELAY=1000
```

## Security Considerations

### Current Scope (v0.3.0)

1. **Same-Origin Policy**: WebSocket inherits HTTP CORS settings
2. **Local Network Only**: Default bind to localhost
3. **No Authentication**: Follows HTTP RPC API model

### Future Enhancements (v0.5.0+)

1. **Token-Based Auth**: Validate JWT in WebSocket handshake
2. **Rate Limiting**: Limit message frequency per client
3. **TLS Support**: `wss://` for encrypted connections

## Performance Targets

| Metric | Target | Notes |
|--------|--------|-------|
| Connection latency | < 50ms | Time to establish connection |
| Message latency | < 10ms | Time from event to client receipt |
| Memory per connection | < 100KB | Per connected client |
| Max connections | 100 | Simultaneous clients |
| Stats broadcast CPU | < 1% | At 1000ms interval |

## Future Enhancements

1. **Per-Topic Statistics**: Real-time bandwidth/latency per topic
2. **Real-Time Logs**: Stream recorder logs to connected clients
3. **Alert System**: Configurable thresholds with push notifications
4. **Historical Data**: Time-series data for charts
5. **Binary Protocol**: Protocol Buffers for efficiency (if needed)

## Related Documentation

- [HTTP RPC API Design](rpc-api-design.md) - Existing REST API
- [Frontend Design](frontend-design.md) - AxonPanel architecture
- [State Machine](../CLAUDE.md) - State transitions and callbacks

## Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2026-02-24 | - | Initial design |
| 1.1 | 2026-02-24 | - | Added heartbeat/ping-pong, thread safety, rate limiting |
