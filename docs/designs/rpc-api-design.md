# HTTP RPC API Design for Axon Recorder

**Date:** 2025-01-12
**Last Updated:** 2025-01-21

## Overview

This document defines the HTTP RPC API design for Axon Recorder, which replaces the original ROS service interface. The design provides a JSON-based RESTful API that supports remote recording control via HTTP protocol.

## Design Goals

1. **HTTP-Based Control**: Use HTTP protocol instead of ROS services to reduce dependencies
2. **JSON Format**: Unified JSON format for data exchange
3. **RESTful Design**: Follow REST design principles with appropriate HTTP methods and paths
4. **State Machine Alignment**: Tightly aligned with state machine design to ensure correct state transitions
5. **Remote Control**: Support server-side remote control of recording tasks

## Architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│                      HTTP RPC Architecture                           │
├──────────────────────────────────────────────────────────────────────┤
│                                                                              │
│   ┌──────────────┐         ┌──────────────┐                          │
│   │ HTTP Client  │         │  HTTP Client  │                          │
│   │  (curl/Post) │         │  (Browser/Web) │                          │
│   └──────┬───────┘         └──────┬───────┘                          │
│          │                       │                                     │
│          └───────────┬───────────┘                                     │
│                      ▼                                                  │
│           ┌─────────────────────┐                                      │
│           │   HTTP RPC Server   │                                      │
│           │   (port 8080)       │                                      │
│           └──────────┬──────────┘                                      │
│                      │                                                     │
│                      ▼                                                     │
│           ┌─────────────────────┐                                      │
│           │   AxonRecorder      │                                      │
│           │   - StateMachine     │                                      │
│           │   - RecordingSession│                                      │
│           │   - WorkerThreadPool │                                      │
│           └─────────────────────┘                                      │
│                                                                              │
└──────────────────────────────────────────────────────────────────────┘
```

## State Machine

The HTTP RPC API is tightly integrated with the state machine:

```
IDLE ──(POST /rpc/config)──► READY ──(POST /rpc/begin)──► RECORDING ↔ PAUSED
  ▲                             │                                       │
  │                             │                                       │
  └────────(POST /rpc/finish)───┴──────────────(POST /rpc/finish)──────┘
                                │
                                ▼
                        (POST /rpc/quit)
                                │
                                ▼
                          Program Exit
```

### State Descriptions

| State | Description |
|-------|-------------|
| `IDLE` | Initial state, waiting for task configuration |
| `READY` | Task configuration cached, waiting to start recording |
| `RECORDING` | Currently recording ROS messages to MCAP file |
| `PAUSED` | Recording paused, can be resumed or finished |

## RPC Endpoints

### 1. POST /rpc/config

Set task configuration (cache metadata), state transition: **IDLE → READY**

**Request:**
```http
POST /rpc/config HTTP/1.1
Content-Type: application/json

{
  "task_config": {
    "task_id": "task_123",
    "device_id": "robot_01",
    "data_collector_id": "collector_01",
    "scene": "warehouse_navigation",
    "subscene": "aisle_traversal",
    "skills": ["navigation", "obstacle_avoidance"],
    "factory": "factory_shenzhen",
    "operator_name": "operator_001",
    "topics": ["/camera/image", "/lidar/scan", "/odom"],
    "start_callback_url": "http://server.example.com/api/v1/tasks/123/start",
    "finish_callback_url": "http://server.example.com/api/v1/tasks/123/finish",
    "user_token": "eyJhbGciOiJIUzI1NiIs..."
  }
}
```

**Response (200 OK):**
```json
{
  "success": true,
  "message": "Task configuration set successfully",
  "data": {
    "state": "ready",
    "task_id": "task_123"
  }
}
```

**Error Response (400 Bad Request):**
```json
{
  "success": false,
  "message": "Missing 'task_config' in request parameters"
}
```

**TaskConfig Fields:**
| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `task_id` | string | Yes | Server-assigned task identifier |
| `device_id` | string | Yes | Robot/device identifier |
| `data_collector_id` | string | No | Data collector identifier |
| `scene` | string | No | Recording scene/context tag |
| `subscene` | string | No | Recording subscene |
| `skills` | array | No | List of associated skills |
| `factory` | string | No | Factory identifier (identifies the factory producing the data) |
| `operator_name` | string | No | Operator identifier |
| `topics` | array | No | List of topics to record |
| `start_callback_url` | string | No | Start recording callback URL |
| `finish_callback_url` | string | No | Finish recording callback URL |
| `user_token` | string | No | JWT token for callback authentication |

---

### 2. POST /rpc/begin

Start recording, state transition: **READY → RECORDING**

**Request:**
```http
POST /rpc/begin HTTP/1.1
Content-Type: application/json

{
  "task_id": "task_123"
}
```

**Note:** The `task_id` parameter is required and must match the task ID previously set via `/rpc/config`.

**Response (200 OK):**
```json
{
  "success": true,
  "message": "Recording started successfully",
  "data": {
    "state": "recording",
    "task_id": "task_123"
  }
}
```

**Error Response (400 Bad Request) - Missing task_id:**
```json
{
  "success": false,
  "message": "Missing required parameter: task_id",
  "data": {
    "state": "ready"
  }
}
```

**Error Response (400 Bad Request) - Wrong state:**
```json
{
  "success": false,
  "message": "Cannot start recording from state: idle. Must be in READY state (call /rpc/config first).",
  "data": {
    "state": "idle"
  }
}
```

**Error Response (400 Bad Request) - task_id mismatch:**
```json
{
  "success": false,
  "message": "task_id mismatch: expected 'task_123' but got 'wrong_task'",
  "data": {
    "state": "ready",
    "expected_task_id": "task_123",
    "provided_task_id": "wrong_task"
  }
}
```

**Valid From States:**
- ✅ `READY` (must call /rpc/config first)
- ❌ `IDLE` (must configure task first)
- ❌ `RECORDING` (already recording)
- ❌ `PAUSED` (already paused)

---

### 3. POST /rpc/finish

Finish recording, state transition: **RECORDING/PAUSED → IDLE**

**Request:**
```http
POST /rpc/finish HTTP/1.1
Content-Type: application/json

{
  "task_id": "task_123"
}
```

**Response (200 OK):**
```json
{
  "success": true,
  "message": "Recording finished successfully",
  "data": {
    "state": "idle",
    "task_id": "task_123"
  }
}
```

**Error Response (400 Bad Request):**
```json
{
  "success": false,
  "message": "Missing required parameter: task_id",
  "data": {
    "state": "recording"
  }
}
```

**Error Response (400 Bad Request) - task_id mismatch:**
```json
{
  "success": false,
  "message": "task_id mismatch: expected 'task_123' but got 'wrong_task_id'",
  "data": {
    "state": "recording",
    "expected_task_id": "task_123",
    "provided_task_id": "wrong_task_id"
  }
}
```

**Error Response (400 Bad Request) - Invalid state:**
```json
{
  "success": false,
  "message": "Cannot finish recording from state: idle. Must be in RECORDING or PAUSED state.",
  "data": {
    "state": "idle"
  }
}
```

**Valid From States:**
- ✅ `RECORDING`
- ✅ `PAUSED`
- ❌ `IDLE`
- ❌ `READY`

**Behavior:**
- Stop recording and save MCAP file
- Inject metadata and generate sidecar JSON
- Return to IDLE state
- **Program continues running**, HTTP server keeps listening
- Can call `/rpc/begin` again to start a new recording

---

### 4. POST /rpc/quit

Exit program (saves data first)

**Request:**
```http
POST /rpc/quit HTTP/1.1
Content-Type: application/json

{}
```

**Note:** This endpoint does not require a `task_id` parameter and can be called from any state.

**Response (200 OK):**
```json
{
  "success": true,
  "message": "Program quitting. Recording stopped and data saved.",
  "data": {
    "state": "idle"
  }
}
```

**Behavior:**
1. If currently recording, automatically stop recording and save data first
2. Stop HTTP server
3. Gracefully exit program

**Valid From States:**
- ✅ Any state (IDLE, READY, RECORDING, PAUSED)

---

### 5. POST /rpc/pause

Pause recording, state transition: **RECORDING → PAUSED**

**Request:**
```http
POST /rpc/pause HTTP/1.1
Content-Type: application/json

{}
```

**Response (200 OK):**
```json
{
  "success": true,
  "message": "Recording paused successfully",
  "data": {
    "state": "paused"
  }
}
```

**Error Response (501 Not Implemented):**
```json
{
  "success": false,
  "message": "Pause functionality not yet implemented",
  "data": {
    "state": "recording"
  }
}
```

**Valid From States:**
- ✅ `RECORDING`
- ❌ Other states

---

### 6. POST /rpc/resume

Resume recording, state transition: **PAUSED → RECORDING**

**Request:**
```http
POST /rpc/resume HTTP/1.1
Content-Type: application/json

{}
```

**Response (200 OK):**
```json
{
  "success": true,
  "message": "Recording resumed successfully",
  "data": {
    "state": "recording"
  }
}
```

**Error Response (501 Not Implemented):**
```json
{
  "success": false,
  "message": "Resume functionality not yet implemented",
  "data": {
    "state": "paused"
  }
}
```

**Valid From States:**
- ✅ `PAUSED`
- ❌ Other states

---

### 7. POST /rpc/cancel

Cancel recording (discard partial data), state transition: **RECORDING/PAUSED → IDLE**

**Request:**
```http
POST /rpc/cancel HTTP/1.1
Content-Type: application/json

{
  "task_id": "task_123"
}
```

**Response (200 OK):**
```json
{
  "success": true,
  "message": "Recording cancelled successfully",
  "data": {
    "state": "idle",
    "task_id": "task_123"
  }
}
```

**Behavior:**
- TODO: Currently implemented the same as finish, should discard partial recording
- Should delete incomplete MCAP files in the future

**Valid From States:**
- ✅ `RECORDING`
- ✅ `PAUSED`
- ❌ Other states

---

### 8. POST /rpc/clear

Clear configuration, state transition: **READY → IDLE**

**Request:**
```http
POST /rpc/clear HTTP/1.1
Content-Type: application/json

{}
```

**Response (200 OK):**
```json
{
  "success": true,
  "message": "Configuration cleared successfully",
  "data": {
    "state": "idle"
  }
}
```

**Error Response (501 Not Implemented):**
```json
{
  "success": false,
  "message": "Clear functionality not yet implemented",
  "data": {
    "state": "ready"
  }
}
```

**Behavior:**
- TODO: Reset task configuration and return to IDLE state
- Used to abandon unstarted recording tasks

**Valid From States:**
- ✅ `READY`
- ❌ Other states

---

### 9. GET /rpc/state

Query current state

**Request:**
```http
GET /rpc/state HTTP/1.1
```

**Response (200 OK) - IDLE state (no config):**
```json
{
  "success": true,
  "message": "State retrieved successfully",
  "data": {
    "state": "idle",
    "running": false
  }
}
```

**Response (200 OK) - READY state (config set):**
```json
{
  "success": true,
  "message": "State retrieved successfully",
  "data": {
    "state": "ready",
    "running": false,
    "task_config": {
      "task_id": "task_123",
      "device_id": "robot_01",
      "data_collector_id": "collector_01",
      "scene": "warehouse_navigation",
      "subscene": "aisle_traversal",
      "skills": ["navigation", "obstacle_avoidance"],
      "factory": "factory_01",
      "operator_name": "operator_01",
      "topics": ["/camera/image", "/lidar/scan", "/odom"]
    }
  }
}
```

**Response (200 OK) - RECORDING state:**
```json
{
  "success": true,
  "message": "State retrieved successfully",
  "data": {
    "state": "recording",
    "running": true,
    "task_config": {
      "task_id": "task_123",
      "device_id": "robot_01",
      "scene": "warehouse_navigation",
      "factory": "factory_01",
      "operator_name": "operator_01",
      "topics": ["/camera/image", "/lidar/scan", "/odom"]
    }
  }
}
```

**State Values:**
- `"idle"` - Idle state, no configuration
- `"ready"` - Configured, waiting to start (includes task_config)
- `"recording"` - Currently recording (includes task_config)
- `"paused"` - Currently paused (includes task_config)

**Notes:**
- When state is `IDLE`, the `task_config` field is not present
- When state is `READY`, `RECORDING`, or `PAUSED`, the response includes task configuration information
- `task_config` only includes business fields (task_id, device_id, scene, topics, etc.), excluding callback URLs and authentication tokens

---

### 10. GET /rpc/stats

Query recording statistics

**Request:**
```http
GET /rpc/stats HTTP/1.1
```

**Response (200 OK):**
```json
{
  "success": true,
  "message": "Statistics retrieved successfully",
  "data": {
    "messages_received": 1523456,
    "messages_written": 1523450,
    "messages_dropped": 6,
    "bytes_written": 2147483648
  }
}
```

**Statistics Fields:**
| Field | Type | Description |
|-------|------|-------------|
| `messages_received` | uint64 | Total messages received |
| `messages_written` | uint64 | Messages written to MCAP |
| `messages_dropped` | uint64 | Messages dropped (queue full) |
| `bytes_written` | uint64 | Bytes written |

---

### 11. GET / or /health

Health check

**Request:**
```http
GET / HTTP/1.1
```

**Response (200 OK):**
```json
{
  "status": "ok",
  "service": "AxonRecorder",
  "version": "0.1.0",
  "running": true,
  "state": "recording"
}
```

## Error Handling

### Error Response Format

All error responses follow a unified format:

```json
{
  "success": false,
  "message": "Human-readable error message",
  "data": {
    "state": "current_state",
    "task_id": "task_123"
  }
}
```

### Common Error Codes

| HTTP Code | Description |
|------------|-------------|
| 200 | Success |
| 400 | Bad Request - Invalid parameters or state transition |
| 404 | Not Found - Unknown RPC endpoint |
| 500 | Internal Server Error |
| 501 | Not Implemented - Feature not yet available |

### State Transition Errors

| Current State | Invalid Commands |
|---------------|-----------------|
| `IDLE` | `pause`, `resume`, `cancel`, `finish` |
| `READY` | `pause`, `resume`, `cancel`, `finish` |
| `RECORDING` | `begin`, `resume`, `clear` |
| `PAUSED` | `begin`, `pause`, `clear` |

## Usage Examples

### Example 1: Complete Recording Workflow

```bash
# 1. Health check
curl http://localhost:8080/health

# 2. Check initial state (should be "idle")
curl http://localhost:8080/rpc/state

# 3. Configure recording task
curl -X POST http://localhost:8080/rpc/config \
  -H "Content-Type: application/json" \
  -d '{
    "task_config": {
      "task_id": "task_123",
      "device_id": "robot_01",
      "scene": "warehouse",
      "factory": "factory_01",
      "operator_name": "operator_01",
      "topics": ["/camera/image", "/lidar/scan"],
      "start_callback_url": "http://server/api/start",
      "finish_callback_url": "http://server/api/finish",
      "user_token": "jwt_token_here"
    }
  }'

# 4. Check state (should be "ready")
curl http://localhost:8080/rpc/state

# 5. Begin recording
curl -X POST http://localhost:8080/rpc/begin \
  -H "Content-Type: application/json" \
  -d '{"task_id": "task_123"}'

# 6. Check state (should be "recording")
curl http://localhost:8080/rpc/state

# 7. Monitor statistics
curl http://localhost:8080/rpc/stats

# 8. Finish recording (but keep program running)
curl -X POST http://localhost:8080/rpc/finish \
  -H "Content-Type: application/json" \
  -d '{"task_id": "task_123"}'

# 9. Check state (should be "idle")
curl http://localhost:8080/rpc/state

# 10. Begin another recording
curl -X POST http://localhost:8080/rpc/begin \
  -H "Content-Type: application/json" \
  -d '{"task_id": "task_123"}'

# 11. When done, quit program
curl -X POST http://localhost:8080/rpc/quit
```

### Example 2: Python Client

```python
import requests
import json

BASE_URL = "http://localhost:8080"

def configure_recording(task_id, device_id, factory, operator_name, topics):
    """Configure recording task"""
    response = requests.post(
        f"{BASE_URL}/rpc/config",
        json={
            "task_config": {
                "task_id": task_id,
                "device_id": device_id,
                "factory": factory,
                "operator_name": operator_name,
                "topics": topics
            }
        }
    )
    return response.json()

def begin_recording(task_id):
    """Begin recording"""
    response = requests.post(
        f"{BASE_URL}/rpc/begin",
        json={"task_id": task_id}
    )
    return response.json()

def finish_recording(task_id):
    """Finish recording"""
    response = requests.post(
        f"{BASE_URL}/rpc/finish",
        json={"task_id": task_id}
    )
    return response.json()

def get_state():
    """Get current state"""
    response = requests.get(f"{BASE_URL}/rpc/state")
    return response.json()

def get_stats():
    """Get recording statistics"""
    response = requests.get(f"{BASE_URL}/rpc/stats")
    return response.json()

# Usage
configure_recording("task_123", "robot_01", "factory_01", "operator_01", ["/camera/image", "/lidar/scan"])
begin_recording("task_123")
print(f"State: {get_state()['data']['state']}")
finish_recording("task_123")
```

### Example 3: JavaScript/Node.js Client

```javascript
const axios = require('axios');

const BASE_URL = 'http://localhost:8080';

async function configureRecording(taskId, deviceId, factory, operatorName, topics) {
  const response = await axios.post(`${BASE_URL}/rpc/config`, {
    task_config: {
      task_id: taskId,
      device_id: deviceId,
      factory: factory,
      operator_name: operatorName,
      topics: topics
    }
  });
  return response.data;
}

async function beginRecording(taskId) {
  const response = await axios.post(`${BASE_URL}/rpc/begin`, {
    task_id: taskId
  });
  return response.data;
}

async function finishRecording(taskId) {
  const response = await axios.post(`${BASE_URL}/rpc/finish`, {
    task_id: taskId
  });
  return response.data;
}

async function getState() {
  const response = await axios.get(`${BASE_URL}/rpc/state`);
  return response.data;
}

// Usage
(async () => {
  await configureRecording('task_123', 'robot_01', 'factory_01', 'operator_01', ['/camera/image', '/lidar/scan']);
  await beginRecording('task_123');
  console.log('State:', (await getState()).data.state);
  await finishRecording('task_123');
})();
```

## Security Considerations

### Authentication

The current version does not implement authentication. Recommended authentication methods:

1. **JWT Token**: Pass JWT token in HTTP Header
   ```http
   POST /rpc/begin HTTP/1.1
   Authorization: Bearer eyJhbGciOiJIUzI1NiIs...
   ```

2. **API Key**: Pass via query parameter or Header
   ```http
   POST /rpc/begin?api_key=your_api_key HTTP/1.1
   ```

3. **Mutual TLS**: Two-way TLS authentication

### CORS

If accessing from a browser, configure CORS:

```cpp
response.set(http::field::access_control_allow_origin, "*");
response.set(http::field::access_control_allow_methods, "GET, POST, OPTIONS");
response.set(http::field::access_control_allow_headers, "Content-Type, Authorization");
```

## Rate Limiting

Recommend adding rate limiting to prevent abuse:

- Maximum 10 requests per second per client
- `/rpc/begin` and `/rpc/finish` require stricter limits

## Future Enhancements

### Planned Features

1. **WebSocket Support**: Real-time status updates and statistics push
2. **Batch Operations**: Configure multiple tasks in batch
3. **File Upload**: Upload MCAP files directly to server
4. **Task Queue**: Support task queues with sequential execution
5. **OAuth 2.0**: Integrate OAuth 2.0 authentication

## Appendix

### A. Complete Endpoint Reference

| Method | Endpoint | Description | Valid States |
|--------|----------|-------------|--------------|
| POST | `/rpc/config` | Set task configuration | Any |
| POST | `/rpc/begin` | Start recording | READY |
| POST | `/rpc/finish` | Finish recording, keep running | RECORDING, PAUSED |
| POST | `/rpc/quit` | Stop and exit program | Any |
| POST | `/rpc/pause` | Pause recording | RECORDING |
| POST | `/rpc/resume` | Resume recording | PAUSED |
| POST | `/rpc/cancel` | Cancel recording | RECORDING, PAUSED |
| POST | `/rpc/clear` | Clear configuration | READY |
| GET | `/rpc/state` | Get current state | Any |
| GET | `/rpc/stats` | Get statistics | Any |
| GET | `/` or `/health` | Health check | Any |

### B. HTTP Status Codes

| Code | Description |
|------|-------------|
| 200 | OK - Request successful |
| 400 | Bad Request - Invalid parameters or state |
| 404 | Not Found - Unknown endpoint |
| 500 | Internal Server Error |
| 501 | Not Implemented - Feature not available |

### C. Data Types

**TaskConfig:**
```typescript
interface TaskConfig {
  task_id: string;
  device_id: string;
  data_collector_id?: string;
  scene?: string;
  subscene?: string;
  skills?: string[];
  factory?: string;
  operator_name?: string;
  topics?: string[];
  start_callback_url?: string;
  finish_callback_url?: string;
  user_token?: string;
}
```

**RpcResponse:**
```typescript
interface RpcResponse {
  success: boolean;
  message: string;
  data: {
    state?: string;
    task_id?: string;
    running?: boolean;
    [key: string]: any;
  };
}
```

**Statistics:**
```typescript
interface Statistics {
  messages_received: number;
  messages_written: number;
  messages_dropped: number;
  bytes_written: number;
}
```

## Related Documentation

- [Frontend Design](frontend-design.md) - AxonPanel web control panel architecture and implementation
- [tools/axon_panel/](../../tools/axon_panel/) - Vue 3-based web interface source code

## Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-01-12 | - | Initial HTTP RPC API design |
