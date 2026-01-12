# HTTP RPC API Design for Axon Recorder

**Date:** 2025-01-12

## Overview

本文档定义了 Axon Recorder 的 HTTP RPC API 设计，用于替代原有的 ROS 服务接口。该设计提供了基于 JSON 的 RESTful API，支持通过 HTTP 协议进行远程录制控制。

## Design Goals

1. **HTTP-Based Control**: 使用 HTTP 协议替代 ROS 服务，降低依赖
2. **JSON Format**: 统一使用 JSON 格式进行数据交换
3. **RESTful Design**: 遵循 REST 设计原则，使用合适的 HTTP 方法和路径
4. **State Machine Alignment**: 与状态机设计紧密对齐，确保状态转换的正确性
5. **Remote Control**: 支持服务器端远程控制录制任务

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

HTTP RPC API 与状态机紧密集成：

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
| `IDLE` | 初始状态，等待任务配置 |
| `READY` | 任务配置已缓存，等待开始录制 |
| `RECORDING` | 正在录制 ROS 消息到 MCAP 文件 |
| `PAUSED` | 录制已暂停，可以恢复或完成 |

## RPC Endpoints

### 1. POST /rpc/config

设置任务配置（缓存元数据），状态转换：**IDLE → READY**

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
| `task_id` | string | Yes | 服务器分配的任务标识符 |
| `device_id` | string | Yes | 机器人/设备标识符 |
| `data_collector_id` | string | No | 数据采集器标识符 |
| `scene` | string | No | 录制场景/上下文标签 |
| `subscene` | string | No | 录制子场景 |
| `skills` | array | No | 关联的技能列表 |
| `topics` | array | No | 要录制的主题列表 |
| `start_callback_url` | string | No | 开始录制回调 URL |
| `finish_callback_url` | string | No | 完成录制回调 URL |
| `user_token` | string | No | 回调认证的 JWT 令牌 |

---

### 2. POST /rpc/begin

开始录制，状态转换：**READY → RECORDING**

**Request:**
```http
POST /rpc/begin HTTP/1.1
Content-Type: application/json

{
  "task_id": "task_123"
}
```

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

**Error Response (400 Bad Request):**
```json
{
  "success": false,
  "message": "Cannot start recording from state: idle. Must be in READY state (call /rpc/config first).",
  "data": {
    "state": "idle"
  }
}
```

**Valid From States:**
- ✅ `READY` (必须先调用 /rpc/config)
- ❌ `IDLE` (必须先配置任务)
- ❌ `RECORDING` (已在录制)
- ❌ `PAUSED` (已暂停)

---

### 3. POST /rpc/finish

结束录制，状态转换：**RECORDING/PAUSED → IDLE**

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
- 停止录制，保存 MCAP 文件
- 注入元数据，生成 sidecar JSON
- 状态回到 IDLE
- **程序继续运行**，HTTP 服务器继续监听
- 可以再次调用 `/rpc/begin` 开始新的录制

---

### 4. POST /rpc/quit

退出程序（先保存数据）

**Request:**
```http
POST /rpc/quit HTTP/1.1
Content-Type: application/json

{}
```

**Note:** 此接口不需要 `task_id` 参数，可以在任何状态下调用。

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
1. 如果正在录制，先自动停止录制并保存数据
2. 停止 HTTP 服务器
3. 程序优雅退出

**Valid From States:**
- ✅ 任何状态（IDLE, READY, RECORDING, PAUSED）

---

### 5. POST /rpc/pause

暂停录制，状态转换：**RECORDING → PAUSED**

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
- ❌ 其他状态

---

### 6. POST /rpc/resume

恢复录制，状态转换：**PAUSED → RECORDING**

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
- ❌ 其他状态

---

### 7. POST /rpc/cancel

取消录制（丢弃部分数据），状态转换：**RECORDING/PAUSED → IDLE**

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
- TODO: 当前实现等同于 finish，应该丢弃部分录制
- 未来应该删除不完整的 MCAP 文件

**Valid From States:**
- ✅ `RECORDING`
- ✅ `PAUSED`
- ❌ 其他状态

---

### 8. POST /rpc/clear

清除配置，状态转换：**READY → IDLE**

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
- TODO: 重置任务配置，返回到 IDLE 状态
- 用于放弃未开始的录制任务

**Valid From States:**
- ✅ `READY`
- ❌ 其他状态

---

### 9. GET /rpc/state

查询当前状态

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
      "topics": ["/camera/image", "/lidar/scan", "/odom"]
    }
  }
}
```

**State Values:**
- `"idle"` - 空闲状态，无配置
- `"ready"` - 已配置，等待开始（包含 task_config）
- `"recording"` - 正在录制（包含 task_config）
- `"paused"` - 已暂停（包含 task_config）

**Notes:**
- 当状态为 `IDLE` 时，`task_config` 字段不存在
- 当状态为 `READY`、`RECORDING` 或 `PAUSED` 时，响应包含任务配置信息
- `task_config` 仅包含业务字段（task_id, device_id, scene, topics 等），不包含回调 URL 和认证令牌

---

### 10. GET /rpc/stats

查询录制统计信息

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
| `messages_received` | uint64 | 接收到的消息总数 |
| `messages_written` | uint64 | 写入 MCAP 的消息数 |
| `messages_dropped` | uint64 | 丢弃的消息数（队列满） |
| `bytes_written` | uint64 | 写入的字节数 |

---

### 11. GET / or /health

健康检查

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

所有错误响应遵循统一格式：

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
      "topics": ["/camera/image", "/lidar/scan"],
      "start_callback_url": "http://server/api/start",
      "finish_callback_url": "http://server/api/finish",
      "user_token": "jwt_token_here"
    }
  }'

# 4. Check state (should be "ready")
curl http://localhost:8080/rpc/state

# 5. Begin recording
curl -X POST http://localhost:8080/rpc/begin

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
curl -X POST http://localhost:8080/rpc/begin

# 11. When done, quit program
curl -X POST http://localhost:8080/rpc/quit
```

### Example 2: Python Client

```python
import requests
import json

BASE_URL = "http://localhost:8080"

def configure_recording(task_id, device_id, topics):
    """Configure recording task"""
    response = requests.post(
        f"{BASE_URL}/rpc/config",
        json={
            "task_config": {
                "task_id": task_id,
                "device_id": device_id,
                "topics": topics
            }
        }
    )
    return response.json()

def begin_recording():
    """Begin recording"""
    response = requests.post(f"{BASE_URL}/rpc/begin")
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
configure_recording("task_123", "robot_01", ["/camera/image", "/lidar/scan"])
begin_recording()
print(f"State: {get_state()['data']['state']}")
finish_recording("task_123")
```

### Example 3: JavaScript/Node.js Client

```javascript
const axios = require('axios');

const BASE_URL = 'http://localhost:8080';

async function configureRecording(taskId, deviceId, topics) {
  const response = await axios.post(`${BASE_URL}/rpc/config`, {
    task_config: {
      task_id: taskId,
      device_id: deviceId,
      topics: topics
    }
  });
  return response.data;
}

async function beginRecording() {
  const response = await axios.post(`${BASE_URL}/rpc/begin`);
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
  await configureRecording('task_123', 'robot_01', ['/camera/image', '/lidar/scan']);
  await beginRecording();
  console.log('State:', (await getState()).data.state);
  await finishRecording('task_123');
})();
```

## Security Considerations

### Authentication

当前版本未实现认证。建议的认证方式：

1. **JWT Token**: 在 HTTP Header 中传递 JWT token
   ```http
   POST /rpc/begin HTTP/1.1
   Authorization: Bearer eyJhbGciOiJIUzI1NiIs...
   ```

2. **API Key**: 通过查询参数或 Header 传递
   ```http
   POST /rpc/begin?api_key=your_api_key HTTP/1.1
   ```

3. **Mutual TLS**: 双向 TLS 认证

### CORS

如果需要从浏览器访问，需要配置 CORS：

```cpp
response.set(http::field::access_control_allow_origin, "*");
response.set(http::field::access_control_allow_methods, "GET, POST, OPTIONS");
response.set(http::field::access_control_allow_headers, "Content-Type, Authorization");
```

## Rate Limiting

建议添加速率限制以防止滥用：

- 每个客户端每秒最多 10 个请求
- `/rpc/begin` 和 `/rpc/finish` 需要更严格的限制

## Future Enhancements

### Planned Features

1. **WebSocket Support**: 实时状态更新和统计推送
2. **Batch Operations**: 批量配置多个任务
3. **File Upload**: 直接上传 MCAP 文件到服务器
4. **Task Queue**: 支持任务队列，按顺序执行
5. **OAuth 2.0**: 集成 OAuth 2.0 认证

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

## Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-01-12 | - | Initial HTTP RPC API design |
