# Recording Service API Design

## Overview

This document defines the ROS service API for `axon_recorder`, designed for:
- **Production**: Server-controlled recording via ros-bridge (data collector pushes tasks)
- **Testing**: ROS-only integration testing (mock topics + mock config)

## Design Goals

1. **Server Integration First** - Designed for fleet management via ros-bridge
2. **Task-Centric** - One task = one MCAP file, identified by `task_id`
3. **Unified Control Interface** - Single service for all recording lifecycle operations
4. **Callback Notifications** - Server notified via HTTP callbacks on start/finish
5. **Simple State Machine** - Clear lifecycle: `idle` → `ready` → `recording` → `idle`
6. **Integration Testing Support** - ROS-only mode for testing without server

## Architecture

```
┌───────────────────────────────────────────────────────────────────────────┐
│                        Service Architecture                                │
├───────────────────────────────────────────────────────────────────────────┤
│                                                                            │
│   ┌──────────────────┐         ┌──────────────────┐                       │
│   │   ROS Client     │         │   Server (HTTP)  │                       │
│   │  (rosservice)    │         │  via ros-bridge  │                       │
│   └────────┬─────────┘         └────────┬─────────┘                       │
│            │                            │                                  │
│            └────────────┬───────────────┘                                  │
│                         ▼                                                  │
│            ┌──────────────────────────────┐                                │
│            │        axon_recorder         │                                │
│            │  ┌────────────────────────┐  │                                │
│            │  │ CachedRecordingConfig  │◄─┼── Push task config + callbacks │
│            │  ├────────────────────────┤  │                                │
│            │  │ IsRecordingReady       │◄─┼── Query recording readiness     │
│            │  ├────────────────────────┤  │                                │
│            │  │ RecordingControl       │◄─┼── Unified control interface    │
│            │  ├────────────────────────┤  │                                │
│            │  │ RecordingStatus        │◄─┼── Rich recording metrics       │
│            │  └────────────────────────┘  │                                │
│            └──────────────────────────────┘                                │
│                         │                                                  │
│                         ▼                                                  │
│            ┌────────────────────────┐                                      │
│            │  Callback to Server    │                                      │
│            │  (start/finish notify) │                                      │
│            └────────────────────────┘                                      │
└───────────────────────────────────────────────────────────────────────────┘
```

## State Machine

The state machine is **task-centric**: each task corresponds to one MCAP recording file. 
After a task completes (finish or cancel), the recorder returns to `idle` waiting for the next task from data collector.

```
        ┌─────────────────────────────────────────────────────┐
        │                                                     │
        ▼                                                     │
   ┌─────────┐                                                │
   │  IDLE   │ ◄──────────────────────────────────────────────┤
   └────┬────┘                                                │
        │ configure (CachedRecordingConfig)                   │
        ▼                                                     │
   ┌─────────┐                                                │
   │  READY  │────────(clear/timeout)─────────────────────────┤
   └────┬────┘                                                │
        │ start                                               │
        ▼                                                     │
   ┌─────────┐  pause   ┌─────────┐                           │
   │RECORDING│─────────►│ PAUSED  │                           │
   │         │◄─────────│         │                           │
   └────┬────┘  resume  └────┬────┘                           │
        │                    │                                │
        └────────┬───────────┘                                │
                 │                                            │
         ┌───────┴───────┐                                    │
         │               │                                    │
      finish           cancel                                 │
         │               │                                    │
         ▼               ▼                                    │
   (upload MCAP)    (cleanup)                                 │
         │               │                                    │
         └───────────────┴────────────────────────────────────┘
                         │
                  back to IDLE
              (ready for next task)
```

### State Descriptions

| State | Description |
|-------|-------------|
| `idle` | No active task, waiting for data collector to push configuration |
| `ready` | Task metadata received, MCAP file prepared, ready to start recording |
| `recording` | Actively recording ROS messages to MCAP file |
| `paused` | Recording paused, can resume or finish |

### State Transitions

| From | To | Trigger | Action |
|------|-----|---------|--------|
| `idle` | `ready` | `CachedRecordingConfig` service call | Cache task metadata, prepare MCAP file |
| `ready` | `idle` | `clear` command or timeout | Clear cached config |
| `ready` | `recording` | `start` command | Begin recording, POST to `start_callback_url` |
| `recording` | `paused` | `pause` command | Pause message subscription |
| `paused` | `recording` | `resume` command | Resume message subscription |
| `recording`/`paused` | `idle` | `finish` command | Finalize MCAP, upload, POST to `finish_callback_url` |
| `recording`/`paused` | `idle` | `cancel` command | Cleanup partial data, return to idle |

## Service Definitions

### 1. CachedRecordingConfig.srv

Caches configuration for an upcoming recording session. Supports server integration with callback URLs.
This configuration is subject to change until recording starts.

```
# Request - Cache configuration for upcoming recording
# ============================================================================

# Task & Device Identification
string task_id                # Server-assigned task identifier (required for server mode)
string device_id              # Robot/device identifier
string data_collector_id      # Data collector identifier

# Recording Context
string scene                  # Recording scene/context label
string subscene               # Recording subscene/sub-context label
string[] skills               # Associated skills for this recording
string organization           # Organization identifier

# Topic Selection
string[] topics               # Topics to record (empty = use default from YAML config)

# Server Callback Integration (leave empty for ROS-only mode)
string start_callback_url     # POST notification when recording starts
string finish_callback_url    # POST notification when recording finishes
string user_token             # JWT token for callback authentication

---
# Response
# ============================================================================
bool success                  # True if configuration cached successfully
string message                # Human-readable status message
```

### 2. IsRecordingReady.srv

Query whether the recorder has a cached recording configuration and is ready to start. Simple yes/no check for other services.

```
# Request (empty)
# ============================================================================
# No parameters needed - just check if recorder has cached config

---
# Response
# ============================================================================
bool success                  # True if query executed successfully
string message                # Human-readable status message

# Task Configuration Status
bool is_configured            # True if a recording config is cached (READY state)
bool is_recording             # True if currently recording

# Cached Configuration Details (populated only if is_configured=true)
string task_id                # Server-assigned task identifier
string device_id              # Robot/device identifier
string scene                  # Recording scene/context
string subscene               # Recording subscene/sub-context
string[] skills               # Associated skills
string organization           # Organization identifier
string data_collector_id      # Data collector identifier
string[] topics               # Topics to be recorded
```

### 3. RecordingControl.srv

Unified control interface for all recording lifecycle operations.

```
# Request - Control recording lifecycle
# ============================================================================

# Command Selection
string command                # "start", "pause", "resume", "cancel", "finish", "clear"

# Task Reference (for pause, resume, cancel, finish commands)
string task_id                # Task identifier from cached config
                              # Required for: pause, resume, cancel, finish
                              # Ignored for: start, clear

---
# Response
# ============================================================================
bool success                  # True if command executed successfully
string message                # Human-readable status message
string task_id                # Task identifier (echoed from request or from cached config)
```

### 4. RecordingStatus.srv

Rich status query with full session information and runtime metrics.

```
# Request - Query recording status
# ============================================================================
string task_id                # Optional: query specific task
                              # Empty = query current active task

---
# Response
# ============================================================================
bool success                  # True if status retrieved successfully
string message                # Human-readable status message

# State Machine
string status                 # "idle", "ready", "recording", "paused"

# Task Identification
string task_id                # Server-assigned task identifier
string device_id              # Robot/device identifier
string data_collector_id      # Data collector identifier

# Recording Context
string scene                  # Recording scene/context label
string subscene               # Recording subscene/sub-context label
string[] skills               # Associated skills
string organization           # Organization identifier

# Runtime Metrics
string[] active_topics        # Currently subscribed topics
string output_path            # Path to output file/directory
float64 disk_usage_gb         # Current disk usage in GB
float64 duration_sec          # Recording duration in seconds
int64 message_count           # Total messages recorded
float64 throughput_mb_sec     # Current throughput in MB/s

# Error Information
string last_error             # Most recent error message (empty if none)
```

## Callback Specification

When `start_callback_url` and `finish_callback_url` are provided, the recorder will POST notifications to these URLs.

### Start Callback

**Trigger**: When recording transitions from `idle` to `recording`

**Request**:
```json
POST {start_callback_url}
Authorization: Bearer {user_token}
Content-Type: application/json

{
  "task_id": "task_123",
  "device_id": "robot_01",
  "status": "recording",
  "started_at": "2025-12-20T14:30:52Z",
  "topics": ["topic1", "topic2"]
}
```

### Finish Callback

**Trigger**: When recording transitions to `finished` or `cancelled`

**Request**:
```json
POST {finish_callback_url}
Authorization: Bearer {user_token}
Content-Type: application/json

{
  "task_id": "task_123",
  "device_id": "robot_01",
  "status": "finished",
  "started_at": "2025-12-20T14:30:52Z",
  "finished_at": "2025-12-20T15:45:30Z",
  "duration_sec": 4478.0,
  "message_count": 1523456,
  "file_size_bytes": 2147483648,
  "output_path": "/data/recordings/task_123.mcap",
  "topics": ["topic1", "topic2"],
  "error": null
}
```

## Workflow Patterns

### Production: Server-Controlled Recording (Fleet Mode)

Two-phase workflow with explicit configuration step. Each task = one MCAP file.

```
                    ┌──────────────────────────────────────────────┐
                    │                                              │
                    ▼                                              │
IDLE ──(CachedRecordingConfig)──► READY ──(start)──► RECORDING ───┤
                    ▲                │                     │       │
                    │                │                     │       │
                    │         (clear/timeout)           (finish)   │
                    │                │                     │       │
                    │                ▼                     ▼       │
                    └────────────── IDLE ◄──── (upload + callback) │
                                     ▲                             │
                                     │         (cancel + cleanup)  │
                                     └─────────────────────────────┘
```

- Data collector pushes task metadata to device first
- Device enters `READY` state, waiting for start command
- `finish` triggers upload and POST to `finish_callback_url`
- After task completes, returns to `IDLE` for next task
- Suitable for fleet management and remote control

---

## Usage Examples

### Integration Testing: ROS-Only Mode

For integration testing, use ROS tools to simulate the server workflow:

```bash
# ============================================================================
# Step 1: Publish mock topics for testing
# ============================================================================
# Terminal 1: Publish mock sensor data
rostopic pub /camera/image sensor_msgs/Image "header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: 'camera'}" -r 10 &
rostopic pub /lidar/scan sensor_msgs/LaserScan "header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: 'lidar'}" -r 10 &

# ============================================================================
# Step 2: Push mock config (simulates server pushing task)
# ============================================================================
rosservice call /axon_recorder/cached_recording_config "{
  task_id: 'test_task_001',
  device_id: 'test_robot',
  scene: 'integration_test',
  subscene: 'basic_recording',
  skills: ['test'],
  organization: 'test_org',
  data_collector_id: 'test_collector',
  topics: ['/camera/image', '/lidar/scan'],
  start_callback_url: '',
  finish_callback_url: '',
  user_token: ''
}"
# Response: {success: true, message: "Config cached"}

# ============================================================================
# Step 3: Verify config is cached
# ============================================================================
rosservice call /axon_recorder/is_recording_ready "{}"
# Response: {success: true, is_configured: true, task_id: "test_task_001", ...}

# ============================================================================
# Step 4: Start recording
# ============================================================================
rosservice call /axon_recorder/recording_control "{command: 'start'}"
# Response: {success: true, message: "Recording started", task_id: "test_task_001"}

# ============================================================================
# Step 5: Check status during recording
# ============================================================================
rosservice call /axon_recorder/recording_status "{}"
# Response: {success: true, status: "recording", task_id: "test_task_001", ...}

# ============================================================================
# Step 6: Finish recording
# ============================================================================
rosservice call /axon_recorder/recording_control "{command: 'finish', task_id: 'test_task_001'}"
# Response: {success: true, message: "Recording finished", task_id: "test_task_001"}

# ============================================================================
# Step 7: Verify MCAP file created
# ============================================================================
ls -la /path/to/output/test_task_001.mcap
```

**Note**: In integration testing mode, leave `start_callback_url` and `finish_callback_url` empty to skip server callbacks.

### Example 2: Server-Controlled Recording via ros-bridge (C++)

```cpp
#include <ros/ros.h>
#include <axon_recorder/CachedRecordingConfig.h>
#include <axon_recorder/IsRecordingReady.h>
#include <axon_recorder/RecordingControl.h>
#include <axon_recorder/RecordingStatus.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "recording_client");
    ros::NodeHandle nh;

    // ========================================================================
    // Step 1: Server pushes task metadata to device
    //         State transition: IDLE → READY
    // ========================================================================
    ros::ServiceClient config_client = 
        nh.serviceClient<axon_recorder::CachedRecordingConfig>("/axon_recorder/cached_recording_config");
    
    axon_recorder::CachedRecordingConfig config_srv;
    config_srv.request.task_id = "task_123";
    config_srv.request.device_id = "robot_01";
    config_srv.request.scene = "warehouse_navigation";
    config_srv.request.subscene = "aisle_traversal";
    config_srv.request.skills = {"navigation", "obstacle_avoidance"};
    config_srv.request.organization = "acme_corp";
    config_srv.request.data_collector_id = "collector_01";
    config_srv.request.topics = {"/camera/image", "/lidar/scan", "/odom"};
    config_srv.request.start_callback_url = "http://server.example.com/api/v1/tasks/123/start-recording";
    config_srv.request.finish_callback_url = "http://server.example.com/api/v1/tasks/123/finish-recording";
    config_srv.request.user_token = "eyJhbGciOiJIUzI1NiIs...";

    if (!config_client.call(config_srv) || !config_srv.response.success) {
        ROS_ERROR("Failed to cache recording config: %s", config_srv.response.message.c_str());
        return 1;
    }
    std::string task_id = config_srv.request.task_id;  // Use task_id for subsequent commands
    ROS_INFO("Config cached for task_id: %s", task_id.c_str());
    // Device is now in READY state, waiting for start command

    // ========================================================================
    // Step 2: (Optional) Verify recorder has cached config before starting
    //         Other services can also query this to check if config is ready
    // ========================================================================
    ros::ServiceClient is_ready_client = 
        nh.serviceClient<axon_recorder::IsRecordingReady>("/axon_recorder/is_recording_ready");
    
    axon_recorder::IsRecordingReady is_ready_srv;
    if (is_ready_client.call(is_ready_srv) && is_ready_srv.response.is_configured) {
        ROS_INFO("Config cached: task_id=%s", is_ready_srv.response.task_id.c_str());
    } else {
        ROS_WARN("No config cached, recorder not ready");
    }

    // ========================================================================
    // Step 3: Server triggers recording start
    //         State transition: READY → RECORDING
    // ========================================================================
    ros::ServiceClient control_client = 
        nh.serviceClient<axon_recorder::RecordingControl>("/axon_recorder/recording_control");
    
    axon_recorder::RecordingControl start_srv;
    start_srv.request.command = "start";
    // No config_id needed - uses current cached config

    if (!control_client.call(start_srv) || !start_srv.response.success) {
        ROS_ERROR("Failed to start recording: %s", start_srv.response.message.c_str());
        return 1;
    }
    ROS_INFO("Recording started for task_id: %s", start_srv.response.task_id.c_str());
    // Recorder POSTs to start_callback_url

    // ========================================================================
    // Step 4: Monitor status during recording
    // ========================================================================
    ros::ServiceClient status_client = 
        nh.serviceClient<axon_recorder::RecordingStatus>("/axon_recorder/recording_status");
    
    axon_recorder::RecordingStatus status_srv;
    status_srv.request.task_id = task_id;
    if (status_client.call(status_srv)) {
        ROS_INFO("Status: %s, Messages: %ld", 
                 status_srv.response.status.c_str(),
                 status_srv.response.message_count);
    }

    // ========================================================================
    // Step 5: Pause/Resume if needed
    //         State transitions: RECORDING ↔ PAUSED
    // ========================================================================
    axon_recorder::RecordingControl pause_srv;
    pause_srv.request.command = "pause";
    pause_srv.request.task_id = task_id;
    control_client.call(pause_srv);

    axon_recorder::RecordingControl resume_srv;
    resume_srv.request.command = "resume";
    resume_srv.request.task_id = task_id;
    control_client.call(resume_srv);

    // ========================================================================
    // Step 6: Finish recording
    //         State transition: RECORDING → (upload) → IDLE
    //         Recorder uploads MCAP and POSTs to finish_callback_url
    //         Task complete, recorder ready for next task
    // ========================================================================
    axon_recorder::RecordingControl finish_srv;
    finish_srv.request.command = "finish";
    finish_srv.request.task_id = task_id;
    if (!control_client.call(finish_srv) || !finish_srv.response.success) {
        ROS_ERROR("Failed to finish recording: %s", finish_srv.response.message.c_str());
        return 1;
    }
    ROS_INFO("Recording finished for task_id: %s", task_id.c_str());

    // ========================================================================
    // Alternative: Clear config without starting (abort task)
    //              State transition: READY → IDLE
    // ========================================================================
    // axon_recorder::RecordingControl clear_srv;
    // clear_srv.request.command = "clear";
    // control_client.call(clear_srv);

    return 0;
}
```

### Example 3: ros-bridge WebSocket (JavaScript)

```javascript
const ROSLIB = require('roslib');

const ros = new ROSLIB.Ros({ url: 'ws://robot:9090' });

// Configure recording
const configService = new ROSLIB.Service({
  ros: ros,
  name: '/axon_recorder/cached_recording_config',
  serviceType: 'axon_recorder/CachedRecordingConfig'
});

configService.callService({
  task_id: 'task_123',
  device_id: 'robot_01',
  scene: 'warehouse_navigation',
  subscene: 'aisle_traversal',
  skills: ['navigation'],
  organization: 'acme_corp',
  data_collector_id: 'collector_01',
  topics: ['/camera/image', '/lidar/scan'],
  start_callback_url: 'http://server/api/tasks/123/start',
  finish_callback_url: 'http://server/api/tasks/123/finish',
  user_token: 'jwt_token_here'
}, (result) => {
  console.log('Config cached for task_id: task_123');
  
  // Start recording (uses cached config)
  const controlService = new ROSLIB.Service({
    ros: ros,
    name: '/axon_recorder/recording_control',
    serviceType: 'axon_recorder/RecordingControl'
  });
  
  controlService.callService({
    command: 'start'
  }, (startResult) => {
    console.log('Recording started for task_id:', startResult.task_id);
  });
});
```

## Command Reference

| Command | Required Fields | Valid From State | Description |
|---------|-----------------|------------------|-------------|
| `start` | - | `ready` | Start recording with cached config |
| `pause` | `task_id` | `recording` | Pause active recording |
| `resume` | `task_id` | `paused` | Resume paused recording |
| `cancel` | `task_id` | `recording`, `paused` | Cancel task, cleanup, return to `idle` |
| `finish` | `task_id` | `recording`, `paused` | Finalize MCAP, upload, return to `idle` |
| `clear` | - | `ready` | Clear cached config, return to `idle` |

## Error Handling

### Error Codes in Response Message

| Error Pattern | Description |
|---------------|-------------|
| `ERR_INVALID_COMMAND` | Unknown command string |
| `ERR_INVALID_STATE` | Command not valid in current state |
| `ERR_RECORDING_NOT_FOUND` | Recording ID does not exist |
| `ERR_CONFIG_NOT_FOUND` | Config ID does not exist |
| `ERR_DISK_FULL` | Insufficient disk space |
| `ERR_TOPIC_NOT_FOUND` | Requested topic does not exist |
| `ERR_CALLBACK_FAILED` | Server callback POST failed |

### State Transition Errors

| Current State | Invalid Commands |
|---------------|------------------|
| `idle` | `start`, `pause`, `resume`, `cancel`, `finish`, `clear` |
| `ready` | `pause`, `resume`, `cancel`, `finish` |
| `recording` | `start`, `resume`, `clear` |
| `paused` | `start`, `pause`, `clear` |

*Note: `start` requires recorder to be in `ready` state (config must be cached first via `CachedRecordingConfig`).

## Migration from Legacy API

### Deprecated Services

The following services are deprecated and will be removed in a future version:

| Legacy Service | Replacement |
|----------------|-------------|
| `StartRecording.srv` | `RecordingControl.srv` with `command: "start"` |
| `StopRecording.srv` | `RecordingControl.srv` with `command: "finish"` |
| `GetStatus.srv` | `RecordingStatus.srv` |
| `UpdateConfig.srv` | `CachedRecordingConfig.srv` |

### Migration Example

**Before (Legacy)**:
```bash
rosservice call /axon_recorder/start_recording "{config_path: '/path/to/config.yaml'}"
rosservice call /axon_recorder/stop_recording "{}"
```

**After (New API)**:
```bash
rosservice call /axon_recorder/recording_control "{command: 'start'}"
rosservice call /axon_recorder/recording_control "{command: 'finish', recording_id: 'rec_001'}"
```

## Configuration File Support

The new API maintains support for YAML configuration files for default settings (e.g., output directory, topics).

Configuration priority (highest to lowest):
1. Parameters in `CachedRecordingConfig` request (from server)
2. Default YAML configuration file (for unspecified parameters)

## Appendix: Service File Locations

```
middlewares/axon_recorder/srv/
├── CachedRecordingConfig.srv   # Configuration caching
├── IsRecordingReady.srv        # Recording readiness query
├── RecordingControl.srv   # Unified control
└── RecordingStatus.srv    # Status query
```

## Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-20 | - | Initial design |

