# Axon Recorder

High-performance ROS data recorder that captures topic data and writes to MCAP format.

## Features

- **Unified codebase**: Single package supports both ROS 1 (Noetic) and ROS 2 (Humble/Jazzy/Rolling)
- **MCAP format**: Efficient append-only container format compatible with Foxglove Studio
- **Zero-copy pipeline**: Lock-free queues for high-throughput recording
- **Server integration**: HTTP callbacks for recording lifecycle events
- **Service API**: Control recording via ROS services or ros-bridge
- **Modular design**: Clean separation of concerns with testable components

## Quick Start with Docker

```bash
cd docker

# Build and run (ROS 2 Humble)
docker-compose -f docker-compose.yml up ros2-humble --build

# Run tests
docker-compose -f docker-compose.test.yml up test-ros2-humble --build --abort-on-container-exit --remove-orphans
```

## Building Locally

```bash
# ROS 1
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws && catkin_make

# ROS 2
source /opt/ros/humble/setup.bash
cd ~/ros2_ws && colcon build --packages-select axon_recorder
```

## Usage

```bash
# ROS 1
roslaunch axon_recorder recorder.launch.ros1.xml

# ROS 2
ros2 launch axon_recorder recorder.launch.ros2.xml
```

## Configuration

See `config/default_config.yaml`:

```yaml
dataset:
  path: /data/recordings/dataset.mcap

topics:
  - name: /camera/image_raw
    message_type: sensor_msgs/Image
    batch_size: 100

recording:
  max_disk_usage_gb: 100
```

## Service API

| Service | Description |
|---------|-------------|
| `~/cached_recording_config` | Cache task configuration from server |
| `~/is_recording_ready` | Check if recorder has cached config |
| `~/recording_control` | Control recording (start/pause/resume/finish/cancel/clear) |
| `~/recording_status` | Get current status and metrics |

## Architecture

The recorder follows a modular design with clear separation of concerns:

### Component Overview

| Component | File | Responsibility |
|-----------|------|----------------|
| **RecorderNode** | `recorder_node.hpp` | Main coordinator, implements `IRecorderContext` |
| **RecordingSession** | `recording_session.hpp` | MCAP file lifecycle, message writing |
| **TopicManager** | `topic_manager.hpp` | ROS subscription management |
| **WorkerThreadPool** | `worker_thread_pool.hpp` | Parallel message processing with bounded threads |
| **StateManager** | `state_machine.hpp` | Single source of truth for recording state |
| **ServiceAdapter** | `service_adapter.hpp` | ROS service registration and version abstraction |
| **RecordingServiceImpl** | `recording_service_impl.hpp` | Service business logic |

### Interface Pattern

Services depend on `IRecorderContext` interface rather than concrete `RecorderNode`:

```
┌─────────────────────────────────────────────────────────────────┐
│                       RecorderNode                              │
│  (implements IRecorderContext, enable_shared_from_this)         │
├─────────────────────────────────────────────────────────────────┤
│  ┌──────────────┐  ┌──────────────┐  ┌─────────────────────┐   │
│  │ Recording    │  │ Topic        │  │ Worker              │   │
│  │ Session      │  │ Manager      │  │ ThreadPool          │   │
│  │ (MCAP)       │  │ (Subs)       │  │ (Queues)            │   │
│  └──────────────┘  └──────────────┘  └─────────────────────┘   │
│  ┌──────────────┐  ┌──────────────┐  ┌─────────────────────┐   │
│  │ State        │  │ TaskConfig   │  │ HTTP Callback       │   │
│  │ Manager      │  │ Cache        │  │ Client              │   │
│  └──────────────┘  └──────────────┘  └─────────────────────┘   │
└────────────────────────────┬────────────────────────────────────┘
                             │ IRecorderContext (shared_ptr)
                             ▼
              ┌───────────────────────────────┐
              │        ServiceAdapter         │
              │  ┌───────────────────────┐   │
              │  │ RecordingServiceImpl  │   │
              │  └───────────────────────┘   │
              └───────────────────────────────┘
```

### Data Flow

```
[ROS Topics] → [TopicManager] → [Zero-copy Callback] → [SPSC Queue]
                                                              │
                                                              ▼
[MCAP File] ← [RecordingSession] ← [WorkerThreadPool] ← [Drain Queue]
```

### Threading Model

- **ROS executor threads**: Handle subscription callbacks (parallel per-topic)
- **Worker pool (1 per topic)**: Drain lock-free queues and write to MCAP
- **Lock-free SPSC queues**: Per-topic for zero-copy message transfer

### Key Design Decisions

1. **Single Source of Truth**: `StateManager` is the only place that tracks recording state (IDLE, READY, RECORDING, PAUSED). The `is_recording()` and `is_paused()` methods derive from `StateManager`.

2. **Dependency Injection**: `RecordingServiceImpl` and `ServiceAdapter` depend on `IRecorderContext` interface (via `shared_ptr`), enabling mock implementations for testing.

3. **Transaction Guards**: `StateTransactionGuard` provides RAII-style rollback for operations that span state transitions.

4. **Factory Pattern**: `RecorderNode::create()` returns a `shared_ptr` to enable `shared_from_this()` for safe dependency injection.

## State Machine

```
IDLE → READY → RECORDING ↔ PAUSED → IDLE
       ↓                           ↑
       └─────── (clear) ───────────┘
```

## Documentation

- [Recording Service API Design](../../docs/recording-service-api-design-2025-12-20.md)
- [Test Documentation](test/README.md)
