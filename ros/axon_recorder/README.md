# Axon Recorder

High-performance ROS data recorder that captures topic data and writes to MCAP format.

## Features

- **Unified codebase**: Single package supports both ROS 1 (Noetic) and ROS 2 (Humble/Jazzy/Rolling)
- **MCAP format**: Efficient append-only container format compatible with Foxglove Studio
- **Zero-copy pipeline**: Lock-free queues for high-throughput recording
- **Server integration**: HTTP callbacks for recording lifecycle events
- **Service API**: Control recording via ROS services or ros-bridge

## Quick Start with Docker

```bash
cd ros/docker

# Build and run (ROS 2 Humble)
docker-compose -f docker-compose.yml up ros2-humble --build

# Run tests
docker-compose -f docker-compose.test.yml up test-ros2-humble --build --abort-on-container-exit
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

## State Machine

```
IDLE → READY → RECORDING ↔ PAUSED → IDLE
       ↓                           ↑
       └─────── (clear) ───────────┘
```

## Documentation

- [Recording Service API Design](../../docs/recording-service-api-design-2025-12-20.md)
- [Test Documentation](test/README.md)
