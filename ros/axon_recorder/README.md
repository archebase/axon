# Axon Recorder

ROS data recorder that captures topic data and writes to MCAP format.

## Overview

`axon_recorder` is a unified ROS package that works with both ROS 1 (Noetic) and ROS 2 (Humble/Jazzy/Rolling). It subscribes to configured topics and writes data to MCAP files for efficient storage and playback.

## Features

- **Unified codebase**: Single package supports both ROS 1 and ROS 2
- **Compile-time ROS selection**: Uses preprocessor macros (`AXON_ROS1`/`AXON_ROS2`) for version-specific code
- **MCAP format**: Efficient append-only container format compatible with Foxglove Studio
- **Configurable batching**: Control batch sizes and flush intervals per topic
- **Service interface**: Start/stop recording, update config at runtime

## Directory Structure

```
axon_recorder/
├── src/
│   ├── ros_interface.hpp/cpp        # ROS abstraction layer
│   ├── message_registry.hpp/cpp     # Message type registry
│   ├── ros_introspection.hpp/cpp    # Message introspection
│   ├── message_factory.hpp/cpp      # Message creation/deserialization
│   ├── register_common_messages.hpp/cpp  # Common message type registration
│   ├── recorder_node.hpp/cpp        # Main recorder node
│   └── recorder_service.hpp/cpp     # Service handlers
├── launch/
    │   ├── recorder.launch              # ROS 1 launch file (XML)
    │   └── recorder.launch.xml          # ROS 2 launch file (XML)
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Building

### ROS 1 (Noetic)

```bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
catkin_make
```

### ROS 2 (Humble/Jazzy/Rolling)

```bash
source /opt/ros/<distro>/setup.bash
cd ~/ros2_ws
colcon build --packages-select axon_recorder
```

## Usage

### ROS 1

```bash
roslaunch axon_recorder recorder.launch
```

With custom parameters:

```bash
roslaunch axon_recorder recorder.launch \
    config_path:=/path/to/config.yaml \
    dataset_path:=/data/my_dataset.mcap \
    auto_start:=false
```

### ROS 2

```bash
ros2 launch axon_recorder recorder.launch.xml
```

With custom parameters:

```bash
ros2 launch axon_recorder recorder.launch.xml \
    config_path:=/path/to/config.yaml \
    dataset_path:=/data/my_dataset.mcap \
    auto_start:=false
```

## Configuration

See `config/default_config.yaml` for configuration options:

```yaml
dataset:
  path: /data/recordings/dataset.mcap

topics:
  - name: /camera/image_raw
    message_type: sensor_msgs/Image  # or sensor_msgs/msg/Image for ROS 2
    batch_size: 100
    flush_interval_ms: 1000

recording:
  auto_start: true
```

## Services

The recorder provides the following services:

| Service | Description |
|---------|-------------|
| `/axon_recorder/start_recording` | Start recording with optional config |
| `/axon_recorder/stop_recording` | Stop recording and flush data |
| `/axon_recorder/get_status` | Get current recording status |
| `/axon_recorder/update_config` | Update configuration at runtime |

## Architecture

The recorder uses a layered architecture:

1. **RosInterface**: Abstract interface for ROS operations, with ROS 1 and ROS 2 implementations selected at compile time
2. **MessageFactory**: Creates and deserializes typed messages dynamically
3. **MessageRegistry**: Maps message types to serialization handlers
4. **MCAP Writer**: Thread-safe MCAP file writer with compression support

## Compile-Time ROS Selection

The package uses preprocessor macros to select ROS-version-specific code:

```cpp
#if defined(AXON_ROS1)
    #include <ros/ros.h>
    // ROS 1 specific code
#elif defined(AXON_ROS2)
    #include <rclcpp/rclcpp.hpp>
    // ROS 2 specific code
#endif
```

The CMakeLists.txt automatically detects the ROS version from the `ROS_VERSION` environment variable.

## See Also

- [C++ SDK](../../cpp/README.md)
- [C SDK (FFI Bridge)](../../c/README.md)
- [Main README](../../README.md)
