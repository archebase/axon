# Axon ROS Packages

ROS-specific packages for Axon data recorder.

## Overview

This directory contains ROS packages that provide the recorder nodes and ROS-specific utilities.

## Packages

### axon_recorder

Unified ROS recorder that works with both ROS 1 (Noetic) and ROS 2 (Humble/Jazzy/Rolling):

- `ros_interface.cpp/.hpp` - ROS abstraction layer (ROS 1/ROS 2 via macros)
- `message_registry.cpp/.hpp` - Message type registry
- `ros_introspection.cpp/.hpp` - Message introspection
- `message_factory.cpp/.hpp` - Message creation/deserialization
- `register_common_messages.cpp/.hpp` - Common message type registration
- `recorder_node.cpp/.hpp` - Main recorder node
- `recorder_service.cpp/.hpp` - Service handlers

## Directory Structure

```
ros/
├── README.md
└── axon_recorder/
    ├── src/
    │   ├── ros_interface.cpp/.hpp
    │   ├── message_registry.cpp/.hpp
    │   ├── ros_introspection.cpp/.hpp
    │   ├── message_factory.cpp/.hpp
    │   ├── register_common_messages.cpp/.hpp
    │   ├── recorder_node.cpp/.hpp
    │   └── recorder_service.cpp/.hpp
    ├── launch/
    │   ├── recorder.launch        # ROS 1 (XML)
    │   └── recorder.launch.xml    # ROS 2 (XML)
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

### ROS 2

```bash
ros2 launch axon_recorder recorder.launch.xml
```

## Services

The recorder provides the following ROS services:

| Service | Description |
|---------|-------------|
| `/axon_recorder/start_recording` | Start recording with optional config |
| `/axon_recorder/stop_recording` | Stop recording and flush data |
| `/axon_recorder/get_status` | Get current recording status |
| `/axon_recorder/update_config` | Update configuration at runtime |

## Compile-Time ROS Selection

The package uses preprocessor macros (`AXON_ROS1`/`AXON_ROS2`) for version-specific code. The CMakeLists.txt automatically detects the ROS version from the `ROS_VERSION` environment variable.

## See Also

- [C++ SDK](../cpp/README.md)
- [C SDK (FFI Bridge)](../c/README.md)
- [Main README](../README.md)
