# ROS1 Middleware

This directory contains ROS1 (Noetic) specific middleware implementations for the Axon recorder.

## Overview

The ROS1 plugin provides a dynamic library (`libaxon_ros1_plugin.so`) that allows the Axon recorder to subscribe to any ROS1 topic using a type-erased subscription pattern similar to the ROS2 plugin.

## Architecture

```
middlewares/ros1/
└── src/ros1_plugin/          # ROS1 plugin implementation
    ├── include/              # Public header files
    │   ├── ros1_plugin.hpp               # Main plugin class interface
    │   └── ros1_subscription_wrapper.hpp # Subscription management interface
    ├── src/                  # Implementation files
    │   ├── ros1_plugin.cpp              # Main plugin implementation
    │   ├── ros1_subscription_wrapper.cpp # Subscription manager implementation
    │   └── ros1_plugin_export.cpp        # C ABI export functions
    ├── CMakeLists.txt        # Build configuration
    └── package.xml           # ROS1 package manifest
```

## Key Design Patterns

1. **Plugin Architecture**: Clean separation between C ABI and C++ implementation
2. **Type Erasure**: `topic_tools::ShapeShifter` handles any ROS1 message type
3. **Thread Safety**: Mutex-protected subscription management
4. **RAII**: Smart pointers for automatic resource cleanup
5. **JSON Configuration**: Flexible plugin configuration via JSON
6. **Asynchronous Execution**: `ros::AsyncSpinner` for callback handling

## Key Components

### Ros1Plugin Class

Manages the ROS1 node lifecycle and async spinner:

- Initializes ROS1 with optional node name configuration
- Creates `ros::NodeHandle` for subscriptions
- Manages `ros::AsyncSpinner` for callback processing
- Delegates subscription management to `SubscriptionManager`

### SubscriptionManager Class

Handles type-erased subscriptions using `topic_tools::ShapeShifter`:

- Subscribes to any ROS1 message type without compile-time dependency
- Serializes messages using `ros::serialization`
- Extracts raw bytes and passes to user callback
- Thread-safe subscription management with mutex

### C ABI Interface

The plugin exports a C ABI interface defined in `ros1_plugin_export.cpp`:

- `axon_init(const char* config_json)`: Initialize plugin with JSON config
- `axon_start()`: Start async spinner
- `axon_stop()`: Stop plugin and cleanup
- `axon_subscribe()`: Subscribe to topic with callback
- `axon_publish()`: Publish message (placeholder, not implemented)
- `axon_get_plugin_descriptor()`: Export plugin metadata

## Building

**Note:** Building the ROS1 plugin requires either:
1. A ROS Noetic environment installed (Ubuntu 20.04)
2. Docker container (recommended for CI/testing)

### Using Docker (Recommended)

```bash
# From project root
make docker-build-ros1

# Or use docker-compose
cd docker
docker-compose build ros1-noetic
```

### Using catkin (Local ROS Noetic Required)

```bash
# Source ROS1 Noetic
source /opt/ros/noetic/setup.bash

# Navigate to ros1 middleware directory
cd middlewares/ros1

# Build with catkin
catkin build

# Or with catkin_make (if no catkin_tools installed)
mkdir -p build
cd build
cmake ../src/ros1_plugin
make
```

### Using top-level Makefile (Local ROS Noetic Required)

```bash
# From project root - source ROS first
source /opt/ros/noetic/setup.bash

# Build ROS1 middleware
make build-ros1
```

## Configuration

The plugin accepts JSON configuration on initialization:

```json
{
  "node_name": "optional_custom_node_name"
}
```

Default node name: `"axon_ros1_plugin"`

## Message Serialization

ROS1 uses a different serialization approach than ROS2:

1. **ROS2**: Uses `rclcpp::SerializedMessage` with direct buffer access
2. **ROS1**: Uses `topic_tools::ShapeShifter` with `ros::serialization`

The ROS1 plugin serializes messages using:

```cpp
uint32_t serial_size = ros::serialization::serializationLength(*msg);
std::vector<uint8_t> data(serial_size);
ros::serialization::OStream stream(data.data(), serial_size);
ros::serialization::serialize(stream, *msg);
```

## Threading Model

| Component | Threading |
|-----------|-----------|
| ROS1 Master | External (roscore) |
| AsyncSpinner | Single-threaded (configurable) |
| Callbacks | Executed in AsyncSpinner thread |
| Subscription Manager | Mutex-protected |

## Dependencies

- **roscpp**: ROS1 C++ client library
- **roslib**: ROS1 utilities
- **topic_tools**: Provides `ShapeShifter` for type-erased subscriptions
- **std_msgs**, **sensor_msgs**: Common message types
- **nlohmann/json**: JSON parsing (fetched via FetchContent)

## Differences from ROS2 Plugin

| Feature | ROS1 Plugin | ROS2 Plugin |
|---------|-------------|-------------|
| Node Handle | `ros::NodeHandlePtr` | `rclcpp::Node::SharedPtr` |
| Executor | `ros::AsyncSpinner` | `rclcpp::SingleThreadedExecutor` |
| Generic Subscription | `topic_tools::ShapeShifter` | `rclcpp::GenericSubscription` |
| Serialization | `ros::serialization` | `rclcpp::SerializedMessage` |
| Timestamp | `ros::Time::now().toNSec()` | `rclcpp::Clock().now().nanoseconds()` |
| Logging | `ROS_INFO/ERROR/WARN` | `RCUTILS_LOG_INFO/ERROR/WARN` |
| Build System | catkin | ament_cmake |

## Testing

The plugin is tested through:

1. Integration tests in the main test suite
2. Docker-based CI testing with ROS1 Noetic
3. Manual testing with ROS1 bags and live topics

## Future Work

- [ ] Implement `axon_publish()` for message publishing
- [ ] Add service support (server/client)
- [ ] Add QoS configuration options
- [ ] Performance benchmarking against ROS2 plugin
