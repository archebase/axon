# Axon - Universal Middleware Plugin System

**Axon** is a universal middleware plugin system for recording data from different middleware (ROS2, ROS1) to MCAP format without knowing message types at compile time.

## 🌟 Key Features

- ✅ **Universal Subscription** - Subscribe to ANY ROS1 or ROS2 message type via string
- ✅ **Type-Agnostic** - No need to know message structure at compile time
- ✅ **Dynamic Loading** - Load plugins at runtime via `dlopen()`
- ✅ **Zero Compile-Time Dependencies** - Test program only needs `libdl`
- ✅ **Dual ROS Support** - Single binary works with both ROS1 and ROS2

## Quick Start

### Using Makefile (Recommended)

```bash
# Show all available commands
make help

# Build ROS2 plugin and unified test program
make build

# Run unified test with ROS2
make run-ros2

# Run unified test with ROS1
make run-ros1

# Clean all build artifacts
make clean
```

### Manual Build

```bash
# Build ROS2 plugin
./scripts/build.sh

# Build unified test program
cd examples && ./build.sh

# Run with ROS2
./run_ros2.sh /chatter std_msgs/msg/String

# Run with ROS1
./run_ros1.sh /chatter std_msgs/String
```

## Project Structure

```
axon/
├── middlewares/    # 🌟 Middleware implementations
│   ├── ros2/       # ROS2 plugin workspace
│   └── ros1/       # ROS1 plugin workspace
├── examples/       # 🌟 Unified test program (NO ROS deps at compile time!)
├── scripts/        # Build and clean scripts
├── Makefile        # 🌟 Unified build system
├── CLAUDE.md       # Complete technical documentation
└── README.md       # This file
```

## The Magic: Zero Compile-Time ROS Dependencies

The unified test program in `examples/` has **ZERO compile-time dependencies** on ROS1 or ROS2!

- Compiles with only `libdl` (dynamic loading library)
- Loads ROS1 or ROS2 plugin at runtime via `dlopen()`
- Works with ANY ROS message type without knowing it at compile time

## Documentation

- **[CLAUDE.md](CLAUDE.md)** - Complete technical documentation
- **[ARCH.md](ARCH.md)** - Original project requirements
- **[MCAP_INTEGRATION.md](MCAP_INTEGRATION.md)** - MCAP integration guide

## Example Usage

### ROS2

```bash
# Terminal 1: Publish messages
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello World'"

# Terminal 2: Run subscriber (works with any message type!)
cd examples
./run_ros2.sh /scan sensor_msgs/msg/LaserScan
```

### ROS1

```bash
# Terminal 1: Start ROS1 master
roscore

# Terminal 2: Publish messages
rostopic pub /chatter std_msgs/String "data: 'Hello'" -r 1

# Terminal 3: Run subscriber (works with any message type!)
cd examples
./run_ros1.sh /chatter std_msgs/String
```

## Key Benefit

**One binary, unlimited possibilities:**

```bash
# The same subscriber_test binary works with ALL of these:
./subscriber_test ros2 /chatter std_msgs/msg/String
./subscriber_test ros2 /scan sensor_msgs/msg/LaserScan
./subscriber_test ros2 /odom nav_msgs/msg/Odometry
./subscriber_test ros2 /camera sensor_msgs/msg/Image
./subscriber_test ros2 /custom_topic my_custom_pkg/msg/CustomMessage
```

## Status

- ✅ **Production Ready** - Fully functional and tested
- ✅ **ROS2 Support** - Tested with Humble/Iron/Jazzy
- ✅ **ROS1 Support** - Tested with Noetic/Melodic
- ✅ **Zero Dependencies** - Test program only needs `libdl`

## License

Apache-2.0
