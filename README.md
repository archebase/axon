# Axon

[![CI](https://github.com/ArcheBase/Axon/actions/workflows/ci.yml/badge.svg)](https://github.com/ArcheBase/Axon/actions/workflows/ci.yml)
[![codecov](https://codecov.io/gh/ArcheBase/Axon/graph/badge.svg?token=2NJARPM7KH)](https://codecov.io/gh/ArcheBase/Axon)
[![License](https://img.shields.io/badge/License-Mulan%20PSL%20v2-blue)](LICENSE)

A high-performance, plugin-based data recorder with ROS1/ROS2 compatibility and fleet-ready HTTP RPC control for task-centric recording to MCAP format.

**Current Version:** 0.2.0

## Architecture

See [ARCHITECTURE.md](ARCHITECTURE.md) for detailed system design.

```
Server/Fleet Manager → Recording Services → State Machine → MCAP
         ↓                    ↓                 ↓
   HTTP RPC API         HTTP Callbacks    Worker Threads
         ↓                    ↓                 ↓
   Task Config          Start/Finish      SPSC Queues
   (YAML)                Notify           (lock-free)
```

**Plugin-Based Architecture:**
- Middleware-agnostic core with zero ROS dependencies
- ROS1/ROS2 plugins dynamically loaded at runtime via `dlopen/dlsym`
- Unified C ABI interface for extensibility
- Core libraries testable without ROS environment

## Features

- **Task-Centric Design**: One task = one MCAP file with full lifecycle management
- **HTTP RPC API**: RESTful API for remote control (config/begin/end/pause/resume/quit/status)
- **Plugin Architecture**: Middleware-agnostic core with dynamic ROS1/ROS2 plugin loading
- **State Machine**: 4-state FSM (IDLE → READY → RECORDING ↔ PAUSED)
- **MCAP Format**: Efficient append-only container compatible with Foxglove Studio
- **Lock-Free Queues**: Per-topic SPSC queues for zero-copy message handling
- **Multi-ROS Support**: ROS 1 Noetic and ROS 2 Humble/Jazzy/Rolling
- **Metadata Injection**: Embeds task/device/recording metadata + sidecar JSON
- **Structured Logging**: Boost.Log with console/file/ROS sinks

## Dependencies

### System Dependencies

- CMake 3.12+
- GCC 7+ or Clang 6+ (C++17 support)
- Boost 1.71+ (`libboost-all-dev`)
- yaml-cpp (`libyaml-cpp-dev`)
- OpenSSL (`libssl-dev`) - for HTTPS callbacks and RPC server
- zstd (`libzstd-dev`) - optional, for MCAP compression
- lz4 (`liblz4-dev`) - optional, for MCAP compression
- cppcheck - optional, for static code analysis
- clang-format - required, for code formatting

### ROS Dependencies

**ROS 1 (Noetic):**
- roscpp, roslib
- std_msgs, sensor_msgs, nav_msgs, geometry_msgs
- topic_tools
- message_generation, message_runtime

**ROS 2 (Humble/Jazzy/Rolling):**
- rclcpp, ament_index_cpp
- std_msgs, sensor_msgs, nav_msgs, geometry_msgs
- rosidl_default_generators, rosidl_default_runtime

## Building

### Quick Start (Using Makefile)

The easiest way to build and test:

```bash
# Build C++ core libraries (no ROS required)
make build-core

# Build all applications
make app

# Build ROS1 middleware
make build-ros1

# Build ROS2 middleware
make build-ros2

# Build everything (auto-detects ROS version)
make build

# Run all tests
make test

# Clean all build artifacts
make clean

# See all available targets
make help
```

### Build Modes

```bash
# Debug build
make debug

# Release build (default)
make release
```

### Manual Build

#### ROS 1 (Noetic)

```bash
# Source ROS setup
source /opt/ros/noetic/setup.bash

# Create catkin workspace
mkdir -p catkin_ws/src
cd catkin_ws/src
ln -s /path/to/axon .

# Build
cd ..
catkin_make
source devel/setup.bash
```

#### ROS 2 (Humble/Jazzy/Rolling)

```bash
# Source ROS setup
source /opt/ros/<distro>/setup.bash

# Create colcon workspace
mkdir -p colcon_ws/src
cd colcon_ws/src
ln -s /path/to/axon .

# Build
cd ..
colcon build
source install/setup.bash
```

### Makefile Targets

**Local Build:**
- `make build-core` - Build C++ core libraries (no ROS required)
- `make build-ros1` - Build ROS1 middleware plugin
- `make build-ros2` - Build ROS2 middleware plugin
- `make build` - Build everything (auto-detects ROS version)
- `make app` - Build all applications
- `make test` - Run all tests
- `make clean` - Clean all build artifacts

**Docker Testing (No Local ROS Required):**
- `make docker-test-cpp` - C++ core library tests
- `make docker-test-ros1` - ROS1 Noetic tests
- `make docker-test-ros2-humble` - ROS2 Humble tests
- `make docker-test-ros2-jazzy` - ROS2 Jazzy tests
- `make docker-test-ros2-rolling` - ROS2 Rolling tests
- `make docker-test-all` - Test all ROS versions sequentially
- `make docker-test-compose` - Test all versions in parallel (faster)

**Code Quality:**
- `make format` - Format code (requires clang-format and cargo)
- `make lint` - Lint code (requires cppcheck and clippy)

**Coverage:**
- `make coverage` - Generate coverage report (requires lcov, ROS2 recommended)
- `make coverage-html` - Generate HTML coverage report
- `make docker-coverage` - Coverage in Docker

### Docker Testing

The easiest way to test across all ROS versions:

```bash
# Test in a specific ROS version
make docker-test-ros1
make docker-test-ros2-humble

# Test in all ROS versions (sequential)
make docker-test-all

# Test in all ROS versions (parallel using docker-compose)
make docker-test-compose

# Or use docker-compose directly
cd docker
docker-compose -f docker-compose.test.yml up --build
```

Docker containers include:
- All dependencies pre-installed
- GTest for C++ unit tests
- Rust toolchain for Rust tests
- Consistent test environment across platforms

## Configuration

Create a YAML configuration file (see `apps/axon_recorder/config/default_config_ros2.yaml` for example):

```yaml
dataset:
  path: "/data/recordings/dataset.mcap"
  mode: "append"  # or "create"

topics:
  - name: "/camera/image_raw"
    message_type: "sensor_msgs/Image"
    batch_size: 100
    flush_interval_ms: 1000

  - name: "/lidar/points"
    message_type: "sensor_msgs/PointCloud2"
    batch_size: 50
    flush_interval_ms: 500

recording:
  auto_start: true
  max_disk_usage_gb: 100.0

# HTTP RPC server configuration
http_server:
  port: 8080
  host: "0.0.0.0"
```

## Usage

### Command Line Interface

```bash
# Display version
./build/axon_recorder/axon_recorder --version

# Run with default configuration
./build/axon_recorder/axon_recorder

# Run with custom configuration
./build/axon_recorder/axon_recorder --config /path/to/config.yaml

# Run with plugin
./build/axon_recorder/axon_recorder --plugin /path/to/libaxon_ros2.so

# Specify output directory
./build/axon_recorder/axon_recorder --path /data/recordings

# Subscribe to specific topics
./build/axon_recorder/axon_recorder --topic /camera/image --type sensor_msgs/msg/Image
```

### HTTP RPC API

The recorder exposes an HTTP RPC server (default port 8080) for remote control:

**Quick Start with curl:**

```bash
# Set task configuration (IDLE → READY)
curl -X POST http://localhost:8080/rpc/config \
  -H "Content-Type: application/json" \
  -d '{
    "task_id": "task_123",
    "device_id": "robot_01",
    "scene": "warehouse",
    "topics": ["/camera/image", "/lidar/scan"],
    "output_path": "/data/recordings/task_123.mcap"
  }'

# Start recording (READY → RECORDING)
curl -X POST http://localhost:8080/rpc/begin

# Pause recording (RECORDING → PAUSED)
curl -X POST http://localhost:8080/rpc/pause

# Resume recording (PAUSED → RECORDING)
curl -X POST http://localhost:8080/rpc/resume

# Finish recording (RECORDING/PAUSED → IDLE)
curl -X POST http://localhost:8080/rpc/end

# Query status
curl http://localhost:8080/rpc/status

# Get statistics
curl http://localhost:8080/rpc/stats
```

**Complete API Documentation:**

See [docs/designs/rpc-api-design.md](docs/designs/rpc-api-design.md) for:
- Full API specification with all endpoints
- Request/response formats
- State machine transitions
- Error handling
- Usage examples (curl, Python, JavaScript)

### Running with Plugins

Axon uses a plugin-based architecture. Specify the plugin path:

```bash
# ROS 1 plugin
./build/axon_recorder/axon_recorder --plugin ./build/middlewares/libaxon_ros1.so

# ROS 2 plugin
./build/axon_recorder/axon_recorder --plugin ./build/middlewares/libaxon_ros2.so
```

### ROS Integration

The recorder integrates with ROS through middleware plugins. See the plugin documentation for details:

- **ROS 1 Plugin**: `middlewares/ros1/`
- **ROS 2 Plugin**: `middlewares/ros2/`

## Docker

### Build Docker Images

```bash
# ROS 1
docker build -f docker/Dockerfile.ros1 -t axon:ros1 .

# ROS 2 Humble
docker build -f docker/Dockerfile.ros2.humble -t axon:ros2-humble .

# ROS 2 Jazzy
docker build -f docker/Dockerfile.ros2.jazzy -t axon:ros2-jazzy .

# ROS 2 Rolling
docker build -f docker/Dockerfile.ros2.rolling -t axon:ros2-rolling .
```

### Using Docker Compose

```bash
cd docker
docker-compose up -d ros1-noetic  # or ros2-humble, ros2-jazzy, ros2-rolling
docker-compose exec ros1-noetic /bin/bash
```

## Performance Considerations

- **Lock-Free Queues**: Per-topic SPSC queues with cache-line alignment prevent false sharing
- **Copy-Minimized**: Minimized allocations and copies in message path
- **Direct Serialization**: MCAP stores ROS messages directly without conversion overhead
- **Bounded Memory**: Fixed-capacity queues with backpressure
- **Async I/O**: Non-blocking HTTP callbacks

## Testing

The project includes comprehensive unit and integration tests.

### Running Tests

**ROS Package Tests:**
```bash
make test  # Builds and runs all tests
```

**C++ Library Tests:**
```bash
cd core/axon_mcap
mkdir -p build && cd build
cmake .. -DAXON_MCAP_BUILD_TESTS=ON
cmake --build .
ctest --output-on-failure
```

### Test Coverage

- ✅ MCAP writer (file operations, compression, thread safety)
- ✅ MCAP validator (file integrity checks)
- ✅ Logging infrastructure (console/file sinks)
- ✅ SPSC queue (lock-free operations)
- ✅ State machine (transitions, guards)
- ✅ Metadata injector (MCAP metadata, sidecar JSON)
- ✅ HTTP RPC server (all endpoints)
- ✅ Plugin interface (ABI compatibility)
- ✅ Configuration parser (YAML validation)

See `core/*/test/` and `apps/axon_recorder/test/` for the full test suite.

### Generating Coverage Reports

**Local Coverage (requires ROS 2 sourced and lcov installed):**

```bash

# Generate coverage report
make coverage

# Generate HTML report (opens in browser)
make coverage-html
open ../coverage/html/index.html

# Run coverage in Docker (no local ROS required)
make docker-coverage
```

**Coverage in CI:**

Coverage reports are automatically generated on every push to `main` and uploaded to [Codecov](https://codecov.io). See the badge at the top of this README for current coverage status.

## Troubleshooting

### MCAP Library Not Found

Ensure the MCAP library is built before compiling applications:

```bash
cd core/axon_mcap
mkdir -p build && cd build
cmake ..
cmake --build .
```

### Dependencies Not Found

Install required development packages:

```bash
# Ubuntu/Debian
sudo apt-get install libyaml-cpp-dev libzstd-dev liblz4-dev libssl-dev
```

### HTTP Server Connection Refused

Check if the HTTP RPC server is running and accessible:

```bash
# Query server status
curl http://localhost:8080/rpc/status

# Check if port is in use
netstat -tlnp | grep 8080
```

### Message Type Not Supported

The system uses ROS message introspection to handle all message types dynamically. If a specific message type fails, check:

1. Message type name is correct in config
2. ROS message packages are installed
3. Check logs for conversion errors

## Documentation

- **[ARCHITECTURE.md](ARCHITECTURE.md)** - Detailed system architecture
- **[ROADMAP.md](ROADMAP.md)** - Project development roadmap ([中文版](ROADMAP_ZH.md))
- **[docs/designs/rpc-api-design.md](docs/designs/rpc-api-design.md)** - HTTP RPC API specification
- **[CONTRIBUTING.md](CONTRIBUTING.md)** - Contributor guidelines ([中文版](CONTRIBUTING_ZH.md))
- **[CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md)** - Code of conduct ([中文版](CODE_OF_CONDUCT_ZH.md))

## Contributing

We welcome contributions! Please see [CONTRIBUTING.md](CONTRIBUTING.md) for details.

**Quick Links:**
- [Code of Conduct](CODE_OF_CONDUCT.md)
- [Development Workflow](CONTRIBUTING.md#development-workflow)
- [Code Style Guidelines](CONTRIBUTING.md#code-style)
- [Pull Request Process](CONTRIBUTING.md#submitting-changes)

## License

**Mulan PSL v2**

Copyright (c) 2026 ArcheBase

Axon is licensed under Mulan PSL v2. You can use this software according to the terms and conditions of the Mulan PSL v2.

You may obtain a copy of Mulan PSL v2 at:
http://license.coscl.org.cn/MulanPSL2

THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT, MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE. See the Mulan PSL v2 for more details.

## Acknowledgments

Built with:
- [MCAP](https://mcap.dev/) - Modern robotics data format
- [Boost.Beast](https://github.com/boostorg/beast) - HTTP/WebSocket library
- [yaml-cpp](https://github.com/jbeder/yaml-cpp) - YAML parser

