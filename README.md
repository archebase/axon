# Edge Lance Recorder

A high-performance ROS recorder that writes data to Lance format using a C++/Rust FFI bridge. Supports both ROS 1 (Noetic) and ROS 2 (Humble, Jazzy, Rolling).

## Architecture

The system uses a three-layer architecture:

- **Rust Layer**: Lance dataset writing via C FFI interface
- **C++ Core Layer**: Arrow builders, message conversion, batch management
- **ROS Abstraction Layer**: ROS 1/2 compatibility with unified interface

```
ROS Messages → C++ Arrow Builders → C FFI → Rust Lance Writer → Disk
     ↓              ↓                    ↓
  YAML Config   Batch Queue      Async Write Thread
     ↓              ↓
  ROS Service  Status/Control
```

## Features

- **Zero-Copy Data Transfer**: Uses Apache Arrow C Data Interface for efficient data transfer between C++ and Rust
- **Asynchronous Writes**: Background thread prevents blocking ROS callbacks
- **Multi-ROS Support**: Compatible with ROS 1 Noetic and ROS 2 Humble/Jazzy/Rolling
- **All Message Types**: Dynamically handles all ROS message types via introspection
- **YAML Configuration**: Flexible configuration via YAML files
- **ROS Services**: Control recording via ROS services (compatible with rosbridge)

## Dependencies

### System Dependencies

- CMake 3.15+
- Rust toolchain (cargo)
- Apache Arrow C++ (libarrow-dev)
- yaml-cpp (libyaml-cpp-dev)

### ROS Dependencies

**ROS 1:**
- roscpp
- std_msgs
- sensor_msgs
- nav_msgs
- message_generation

**ROS 2:**
- rclcpp
- std_msgs
- sensor_msgs
- nav_msgs
- rosidl_default_generators

## Building

### Quick Start (Using Makefile)

The easiest way to build and test:

```bash
# Source ROS setup
source /opt/ros/noetic/setup.bash  # For ROS1
# OR
source /opt/ros/humble/setup.bash  # For ROS2

# Build everything
make build

# Run tests
make test

# Clean build artifacts
make clean

# See all available targets
make help
```

### Manual Build

#### ROS 1 (Noetic)

```bash
# Source ROS setup
source /opt/ros/noetic/setup.bash

# Create catkin workspace
mkdir -p catkin_ws/src
cd catkin_ws/src
ln -s /path/to/edge_lance_recorder .

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
ln -s /path/to/edge_lance_recorder .

# Build
cd ..
colcon build
source install/setup.bash
```

### Makefile Targets

**Local Build:**
- `make build` - Build Rust library and C++ code
- `make test` - Run all tests (Rust + C++)
- `make rust-build` - Build only Rust library
- `make rust-test` - Run only Rust tests
- `make cpp-build` - Build only C++ code
- `make cpp-test` - Run only C++ tests
- `make clean` - Clean all build artifacts
- `make install` - Install the package

**Docker Build & Test:**
- `make docker-build` - Build all Docker images
- `make docker-test` - Run tests in Docker (auto-detect ROS version)
- `make docker-test-ros1` - Run tests in ROS1 Docker container
- `make docker-test-ros2-humble` - Run tests in ROS2 Humble Docker container
- `make docker-test-ros2-jazzy` - Run tests in ROS2 Jazzy Docker container
- `make docker-test-ros2-rolling` - Run tests in ROS2 Rolling Docker container
- `make docker-test-all` - Run tests in all Docker containers
- `make docker-test-compose` - Run tests using docker-compose (parallel)

**Code Quality:**
- `make debug` - Build in debug mode
- `make release` - Build in release mode (default)
- `make format` - Format code (requires formatters)
- `make lint` - Lint code (requires linters)

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

Create a YAML configuration file (see `config/default_config.yaml` for example):

```yaml
dataset:
  path: "/data/recordings/dataset.lance"
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
```

## Usage

### ROS 1

```bash
# Launch with default config
roslaunch edge_lance_recorder recorder_ros1.launch

# Launch with custom config
roslaunch edge_lance_recorder recorder_ros1.launch config_path:=/path/to/config.yaml
```

### ROS 2

```bash
# Launch with default config
ros2 launch edge_lance_recorder recorder_ros2.launch.py

# Launch with custom config
ros2 launch edge_lance_recorder recorder_ros2.launch.py config_path:=/path/to/config.yaml
```

## ROS Services

The recorder provides the following services:

### StartRecording

Start recording with optional config override.

```bash
# ROS 1
rosservice call /lance_recorder/start_recording "config_path: ''"

# ROS 2
ros2 service call /lance_recorder/start_recording edge_lance_recorder/srv/StartRecording "{config_path: ''}"
```

### StopRecording

Stop recording and flush current batch.

```bash
# ROS 1
rosservice call /lance_recorder/stop_recording

# ROS 2
ros2 service call /lance_recorder/stop_recording edge_lance_recorder/srv/StopRecording
```

### UpdateConfig

Update configuration at runtime.

```bash
# ROS 1
rosservice call /lance_recorder/update_config "config_path: '/path/to/new_config.yaml'"

# ROS 2
ros2 service call /lance_recorder/update_config edge_lance_recorder/srv/UpdateConfig "{config_path: '/path/to/new_config.yaml'}"
```

### GetStatus

Get current recording status.

```bash
# ROS 1
rosservice call /lance_recorder/get_status

# ROS 2
ros2 service call /lance_recorder/get_status edge_lance_recorder/srv/GetStatus
```

## Docker

### Build Docker Images

```bash
# ROS 1
docker build -f docker/Dockerfile.ros1 -t lance_recorder:ros1 .

# ROS 2 Humble
docker build -f docker/Dockerfile.ros2.humble -t lance_recorder:ros2-humble .

# ROS 2 Jazzy
docker build -f docker/Dockerfile.ros2.jazzy -t lance_recorder:ros2-jazzy .

# ROS 2 Rolling
docker build -f docker/Dockerfile.ros2.rolling -t lance_recorder:ros2-rolling .
```

### Using Docker Compose

```bash
cd docker
docker-compose up -d ros1-noetic  # or ros2-humble, ros2-jazzy, ros2-rolling
docker-compose exec ros1-noetic /bin/bash
```

## Performance Considerations

- **Batch Size**: Larger batch sizes improve write throughput but increase memory usage
- **Flush Interval**: Shorter intervals reduce data loss risk but may impact performance
- **Zero-Copy**: The Arrow C Data Interface enables zero-copy transfer between C++ and Rust
- **Async Writes**: Background writer thread prevents blocking ROS callbacks

## Testing

The project includes a comprehensive test suite covering Rust and C++ components.

### Running Tests

**Rust Tests:**
```bash
cd src/bridge
cargo test
```

**C++ Tests:**
```bash
mkdir -p build
cd build
cmake ../test
make
ctest --output-on-failure
```

**All Tests:**
```bash
# Run Rust tests
cd src/bridge && cargo test && cd ../..

# Build and run C++ tests
mkdir -p build && cd build
cmake ../test && make && ctest --output-on-failure
```

### Test Coverage

- ✅ Rust bridge library (FFI functions, error handling, dataset operations)
- ✅ Configuration parser (YAML loading, validation, saving)
- ✅ Batch manager (batch collection, flushing, async writes)
- ✅ Arrow builder (type-specific builders, reset functionality)

See `test/README.md` for more details on the test suite.

## Troubleshooting

### Rust Library Not Found

Ensure the Rust library is built before compiling C++ code:

```bash
cd src/bridge
cargo build --release
```

### Arrow Library Not Found

Install Apache Arrow C++ development packages:

```bash
# Ubuntu/Debian
sudo apt-get install libarrow-dev libarrow-glib-dev
```

### Message Type Not Supported

The system uses ROS message introspection to handle all message types dynamically. If a specific message type fails, check:

1. Message type name is correct in config
2. ROS message packages are installed
3. Check logs for conversion errors

## License

Apache-2.0

