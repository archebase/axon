# Axon

[![CI](https://github.com/ArcheBase/Axon/actions/workflows/ci.yml/badge.svg)](https://github.com/ArcheBase/Axon/actions/workflows/ci.yml)
[![codecov](https://codecov.io/gh/ArcheBase/Axon/graph/badge.svg?token=2NJARPM7KH)](https://codecov.io/gh/ArcheBase/Axon)

A high-performance ROS recorder by ArcheBase that writes data to MCAP format. Supports both ROS 1 (Noetic) and ROS 2 (Humble, Jazzy, Rolling). Designed for fleet management with server-controlled recording via ros-bridge.

## Architecture

See [ARCHITECTURE.md](ARCHITECTURE.md) for detailed system design.

```
Server (ros-bridge) → Recording Services → State Machine → MCAP
         ↓                    ↓                 ↓
   Task Config          HTTP Callbacks    Worker Threads
         ↓                    ↓                 ↓
   CachedRecording      Start/Finish      SPSC Queues
       Config             Notify           (lock-free)
```

## Features

- **Task-Centric Design**: One task = one MCAP file with full lifecycle management
- **Server Integration**: Fleet management via ros-bridge with HTTP callbacks
- **MCAP Format**: Efficient append-only container compatible with Foxglove Studio
- **Lock-Free Queues**: Per-topic SPSC queues for zero-copy message handling
- **Multi-ROS Support**: ROS 1 Noetic and ROS 2 Humble/Jazzy/Rolling
- **Metadata Injection**: Embeds task/device/recording metadata + sidecar JSON
- **Edge Uploader**: S3 upload with retry, crash recovery, and backpressure
- **Structured Logging**: Boost.Log with console/file/ROS sinks

## Dependencies

### System Dependencies

- CMake 3.12+
- Boost 1.71+ (`libboost-log-dev`, `libboost-filesystem-dev`, `libboost-thread-dev`)
- yaml-cpp (`libyaml-cpp-dev`)
- OpenSSL (`libssl-dev`) - for HTTPS callbacks
- zstd (`libzstd-dev`) - optional, for MCAP compression
- lz4 (`liblz4-dev`) - optional, for MCAP compression

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
- `make build` - Build C++ code
- `make test` - Run all tests
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

Create a YAML configuration file (see `ros/src/axon_recorder/config/default_config.yaml` for example):

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
```

## Usage

### ROS 1

```bash
# Launch with default config
roslaunch axon recorder_ros1.launch

# Launch with custom config
roslaunch axon recorder_ros1.launch config_path:=/path/to/config.yaml
```

### ROS 2

```bash
# Launch with default config
ros2 launch axon recorder_ros2.launch.py

# Launch with custom config
ros2 launch axon recorder_ros2.launch.py config_path:=/path/to/config.yaml
```

## ROS Services

The recorder uses a task-centric service API designed for server integration:

| Service | Purpose |
|---------|---------|
| `CachedRecordingConfig` | Cache task configuration (IDLE → READY) |
| `IsRecordingReady` | Query if recorder has cached config |
| `RecordingControl` | Control lifecycle: start/pause/resume/cancel/finish/clear |
| `RecordingStatus` | Query status, metrics, and task info |

### CachedRecordingConfig

Cache task configuration from server before starting.

```bash
# ROS 2
ros2 service call /axon_recorder/cached_recording_config axon_recorder/srv/CachedRecordingConfig "{
  task_id: 'task_123',
  device_id: 'robot_01',
  scene: 'warehouse',
  topics: ['/camera/image', '/lidar/scan'],
  start_callback_url: 'http://server/api/start',
  finish_callback_url: 'http://server/api/finish',
  user_token: 'jwt_token'
}"
```

### RecordingControl

Unified control interface for all lifecycle operations.

```bash
# Start recording (requires READY state)
ros2 service call /axon_recorder/recording_control axon_recorder/srv/RecordingControl "{command: 'start'}"

# Pause recording
ros2 service call /axon_recorder/recording_control axon_recorder/srv/RecordingControl "{command: 'pause', task_id: 'task_123'}"

# Resume recording
ros2 service call /axon_recorder/recording_control axon_recorder/srv/RecordingControl "{command: 'resume', task_id: 'task_123'}"

# Finish recording (finalizes MCAP, triggers upload)
ros2 service call /axon_recorder/recording_control axon_recorder/srv/RecordingControl "{command: 'finish', task_id: 'task_123'}"

# Cancel recording (cleanup without upload)
ros2 service call /axon_recorder/recording_control axon_recorder/srv/RecordingControl "{command: 'cancel', task_id: 'task_123'}"
```

### RecordingStatus

Query current status and metrics.

```bash
ros2 service call /axon_recorder/recording_status axon_recorder/srv/RecordingStatus "{}"
```

See [docs/recording-service-api-design-2025-12-20.md](docs/recording-service-api-design-2025-12-20.md) for complete API documentation.

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
- **Zero-Copy Transfer**: Messages moved through queue without copying
- **Direct Serialization**: MCAP stores ROS messages directly without conversion overhead
- **Bounded Memory**: Fixed-capacity queues (default 4096 per topic) with backpressure
- **Async I/O**: Non-blocking HTTP callbacks and S3 uploads

## Testing

The project includes comprehensive unit and integration tests.

### Running Tests

**ROS Package Tests:**
```bash
cd ros
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
- ✅ Edge uploader (retry, state management)
- ✅ SPSC queue (lock-free operations)
- ✅ State machine (transitions, guards)
- ✅ Metadata injector (MCAP metadata, sidecar JSON)
- ✅ Recording service (all 4 services)

See `ros/src/axon_recorder/test/` for the full test suite.

### Generating Coverage Reports

**Local Coverage (requires ROS 2 sourced and lcov installed):**

```bash
cd ros

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

Ensure the MCAP library is built before compiling ROS packages:

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
sudo apt-get install libyaml-cpp-dev libzstd-dev liblz4-dev
```

### Message Type Not Supported

The system uses ROS message introspection to handle all message types dynamically. If a specific message type fails, check:

1. Message type name is correct in config
2. ROS message packages are installed
3. Check logs for conversion errors

## License

Apache-2.0

