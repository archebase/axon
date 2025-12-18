# Docker Testing Guide

This directory contains Docker configurations for building and testing Axon across different ROS versions.

## Quick Start

### Run Tests in Docker

```bash
# Test in ROS1 (Noetic)
make docker-test-ros1

# Test in ROS2 Humble
make docker-test-ros2-humble

# Test in all ROS versions (sequential)
make docker-test-all

# Test in all ROS versions (parallel)
make docker-test-compose
```

### Using Docker Compose

```bash
# Run tests in all containers
cd docker
docker-compose -f docker-compose.test.yml up --build

# Run tests in specific container
docker-compose -f docker-compose.test.yml up test-ros1
```

## Docker Images

- `axon:ros1` - ROS1 Noetic
- `axon:ros2-humble` - ROS2 Humble
- `axon:ros2-jazzy` - ROS2 Jazzy
- `axon:ros2-rolling` - ROS2 Rolling

## Test Scripts

### `run_tests.sh`
Runs all tests (Rust + C++ + Integration):
- Rust unit tests
- C++ unit tests
- Integration tests (if available)

### `run_build.sh`
Builds the project:
- Rust library
- C++ code

## Manual Docker Usage

### Build Image
```bash
docker build -f docker/Dockerfile.ros1 -t axon:ros1 .
```

### Run Tests
```bash
docker run --rm \
  -v $(pwd):/workspace/axon \
  -e ROS_DISTRO=noetic \
  -e ROS_VERSION=1 \
  axon:ros1 \
  /usr/local/bin/run_tests.sh
```

### Interactive Shell
```bash
docker run -it --rm \
  -v $(pwd):/workspace/axon \
  -e ROS_DISTRO=noetic \
  -e ROS_VERSION=1 \
  axon:ros1 \
  /bin/bash
```

## CI/CD Integration

The Docker setup is designed for CI/CD pipelines:

```yaml
# Example GitHub Actions
- name: Test ROS1
  run: make docker-test-ros1

- name: Test ROS2
  run: make docker-test-ros2-humble
```

## Running Performance Tests

Performance tests measure recording throughput, CPU usage, memory, and message drop rates.

### Quick Start

```bash
# Run perf tests in ROS 2 Humble
cd ros/docker
docker-compose -f docker-compose.perf.yml up perf-ros2-humble --build

# Run perf tests in ROS 1 Noetic
docker-compose -f docker-compose.perf.yml up perf-ros1 --build
```

### Custom Parameters

```bash
# Run with custom test duration and rates
docker-compose -f docker-compose.perf.yml run --rm perf-ros2-humble \
  /usr/local/bin/run_perf_tests.sh --duration 30 --imu-rate 2000 --camera-rate 60

# Available options:
#   --duration <sec>     Test duration (default: 10)
#   --imu-rate <hz>      IMU rate (default: 1000)
#   --camera-rate <hz>   Camera rate (default: 30)
#   --num-cameras <n>    Number of cameras (default: 3)
#   --output <file>      JSON output file
#   --skip-build         Skip building (use cached build)
```

### Interactive Debugging

```bash
# Open shell for debugging
docker-compose -f docker-compose.perf.yml run --rm perf-ros2-humble /bin/bash

# Inside container:
source /opt/ros/humble/setup.bash
cd /workspace/axon
# ... debug as needed
```

### Skip Build (After First Run)

```bash
# Skip rebuilding to save time on subsequent runs
docker-compose -f docker-compose.perf.yml run --rm perf-ros2-humble \
  /usr/local/bin/run_perf_tests.sh --skip-build
```

## Troubleshooting

### Build Fails
- Ensure Docker has enough memory (recommended: 4GB+)
- Check Docker logs: `docker logs <container_id>`

### Tests Fail
- Verify source code is mounted correctly
- Check ROS environment variables are set
- Review test output for specific failures

### Permission Issues
- Ensure scripts are executable: `chmod +x docker/scripts/*.sh`
