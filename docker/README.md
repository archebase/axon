# Docker Testing Guide

This directory contains Docker configurations for building and testing Edge Lance Recorder across different ROS versions.

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

- `lance_recorder:ros1` - ROS1 Noetic
- `lance_recorder:ros2-humble` - ROS2 Humble
- `lance_recorder:ros2-jazzy` - ROS2 Jazzy
- `lance_recorder:ros2-rolling` - ROS2 Rolling

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
docker build -f docker/Dockerfile.ros1 -t lance_recorder:ros1 .
```

### Run Tests
```bash
docker run --rm \
  -v $(pwd):/workspace/edge_lance_recorder \
  -e ROS_DISTRO=noetic \
  -e ROS_VERSION=1 \
  lance_recorder:ros1 \
  /usr/local/bin/run_tests.sh
```

### Interactive Shell
```bash
docker run -it --rm \
  -v $(pwd):/workspace/edge_lance_recorder \
  -e ROS_DISTRO=noetic \
  -e ROS_VERSION=1 \
  lance_recorder:ros1 \
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








