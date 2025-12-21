# Docker Testing Guide

Docker configurations for building and testing Axon across different ROS versions.

## Quick Start

```bash
cd ros/docker

# Run tests
docker-compose -f docker-compose.test.yml up test-ros2-humble --build --abort-on-container-exit

# Run performance tests
docker-compose -f docker-compose.perf.yml up perf-ros2-humble --build

# Interactive shell
docker-compose -f docker-compose.test.yml run test-ros2-humble /bin/bash
```

## Available Images

| Image | ROS Version |
|-------|-------------|
| `test-ros1` | ROS 1 Noetic |
| `test-ros2-humble` | ROS 2 Humble |
| `test-ros2-jazzy` | ROS 2 Jazzy |
| `test-ros2-rolling` | ROS 2 Rolling |

## CI ↔ Local Docker Mapping

Local Docker testing mirrors CI exactly. Both use ROS's native test infrastructure.

| CI Job (`ci.yml`) | Local Docker (`run_tests.sh`) | What it does |
|-------------------|-------------------------------|--------------|
| `ros-unit-tests` | Part 1: ROS Tests | `colcon test` / `catkin_make run_tests` |
| `ros-integration-tests` | Part 2: Integration Tests | `run_integration_tests.sh` |

### Shared Scripts

| Script | Used by |
|--------|---------|
| `test/integration/run_integration_tests.sh` | CI `AFTER_SCRIPT` + Docker `run_tests.sh` |
| `test/integration/test_ros_services.sh` | Called by `run_integration_tests.sh` |

### Test Flow

```
┌─────────────────────────────────────────────────────────────┐
│  Part 1: ROS Tests                                          │
│    - Build package with colcon/catkin                       │
│    - Run colcon test / catkin_make run_tests                │
│    - Executes all GTest-based unit tests                    │
├─────────────────────────────────────────────────────────────┤
│  Part 2: Integration Tests                                  │
│    - Start axon_recorder_node                               │
│    - Run test_ros_services.sh (actual ROS service calls)    │
│    - Cleanup                                                │
└─────────────────────────────────────────────────────────────┘
```

## Performance Tests

```bash
# Basic perf test
docker-compose -f docker-compose.perf.yml up perf-ros2-humble --build

# Custom parameters
docker-compose -f docker-compose.perf.yml run --rm perf-ros2-humble \
  /usr/local/bin/run_perf_tests.sh --duration 30 --imu-rate 2000

# With ASAN (memory debugging)
docker-compose -f docker-compose.perf.yml run --rm perf-ros2-humble \
  /usr/local/bin/run_perf_tests.sh --asan

# With flamegraph profiling (Linux host only)
docker-compose -f docker-compose.perf.yml run --rm --privileged perf-ros2-humble \
  /usr/local/bin/run_perf_tests.sh --flamegraph
```

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Build fails | Ensure Docker has 4GB+ memory |
| Permission issues | Run `chmod +x docker/scripts/*.sh` |
| Tests fail | Check ROS environment with `printenv \| grep ROS` |
