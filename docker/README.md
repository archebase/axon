# Docker Testing Guide

Docker configurations for building and testing Axon across different ROS versions.

## Quick Start

```bash
cd docker

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

Local Docker testing mirrors CI exactly. Both use unified CMake build system.

| CI Job (`ci.yml`) | Local Docker (`run_integration.sh`) | What it does |
|-------------------|-------------------------------|--------------|
| `ros-unit-tests` | Part 1: ROS Tests | `ctest` (unified CMake build) |
| `ros-integration-tests` | Part 2: E2E Tests | `run_e2e_tests.sh` |

### Shared Scripts

| Script | Used by |
|--------|---------|
| `docker/scripts/run_e2e_tests.sh` | CI E2E workflow + Docker testing (unified script with --coverage support) |
| `test/e2e/test_ros_services.sh` | Called by `run_e2e_tests.sh` |

### Test Flow

```
┌─────────────────────────────────────────────────────────────┐
│  Part 1: ROS Tests                                          │
│    - Build package with unified CMake                       │
│    - Run ctest for plugin tests                             │
│    - Executes all GTest-based unit tests                    │
├─────────────────────────────────────────────────────────────┤
│  Part 2: Integration Tests                                  │
│    - Start axon_recorder_node                               │
│    - Run test_ros_services.sh (actual ROS service calls)    │
│    - Cleanup                                                │
└─────────────────────────────────────────────────────────────┘
```

## Coverage Tests

Generate coverage reports to identify untested code:

```bash
cd docker

# Run coverage tests (builds with instrumentation + generates lcov report)
docker compose -f docker-compose.test.yml run --rm test-ros2-humble \
  /workspace/axon/docker/scripts/run_integration.sh --coverage --coverage-output /workspace/coverage

# Coverage output is saved to /workspace/coverage inside container
# Mount a local directory to extract it:
docker compose -f docker-compose.test.yml run --rm \
  -v $(pwd)/coverage-output:/workspace/coverage \
  test-ros2-humble /workspace/axon/docker/scripts/run_integration.sh --coverage --coverage-output /workspace/coverage

# View HTML report
open coverage-output/html/index.html
```

### Coverage for Different ROS Versions

```bash
# ROS 2 Humble
docker compose -f docker-compose.test.yml run --rm test-ros2-humble \
  /workspace/axon/docker/scripts/run_integration.sh --coverage --coverage-output /workspace/coverage

# ROS 2 Jazzy
docker compose -f docker-compose.test.yml run --rm test-ros2-jazzy \
  /workspace/axon/docker/scripts/run_integration.sh --coverage --coverage-output /workspace/coverage

# ROS 1 Noetic
docker compose -f docker-compose.test.yml run --rm test-ros1 \
  /workspace/axon/docker/scripts/run_integration.sh --coverage --coverage-output /workspace/coverage
```

### Interactive Coverage Debugging

```bash
# Start interactive shell with coverage build
docker compose -f docker-compose.test.yml run --rm test-ros2-humble bash

# Inside container:
source /opt/ros/humble/setup.bash
cd /workspace/axon
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug -DAXON_ENABLE_COVERAGE=ON -DAXON_BUILD_ROS2_PLUGIN=ON
cmake --build . -j$(nproc)
ctest --output-on-failure

# Generate coverage
lcov --capture --directory . --output-file coverage.info --rc lcov_branch_coverage=1
lcov --list coverage.info
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
| No .gcda files | Ensure `-DAXON_ENABLE_COVERAGE=ON` was passed to cmake |
| Empty coverage | Check tests actually ran: `ctest --output-on-failure --verbose` |
| lcov errors | lcov 1.x doesn't support `--ignore-errors mismatch` (lcov 2.0+ only) |
