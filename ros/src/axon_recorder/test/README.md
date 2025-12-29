# Axon Recorder Tests

## Build Context

Tests are built as part of the ROS package via `colcon` (ROS 2) or `catkin` (ROS 1). You **cannot** build/run tests directly from the `test/` directory.

**Required dependencies:** `axon_mcap`, `axon_logging`, ROS-generated message headers.

## Running Tests

### Docker (Recommended)

Ensures consistent environment and library versions. Clean local build artifacts first if you have existing builds:

```bash
cd ros/docker
rm -rf ../../ros/build ../../ros/devel ../../ros/install ../../ros/log
rm -rf ../../install ../../build ../../log
```

**With coverage:**
```bash
# ROS 1
docker compose -f docker-compose.test.yml build test-ros1
docker compose -f docker-compose.test.yml run --rm \
  -v $(pwd)/../../coverage_output:/coverage_output \
  test-ros1 /workspace/axon/ros/docker/scripts/run_e2e_tests.sh --coverage --coverage-output /coverage_output

# ROS 2 (Humble/Jazzy/Rolling)
docker compose -f docker-compose.test.yml build test-ros2-humble  # or -jazzy, -rolling
docker compose -f docker-compose.test.yml run --rm \
  -v $(pwd)/../../coverage_output:/coverage_output \
  test-ros2-humble /workspace/axon/ros/docker/scripts/run_e2e_tests.sh --coverage --coverage-output /coverage_output
```

**Without coverage:**
```bash
docker compose -f docker-compose.test.yml run --rm \
  test-ros2-humble /workspace/axon/ros/docker/scripts/run_e2e_tests.sh
```

### Local ROS Workspace

For developers with ROS installed locally:

```bash
cd ros

# ROS 1
source /opt/ros/noetic/setup.bash
catkin_make
catkin_make run_tests

# ROS 2
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select axon_recorder --cmake-args -DENABLE_COVERAGE=ON
colcon test --packages-select axon_recorder
colcon test-result --verbose
```

## Test Types

| Test | Framework | Category |
|------|-----------|----------|
| Unit tests | GTest | `unit/` |
| Integration tests | GTest | `integration/` |
| E2E tests | Shell | `run_e2e_tests.sh` |

## CI Pipeline

| Job | Command |
|-----|---------|
| `ros-unit-tests` | `colcon test` / `catkin_make run_tests` |
| `ros-integration-tests` | `run_e2e_tests.sh` |

## Performance Tests

```bash
cd ros/docker
docker compose -f docker-compose.perf.yml up perf-ros2-humble --build
```
