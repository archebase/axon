# Axon Recorder Tests

## Running Tests

### Local Docker

#### E2E Tests (End-to-End Service Tests)

E2E tests start the `axon_recorder_node` and make actual ROS service calls.

**Important:** If you have pre-built binaries on your host machine, they may have been built with different library versions (e.g., Boost) than what's in Docker. To avoid library version mismatches, either:

1. **Clean build artifacts before running** (recommended):
   ```bash
   cd ros/docker
   # Clean build artifacts to force rebuild inside Docker
   rm -rf ../../ros/build ../../ros/devel ../../ros/install ../../ros/log
   rm -rf ../../install ../../build ../../log
   ```

2. **Use the unified E2E test script** which builds inside Docker:
   ```bash
   cd ros/docker
   # Without coverage (builds Release, runs tests)
   docker compose -f docker-compose.test.yml run --rm test-ros1 \
     /workspace/axon/ros/docker/scripts/run_e2e_tests.sh
   
   # With coverage (builds Debug with coverage, runs tests, generates coverage)
   docker compose -f docker-compose.test.yml run --rm \
     -v $(pwd)/../../coverage_output:/coverage_output \
     test-ros1 \
     /workspace/axon/ros/docker/scripts/run_e2e_tests.sh --coverage --coverage-output /coverage_output
   ```

```bash
cd ros/docker

# ROS 1 Noetic (with coverage)
docker compose -f docker-compose.test.yml build test-ros1
docker compose -f docker-compose.test.yml run --rm \
  -v $(pwd)/../../coverage_output:/coverage_output \
  test-ros1 \
  /workspace/axon/ros/docker/scripts/run_e2e_tests.sh --coverage --coverage-output /coverage_output

# ROS 2 Humble (with coverage)
docker compose -f docker-compose.test.yml build test-ros2-humble
docker compose -f docker-compose.test.yml run --rm \
  -v $(pwd)/../../coverage_output:/coverage_output \
  test-ros2-humble \
  /workspace/axon/ros/docker/scripts/run_e2e_tests.sh --coverage --coverage-output /coverage_output

# ROS 2 Jazzy (with coverage)
docker compose -f docker-compose.test.yml build test-ros2-jazzy
docker compose -f docker-compose.test.yml run --rm \
  -v $(pwd)/../../coverage_output:/coverage_output \
  test-ros2-jazzy \
  /workspace/axon/ros/docker/scripts/run_e2e_tests.sh --coverage --coverage-output /coverage_output

# ROS 2 Rolling (with coverage)
docker compose -f docker-compose.test.yml build test-ros2-rolling
docker compose -f docker-compose.test.yml run --rm \
  -v $(pwd)/../../coverage_output:/coverage_output \
  test-ros2-rolling \
  /workspace/axon/ros/docker/scripts/run_e2e_tests.sh --coverage --coverage-output /coverage_output
```

**Note:** The same build artifact cleanup applies to all ROS distributions if you encounter library version mismatch errors.
```

#### Full Test Suite (Unit + Integration + E2E)

```bash
cd ros/docker

# ROS 1 Noetic
docker-compose -f docker-compose.test.yml up test-ros1 --build --abort-on-container-exit

# ROS 2 Humble
docker-compose -f docker-compose.test.yml up test-ros2-humble --build --abort-on-container-exit

# ROS 2 Jazzy
docker-compose -f docker-compose.test.yml up test-ros2-jazzy --build --abort-on-container-exit

# ROS 2 Rolling
docker-compose -f docker-compose.test.yml up test-ros2-rolling --build --abort-on-container-exit
```

### CI Pipeline

Tests run via `ros-industrial/industrial_ci`:

| Job | Description |
|-----|-------------|
| `ros-unit-tests` | `colcon test` / `catkin_make run_tests` |
| `ros-integration-tests` | End-to-end ROS service calls |

## Unified Test Flow

Both CI and local Docker use the same approach:

| Step | CI (industrial_ci) | Local Docker (run_tests.sh) |
|------|--------------------|-----------------------------|
| **Part 1** | `colcon test` / `catkin_make run_tests` | Same |
| **Part 2** | `run_e2e_tests.sh` (unified script) | Same |

## Test Types

| Test | Framework | Run by |
|------|-----------|--------|
| `test_task_config` | GTest | colcon/catkin |
| `test_state_machine` | GTest | colcon/catkin |
| `test_http_callback_client` | GTest | colcon/catkin |
| `test_recording_workflow` | GTest | colcon/catkin |
| `test_service_adapter` | GTest | colcon/catkin |
| `test_ros_services.sh` | Shell | run_e2e_tests.sh |

## Manual Testing

```bash
# Terminal 1: Start recorder
ros2 run axon_recorder axon_recorder_node

# Terminal 2: Run E2E tests
./test/e2e/test_ros_services.sh
```

## Performance Tests

```bash
cd ros/docker
docker-compose -f docker-compose.perf.yml up perf-ros2-humble --build
```
