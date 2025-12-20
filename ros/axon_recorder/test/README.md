# Axon Recorder Tests

## Running Tests

### Local Docker

```bash
cd ros/docker

# ROS 2 Humble
docker-compose -f docker-compose.test.yml up test-ros2-humble --build --abort-on-container-exit

# ROS 1 Noetic
docker-compose -f docker-compose.test.yml up test-ros1 --build --abort-on-container-exit
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
| **Part 2** | `run_integration_tests.sh` | Same |

## Test Types

| Test | Framework | Run by |
|------|-----------|--------|
| `test_task_config` | GTest | colcon/catkin |
| `test_state_machine` | GTest | colcon/catkin |
| `test_http_callback_client` | GTest | colcon/catkin |
| `test_recording_workflow` | GTest | colcon/catkin |
| `test_service_adapter` | GTest | colcon/catkin |
| `test_ros_services.sh` | Shell | run_integration_tests.sh |

## Manual Testing

```bash
# Terminal 1: Start recorder
ros2 run axon_recorder axon_recorder_node

# Terminal 2: Run integration tests
./test/integration/test_ros_services.sh
```

## Performance Tests

```bash
cd ros/docker
docker-compose -f docker-compose.perf.yml up perf-ros2-humble --build
```
