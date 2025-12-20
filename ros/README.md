# Axon ROS Packages

ROS packages for high-performance data recording.

## Quick Start

```bash
cd ros/docker

# Run with Docker (recommended)
docker-compose -f docker-compose.yml up ros2-humble --build

# Run tests
docker-compose -f docker-compose.test.yml up test-ros2-humble --build --abort-on-container-exit
```

## Packages

| Package | Description |
|---------|-------------|
| `axon_recorder` | High-performance ROS data recorder with MCAP output |

## Building Locally

```bash
# ROS 1 (Noetic)
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws && catkin_make

# ROS 2 (Humble/Jazzy/Rolling)
source /opt/ros/humble/setup.bash
cd ~/ros2_ws && colcon build --packages-select axon_recorder
```

## Usage

```bash
# ROS 1
roslaunch axon_recorder recorder.launch.ros1.xml

# ROS 2
ros2 launch axon_recorder recorder.launch.ros2.xml
```

## Documentation

- [axon_recorder README](axon_recorder/README.md)
- [Docker Guide](docker/README.md)
- [Recording Service API Design](../docs/recording-service-api-design-2025-12-20.md)
