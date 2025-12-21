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

## Recording Service API

The recorder exposes ROS services for programmatic control. The API is identical for ROS 1 and ROS 2, only the command syntax differs.

### Available Commands

| Command | Description |
|---------|-------------|
| `start` | Start recording with cached config |
| `pause` | Pause active recording |
| `resume` | Resume paused recording |
| `finish` | Stop and save recording |
| `cancel` | Stop and discard recording |
| `clear` | Clear cached config (when in READY state) |

---

## ROS 1 (Noetic)

### 1. Cache Recording Configuration

```bash
rosservice call /axon_recorder/cached_recording_config "{
  task_id: 'my_task_001',
  device_id: 'robot_01',
  data_collector_id: 'collector_01',
  order_id: 'order_batch_001',
  operator_name: 'john.doe',
  scene: 'warehouse',
  subscene: 'picking',
  skills: ['navigation', 'manipulation'],
  factory: 'factory_shanghai_01',
  topics: ['/camera/image', '/lidar/scan', '/imu/data'],
  start_callback_url: '',
  finish_callback_url: '',
  user_token: ''
}"
```

### 2. Start Recording

```bash
rosservice call /axon_recorder/recording_control "{command: 'start', task_id: ''}"
```

### 3. Check Recording Status

```bash
rosservice call /axon_recorder/recording_status "{task_id: ''}"
```

### 4. Pause/Resume Recording

```bash
# Pause
rosservice call /axon_recorder/recording_control "{command: 'pause', task_id: 'my_task_001'}"

# Resume
rosservice call /axon_recorder/recording_control "{command: 'resume', task_id: 'my_task_001'}"
```

### 5. Finish Recording

```bash
rosservice call /axon_recorder/recording_control "{command: 'finish', task_id: 'my_task_001'}"
```

### 6. Cancel Recording (Discard Data)

```bash
rosservice call /axon_recorder/recording_control "{command: 'cancel', task_id: 'my_task_001'}"
```

### 7. Check if Recorder is Ready

```bash
rosservice call /axon_recorder/is_recording_ready "{}"
```

---

## ROS 2 (Humble/Jazzy/Rolling)

### 1. Cache Recording Configuration

```bash
ros2 service call /axon_recorder/cached_recording_config axon_recorder/srv/CachedRecordingConfig "{
  task_id: 'my_task_001',
  device_id: 'robot_01',
  data_collector_id: 'collector_01',
  order_id: 'order_batch_001',
  operator_name: 'john.doe',
  scene: 'warehouse',
  subscene: 'picking',
  skills: ['navigation', 'manipulation'],
  factory: 'factory_shanghai_01',
  topics: ['/camera/image', '/lidar/scan', '/imu/data'],
  start_callback_url: '',
  finish_callback_url: '',
  user_token: ''
}"
```

### 2. Start Recording

```bash
ros2 service call /axon_recorder/recording_control axon_recorder/srv/RecordingControl "{
  command: 'start',
  task_id: ''
}"
```

### 3. Check Recording Status

```bash
ros2 service call /axon_recorder/recording_status axon_recorder/srv/RecordingStatus "{
  task_id: ''
}"
```

### 4. Pause/Resume Recording

```bash
# Pause
ros2 service call /axon_recorder/recording_control axon_recorder/srv/RecordingControl "{
  command: 'pause',
  task_id: 'my_task_001'
}"

# Resume
ros2 service call /axon_recorder/recording_control axon_recorder/srv/RecordingControl "{
  command: 'resume',
  task_id: 'my_task_001'
}"
```

### 5. Finish Recording

```bash
ros2 service call /axon_recorder/recording_control axon_recorder/srv/RecordingControl "{
  command: 'finish',
  task_id: 'my_task_001'
}"
```

### 6. Cancel Recording (Discard Data)

```bash
ros2 service call /axon_recorder/recording_control axon_recorder/srv/RecordingControl "{
  command: 'cancel',
  task_id: 'my_task_001'
}"
```

### 7. Check if Recorder is Ready

```bash
ros2 service call /axon_recorder/is_recording_ready axon_recorder/srv/IsRecordingReady "{}"
```

## Documentation

- [axon_recorder README](axon_recorder/README.md)
- [Docker Guide](docker/README.md)
- [Recording Service API Design](../docs/recording-service-api-design-2025-12-20.md)
