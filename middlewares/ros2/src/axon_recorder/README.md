# Axon Recorder for ROS 2 (Humble/Jazzy/Rolling)

High-performance ROS 2 recorder that writes sensor data to MCAP format.

## Features

- Task-centric recording with full lifecycle management
- Lock-free per-topic SPSC queues for zero-copy message handling
- Fleet-ready: Server-controlled recording via HTTP callbacks
- Crash-resilient: S3 uploader with SQLite state persistence

## Building

```bash
# From project root
source /opt/ros/$ROS_DISTRO/setup.bash  # humble, jazzy, or rolling
make build-ros2

# Or directly with colcon
cd middlewares/ros2
colcon build
```

## Usage

```bash
# Launch the recorder
ros2 launch axon_recorder recorder.launch

# Control recording via services
ros2 service call /axon_recorder/cached_recording_config axon_recorder/srv/CachedRecordingConfig "{...}"
ros2 service call /axon_recorder/recording_control axon_recorder/srv/RecordingControl "{command: START}"
```

## Services

- `~/cached_recording_config` - Cache recording configuration
- `~/is_recording_ready` - Query recorder state
- `~/recording_control` - Start/stop/pause/finish recording
- `~/recording_status` - Get recording status

## Documentation

See [project README](../../../../README.md) for more details.
