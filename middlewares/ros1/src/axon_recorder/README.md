# Axon Recorder for ROS 1 (Noetic)

High-performance ROS 1 recorder that writes sensor data to MCAP format.

## Features

- Task-centric recording with full lifecycle management
- Lock-free per-topic SPSC queues for zero-copy message handling
- Fleet-ready: Server-controlled recording via HTTP callbacks
- Crash-resilient: S3 uploader with SQLite state persistence

## Building

```bash
# From project root
source /opt/ros/noetic/setup.bash
make build-ros1

# Or directly with catkin
cd middlewares/ros1
catkin build
```

## Usage

```bash
# Launch the recorder
roslaunch axon_recorder recorder.launch

# Control recording via services
rosservice call /axon_recorder/cached_recording_config "..."
rosservice call /axon_recorder/recording_control "{command: 'START'}"
```

## Services

- `~/cached_recording_config` - Cache recording configuration
- `~/is_recording_ready` - Query recorder state
- `~/recording_control` - Start/stop/pause/finish recording
- `~/recording_status` - Get recording status

## Documentation

See [project README](../../../../README.md) for more details.
