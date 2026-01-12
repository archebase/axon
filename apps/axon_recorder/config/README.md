# Axon Recorder Configuration Files

This directory contains configuration files for the Axon recorder.

## Configuration Files

### ROS1 Configuration
- **[default_config_ros1.yaml](default_config_ros1.yaml)** - Default configuration for ROS1 (Noetic)
  - Sets `profile: ros1`
  - Use with ROS1 plugin: `libaxon_ros1_plugin.so`

### ROS2 Configuration
- **[default_config_ros2.yaml](default_config_ros2.yaml)** - Default configuration for ROS2 (Humble/Jazzy/Rolling)
  - Sets `profile: ros2`
  - Use with ROS2 plugin: `libaxon_ros2_plugin.so`

## Usage

### Method 1: Plugin path in config file (Recommended)

Set the `plugin.path` in the config file:

```bash
# ROS1
./axon_recorder --config config/default_config_ros1.yaml

# ROS2
./axon_recorder --config config/default_config_ros2.yaml
```

### Method 2: Plugin path via command line

```bash
# ROS1
./axon_recorder --config config/default_config_ros1.yaml --plugin /path/to/libaxon_ros1_plugin.so

# ROS2
./axon_recorder --config config/default_config_ros2.yaml --plugin /path/to/libaxon_ros2_plugin.so
```

## Configuration Sections

### Plugin
- `path`: Path to the plugin shared library (`.so` file)
  - If specified, the recorder will automatically load the plugin on startup
  - If empty, the `--plugin` command line argument must be provided
  - Common locations:
    - Build: `middlewares/ros1/build/axon_ros1_plugin/libaxon_ros1_plugin.so`
    - Install: `/usr/local/lib/axon/plugins/libaxon_ros1_plugin.so`
- `config`: Optional JSON configuration passed to the plugin during initialization
  - ROS1: `{"node_name": "custom_name"}`
  - ROS2: `{"node_name": "custom_name", "namespace": "custom_ns"}`

### Dataset
- `path`: Output directory for MCAP files
- `output_file`: MCAP filename
- `queue_size`: Per-topic message queue capacity

### Subscriptions
List of topics to record with batching settings:
- `name`: Topic name
- `message_type`: ROS message type
- `batch_size`: Number of messages to batch before writing
- `flush_interval_ms`: Maximum time to wait before flushing (ms)

### Recording
- `profile`: ROS profile (`ros1` or `ros2`)
- `compression`: MCAP compression (`none`, `zstd`, `lz4`)
- `compression_level`: Compression level (zstd: 1-19, lz4: 1-12)

### Logging
- `console`: Console logging configuration
- `file`: File logging configuration

### Upload
- `enabled`: Enable/disable automatic S3 upload
- `s3`: S3-compatible storage configuration
- `retry`: Retry policy for uploads
- `state_db_path`: SQLite database for crash recovery

## Environment Variables

Logging can be configured via environment variables:
- `AXON_LOG_LEVEL` - Global log level
- `AXON_LOG_CONSOLE_LEVEL` - Console log level
- `AXON_LOG_FILE_LEVEL` - File log level
- `AXON_LOG_FILE_DIR` - Log file directory
- `AXON_LOG_FORMAT` - Log format (`json` or `text`)
- `AXON_LOG_FILE_ENABLED` - Enable file logging
- `AXON_LOG_CONSOLE_ENABLED` - Enable console logging
