# Axon Recorder Configuration Files

This directory contains configuration files for the Axon recorder. The ROS1 and ROS2
samples are also Keystone-ready `recorder.yaml` templates for `axon_config register`.

## Configuration Files

### ROS1 Configuration
- **[default_config_ros1.yaml](default_config_ros1.yaml)** - Default configuration for ROS1 (Noetic)
  - Sets `profile: ros1`
  - Sets `rpc.mode: ws_client` and fills the RPC URL during registration
  - Use with ROS1 plugin: `libaxon_ros1_plugin.so`

### ROS2 Configuration
- **[default_config_ros2.yaml](default_config_ros2.yaml)** - Default configuration for ROS2 (Humble/Jazzy/Rolling)
  - Sets `profile: ros2`
  - Sets `rpc.mode: ws_client` and fills the RPC URL during registration
  - Use with ROS2 plugin: `libaxon_ros2_plugin.so`

## Usage

### Method 1: Keystone Template Workflow

Edit the `subscriptions` list for the robot type, then upload the file to Keystone as
`recorder.yaml`. `axon_config register` renders the WebSocket RPC URL and writes the final
config to `/etc/axon/recorder.yaml`.

### Method 2: Plugin path in config file

Set the `plugin.path` in the config file. For local-only runs without Keystone, also change
`rpc.mode` to `http_server` or replace `rpc.ws_client.url` with a concrete WebSocket URL:

```bash
# ROS1
./axon_recorder --config config/default_config_ros1.yaml

# ROS2
./axon_recorder --config config/default_config_ros2.yaml
```

### Method 3: Plugin path via command line

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
  - ROS1: `{"node_name": "custom_name", "namespace": "/custom_ns"}`
  - ROS2: `{"node_name": "custom_name", "namespace": "/custom_ns"}`

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
- `qos_depth`: Backward-compatible ROS2 subscription history depth (default `10`).
- `qos`: Optional ROS2 QoS object. Any empty field falls back to the default.
  - `mode`: `auto` to detect publisher QoS when available
  - `depth`: History depth, or `auto` (default `10`)
  - `reliability`: `reliable`, `best_effort`, or `auto` (default `reliable`)
  - `durability`: `volatile`, `transient_local`, or `auto` (default `volatile`)
  - `history`: `keep_last`, `keep_all`, or `auto` (default `keep_last`)

QoS depth controls the ROS2 middleware queue, not the recorder's
`dataset.queue_size` worker queue.

### Recording
- `profile`: ROS profile (`ros1` or `ros2`)
- `compression`: MCAP compression (`none`, `zstd`, `lz4`)
- `compression_level`: Compression level preset (applies to both zstd and lz4).
  Maps recorder/CLI values to MCAP presets: `0`=Default, `1`=Fastest, `2`=Fast, `3`=Default, `4`=Slow, `>=5`=Slowest.
  This is *not* the native zstd (1-19) or lz4 (1-12) range; the underlying MCAP C++ library only exposes 5 presets.
- `max_disk_usage_gb`: Backward-compatible alias for `disk_usage.hard_limit_gb`
- `disk_usage`: Recorder-managed disk budget
  - `enabled`: Enable warn/hard disk guard checks
  - `warn_usage_gb`: Log warning and expose `disk_usage.state=warn`
  - `hard_limit_gb`: Refuse new recordings and pause active writes at hard limit
  - `max_task_size_gb`: Optional per-task MCAP size limit (`0` disables)
  - `cleanup_enabled`: Delete oldest completed `.mcap`/`.json` files before refusing start
  - `cleanup_target_gb`: Cleanup target when cleanup is enabled
  - `cleanup_upload_backlog`: Also allow cleanup in failed upload backlog directory

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
