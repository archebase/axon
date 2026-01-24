# Axon HTTP Client - Python

Python HTTP client for controlling [Axon recorder](https://github.com/archebase/axon) via its HTTP RPC API.

## Installation

```bash
# Basic installation
pip install -r requirements.txt

# With Zenoh support
pip install -r requirements.txt
pip install eclipse-zenoh

# Development installation
pip install -r requirements-dev.txt
pip install -e .
```

## Quick Start

```python
from axon_client import AxonRecorderClient, TaskConfig

# Connect to recorder
client = AxonRecorderClient(host="localhost", port=8080)

# Configure recording task
config = TaskConfig(
    task_id="task_001",
    device_id="robot_01",
    scene="warehouse",
    topics=["/camera/image_raw", "/lidar/scan"]
)

# Start recording
client.config(config)
client.begin("task_001")

# Record for 10 seconds
import time
time.sleep(10)

# Stop recording
client.finish("task_001")

# Get statistics
stats = client.get_stats()
print(f"Written: {stats.messages_written} messages")
```

## Features

- **Synchronous Client**: Simple blocking API with `requests`
- **Asynchronous Client**: Concurrent operations with `aiohttp`
- **Type Safety**: Dataclasses matching C++ structures
- **Error Handling**: Custom exception hierarchy
- **Retry Logic**: Exponential backoff for network resilience
- **Zenoh Integration**: Optional status publishing

## Examples

See the `examples/` directory for complete examples:

- `basic_recording.py` - Simple recording workflow
- `async_recording.py` - Async operations with monitoring
- `zenoh_integration.py` - Status publishing to Zenoh

```bash
# Run examples
python examples/basic_recording.py
python examples/async_recording.py
python examples/zenoh_integration.py
```

## API Reference

### Client Methods

| Method | Description |
|--------|-------------|
| `config(task_config)` | Cache task configuration (IDLE → READY) |
| `begin(task_id)` | Start recording (READY → RECORDING) |
| `finish(task_id)` | Stop recording (RECORDING → IDLE) |
| `cancel(task_id)` | Cancel recording (discards data) |
| `pause()` | Pause recording (RECORDING → PAUSED) |
| `resume()` | Resume recording (PAUSED → RECORDING) |
| `get_state()` | Get current recorder state |
| `get_stats()` | Get recording statistics |
| `health()` | Check if recorder is running |

### Data Models

```python
from axon_client import RecorderState, Statistics, TaskConfig

# Recorder states: IDLE, READY, RECORDING, PAUSED
state = RecorderState.RECORDING

# Task configuration
config = TaskConfig(
    task_id="task_001",
    device_id="robot_01",
    topics=["/camera/image_raw"]
)

# Statistics
stats = Statistics(
    messages_received=1000,
    messages_written=995,
    messages_dropped=5
)
print(f"Drop rate: {stats.drop_rate():.1%}")
```

## Zenoh Integration

```python
from axon_client import ZenohStatusPublisher, ZenohConfig

# Create publisher
publisher = ZenohStatusPublisher(ZenohConfig())
publisher.start()

# Publish state updates
publisher.publish_state("robot_01", RecorderState.RECORDING)

# Publish statistics
publisher.publish_stats("robot_01", stats)

# Stop publisher
publisher.stop()
```

## Requirements

- Python 3.8+
- `requests` >= 2.31.0
- `aiohttp` >= 3.9.0
- `eclipse-zenoh` >= 1.0.0 (optional)

## License

MulanPSL-2.0

## See Also

- [Axon Recorder Documentation](https://github.com/archebase/axon)
- [HTTP RPC API Design](../docs/designs/rpc-api-design.md)
