# UDP Reception and JSON Recording Design Document

**Date:** 2026-02-26
**Status:** In Design
**Target Version:** 0.3.0

## Overview

This document describes the design for UDP-based JSON data reception and recording in Axon. The goal is to enable recording of non-ROS data sources (custom sensors, external systems, IoT devices) alongside ROS messages in a unified MCAP file, supporting hybrid recording scenarios common in robotics deployments.

## Motivation

### Current Limitations

1. **ROS-Only Data Sources**: Axon currently records only from ROS middleware plugins (ROS1/ROS2)
2. **No JSON Support**: Many external systems output JSON telemetry data that cannot be recorded
3. **Data Silos**: Non-ROS sensor data must be converted to ROS messages or stored separately
4. **Deployment Complexity**: Each non-ROS data source requires custom integration

### Use Cases

1. **Custom Sensors**: LiDAR, IMU, or cameras with proprietary UDP output
2. **External Systems**: GPS receivers, CAN bus bridges, industrial PLCs
3. **IoT Devices**: Environmental sensors, power monitors, network equipment
4. **Hybrid Recording**: Both ROS binary data and UDP JSON in a single MCAP
5. **Testing & Simulation**: Inject test data via UDP without ROS infrastructure

### Benefits

1. **Unified Storage**: All data in one MCAP file with synchronized timestamps
2. **Zero-Copy Potential**: UDP datagrams can be written directly without ROS serialization
3. **Language Agnostic**: Any system that can send UDP packets can be recorded
4. **Low Latency**: UDP provides minimal overhead for high-frequency data

## Architecture

### System Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           Axon Recorder (C++)                                │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────────────┐  │
│  │  HTTP RPC API   │    │  State Machine  │    │    MCAP Writer          │  │
│  └────────┬────────┘    └────────┬────────┘    └───────────▲─────────────┘  │
│           │                      │                         │                 │
│           │                      │                         │                 │
│  ┌────────▼──────────────────────▼─────────────────────────┴──────────────┐ │
│  │                        Plugin Loader                                    │ │
│  │  - Loads libaxon_ros2.so (ROS2 messages)                              │ │
│  │  - Loads libaxon_udp.so (UDP JSON messages)                           │ │
│  └────────────────────────────┬───────────────────────────────────────────┘ │
│                               │                                              │
│           ┌───────────────────┼───────────────────┐                         │
│           │                   │                   │                          │
│  ┌────────▼────────┐  ┌───────▼───────┐  ┌───────▼────────┐                 │
│  │  ROS2 Plugin    │  │  UDP Plugin   │  │  (Future)      │                 │
│  │  libaxon_ros2   │  │  libaxon_udp  │  │  Zenoh Plugin  │                 │
│  │                 │  │               │  │                │                 │
│  │  - rclcpp       │  │  - UDP Server │  │                │                 │
│  │  - CDR encoding │  │  - JSON parse │  │                │                 │
│  └─────────────────┘  └───────────────┘  └────────────────┘                 │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
                                    ▲
                    UDP Datagrams   │
                                    │
┌───────────────────────────────────┴─────────────────────────────────────────┐
│                           External Systems                                   │
│                                                                              │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │ GPS Receiver│  │ CAN Bridge  │  │ IoT Sensor  │  │ Test Tool   │        │
│  │             │  │             │  │             │  │             │        │
│  │ UDP:4242    │  │ UDP:4243    │  │ UDP:4244    │  │ UDP:4245    │        │
│  └─────────────┘  └─────────────┘  └─────────────┘  └─────────────┘        │
│                                                                              │
│  JSON Payload: {"timestamp": 1234567890, "lat": 37.7749, "lon": -122.4194}  │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Plugin Architecture Integration

The UDP receiver is implemented as a **middleware plugin** (`libaxon_udp.so`), following the same pattern as ROS1/ROS2 plugins:

```
middlewares/
├── ros1/           # ROS1 Noetic plugin
├── ros2/           # ROS2 Humble/Jazzy/Rolling plugin
├── zenoh/          # (Future) Zenoh plugin
├── udp/            # UDP JSON plugin (New)
│   ├── CMakeLists.txt
│   ├── include/
│   │   └── axon_udp/
│   │       ├── udp_plugin.hpp
│   │       ├── udp_server.hpp
│   │       └── json_schema.hpp
│   └── src/
│       ├── udp_plugin.cpp
│       ├── udp_server.cpp
│       └── json_schema.cpp
├── mock/           # Mock plugin for testing
└── filters/        # Data processing filters
```

### Design Decision: Plugin vs Core Feature

| Aspect | Plugin Approach | Core Feature |
|--------|-----------------|--------------|
| Deployment | Optional, load on demand | Always included |
| Dependencies | Isolated in plugin | Added to core |
| Testing | Independent unit tests | Requires full recorder |
| Consistency | Follows existing patterns | New integration point |

**Decision:** Implement as a **plugin** because:
- Optional deployment (not all robots need UDP)
- Clear dependency isolation (no external libs in core)
- Consistency with ROS plugin architecture
- Easier to test and maintain

## UDP Server Design

### Server Configuration

```yaml
# Task configuration extension
udp:
  enabled: true
  bind_address: "0.0.0.0"
  default_port: 4242
  buffer_size: 65536        # 64KB receive buffer
  max_message_size: 65507   # Max UDP payload

  # Stream definitions (multiple UDP endpoints)
  streams:
    - name: "gps"
      port: 4242
      topic: "/udp/gps"
      schema: "gps_data"
      enabled: true

    - name: "can_bridge"
      port: 4243
      topic: "/udp/can"
      schema: "can_frame"
      enabled: true

    - name: "custom_sensor"
      port: 4244
      topic: "/udp/sensor/imu"
      schema: "raw_json"     # Passthrough mode
      enabled: false
```

### UDP Server Implementation

```cpp
namespace axon {
namespace udp {

struct UdpStreamConfig {
    std::string name;           // Stream identifier
    uint16_t port;              // UDP port to listen on
    std::string topic;          // MCAP topic name
    std::string schema_name;    // Schema identifier
    bool enabled;               // Stream active flag
};

class UdpServer {
public:
    UdpServer(boost::asio::io_context& io_context);
    ~UdpServer();

    // Lifecycle management
    bool start(const std::vector<UdpStreamConfig>& streams);
    void stop();

    // Message callback registration
    using MessageCallback = std::function<void(
        const std::string& topic,
        const uint8_t* data,
        size_t size,
        uint64_t timestamp
    )>;
    void set_message_callback(MessageCallback callback);

    // Statistics
    struct Stats {
        uint64_t packets_received;
        uint64_t bytes_received;
        uint64_t parse_errors;
        uint64_t buffer_overruns;
    };
    Stats get_stats() const;

private:
    // Per-port socket and receive buffer
    struct StreamEndpoint {
        std::unique_ptr<boost::asio::ip::udp::socket> socket;
        std::vector<uint8_t> receive_buffer;
        UdpStreamConfig config;
        Stats stats;
    };

    void async_receive(StreamEndpoint& stream);
    void handle_receive(StreamEndpoint& stream, std::size_t bytes_transferred);

    boost::asio::io_context& io_context_;
    std::unordered_map<uint16_t, StreamEndpoint> streams_;
    MessageCallback message_callback_;
    std::atomic<bool> running_{false};
};

}  // namespace udp
}  // namespace axon
```

### Threading Model

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           Threading Architecture                             │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌─────────────────┐     ┌─────────────────┐     ┌─────────────────────┐    │
│  │ HTTP RPC Thread │     │ State Machine   │     │  Worker Thread Pool │    │
│  │                 │     │ Thread          │     │  (Per-Topic Queues) │    │
│  └─────────────────┘     └─────────────────┘     │  ┌───────────────┐  │    │
│                                                   │  │ /udp/gps     │  │    │
│  ┌─────────────────────────────────────────────┐ │  │ SPSC Queue   │  │    │
│  │ Plugin Executor Thread (UDP)                │ │  └───────────────┘  │    │
│  │                                             │ │                     │    │
│  │  ┌─────────────────────────────────────┐   │ │  ┌───────────────┐  │    │
│  │  │ boost::asio::io_context             │   │ │  │ /udp/can     │  │    │
│  │  │                                     │   │ │  │ SPSC Queue   │  │    │
│  │  │  ┌──────────┐ ┌──────────┐         │   │ │  └───────────────┘  │    │
│  │  │  │ UDP:4242 │ │ UDP:4243 │ ...     │   │ │                     │    │
│  │  │  │ Socket   │ │ Socket   │         │   │ │  ┌───────────────┐  │    │
│  │  │  └──────────┘ └──────────┘         │   │ │  │ /udp/sensor  │  │    │
│  │  │                                     │   │ │  │ SPSC Queue   │  │    │
│  │  └─────────────────────────────────────┘   │ │  └───────────────┘  │    │
│  │                                             │ │                     │    │
│  └─────────────────────────────────────────────┘ └─────────────────────┘    │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

**Thread Responsibilities:**

| Thread | Role | Blocking Operations |
|--------|------|---------------------|
| HTTP RPC | Handle API requests | HTTP I/O (allowed) |
| State Machine | State transitions | None (async) |
| Plugin Executor | UDP receive + dispatch | Socket I/O (allowed) |
| Worker Pool | Write to MCAP | Disk I/O (allowed) |

**Key Principle:** UDP receive thread does **not** block on MCAP writes. Messages are pushed to SPSC queues, worker threads handle disk I/O.

## JSON Schema System

### Schema Definition

The system supports two modes:

1. **Structured Schema**: Define expected JSON fields and types
2. **Raw JSON Passthrough**: Store JSON as-is without validation

#### Structured Schema Definition

```yaml
# Schema registry configuration
schemas:
  gps_data:
    description: "GPS position data"
    fields:
      - name: "timestamp"
        type: "uint64"
        required: true
      - name: "latitude"
        type: "float64"
        required: true
      - name: "longitude"
        type: "float64"
        required: true
      - name: "altitude"
        type: "float64"
        required: false
      - name: "hdop"
        type: "float32"
        required: false
      - name: "satellites"
        type: "uint32"
        required: false

  can_frame:
    description: "CAN bus frame"
    fields:
      - name: "timestamp"
        type: "uint64"
        required: true
      - name: "can_id"
        type: "uint32"
        required: true
      - name: "dlc"
        type: "uint8"
        required: true
      - name: "data"
        type: "bytes"
        required: true
```

#### MCAP Schema Registration

For JSON messages, we use MCAP's native JSON schema support:

```cpp
// JSON Schema for MCAP
// Schema name: "axon_udp/json"
// Schema encoding: "jsonschema"
// Schema data: JSON Schema definition

{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://archebase.io/schemas/gps_data",
  "title": "GPS Data",
  "type": "object",
  "properties": {
    "timestamp": {"type": "integer", "minimum": 0},
    "latitude": {"type": "number", "minimum": -90, "maximum": 90},
    "longitude": {"type": "number", "minimum": -180, "maximum": 180},
    "altitude": {"type": "number"},
    "hdop": {"type": "number"},
    "satellites": {"type": "integer", "minimum": 0}
  },
  "required": ["timestamp", "latitude", "longitude"]
}
```

### Message Encoding

| Mode | MCAP Message Encoding | Schema Encoding | Description |
|------|----------------------|-----------------|-------------|
| Structured | `json` | `jsonschema` | Validated JSON with schema |
| Raw | `json` | `jsonschema` | Minimal schema, no validation |

### Timestamp Extraction

JSON messages must include a timestamp field. The system supports two strategies:

```yaml
udp:
  timestamp_extraction:
    source: "field"           # "field" | "arrival"
    field: "timestamp"        # Field path, supports dot notation (e.g., "timestamp" or "header.stamp")

    # Strategy 2: Use packet arrival time
    # source: "arrival"
```

**Strategy 1 Description**: When `source: "field"`, configure the field path via `field`, supporting dot notation to access nested fields:
- `field: "timestamp"` → `json["timestamp"]`
- `field: "header.stamp"` → `json["header"]["stamp"]`

Timestamp unit is fixed to **nanoseconds**.

```cpp
// Timestamp extraction implementation
class TimestampExtractor {
public:
    struct Config {
        enum class Source { Field, Arrival };
        Source source = Source::Field;
        std::string field = "timestamp";  // Supports dot-notation path
    };

    uint64_t extract(const nlohmann::json& json, uint64_t arrival_time);

private:
    uint64_t extract_from_path(const nlohmann::json& json, const std::string& path);
};
```

## State Machine Integration

UDP streams follow the recorder state machine:

| State | UDP Behavior |
|-------|--------------|
| IDLE | Sockets closed, not receiving |
| READY | Sockets open, receiving but not recording |
| RECORDING | Receiving and writing to MCAP |
| PAUSED | Receiving but discarding messages |

```cpp
// State transition handling
void UdpPlugin::on_state_change(State new_state, State old_state) {
    switch (new_state) {
        case State::Idle:
            stop_servers();
            break;
        case State::Ready:
            start_servers();  // Receive but don't record
            break;
        case State::Recording:
            // Already receiving, now enable recording
            enable_recording(true);
            break;
        case State::Paused:
            enable_recording(false);  // Discard incoming messages
            break;
    }
}
```

## Plugin Implementation

### Plugin ABI Compliance

The UDP plugin implements the standard Axon plugin interface:

```cpp
// middlewares/udp/src/udp_plugin.cpp

extern "C" {

// Plugin descriptor
AxonPluginDescriptor axon_get_plugin_descriptor() {
    static AxonPluginVtable vtable = {
        .init = udp_plugin_init,
        .start = udp_plugin_start,
        .stop = udp_plugin_stop,
        .subscribe = nullptr,  // UDP doesn't subscribe, it receives
        .publish = nullptr,    // UDP doesn't publish
        // ... reserved slots
    };

    static AxonPluginDescriptor descriptor = {
        .abi_version_major = 1,
        .abi_version_minor = 0,
        .middleware_name = "UDP",
        .middleware_version = "1.0.0",
        .plugin_version = "0.3.0",
        .vtable = &vtable,
    };

    return descriptor;
}

AxonStatus udp_plugin_init(const char* config_yaml) {
    // Parse YAML config, set up streams
    return AXON_SUCCESS;
}

AxonStatus udp_plugin_start() {
    // Start UDP servers, begin receiving
    return AXON_SUCCESS;
}

AxonStatus udp_plugin_stop() {
    // Stop servers, cleanup resources
    return AXON_SUCCESS;
}

}  // extern "C"
```

### Message Callback Integration

```cpp
void UdpPlugin::on_udp_message(
    const std::string& topic,
    const uint8_t* data,
    size_t size,
    uint64_t timestamp
) {
    // Forward to recorder via registered callback
    if (message_callback_) {
        message_callback_(
            topic.c_str(),
            data,
            size,
            "axon_udp/json",  // Message type
            timestamp,
            user_data_
        );
    }
}
```

## Hybrid Recording

### Simultaneous ROS + UDP Recording

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           Recording Session                                  │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  Task: task_20260226_143000                                                 │
│  Output: /recordings/task_20260226_143000.mcap                              │
│                                                                              │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                        MCAP File                                     │    │
│  ├─────────────────────────────────────────────────────────────────────┤    │
│  │                                                                     │    │
│  │  Schemas:                                                           │    │
│  │  ├── sensor_msgs/msg/Image (ros2msg)                               │    │
│  │  ├── sensor_msgs/msg/Imu (ros2msg)                                 │    │
│  │  ├── geometry_msgs/msg/Twist (ros2msg)                             │    │
│  │  └── axon_udp/json (jsonschema) - GPS Data                         │    │
│  │                                                                     │    │
│  │  Channels:                                                          │    │
│  │  ├── /camera/image_raw (cdr) - ROS2                                │    │
│  │  ├── /imu/data (cdr) - ROS2                                        │    │
│  │  ├── /cmd_vel (cdr) - ROS2                                         │    │
│  │  └── /udp/gps (json) - UDP                                         │    │
│  │                                                                     │    │
│  │  Messages:                                                          │    │
│  │  ├── t=0.000s: /camera/image_raw (ROS2 CDR)                        │    │
│  │  ├── t=0.001s: /udp/gps (JSON)                                     │    │
│  │  ├── t=0.002s: /imu/data (ROS2 CDR)                                │    │
│  │  ├── t=0.010s: /camera/image_raw (ROS2 CDR)                        │    │
│  │  └── ...                                                            │    │
│  │                                                                     │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Timestamp Synchronization

All messages use **nanoseconds since Unix epoch** for consistent synchronization:

| Source | Timestamp Source | Conversion |
|--------|------------------|------------|
| ROS2 | `header.stamp` or receive time | Already nanoseconds |
| UDP | JSON field or arrival time | Convert from configured units |

```cpp
// Unified timestamp handling
uint64_t normalize_timestamp(uint64_t raw, TimestampSource source) {
    switch (source) {
        case TimestampSource::RosHeader:
            return raw;  // Already nanoseconds
        case TimestampSource::JsonField:
            return convert_to_nanoseconds(raw, config_.units);
        case TimestampSource::ArrivalTime:
            return get_steady_clock_ns();
    }
}
```

## Performance Considerations

### Memory Management

1. **Buffer Reuse**: Pre-allocated receive buffer per stream
2. **Zero-Copy Path**: JSON data written directly to MCAP without additional copies
3. **Queue Sizing**: SPSC queue size based on expected message rate

```cpp
// Recommended buffer sizes
constexpr size_t UDP_RECEIVE_BUFFER = 65536;      // 64KB per socket
constexpr size_t SPSC_QUEUE_CAPACITY = 1024;      // Messages per topic
constexpr size_t JSON_MAX_SIZE = 65507;           // Max UDP payload
```

### High-Frequency Optimization

For streams >1000 messages/second:

1. **Batching**: Aggregate multiple messages before MCAP write
2. **Compression**: Enable Zstd for JSON (good compression ratio)
3. **Dedicated Worker**: Allocate dedicated worker thread for high-frequency topics

```yaml
udp:
  streams:
    - name: "high_freq_imu"
      port: 4250
      topic: "/udp/imu"
      schema: "imu_data"
      high_frequency: true        # Enable optimizations
      batch_size: 100             # Batch 100 messages
      batch_timeout_ms: 10        # Or timeout after 10ms
```

### Benchmark Targets

| Metric | Target | Notes |
|--------|--------|-------|
| UDP Receive Rate | >100K packets/sec | Per socket |
| JSON Parse Rate | >50K messages/sec | Simple schema |
| MCAP Write Rate | >10K messages/sec | With compression |
| Latency (receive to MCAP) | <1ms | 99th percentile |

## Error Handling

### UDP Packet Errors

```cpp
enum class UdpError {
    Success = 0,
    BufferOverflow,       // Packet larger than buffer
    JsonParseError,       // Invalid JSON
    SchemaValidationError,// Missing required field
    TimestampMissing,     // No timestamp in message
    UnknownSchema,        // Schema not registered
};
```

### Error Handling Strategy

| Error Type | Action | Logging |
|------------|--------|---------|
| Buffer overflow | Drop packet, increment counter | WARN (rate-limited) |
| JSON parse error | Drop packet, increment counter | WARN (rate-limited) |
| Schema validation | Drop packet, increment counter | DEBUG |
| Missing timestamp | Use arrival time | DEBUG |
| Unknown schema | Use raw JSON mode | INFO |

```cpp
void UdpPlugin::handle_error(UdpError error, const std::string& stream_name) {
    stats_.error_count[error]++;

    // Rate-limited logging (max 1 per second per error type)
    if (should_log(error)) {
        AXON_LOG_WARN("UDP stream '{}' error: {}",
            stream_name, error_to_string(error));
    }
}
```

## Testing Strategy

### Unit Tests

1. **UDP Server**
   - Socket creation and binding
   - Packet reception and parsing
   - Multi-stream handling
   - Error conditions

2. **JSON Schema**
   - Schema validation
   - Timestamp extraction
   - Unit conversion
   - Edge cases (missing fields, invalid types)

3. **Plugin Interface**
   - Init/start/stop lifecycle
   - Configuration parsing
   - Callback registration

### Integration Tests

1. **Hybrid Recording**
   - ROS2 + UDP in same MCAP
   - Timestamp synchronization
   - State machine transitions

2. **High Load**
   - Multiple concurrent streams
   - High-frequency data
   - Buffer pressure

### E2E Tests

```bash
# Test UDP reception with mock sender
./test_udp_recording.sh

# Test hybrid recording
./test_hybrid_recording.sh --ros2 --udp

# Performance benchmark
./benchmark_udp_throughput.sh --rate 100000
```

## Deployment

### Binary Size Impact

| Component | Size (approx) | Notes |
|-----------|---------------|-------|
| libaxon_udp.so | ~200KB | Includes nlohmann/json |
| Dependencies | 0 | Header-only JSON library |

### Configuration Example

```yaml
# Complete UDP configuration example
version: "1.0"
task_id: "task_001"
device_id: "robot_001"

topics:
  - /camera/image_raw
  - /imu/data

# UDP streams (optional)
udp:
  enabled: true
  bind_address: "0.0.0.0"
  buffer_size: 65536

  streams:
    - name: "gps"
      port: 4242
      topic: "/udp/gps"
      schema: "gps_data"
      enabled: true

    - name: "can"
      port: 4243
      topic: "/udp/can"
      schema: "can_frame"
      enabled: true

  timestamp_extraction:
    source: "field"
    field: "timestamp"
    units: "milliseconds"

# Schema definitions
schemas:
  gps_data:
    fields:
      - name: "timestamp"
        type: "uint64"
        required: true
      - name: "latitude"
        type: "float64"
        required: true
      - name: "longitude"
        type: "float64"
        required: true

  can_frame:
    fields:
      - name: "timestamp"
        type: "uint64"
        required: true
      - name: "can_id"
        type: "uint32"
        required: true
      - name: "data"
        type: "bytes"
        required: true
```

### Startup Example

```bash
# Start recorder with ROS2 and UDP plugins
./axon_recorder \
  --plugin ./libaxon_ros2.so \
  --plugin ./libaxon_udp.so \
  --config /etc/axon/task_config.yaml

# UDP plugin will automatically:
# 1. Parse UDP configuration
# 2. Register schemas
# 3. Open sockets on configured ports
# 4. Begin receiving when state becomes READY
```

## Future Enhancements

### Version 0.4.0+

1. **UDP Multicast Support**: Receive from multicast groups
2. **Binary Protocol Support**: Custom binary encodings beyond JSON
3. **Schema Inference**: Auto-detect schema from received messages
4. **Message Filtering**: Filter UDP messages by content
5. **QoS Settings**: Configurable packet handling policies

### Version 0.5.0+

1. **UDP TLS/DTLS**: Encrypted UDP streams
2. **Compression**: Per-stream compression options
3. **Backpressure**: Flow control for overloaded receivers

## Summary

This design introduces UDP-based JSON recording to Axon through a plugin architecture:

- **Plugin-Based**: `libaxon_udp.so` follows established patterns
- **Schema Support**: Structured JSON with validation or raw passthrough
- **Hybrid Recording**: ROS2 + UDP in a single MCAP
- **High Performance**: Optimized for high-frequency data streams

The implementation enables recording from non-ROS sources while maintaining Axon's core principles of reliability, high performance, and simplicity.
