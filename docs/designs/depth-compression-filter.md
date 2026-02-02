# Depth Compression Filter Design Document

## Overview

This document describes the design and implementation of the depth compression filter in Axon Recorder. Depth data (16-bit depth images) consume significant storage space, and specialized compression algorithms can greatly reduce storage requirements.

**Goals**:
- Compress depth image data during recording
- Transparent to plugin layer, supporting ROS2
- Configurable compression parameters

## Background

### Depth Data Characteristics

| Property | Value |
|----------|-------|
| Data Type | `uint16_t` (16-bit) |
| Typical Resolutions | 640x480, 1280x1024, 1920x1080 |
| Raw Size | 640x480x2 = 614 KB/frame |
| Frame Rate | 30 FPS |
| Uncompressed Bandwidth | ~18 MB/s |

### Depth Data Format

Depth images typically use these ROS message types:
- `sensor_msgs/msg/Image` (ROS2)
- Encoding: `16UC1` (16-bit unsigned integer, 1 channel)

### Compression Library: DepthLiteZ

The `middlewares/filters/depthlitez` submodule provides specialized depth compression algorithms:
- **Input**: 16-bit depth data raw data
- **Output**: Compressed data stream
- **Compression Ratio**: Typically 5-10x
- **Lossy/Lossless**: Supports configurable quality parameters

## Architecture Design

### Overall Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      Axon Recorder                          │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  ┌──────────────┐      ┌──────────────┐      ┌────────────┐│
│  │   ROS2       │      │  SPSC Queue  │      │   MCAP     ││
│  │   Plugin     │─────▶│  (per-topic) │─────▶│  Writer    ││
│  └──────────────┘      └──────────────┘      └────────────┘│
│         ▲                                            ▲      │
│         │                                            │      │
│    Subscribe                                    Write         │
│         │                                            │      │
│         │         ┌──────────────────────────────┐   │      │
│         │         │  Depth Compression Filter    │   │      │
│         │         │  ┌─────────────────────────┐  │   │      │
│         │         │  │  is_depth_image()?     │  │   │      │
│         │         │  └───────────┬─────────────┘  │   │      │
│         │         │              │ Yes            │   │      │
│         │         │  ┌───────────▼─────────────┐  │   │      │
│         │         │  │  DepthCompressor::      │  │   │      │
│         │         │  │  compress()             │  │   │      │
│         │         │  └───────────┬─────────────┘  │   │      │
│         │         │              │                │   │      │
│         │         │  ┌───────────▼─────────────┐  │   │      │
│         │         │  │  Serialize Compressed   │──┼──▶      │
│         │         │  │  Image                  │  │   │      │
│         │         │  └─────────────────────────┘  │   │      │
│         │         └──────────────────────────────┘   │      │
│         │                                              │      │
└─────────┼──────────────────────────────────────────────┼──────┘
          │                                              │
     ROS Topics                                    MCAP File
   /camera/depth/*
```

### Key Design Decisions

1. **Compression Location**: Executed in ROS2 plugin subscription callback
   - ✅ No plugin callback thread blocking
   - ✅ Compression happens before queuing to worker threads
   - ✅ Worker threads write pre-compressed data

2. **Compression Recognition**: Via configuration file
   - Topics with `depth_compression: true` enable compression
   - Validates `message_type` is `sensor_msgs/msg/Image`
   - Validates image encoding is `16UC1`

3. **Compressed Storage**: Uses `sensor_msgs/msg/CompressedImage` message type
   - Compressed data stored in `CompressedImage.data` field
   - Compression format identifier in `CompressedImage.format`: `dlz_fast`, `dlz_medium`, `dlz_max`
   - MCAP schema becomes `sensor_msgs/msg/CompressedImage`
   - Readers identify compression format via `format` field for decompression

## Component Design

### 1. DepthCompressor Class

**Location**: `middlewares/filters/depth_compressor.hpp`

```cpp
#include <dlz.hpp>  // depthlitez compression library

namespace axon {
namespace depth {

class DepthCompressor {
public:
  struct Config {
    dlz::CompressionLevel level = dlz::CompressionLevel::kFast;  // Default: fast
    std::string encoding = "16UC1"; // Input encoding format
  };

  /**
   * Compress depth image data
   * @param depth_data 16-bit depth data (row-major)
   * @param width Image width
   * @param height Image height
   * @param compressed_data Output compressed data
   * @return true if compression succeeded
   */
  bool compress(
    const uint8_t* depth_data,
    size_t width,
    size_t height,
    std::vector<uint8_t>& compressed_data
  );

  /**
   * Get compression format identifier for current config
   * @return Compression format string for CompressedImage.format ("dlz_fast", "dlz_medium", "dlz_max")
   */
  std::string get_compression_format() const;

  /**
   * Set compressor configuration
   */
  void set_config(const Config& config);

private:
  Config config_;
};

} // namespace depth
} // namespace axon
```

### 2. DepthCompressionFilter Class

**Location**: `middlewares/ros2/include/depth_compression_filter.hpp`

```cpp
namespace ros2_plugin {

struct DepthCompressionConfig {
  bool enabled = false;
  std::string level = "fast";  // fast, medium, max (maps to dlz::CompressionLevel)
};

/**
 * @brief Depth image compression filter for ROS2 plugin
 *
 * Processes sensor_msgs::msg::Image messages and compresses 16UC1 depth images
 * to sensor_msgs::msg::CompressedImage format using DepthLiteZ compression.
 */
class DepthCompressionFilter {
public:
  explicit DepthCompressionFilter(const DepthCompressionConfig& config);

  /**
   * Filter and process messages
   * - Checks if message is an Image with 16UC1 encoding
   * - Compresses depth data if applicable
   * - Passes through non-depth or failed compression unchanged
   */
  void filter_and_process(
    const std::string& topic,
    const std::string& message_type,
    const std::vector<uint8_t>& data,
    uint64_t timestamp_ns,
    ProcessedCallback callback
  );

private:
  /**
   * Extract depth data from serialized Image message
   * @return Tuple of (data pointer, width, height) or (nullptr, 0, 0)
   */
  std::tuple<const uint8_t*, size_t, size_t> extract_depth_data(
    const std::vector<uint8_t>& image_msg
  );

  /**
   * Build CompressedImage message with compressed data
   */
  std::vector<uint8_t> build_compressed_image_msg(
    const std::string& format,
    const std::vector<uint8_t>& compressed_data,
    uint64_t timestamp_ns
  );

  DepthCompressor compressor_;
  DepthCompressionConfig config_;
};

} // namespace ros2_plugin
```

### 3. Configuration Extension

**Location**: `apps/axon_recorder/recorder.hpp`

```cpp
struct DepthCompression {
  bool enabled = false;
  std::string level = "fast";  // fast, medium, max (maps to dlz::CompressionLevel)
};

struct SubscriptionConfig {
  std::string topic_name;
  std::string message_type;

  // Depth compression configuration (optional)
  std::optional<DepthCompression> depth_compression;

  // Existing configuration
  size_t batch_size = 1;
  int flush_interval_ms = 100;
};
```

## Configuration

### YAML Configuration Examples

**Simple syntax** (boolean):
```yaml
# config/default_config_ros2.yaml
subscriptions:
  # Depth image with compression enabled
  - name: /camera/depth/image_rect_raw
    message_type: sensor_msgs/msg/Image
    batch_size: 300
    flush_interval_ms: 10000
    depth_compression: true  # Enable with default "fast" level

  # Regular image without compression
  - name: /camera/color/image_raw
    message_type: sensor_msgs/msg/Image
    # No depth_compression field or false = disabled
```

**Advanced syntax** (object):
```yaml
subscriptions:
  - name: /camera/depth/image_rect_raw
    message_type: sensor_msgs/msg/Image
    depth_compression:
      enabled: true
      level: max  # fast, medium, or max
```

**Supported compression levels**:
- `fast` (default): 、fastest speed
- `medium`: balanced
- `max`: best compression

## Recording Flow

### Message Processing Pipeline

```
ROS2 Topic (sensor_msgs::msg::Image)
  │
  ▼
ROS2 Generic Subscription Callback
  │
  ├─▶ DepthCompressionFilter::filter_and_process()
  │       │
  │       ├─▶ Is message_type == sensor_msgs::msg::Image?
  │       │       │ Yes
  │       │       ▼
  │       │   Extract depth data from CDR serialization
  │       │       │
  │       │       ├─▶ Is encoding == 16UC1?
  │       │       │       │ Yes
  │       │       │       ▼
  │       │       │   DepthCompressor::compress()
  │       │       │       │
  │       │       │       ├─▶ Compression successful
  │       │       │       │       │
  │       │       │       │       ▼
  │       │       │       │   Build CompressedImage message
  │       │       │       │       │
  │       │       │       │       ▼
  │       │       │       │   Change message_type to sensor_msgs::msg::CompressedImage
  │       │       │       │
  │       │       │       └─▶ Compression failed → Pass through original
  │       │       │
  │       │       └─▶ Not 16UC1 or compression disabled → Pass through
  │       │
  │       ▼
  │   SPSC Queue (compressed or original message)
  │       │
  ▼       ▼
Worker Thread (per topic)
  │
  ├─▶ Message handler writes to MCAP
  │       │
  │       ▼
  │   MCAP Writer
  │       │
  ▼       ▼
MCAP File (CompressedImage type for compressed depth, Image type for others)
```

**Key Implementation Detail:**
- Compression occurs **before** queuing in the ROS2 subscription callback
- This ensures compression doesn't block worker threads
- Worker threads only handle pre-compressed data for efficient writing

## Implementation Status

### Phase 1: Basic Integration ✅
- [x] Create `DepthCompressor` wrapper class
- [x] Implement `DepthCompressionFilter` with CDR deserialization
- [x] Integrate compression logic in ROS2 subscription callback
- [x] Add configuration file support (boolean and object syntax)
- [x] Auto-register CompressedImage schema in MCAP
- [x] Update channel registration to use composite keys (topic + message_type)

### Phase 2: Testing and Optimization
- [ ] Performance benchmarking
- [ ] Memory usage optimization
- [ ] Unit tests and integration tests

## Performance Considerations

### Expected Performance Metrics

| Metric | Target Value |
|--------|--------------|
| Compression Latency | < 5 ms/frame (640x480) |
| Compression Ratio | 5-10x |
| CPU Usage | < 10% (single core) |
| Memory Overhead | < 50 MB |

### Threading Impact Analysis

**Since compression occurs in the ROS2 subscription callback before queuing:**

```
Compression time (in callback):  T_compress ≈ 3 ms
Queue write time:                T_queue ≈ 0.1 ms
Worker write time:               T_write ≈ 1 ms
Total time per message:          T_total = T_compress + T_queue + T_write ≈ 4.1 ms

Maximum frame rate:              1000 / 4.1 ≈ 244 FPS (per topic)
```

**Advantages of callback-based compression:**
- Worker threads are not blocked by compression
- Multiple topics can process compression in parallel (one callback thread per topic)
- No contention on worker thread pool
- Consistent write throughput

**Trade-offs:**
- Callback thread latency increases (acceptable for typical depth camera frame rates of 30 FPS)
- Memory allocation during compression (mitigated by reusing buffers where possible)

## Error Handling

| Error Scenario | Handling Strategy |
|----------------|-------------------|
| Compression failure | Fall back to writing original data, log warning |
| Unsupported encoding | Skip compression, write original data (16UC1 only) |
| Out of memory | Log error, write original data |
| Configuration error | Validate at startup, refuse to start if invalid |

**Current Implementation:**
- All compression errors are caught in `filter_and_process()` and fall back to pass-through
- Non-16UC1 images are passed through unchanged
- Missing or invalid depth_compression config defaults to disabled

## Usage Examples

### Command Line Usage

```bash
# Using configuration file (recommended)
axon_recorder --config config/default_config_ros2.yaml

# The config file enables compression per topic:
# subscriptions:
#   - name: /camera/depth/image_rect_raw
#     message_type: sensor_msgs/msg/Image
#     depth_compression: true  # Enable with default "fast" level
```

### Configuration Examples

**Simple boolean syntax** (default "fast" level):
```yaml
subscriptions:
  - name: /camera/depth/image_rect_raw
    message_type: sensor_msgs/msg/Image
    batch_size: 300
    flush_interval_ms: 10000
    depth_compression: true  # Enable with default "fast" level
```

**Advanced object syntax** (custom level):
```yaml
subscriptions:
  - name: /camera/depth/image_rect_raw
    message_type: sensor_msgs/msg/Image
    depth_compression:
      enabled: true
      level: max  # Options: fast, medium, max
```

**Compression levels:**
- `fast` (default):fastest speed
- `medium`: balanced
- `max`: best compression

### Programmatic Usage

The depth compression is handled automatically by the ROS2 plugin when enabled in configuration. The compression filter:

1. Intercepts `sensor_msgs::msg::Image` messages in the subscription callback
2. Checks if encoding is `16UC1`
3. Compresses depth data using DepthLiteZ
4. Transforms message to `sensor_msgs::msg::CompressedImage`
5. Updates message type metadata for MCAP schema registration

**Compression format identifiers** (stored in `CompressedImage.format`):
- `dlz::CompressionLevel::kFast` → `"dlz_fast"`
- `dlz::CompressionLevel::kMedium` → `"dlz_medium"`
- `dlz::CompressionLevel::kMax` → `"dlz_max"`

## Testing Plan

### Unit Tests

```cpp
// Test cases for DepthCompressor
TEST(DepthCompressor, Compress16UC1) {
  axon::depth::DepthCompressor compressor;
  std::vector<uint8_t> depth_data(640 * 480 * 2);
  std::vector<uint8_t> compressed;

  ASSERT_TRUE(compressor.compress(depth_data.data(), 640, 480, compressed));
  EXPECT_GT(depth_data.size(), compressed.size());
}

TEST(DepthCompressor, CompressionRatio) {
  // Test real-world compression ratios
}

TEST(DepthCompressor, InvalidInput) {
  // Test error handling
}

// Test cases for DepthCompressionFilter
TEST(DepthCompressionFilter, NonDepthImagePassthrough) {
  // Test that non-16UC1 images pass through unchanged
}

TEST(DepthCompressionFilter, CompressionFailurePassthrough) {
  // Test fallback on compression failure
}
```

### Integration Tests

```bash
# Start ROS2 depth camera publisher
ros2 launch depth_camera_publisher.launch.py

# Record with compression enabled
axon_recorder --config config/default_config_ros2.yaml

# Verify MCAP contents:
# - Topic should use sensor_msgs/msg/CompressedImage schema
# - CompressedImage.format should be "dlz_fast", "dlz_medium", or "dlz_max"
# - Decompressed data should match original depth images
```

### Verification Steps

1. **Check MCAP schema:**
   ```bash
   mcap info recording.mcap | grep CompressedImage
   ```

2. **Verify compression format:**
   - Use Foxglove Studio or `mcap dump` to inspect messages
   - Confirm `CompressedImage.format` field contains compression identifier

3. **Validate decompression:**
   - Implement reader that decompresses based on `format` field
   - Compare with uncompressed depth images

## References

- [depthlitez repository](../middlewares/filters/depthlitez) - Depth compression algorithm library
- [MCAP specification](https://mcap.dev/spec) - MCAP file format documentation
- [ROS2 sensor messages](https://github.com/ros2/common_interfaces) - Message type definitions
- [CDR serialization specification](https://www.omg.org/spec/DDS/1.4/PDF) - ROS2 serialization format
- [ROS2 plugin implementation](middlewares/ros2/src/ros2_plugin/) - Plugin code
- [Configuration examples](apps/axon_recorder/config/default_config_ros2.yaml) - Sample configurations
