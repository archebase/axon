// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_MCAP_WRITER_WRAPPER_HPP
#define AXON_MCAP_WRITER_WRAPPER_HPP

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

// Forward declarations to avoid including mcap headers in this header
namespace mcap {
class McapWriter;
enum class Compression;
}  // namespace mcap

namespace axon {
namespace mcap_wrapper {

/**
 * Compression options for MCAP files
 */
enum class Compression { None, Zstd, Lz4 };

/**
 * Configuration options for MCAP writer
 */
struct McapWriterOptions {
  /// MCAP profile: "ros1" or "ros2"
  std::string profile = "ros2";

  /// Compression algorithm
  Compression compression = Compression::Zstd;

  /// Compression level (1-22 for Zstd, 1-12 for LZ4)
  int compression_level = 3;

  /// Chunk size in bytes (default 4MB)
  uint64_t chunk_size = 4 * 1024 * 1024;

  /// Whether to force flush after each message (slower but safer)
  bool force_flush = false;

  /// Enable statistics in footer
  bool enable_statistics = true;
};

/**
 * Thread-safe wrapper around MCAP writer
 *
 * This class provides a simplified interface for writing ROS messages to MCAP files.
 * It handles schema registration, channel management, and thread-safe message writing.
 *
 * Usage:
 *   McapWriterWrapper writer;
 *   writer.open("/path/to/output.mcap", options);
 *
 *   // Register schemas and channels
 *   uint16_t schema_id = writer.register_schema("sensor_msgs/msg/Image", "ros2msg", definition);
 *   uint16_t channel_id = writer.register_channel("/camera/image", "cdr", schema_id);
 *
 *   // Write messages (thread-safe)
 *   writer.write(channel_id, timestamp_ns, timestamp_ns, data, size);
 *
 *   writer.close();
 */
class McapWriterWrapper {
public:
  McapWriterWrapper();
  ~McapWriterWrapper();

  // Non-copyable, non-movable
  McapWriterWrapper(const McapWriterWrapper&) = delete;
  McapWriterWrapper& operator=(const McapWriterWrapper&) = delete;
  McapWriterWrapper(McapWriterWrapper&&) = delete;
  McapWriterWrapper& operator=(McapWriterWrapper&&) = delete;

  /**
   * Open an MCAP file for writing
   *
   * @param path Path to the output MCAP file
   * @param options Writer configuration options
   * @return true on success, false on failure (check get_last_error())
   */
  bool open(const std::string& path, const McapWriterOptions& options = McapWriterOptions{});

  /**
   * Close the MCAP file
   *
   * Flushes all pending data and writes the footer.
   * Must be called before the writer is destroyed.
   */
  void close();

  /**
   * Check if the writer is currently open
   */
  bool is_open() const;

  /**
   * Register a message schema
   *
   * @param name Full message type name (e.g., "sensor_msgs/msg/Image")
   * @param encoding Schema encoding ("ros1msg", "ros2msg", "ros2idl", "protobuf", etc.)
   * @param data Schema definition data (message definition string or binary)
   * @return Schema ID for use with register_channel(), or 0 on failure
   */
  uint16_t register_schema(
    const std::string& name, const std::string& encoding, const std::string& data
  );

  /**
   * Register a channel (topic)
   *
   * @param topic Topic name (e.g., "/camera/image_raw")
   * @param message_encoding Message encoding ("ros1", "cdr", "protobuf", "json", etc.)
   * @param schema_id Schema ID from register_schema()
   * @param metadata Optional channel metadata (e.g., QoS settings)
   * @return Channel ID for use with write(), or 0 on failure
   */
  uint16_t register_channel(
    const std::string& topic, const std::string& message_encoding, uint16_t schema_id,
    const std::unordered_map<std::string, std::string>& metadata = {}
  );

  /**
   * Write a message to the MCAP file
   *
   * This is the main hot path - optimized for high throughput.
   * Thread-safe: multiple threads can call this concurrently.
   *
   * @param channel_id Channel ID from register_channel()
   * @param log_time_ns Timestamp when message was logged (nanoseconds since epoch)
   * @param publish_time_ns Timestamp when message was published (nanoseconds since epoch)
   * @param data Pointer to serialized message data
   * @param data_size Size of serialized message data in bytes
   * @return true on success, false on failure
   */
  bool write(
    uint16_t channel_id, uint64_t log_time_ns, uint64_t publish_time_ns, const void* data,
    size_t data_size
  );

  /**
   * Write a message with sequence number
   *
   * @param channel_id Channel ID from register_channel()
   * @param sequence Message sequence number
   * @param log_time_ns Timestamp when message was logged (nanoseconds since epoch)
   * @param publish_time_ns Timestamp when message was published (nanoseconds since epoch)
   * @param data Pointer to serialized message data
   * @param data_size Size of serialized message data in bytes
   * @return true on success, false on failure
   */
  bool write(
    uint16_t channel_id, uint32_t sequence, uint64_t log_time_ns, uint64_t publish_time_ns,
    const void* data, size_t data_size
  );

  /**
   * Flush pending data to disk
   *
   * Forces a chunk boundary and writes pending data.
   */
  void flush();

  /**
   * Write a metadata record to the MCAP file
   *
   * Metadata records are used for file-level metadata that describes the entire recording
   * (e.g., task context, device info, recording statistics). This is NOT the hot path.
   * Thread-safe: uses mutex_ for synchronization.
   *
   * @param name Metadata record name (e.g., "axon.task", "axon.device", "axon.recording")
   * @param metadata Key-value pairs of metadata
   * @return true on success, false on failure (check get_last_error())
   */
  bool write_metadata(
    const std::string& name, const std::unordered_map<std::string, std::string>& metadata
  );

  /**
   * Get the last error message
   */
  std::string get_last_error() const;

  /**
   * Get statistics
   */
  struct Statistics {
    uint64_t messages_written;
    uint64_t bytes_written;
    uint64_t schemas_registered;
    uint64_t channels_registered;
  };
  Statistics get_statistics() const;

  /**
   * Get the output file path
   */
  std::string get_path() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mcap_wrapper
}  // namespace axon

#endif  // AXON_MCAP_WRITER_WRAPPER_HPP
