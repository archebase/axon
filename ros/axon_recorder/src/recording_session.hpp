#ifndef AXON_RECORDER_RECORDING_SESSION_HPP
#define AXON_RECORDER_RECORDING_SESSION_HPP

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "mcap_writer_wrapper.hpp"
#include "metadata_injector.hpp"
#include "task_config.hpp"

namespace axon {
namespace recorder {

/**
 * RecordingSession encapsulates a single recording session's MCAP lifecycle.
 *
 * This class is responsible for:
 * - Opening and closing MCAP files
 * - Schema and channel registration
 * - Thread-safe message writing
 * - Session statistics tracking
 * - Duration calculation
 *
 * Extracted from RecorderNode to follow Single Responsibility Principle.
 * Each recording session is independent and can be created/destroyed
 * as recordings start and stop.
 *
 * Thread Safety:
 * - write() is thread-safe (multiple worker threads can write concurrently)
 * - open()/close() should be called from a single thread
 * - Statistics are atomic and can be read from any thread
 */
class RecordingSession {
public:
  /**
   * Session statistics for status reporting.
   */
  struct Stats {
    uint64_t messages_written = 0;
    uint64_t bytes_written = 0;
    uint64_t schemas_registered = 0;
    uint64_t channels_registered = 0;
    std::chrono::system_clock::time_point start_time;
  };

  RecordingSession();
  ~RecordingSession();

  // Non-copyable, non-movable
  RecordingSession(const RecordingSession&) = delete;
  RecordingSession& operator=(const RecordingSession&) = delete;
  RecordingSession(RecordingSession&&) = delete;
  RecordingSession& operator=(RecordingSession&&) = delete;

  /**
   * Open a new MCAP file for recording.
   *
   * @param path Output file path
   * @param options MCAP writer configuration
   * @return true on success, false on failure
   */
  bool open(const std::string& path,
            const mcap_wrapper::McapWriterOptions& options = mcap_wrapper::McapWriterOptions{});

  /**
   * Close the MCAP file.
   * Flushes pending data and writes the footer.
   */
  void close();

  /**
   * Check if the session is currently open.
   */
  bool is_open() const;

  /**
   * Flush pending data to disk.
   */
  void flush();

  /**
   * Register a message schema.
   *
   * @param name Full message type name (e.g., "sensor_msgs/msg/Image")
   * @param encoding Schema encoding ("ros1msg", "ros2msg", etc.)
   * @param definition Schema definition string
   * @return Schema ID for use with register_channel(), or 0 on failure
   */
  uint16_t register_schema(const std::string& name, const std::string& encoding,
                           const std::string& definition);

  /**
   * Register a channel (topic).
   *
   * @param topic Topic name (e.g., "/camera/image_raw")
   * @param message_encoding Message encoding ("ros1", "cdr", etc.)
   * @param schema_id Schema ID from register_schema()
   * @param metadata Optional channel metadata
   * @return Channel ID for use with write(), or 0 on failure
   */
  uint16_t register_channel(const std::string& topic, const std::string& message_encoding,
                            uint16_t schema_id,
                            const std::unordered_map<std::string, std::string>& metadata = {});

  /**
   * Write a message to the MCAP file.
   * Thread-safe: multiple threads can call this concurrently.
   *
   * @param channel_id Channel ID from register_channel()
   * @param sequence Message sequence number
   * @param log_time_ns Timestamp when message was logged (nanoseconds)
   * @param publish_time_ns Timestamp when message was published (nanoseconds)
   * @param data Pointer to serialized message data
   * @param data_size Size of serialized message data
   * @return true on success, false on failure
   */
  bool write(uint16_t channel_id, uint32_t sequence, uint64_t log_time_ns, uint64_t publish_time_ns,
             const uint8_t* data, size_t data_size);

  /**
   * Get the channel ID for a topic.
   *
   * @param topic Topic name
   * @return Channel ID, or 0 if not registered
   */
  uint16_t get_channel_id(const std::string& topic) const;

  /**
   * Get the schema ID for a message type.
   *
   * @param message_type Message type name
   * @return Schema ID, or 0 if not registered
   */
  uint16_t get_schema_id(const std::string& message_type) const;

  /**
   * Get current session statistics.
   */
  Stats get_stats() const;

  /**
   * Get session duration in seconds.
   * @return Duration since session opened, or 0.0 if not open
   */
  double get_duration_sec() const;

  /**
   * Get the last error message.
   */
  std::string get_last_error() const;

  /**
   * Get the output file path.
   */
  std::string get_path() const;

  /**
   * Set the task configuration for metadata injection.
   * Must be called before close() to enable metadata injection.
   *
   * @param config Task configuration
   */
  void set_task_config(const TaskConfig& config);

  /**
   * Get the path to the generated sidecar JSON file.
   * Empty if metadata injection was not enabled or not yet generated.
   */
  std::string get_sidecar_path() const;

  /**
   * Get the computed checksum of the MCAP file.
   * Empty if metadata injection was not enabled or not yet computed.
   */
  std::string get_checksum() const;

  /**
   * Update topic statistics for metadata.
   * Called by RecorderNode when messages are written.
   *
   * @param topic Topic name
   * @param message_type Message type string
   */
  void update_topic_stats(const std::string& topic, const std::string& message_type);

private:
  std::unique_ptr<mcap_wrapper::McapWriterWrapper> writer_;
  std::chrono::system_clock::time_point start_time_;
  std::string output_path_;

  // Schema and channel tracking
  mutable std::mutex registry_mutex_;
  std::unordered_map<std::string, uint16_t> topic_channel_ids_;
  std::unordered_map<std::string, uint16_t> message_type_schema_ids_;

  // Per-topic message counts for metadata
  mutable std::mutex topic_stats_mutex_;
  std::unordered_map<std::string, uint64_t> topic_message_counts_;
  std::unordered_map<std::string, std::string> topic_message_types_;

  // Metadata injection
  MetadataInjector metadata_injector_;
  bool has_task_config_ = false;

  // Statistics (atomic for thread-safe reads)
  std::atomic<uint64_t> messages_written_{0};
};

}  // namespace recorder
}  // namespace axon

#endif  // AXON_RECORDER_RECORDING_SESSION_HPP

