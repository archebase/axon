// Copyright (c) 2026 ArcheBase
// Axon is licensed under Mulan PSL v2.
// You can use this software according to the terms and conditions of the Mulan PSL v2.
// You may obtain a copy of Mulan PSL v2 at:
//          http://license.coscl.org.cn/MulanPSL2
// THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
// EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
// MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
// See the Mulan PSL v2 for more details.

#ifndef AXON_RECORDER_METADATA_INJECTOR_HPP
#define AXON_RECORDER_METADATA_INJECTOR_HPP

#include <chrono>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include "task_config.hpp"

// Forward declaration
namespace axon {
namespace mcap_wrapper {
class McapWriterWrapper;
}
}  // namespace axon

namespace axon {
namespace recorder {

/**
 * Statistics for a single topic
 */
struct TopicStats {
  std::string topic;
  std::string message_type;
  uint64_t message_count = 0;
  std::chrono::system_clock::time_point first_message_time;
  std::chrono::system_clock::time_point last_message_time;

  /**
   * Compute average frequency in Hz
   */
  double compute_frequency_hz() const;
};

/**
 * MetadataInjector handles:
 * - Building metadata maps for axon.task, axon.device, axon.recording
 * - Writing metadata records to MCAP via McapWriterWrapper
 * - Generating sidecar JSON files (always enabled)
 * - Computing SHA-256 checksum of MCAP files (always enabled)
 *
 * Usage:
 *   MetadataInjector injector;
 *   injector.set_task_config(config);
 *   injector.set_recording_start_time(start_time);
 *
 *   // During recording, track topic stats
 *   injector.update_topic_stats("/camera/image", "sensor_msgs/Image", msg_count);
 *
 *   // At finalization - BEFORE writer.close()
 *   injector.inject_metadata(writer, total_messages, file_size);
 *
 *   // AFTER writer.close() - compute checksum and generate sidecar
 *   injector.generate_sidecar_json(mcap_path, actual_file_size);
 */
class MetadataInjector {
public:
  /**
   * Optional callback for loading config values
   * Used for ROS parameter server integration
   */
  using ConfigLoaderCallback = std::function<std::string(const std::string& param_name)>;

  MetadataInjector();
  ~MetadataInjector();

  /**
   * Set the task configuration for this recording
   */
  void set_task_config(const TaskConfig& config);

  /**
   * Set the recording start time
   */
  void set_recording_start_time(std::chrono::system_clock::time_point time);

  /**
   * Set a callback for loading ROS parameters
   * Used as fallback when env vars and config file don't have values
   */
  void set_ros_param_loader(ConfigLoaderCallback callback);

  /**
   * Update statistics for a topic
   *
   * @param topic Topic name
   * @param message_type Message type string
   * @param count Total message count for this topic
   */
  void update_topic_stats(
    const std::string& topic, const std::string& message_type, uint64_t count
  );

  /**
   * Inject metadata records into MCAP file
   *
   * Must be called BEFORE writer.close() to ensure metadata is embedded.
   *
   * @param writer McapWriterWrapper to write metadata to
   * @param message_count Total messages recorded
   * @param file_size Current file size (before close)
   * @return true on success
   */
  bool inject_metadata(
    mcap_wrapper::McapWriterWrapper& writer, uint64_t message_count, uint64_t file_size
  );

  /**
   * Generate sidecar JSON file
   *
   * Must be called AFTER writer.close() to get accurate final file size.
   * Always computes SHA-256 checksum and generates sidecar.
   *
   * @param mcap_path Path to the MCAP file
   * @param actual_file_size Actual file size after close
   * @return true on success
   */
  bool generate_sidecar_json(const std::string& mcap_path, uint64_t actual_file_size);

  /**
   * Get the path to the generated sidecar JSON file
   * Empty if not yet generated
   */
  std::string get_sidecar_path() const;

  /**
   * Get the computed checksum (empty if not yet computed)
   */
  std::string get_checksum() const;

private:
  /**
   * Build task metadata map (axon.task)
   */
  std::unordered_map<std::string, std::string> build_task_metadata() const;

  /**
   * Build device metadata map (axon.device)
   */
  std::unordered_map<std::string, std::string> build_device_metadata() const;

  /**
   * Build recording metadata map (axon.recording)
   */
  std::unordered_map<std::string, std::string> build_recording_metadata(
    uint64_t message_count, uint64_t file_size
  ) const;

  /**
   * Get hostname via gethostname()
   */
  std::string get_hostname() const;

  /**
   * Get ROS distro from $ROS_DISTRO environment variable
   */
  std::string get_ros_distro() const;

  /**
   * Get device model using cascading config resolution:
   * 1. Environment variable: AXON_DEVICE_MODEL
   * 2. Config file (via loaded config)
   * 3. ROS parameter (via callback)
   */
  std::string get_device_model() const;

  /**
   * Get device serial using cascading config resolution:
   * 1. Environment variable: AXON_DEVICE_SERIAL
   * 2. Config file (via loaded config)
   * 3. ROS parameter (via callback)
   */
  std::string get_device_serial() const;

  /**
   * Resolve a config value using cascading priority
   */
  std::string resolve_config_value(
    const char* env_var, const std::string& config_value, const std::string& ros_param_name
  ) const;

  /**
   * Sanitize a field value (truncate to max length, strip control chars)
   */
  std::string sanitize_field(const std::string& value, size_t max_length) const;

  /**
   * Compute SHA-256 checksum of a file
   */
  std::string compute_sha256(const std::string& file_path) const;

  /**
   * Format a time point as ISO 8601 string
   */
  std::string format_iso8601(std::chrono::system_clock::time_point time) const;

  /**
   * Join a vector of strings with a delimiter
   */
  std::string join(const std::vector<std::string>& items, const std::string& delimiter) const;

  // =========================================================================
  // Member variables
  // =========================================================================

  TaskConfig task_config_;
  bool has_task_config_ = false;

  std::chrono::system_clock::time_point start_time_;
  std::chrono::system_clock::time_point finish_time_;

  std::unordered_map<std::string, TopicStats> topic_stats_;

  std::string sidecar_path_;
  std::string checksum_;

  // Optional callback for ROS parameter loading
  ConfigLoaderCallback ros_param_loader_;

  // Field length limits (from design doc)
  static constexpr size_t kMaxTaskIdLength = 256;
  static constexpr size_t kMaxSceneLength = 128;
  static constexpr size_t kMaxSkillLength = 64;
  static constexpr size_t kMaxFactoryLength = 128;
  static constexpr size_t kMaxOperatorNameLength = 128;
  static constexpr size_t kMaxIdLength = 256;
};

}  // namespace recorder
}  // namespace axon

#endif  // AXON_RECORDER_METADATA_INJECTOR_HPP
