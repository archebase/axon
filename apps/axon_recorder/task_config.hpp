// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_RECORDER_TASK_CONFIG_HPP
#define AXON_RECORDER_TASK_CONFIG_HPP

#include <chrono>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace axon {
namespace recorder {

/**
 * TaskConfig holds all metadata for a recording task pushed from the server.
 * This is cached when CachedRecordingConfig service is called and used
 * throughout the recording lifecycle.
 */
struct TaskConfig {
  // Task & Device Identification
  std::string task_id;
  std::string device_id;
  std::string data_collector_id;
  std::string order_id;       // Parent order/job identifier (optional)
  std::string operator_name;  // Human operator identifier (optional)

  // Recording Context
  std::string scene;
  std::string subscene;
  std::vector<std::string> skills;
  std::string factory;  // Factory identifier (which factory produced this data)

  // Topic Selection
  std::vector<std::string> topics;

  // Server Callback Integration
  std::string start_callback_url;
  std::string finish_callback_url;
  std::string user_token;  // JWT token for callback authentication

  // Metadata timestamps
  std::chrono::system_clock::time_point cached_at;

  TaskConfig() = default;

  /**
   * Check if server callbacks are configured
   */
  bool has_callbacks() const {
    return !start_callback_url.empty() || !finish_callback_url.empty();
  }

  /**
   * Check if the config is valid (has required fields)
   */
  bool is_valid() const {
    return !task_id.empty();
  }

  /**
   * Generate output filename based on task_id
   * Format: {task_id}.mcap
   */
  std::string generate_output_filename() const {
    return task_id + ".mcap";
  }
};

/**
 * TaskConfigCache provides thread-safe caching of task configuration.
 * Only one configuration can be cached at a time (single task model).
 */
class TaskConfigCache {
public:
  TaskConfigCache() = default;

  // Non-copyable
  TaskConfigCache(const TaskConfigCache&) = delete;
  TaskConfigCache& operator=(const TaskConfigCache&) = delete;

  /**
   * Cache a new task configuration.
   * Replaces any existing cached configuration.
   *
   * @param config The task configuration to cache
   */
  void cache(TaskConfig config) {
    std::lock_guard<std::mutex> lock(mutex_);
    config.cached_at = std::chrono::system_clock::now();
    cached_config_ = std::move(config);
  }

  /**
   * Get the cached configuration.
   *
   * @return The cached configuration, or std::nullopt if none cached
   */
  std::optional<TaskConfig> get() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return cached_config_;
  }

  /**
   * Check if a configuration is cached.
   *
   * @return true if a configuration is cached
   */
  bool has_config() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return cached_config_.has_value();
  }

  /**
   * Get the task_id of the cached configuration.
   *
   * @return The task_id, or empty string if none cached
   */
  std::string get_task_id() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return cached_config_ ? cached_config_->task_id : "";
  }

  /**
   * Clear the cached configuration.
   * Used when recording finishes or is cancelled.
   */
  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    cached_config_.reset();
  }

  /**
   * Check if the cached config matches a given task_id.
   *
   * @param task_id The task_id to check
   * @return true if cached config exists and matches the task_id
   */
  bool matches_task_id(const std::string& task_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return cached_config_ && cached_config_->task_id == task_id;
  }

private:
  mutable std::mutex mutex_;
  std::optional<TaskConfig> cached_config_;
};

}  // namespace recorder
}  // namespace axon

#endif  // AXON_RECORDER_TASK_CONFIG_HPP
