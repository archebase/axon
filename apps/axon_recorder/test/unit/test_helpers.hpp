/**
 * @file test_helpers.hpp
 * @brief Common test utilities and fixtures for axon_recorder tests
 */

#ifndef AXON_RECORDER_TEST_HELPERS_HPP
#define AXON_RECORDER_TEST_HELPERS_HPP

#include <chrono>
#include <filesystem>
#include <string>
#include <vector>

#include "../src/recorder_adapter.hpp"

namespace axon {
namespace recorder {
namespace test {

/**
 * RAII wrapper for temporary test directories.
 * Automatically creates a unique directory on construction and
 * removes it (with all contents) on destruction.
 */
class TempDirectory {
public:
  TempDirectory() {
    auto base = std::filesystem::temp_directory_path();
    auto timestamp = std::chrono::system_clock::now().time_since_epoch().count();
    path_ = base / ("axon_test_" + std::to_string(timestamp));
    std::filesystem::create_directories(path_);
  }

  ~TempDirectory() {
    if (!path_.empty() && std::filesystem::exists(path_)) {
      std::filesystem::remove_all(path_);
    }
  }

  // Non-copyable, movable
  TempDirectory(const TempDirectory&) = delete;
  TempDirectory& operator=(const TempDirectory&) = delete;
  TempDirectory(TempDirectory&& other) noexcept
      : path_(std::move(other.path_)) {
    other.path_.clear();
  }
  TempDirectory& operator=(TempDirectory&& other) noexcept {
    if (this != &other) {
      if (!path_.empty() && std::filesystem::exists(path_)) {
        std::filesystem::remove_all(path_);
      }
      path_ = std::move(other.path_);
      other.path_.clear();
    }
    return *this;
  }

  std::filesystem::path path() const {
    return path_;
  }
  std::string string() const {
    return path_.string();
  }

  /**
   * Create a subdirectory within the temp directory.
   */
  std::filesystem::path subdir(const std::string& name) const {
    auto sub = path_ / name;
    std::filesystem::create_directories(sub);
    return sub;
  }

  /**
   * Get path to a file within the temp directory.
   */
  std::filesystem::path file(const std::string& name) const {
    return path_ / name;
  }

private:
  std::filesystem::path path_;
};

/**
 * Create a standard TaskConfig for testing.
 */
inline TaskConfig make_test_task_config() {
  TaskConfig config;
  config.task_id = "test_task_123";
  config.device_id = "test_device_001";
  config.data_collector_id = "test_collector_001";
  config.order_id = "test_order_001";
  config.operator_name = "test.operator";
  config.scene = "indoor";
  config.subscene = "kitchen";
  config.skills = {"cooking", "navigation"};
  config.factory = "test_factory";
  config.topics = {"/camera/image", "/imu/data", "/odom"};
  config.start_callback_url = "http://localhost:8080/api/recording/start";
  config.finish_callback_url = "http://localhost:8080/api/recording/finish";
  config.user_token = "test_jwt_token";
  config.cached_at = std::chrono::system_clock::now();
  return config;
}

/**
 * Create a minimal TaskConfig with only required fields.
 */
inline TaskConfig make_minimal_task_config(const std::string& task_id = "minimal_task") {
  TaskConfig config;
  config.task_id = task_id;
  config.device_id = "device_001";
  config.topics = {"/test_topic"};
  config.cached_at = std::chrono::system_clock::now();
  return config;
}

/**
 * Create a standard RecorderConfig for testing.
 * @param output_dir Directory for MCAP output files
 */
inline RecorderConfig make_test_recorder_config(const std::string& output_dir) {
  RecorderConfig config;

  // Dataset config
  config.dataset.path = output_dir;
  config.dataset.mode = "create";

  // Recording config
  config.recording.max_disk_usage_gb = 10.0;

  // Topic config
  TopicConfig topic1;
  topic1.name = "/camera/image";
  topic1.message_type = "sensor_msgs/msg/Image";
  topic1.batch_size = 10;
  topic1.flush_interval_ms = 100;
  config.topics.push_back(topic1);

  TopicConfig topic2;
  topic2.name = "/imu/data";
  topic2.message_type = "sensor_msgs/msg/Imu";
  topic2.batch_size = 100;
  topic2.flush_interval_ms = 50;
  config.topics.push_back(topic2);

  // Logging config
  config.logging.console_enabled = false;
  config.logging.file_enabled = false;

  // Upload config (disabled for tests)
  config.upload.enabled = false;

  return config;
}

/**
 * Create a minimal RecorderConfig with just one topic.
 * @param output_dir Directory for MCAP output files
 * @param topic_name Topic name to record
 */
inline RecorderConfig make_minimal_recorder_config(
  const std::string& output_dir, const std::string& topic_name = "/test_topic"
) {
  RecorderConfig config;
  config.dataset.path = output_dir;
  config.dataset.mode = "create";
  config.recording.max_disk_usage_gb = 1.0;

  TopicConfig topic;
  topic.name = topic_name;
  topic.message_type = "std_msgs/msg/String";
  topic.batch_size = 10;
  topic.flush_interval_ms = 100;
  config.topics.push_back(topic);

  config.logging.console_enabled = false;
  config.logging.file_enabled = false;
  config.upload.enabled = false;

  return config;
}

/**
 * Generate a test message payload of specified size.
 * @param size Size in bytes
 * @return Vector of bytes filled with test pattern
 */
inline std::vector<uint8_t> make_test_message(size_t size) {
  std::vector<uint8_t> data(size);
  for (size_t i = 0; i < size; ++i) {
    data[i] = static_cast<uint8_t>(i & 0xFF);
  }
  return data;
}

/**
 * Get current time in nanoseconds since epoch.
 */
inline uint64_t now_ns() {
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                 std::chrono::system_clock::now().time_since_epoch()
  )
                                 .count());
}

}  // namespace test
}  // namespace recorder
}  // namespace axon

#endif  // AXON_RECORDER_TEST_HELPERS_HPP
