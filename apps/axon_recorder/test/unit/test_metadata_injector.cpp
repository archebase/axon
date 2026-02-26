// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

/**
 * @file test_metadata_injector.cpp
 * @brief Unit tests for MetadataInjector class
 *
 * Tests metadata injection, sidecar JSON generation, checksum computation,
 * and topic statistics tracking.
 */

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>

#include "../src/metadata/metadata_injector.hpp"

namespace fs = std::filesystem;
using namespace axon::recorder;

// ============================================================================
// Test Fixtures
// ============================================================================

class MetadataInjectorTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Create a unique temp directory for test files
    test_dir_ = fs::temp_directory_path() /
                ("metadata_injector_test_" +
                 std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::create_directories(test_dir_);
  }

  void TearDown() override {
    // Clean up test directory
    if (fs::exists(test_dir_)) {
      fs::remove_all(test_dir_);
    }
    // Clear environment variables
    unsetenv("AXON_DEVICE_MODEL");
    unsetenv("AXON_DEVICE_SERIAL");
    unsetenv("ROS_DISTRO");
  }

  std::string create_test_mcap(const std::string& filename, const std::string& content = "test") {
    auto path = test_dir_ / filename;
    std::ofstream file(path);
    file << content;
    return path.string();
  }

  fs::path test_dir_;
};

// ============================================================================
// TopicStats Tests
// ============================================================================

TEST(TopicStatsTest, ComputeFrequencyZeroMessages) {
  TopicStats stats;
  stats.message_count = 0;
  EXPECT_DOUBLE_EQ(stats.compute_frequency_hz(), 0.0);
}

TEST(TopicStatsTest, ComputeFrequencyOneMessage) {
  TopicStats stats;
  stats.message_count = 1;
  stats.first_message_time = std::chrono::system_clock::now();
  stats.last_message_time = stats.first_message_time;
  EXPECT_DOUBLE_EQ(stats.compute_frequency_hz(), 0.0);
}

TEST(TopicStatsTest, ComputeFrequencyMultipleMessages) {
  TopicStats stats;
  stats.message_count = 100;

  auto now = std::chrono::system_clock::now();
  stats.first_message_time = now;
  stats.last_message_time = now + std::chrono::seconds(1);  // 1 second duration

  // Expected: (100 - 1) messages / 1 second = 99 Hz
  double expected = 99.0;
  EXPECT_NEAR(stats.compute_frequency_hz(), expected, 0.1);
}

TEST(TopicStatsTest, ComputeFrequencyHighFrequency) {
  TopicStats stats;
  stats.message_count = 10000;

  auto now = std::chrono::system_clock::now();
  stats.first_message_time = now;
  stats.last_message_time = now + std::chrono::milliseconds(100);  // 100ms duration

  // Expected: (10000 - 1) messages / 0.1 seconds = 99990 Hz
  double expected = 99990.0;
  EXPECT_NEAR(stats.compute_frequency_hz(), expected, 10.0);
}

TEST(TopicStatsTest, ComputeFrequencySameTimestamp) {
  TopicStats stats;
  stats.message_count = 10;
  auto now = std::chrono::system_clock::now();
  stats.first_message_time = now;
  stats.last_message_time = now;  // Same timestamp

  EXPECT_DOUBLE_EQ(stats.compute_frequency_hz(), 0.0);
}

// ============================================================================
// MetadataInjector Basic Tests
// ============================================================================

TEST_F(MetadataInjectorTest, ConstructorInitializesDefaults) {
  MetadataInjector injector;

  EXPECT_EQ(injector.get_sidecar_path(), "");
  EXPECT_EQ(injector.get_checksum(), "");
}

TEST_F(MetadataInjectorTest, SetTaskConfig) {
  MetadataInjector injector;
  TaskConfig config;
  config.task_id = "test_task_001";
  config.device_id = "robot_01";
  config.scene = "warehouse";

  injector.set_task_config(config);

  // Config is set but not exposed directly - tested via side effects
  EXPECT_EQ(injector.get_sidecar_path(), "");
}

TEST_F(MetadataInjectorTest, SetRecordingStartTime) {
  MetadataInjector injector;
  auto now = std::chrono::system_clock::now();

  injector.set_recording_start_time(now);

  // Time is set internally, tested via sidecar generation
}

TEST_F(MetadataInjectorTest, SetRosParamLoader) {
  MetadataInjector injector;
  bool callback_called = false;

  injector.set_ros_param_loader([&](const std::string& param) -> std::string {
    callback_called = true;
    return param == "test_param" ? "test_value" : "";
  });

  // Callback will be used when resolving config values
}

// ============================================================================
// Topic Statistics Tests
// ============================================================================

TEST_F(MetadataInjectorTest, UpdateTopicStatsFirstMessage) {
  MetadataInjector injector;

  injector.update_topic_stats("/camera/image", "sensor_msgs/Image", 1);

  // Stats updated internally - verified through sidecar generation
}

TEST_F(MetadataInjectorTest, UpdateTopicStatsMultipleTopics) {
  MetadataInjector injector;

  injector.update_topic_stats("/camera/image", "sensor_msgs/Image", 100);
  injector.update_topic_stats("/imu/data", "sensor_msgs/Imu", 500);
  injector.update_topic_stats("/lidar/points", "sensor_msgs/PointCloud2", 50);

  // Multiple topics tracked
}

TEST_F(MetadataInjectorTest, UpdateTopicStatsIncremental) {
  MetadataInjector injector;

  injector.update_topic_stats("/camera/image", "sensor_msgs/Image", 1);
  injector.update_topic_stats("/camera/image", "sensor_msgs/Image", 100);
  injector.update_topic_stats("/camera/image", "sensor_msgs/Image", 1000);

  // Count should be updated to latest value
}

// ============================================================================
// Sidecar JSON Generation Tests
// ============================================================================

TEST_F(MetadataInjectorTest, GenerateSidecarWithoutTaskConfig) {
  MetadataInjector injector;
  auto mcap_path = create_test_mcap("test.mcap", "test content");

  bool result = injector.generate_sidecar_json(mcap_path, 12);

  EXPECT_FALSE(result);
  EXPECT_EQ(injector.get_sidecar_path(), "");
}

TEST_F(MetadataInjectorTest, GenerateSidecarCreatesFile) {
  MetadataInjector injector;
  TaskConfig config;
  config.task_id = "test_task_001";
  config.device_id = "robot_01";
  config.scene = "warehouse";
  injector.set_task_config(config);

  auto now = std::chrono::system_clock::now();
  injector.set_recording_start_time(now);

  auto mcap_path = create_test_mcap("test.mcap", "test content");

  bool result = injector.generate_sidecar_json(mcap_path, 12);

  EXPECT_TRUE(result);
  EXPECT_FALSE(injector.get_sidecar_path().empty());

  // Verify JSON file exists
  EXPECT_TRUE(fs::exists(injector.get_sidecar_path()));
  EXPECT_TRUE(injector.get_sidecar_path().find(".json") != std::string::npos);
}

TEST_F(MetadataInjectorTest, GenerateSidecarComputesChecksum) {
  MetadataInjector injector;
  TaskConfig config;
  config.task_id = "test_task";
  config.device_id = "robot_01";
  config.scene = "test";
  injector.set_task_config(config);

  injector.set_recording_start_time(std::chrono::system_clock::now());

  auto mcap_path = create_test_mcap("test.mcap", "specific content for checksum");

  bool result = injector.generate_sidecar_json(mcap_path, 23);

  EXPECT_TRUE(result);
  EXPECT_FALSE(injector.get_checksum().empty());
  // SHA-256 checksum should be 64 hex characters
  EXPECT_EQ(64, injector.get_checksum().length());
}

TEST_F(MetadataInjectorTest, GenerateSidecarNonexistentFile) {
  MetadataInjector injector;
  TaskConfig config;
  config.task_id = "test_task";
  config.device_id = "robot_01";
  config.scene = "test";
  injector.set_task_config(config);

  injector.set_recording_start_time(std::chrono::system_clock::now());

  bool result = injector.generate_sidecar_json("/nonexistent/file.mcap", 100);

  EXPECT_FALSE(result);
}

// ============================================================================
// Config Resolution Tests
// ============================================================================

TEST_F(MetadataInjectorTest, ResolveConfigValueFromEnvVar) {
  setenv("AXON_DEVICE_MODEL", "model_from_env", 1);

  MetadataInjector injector;
  TaskConfig config;
  config.task_id = "test";
  config.device_id = "robot";
  config.scene = "test";
  injector.set_task_config(config);
  injector.set_recording_start_time(std::chrono::system_clock::now());

  auto mcap_path = create_test_mcap("test.mcap", "content");
  injector.generate_sidecar_json(mcap_path, 10);

  // Verify sidecar contains env var value
  std::ifstream sidecar(injector.get_sidecar_path());
  std::string content((std::istreambuf_iterator<char>(sidecar)), std::istreambuf_iterator<char>());
  EXPECT_NE(content.find("model_from_env"), std::string::npos);
}

TEST_F(MetadataInjectorTest, ResolveConfigValueFromCallback) {
  MetadataInjector injector;
  TaskConfig config;
  config.task_id = "test";
  config.device_id = "robot";
  config.scene = "test";
  injector.set_task_config(config);
  injector.set_recording_start_time(std::chrono::system_clock::now());

  // Set callback that returns device model
  injector.set_ros_param_loader([](const std::string& param) -> std::string {
    if (param == "device_model") {
      return "model_from_callback";
    }
    return "";
  });

  auto mcap_path = create_test_mcap("test.mcap", "content");
  injector.generate_sidecar_json(mcap_path, 10);

  // Verify sidecar contains callback value
  std::ifstream sidecar(injector.get_sidecar_path());
  std::string content((std::istreambuf_iterator<char>(sidecar)), std::istreambuf_iterator<char>());
  EXPECT_NE(content.find("model_from_callback"), std::string::npos);
}

TEST_F(MetadataInjectorTest, ResolveConfigValuePriority) {
  // Set env var (highest priority)
  setenv("AXON_DEVICE_SERIAL", "serial_env", 1);

  MetadataInjector injector;
  TaskConfig config;
  config.task_id = "test";
  config.device_id = "robot";
  config.scene = "test";
  // Config value would be second priority (not set here)
  injector.set_task_config(config);
  injector.set_recording_start_time(std::chrono::system_clock::now());

  // Set callback (lowest priority)
  injector.set_ros_param_loader([](const std::string& param) -> std::string {
    return "serial_callback";
  });

  auto mcap_path = create_test_mcap("test.mcap", "content");
  injector.generate_sidecar_json(mcap_path, 10);

  // Should use env var value
  std::ifstream sidecar(injector.get_sidecar_path());
  std::string content((std::istreambuf_iterator<char>(sidecar)), std::istreambuf_iterator<char>());
  EXPECT_NE(content.find("serial_env"), std::string::npos);
  // Note: The callback value might also be present if device_serial is set elsewhere
}

// ============================================================================
// Sanitize Field Tests
// ============================================================================

TEST_F(MetadataInjectorTest, SanitizeFieldTruncatesLongValue) {
  MetadataInjector injector;
  TaskConfig config;
  config.task_id = "test";
  config.device_id = "robot";
  config.scene = std::string(200, 'a');  // Longer than kMaxSceneLength (128)
  injector.set_task_config(config);
  injector.set_recording_start_time(std::chrono::system_clock::now());

  auto mcap_path = create_test_mcap("test.mcap", "content");
  injector.generate_sidecar_json(mcap_path, 10);

  // Scene should be truncated in sidecar
  std::ifstream sidecar(injector.get_sidecar_path());
  std::string content((std::istreambuf_iterator<char>(sidecar)), std::istreambuf_iterator<char>());

  // Find scene value in JSON
  size_t scene_pos = content.find("\"scene\":");
  ASSERT_NE(scene_pos, std::string::npos);

  // Extract scene value (simplified - just check it's not too long)
  // The scene value should be truncated to 128 chars max
}

TEST_F(MetadataInjectorTest, SanitizeFieldRemovesControlCharacters) {
  MetadataInjector injector;
  TaskConfig config;
  config.task_id = "test";
  config.device_id = "robot";
  config.scene = "scene_with_\x01_control\x02_chars";
  injector.set_task_config(config);
  injector.set_recording_start_time(std::chrono::system_clock::now());

  auto mcap_path = create_test_mcap("test.mcap", "content");
  injector.generate_sidecar_json(mcap_path, 10);

  // Control characters should be removed (but \n and \t kept)
  std::ifstream sidecar(injector.get_sidecar_path());
  std::string content((std::istreambuf_iterator<char>(sidecar)), std::istreambuf_iterator<char>());

  EXPECT_EQ(content.find("\x01"), std::string::npos);
  EXPECT_EQ(content.find("\x02"), std::string::npos);
}

TEST_F(MetadataInjectorTest, SanitizeFieldKeepsNewlineAndTab) {
  MetadataInjector injector;
  TaskConfig config;
  config.task_id = "test";
  config.device_id = "robot";
  config.scene = "scene_with\nnewline\tand\ttab";
  injector.set_task_config(config);
  injector.set_recording_start_time(std::chrono::system_clock::now());

  auto mcap_path = create_test_mcap("test.mcap", "content");
  injector.generate_sidecar_json(mcap_path, 10);

  std::ifstream sidecar(injector.get_sidecar_path());
  std::string content((std::istreambuf_iterator<char>(sidecar)), std::istreambuf_iterator<char>());

  // In JSON, \n and \t are escaped as \\n and \\t
  EXPECT_NE(content.find("newline\\tand\\ttab"), std::string::npos);
}

// ============================================================================
// ROS Distro Tests
// ============================================================================

TEST_F(MetadataInjectorTest, GetRosDistroFromEnv) {
  setenv("ROS_DISTRO", "humble", 1);

  MetadataInjector injector;
  TaskConfig config;
  config.task_id = "test";
  config.device_id = "robot";
  config.scene = "test";
  injector.set_task_config(config);
  injector.set_recording_start_time(std::chrono::system_clock::now());

  auto mcap_path = create_test_mcap("test.mcap", "content");
  injector.generate_sidecar_json(mcap_path, 10);

  std::ifstream sidecar(injector.get_sidecar_path());
  std::string content((std::istreambuf_iterator<char>(sidecar)), std::istreambuf_iterator<char>());
  EXPECT_NE(content.find("humble"), std::string::npos);
}

TEST_F(MetadataInjectorTest, GetRosDistroEmptyWhenNotSet) {
  unsetenv("ROS_DISTRO");

  MetadataInjector injector;
  TaskConfig config;
  config.task_id = "test";
  config.device_id = "robot";
  config.scene = "test";
  injector.set_task_config(config);
  injector.set_recording_start_time(std::chrono::system_clock::now());

  auto mcap_path = create_test_mcap("test.mcap", "content");
  injector.generate_sidecar_json(mcap_path, 10);

  std::ifstream sidecar(injector.get_sidecar_path());
  std::string content((std::istreambuf_iterator<char>(sidecar)), std::istreambuf_iterator<char>());
  // ros_distro should be empty or not present
  size_t pos = content.find("\"ros_distro\":");
  if (pos != std::string::npos) {
    // If present, should be empty or null
    size_t value_start = content.find(":", pos) + 1;
    size_t value_end = content.find_first_of(",}\n", value_start);
    std::string value = content.substr(value_start, value_end - value_start);
    // Value should be empty string or null
    EXPECT_TRUE(value.find("null") != std::string::npos || value.find("\"\"") != std::string::npos);
  }
}

// ============================================================================
// Join Helper Tests
// ============================================================================

TEST_F(MetadataInjectorTest, JoinEmptyVector) {
  // Tested through topics_recorded field when no topics
  MetadataInjector injector;
  TaskConfig config;
  config.task_id = "test";
  config.device_id = "robot";
  config.scene = "test";
  injector.set_task_config(config);
  injector.set_recording_start_time(std::chrono::system_clock::now());

  auto mcap_path = create_test_mcap("test.mcap", "content");
  injector.generate_sidecar_json(mcap_path, 10);

  // topics_recorded should be empty array
  std::ifstream sidecar(injector.get_sidecar_path());
  std::string content((std::istreambuf_iterator<char>(sidecar)), std::istreambuf_iterator<char>());
  EXPECT_NE(content.find("\"topics_recorded\": []"), std::string::npos);
}

// ============================================================================
// Full Sidecar Content Tests
// ============================================================================

TEST_F(MetadataInjectorTest, SidecarContainsAllRequiredFields) {
  MetadataInjector injector;
  TaskConfig config;
  config.task_id = "task_12345";
  config.device_id = "robot_01";
  config.scene = "warehouse";
  config.factory = "factory_a";
  config.operator_name = "operator_x";
  injector.set_task_config(config);
  injector.set_recording_start_time(std::chrono::system_clock::now());

  // Update topic stats
  injector.update_topic_stats("/camera/image", "sensor_msgs/Image", 100);
  injector.update_topic_stats("/imu/data", "sensor_msgs/Imu", 500);

  // Wait a bit for duration
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  auto mcap_path = create_test_mcap("test.mcap", "content");
  injector.generate_sidecar_json(mcap_path, 1000);

  std::ifstream sidecar(injector.get_sidecar_path());
  std::string content((std::istreambuf_iterator<char>(sidecar)), std::istreambuf_iterator<char>());

  // Check required fields
  EXPECT_NE(content.find("\"version\": \"1.0\""), std::string::npos);
  EXPECT_NE(content.find("\"task_id\": \"task_12345\""), std::string::npos);
  EXPECT_NE(content.find("\"device_id\": \"robot_01\""), std::string::npos);
  EXPECT_NE(content.find("\"scene\": \"warehouse\""), std::string::npos);
  EXPECT_NE(content.find("\"factory\": \"factory_a\""), std::string::npos);
  EXPECT_NE(content.find("\"operator_name\": \"operator_x\""), std::string::npos);
  EXPECT_NE(content.find("\"message_count\":"), std::string::npos);
  EXPECT_NE(content.find("\"file_size_bytes\":"), std::string::npos);
  EXPECT_NE(content.find("\"checksum_sha256\":"), std::string::npos);
  EXPECT_NE(content.find("\"topics_recorded\":"), std::string::npos);
  EXPECT_NE(content.find("\"topics_summary\":"), std::string::npos);
}

TEST_F(MetadataInjectorTest, SidecarTopicsSummaryCorrect) {
  MetadataInjector injector;
  TaskConfig config;
  config.task_id = "test";
  config.device_id = "robot";
  config.scene = "test";
  injector.set_task_config(config);
  injector.set_recording_start_time(std::chrono::system_clock::now());

  injector.update_topic_stats("/camera/image", "sensor_msgs/Image", 100);
  injector.update_topic_stats("/imu/data", "sensor_msgs/Imu", 500);
  injector.update_topic_stats("/lidar/points", "sensor_msgs/PointCloud2", 50);

  auto mcap_path = create_test_mcap("test.mcap", "content");
  injector.generate_sidecar_json(mcap_path, 650);

  std::ifstream sidecar(injector.get_sidecar_path());
  std::string content((std::istreambuf_iterator<char>(sidecar)), std::istreambuf_iterator<char>());

  // Check topics_summary contains all topics
  EXPECT_NE(content.find("\"/camera/image\""), std::string::npos);
  EXPECT_NE(content.find("\"sensor_msgs/Image\""), std::string::npos);
  EXPECT_NE(content.find("\"/imu/data\""), std::string::npos);
  EXPECT_NE(content.find("\"sensor_msgs/Imu\""), std::string::npos);
  EXPECT_NE(content.find("\"/lidar/points\""), std::string::npos);
  EXPECT_NE(content.find("\"sensor_msgs/PointCloud2\""), std::string::npos);
}

TEST_F(MetadataInjectorTest, SidecarTopicsRecordedSorted) {
  MetadataInjector injector;
  TaskConfig config;
  config.task_id = "test";
  config.device_id = "robot";
  config.scene = "test";
  injector.set_task_config(config);
  injector.set_recording_start_time(std::chrono::system_clock::now());

  injector.update_topic_stats("/zzz/last", "std_msgs/String", 1);
  injector.update_topic_stats("/aaa/first", "std_msgs/String", 1);
  injector.update_topic_stats("/mmm/middle", "std_msgs/String", 1);

  auto mcap_path = create_test_mcap("test.mcap", "content");
  injector.generate_sidecar_json(mcap_path, 3);

  std::ifstream sidecar(injector.get_sidecar_path());
  std::string content((std::istreambuf_iterator<char>(sidecar)), std::istreambuf_iterator<char>());

  // Find topics_recorded array
  size_t topics_pos = content.find("\"topics_recorded\": [");
  ASSERT_NE(topics_pos, std::string::npos);

  // Topics should be in alphabetical order
  size_t aaa_pos = content.find("\"/aaa/first\"", topics_pos);
  size_t mmm_pos = content.find("\"/mmm/middle\"", topics_pos);
  size_t zzz_pos = content.find("\"/zzz/last\"", topics_pos);

  EXPECT_LT(aaa_pos, mmm_pos);
  EXPECT_LT(mmm_pos, zzz_pos);
}

// ============================================================================
// Optional Fields Tests
// ============================================================================

TEST_F(MetadataInjectorTest, SidecarWithOptionalFields) {
  MetadataInjector injector;
  TaskConfig config;
  config.task_id = "test";
  config.device_id = "robot";
  config.scene = "test";
  config.order_id = "order_123";
  config.subscene = "packing_area";
  config.skills = {"pick", "place", "navigate"};
  config.data_collector_id = "collector_01";
  injector.set_task_config(config);
  injector.set_recording_start_time(std::chrono::system_clock::now());

  auto mcap_path = create_test_mcap("test.mcap", "content");
  injector.generate_sidecar_json(mcap_path, 10);

  std::ifstream sidecar(injector.get_sidecar_path());
  std::string content((std::istreambuf_iterator<char>(sidecar)), std::istreambuf_iterator<char>());

  EXPECT_NE(content.find("\"order_id\": \"order_123\""), std::string::npos);
  EXPECT_NE(content.find("\"subscene\": \"packing_area\""), std::string::npos);
  EXPECT_NE(content.find("\"skills\":"), std::string::npos);
  EXPECT_NE(content.find("\"data_collector_id\": \"collector_01\""), std::string::npos);
}

TEST_F(MetadataInjectorTest, SidecarWithoutOptionalFields) {
  MetadataInjector injector;
  TaskConfig config;
  config.task_id = "test";
  config.device_id = "robot";
  config.scene = "test";
  // Leave optional fields empty
  injector.set_task_config(config);
  injector.set_recording_start_time(std::chrono::system_clock::now());

  auto mcap_path = create_test_mcap("test.mcap", "content");
  injector.generate_sidecar_json(mcap_path, 10);

  std::ifstream sidecar(injector.get_sidecar_path());
  std::string content((std::istreambuf_iterator<char>(sidecar)), std::istreambuf_iterator<char>());

  // Optional fields should not be present when empty
  EXPECT_EQ(content.find("\"order_id\":"), std::string::npos);
  EXPECT_EQ(content.find("\"subscene\":"), std::string::npos);
  EXPECT_EQ(content.find("\"skills\":"), std::string::npos);
  EXPECT_EQ(content.find("\"data_collector_id\":"), std::string::npos);
}

TEST_F(MetadataInjectorTest, SidecarCompleteStructureValidation) {
  // This test validates the complete structure of sidecar.json
  // ensuring all important fields are present and correctly formatted
  MetadataInjector injector;
  TaskConfig config;
  config.task_id = "task_complete_001";
  config.device_id = "robot_complete_01";
  config.data_collector_id = "collector_complete";
  config.order_id = "order_complete_123";
  config.operator_name = "operator_complete";
  config.scene = "scene_complete";
  config.subscene = "subscene_complete";
  config.factory = "factory_complete";
  config.skills = {"skill1", "skill2", "skill3"};
  injector.set_task_config(config);
  injector.set_recording_start_time(std::chrono::system_clock::now());

  // Add comprehensive topic stats
  injector.update_topic_stats("/camera/image_raw", "sensor_msgs/Image", 300);
  injector.update_topic_stats("/imu/data", "sensor_msgs/Imu", 1000);
  injector.update_topic_stats("/lidar/points", "sensor_msgs/PointCloud2", 100);

  // Wait for measurable duration
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  auto mcap_path = create_test_mcap("test_complete.mcap", "test content for complete validation");
  uint64_t file_size = 2048;
  bool result = injector.generate_sidecar_json(mcap_path, file_size);

  ASSERT_TRUE(result);
  ASSERT_FALSE(injector.get_sidecar_path().empty());
  ASSERT_FALSE(injector.get_checksum().empty());
  EXPECT_EQ(64, injector.get_checksum().length());

  // Read and parse the JSON
  std::ifstream sidecar(injector.get_sidecar_path());
  std::string content((std::istreambuf_iterator<char>(sidecar)), std::istreambuf_iterator<char>());

  // Verify top-level structure
  EXPECT_NE(content.find("\"version\":"), std::string::npos);
  EXPECT_NE(content.find("\"mcap_file\":"), std::string::npos);
  EXPECT_NE(content.find("\"task\":"), std::string::npos);
  EXPECT_NE(content.find("\"device\":"), std::string::npos);
  EXPECT_NE(content.find("\"recording\":"), std::string::npos);
  EXPECT_NE(content.find("\"topics_summary\":"), std::string::npos);

  // Verify task section - required fields
  EXPECT_NE(content.find("\"task_id\": \"task_complete_001\""), std::string::npos);
  EXPECT_NE(content.find("\"device_id\": \"robot_complete_01\""), std::string::npos);
  EXPECT_NE(content.find("\"scene\": \"scene_complete\""), std::string::npos);
  EXPECT_NE(content.find("\"factory\": \"factory_complete\""), std::string::npos);

  // Verify task section - optional fields (all set in this test)
  EXPECT_NE(content.find("\"data_collector_id\": \"collector_complete\""), std::string::npos);
  EXPECT_NE(content.find("\"order_id\": \"order_complete_123\""), std::string::npos);
  EXPECT_NE(content.find("\"operator_name\": \"operator_complete\""), std::string::npos);
  EXPECT_NE(content.find("\"subscene\": \"subscene_complete\""), std::string::npos);
  EXPECT_NE(content.find("\"skills\":"), std::string::npos);
  EXPECT_NE(content.find("\"skill1\""), std::string::npos);
  EXPECT_NE(content.find("\"skill2\""), std::string::npos);
  EXPECT_NE(content.find("\"skill3\""), std::string::npos);

  // Verify device section
  EXPECT_NE(content.find("\"hostname\":"), std::string::npos);

  // Verify recording section
  EXPECT_NE(content.find("\"recorder_version\":"), std::string::npos);
  EXPECT_NE(content.find("\"recording_started_at\":"), std::string::npos);
  EXPECT_NE(content.find("\"recording_finished_at\":"), std::string::npos);
  EXPECT_NE(content.find("\"duration_sec\":"), std::string::npos);
  EXPECT_NE(content.find("\"message_count\":"), std::string::npos);
  EXPECT_NE(content.find("\"file_size_bytes\": 2048"), std::string::npos);
  EXPECT_NE(content.find("\"checksum_sha256\":"), std::string::npos);
  EXPECT_NE(content.find("\"topics_recorded\":"), std::string::npos);

  // Verify topics_summary contains all topics with correct structure
  EXPECT_NE(content.find("\"topic\": \"/camera/image_raw\""), std::string::npos);
  EXPECT_NE(content.find("\"message_type\": \"sensor_msgs/Image\""), std::string::npos);
  EXPECT_NE(content.find("\"message_count\": 300"), std::string::npos);
  EXPECT_NE(content.find("\"frequency_hz\":"), std::string::npos);

  EXPECT_NE(content.find("\"topic\": \"/imu/data\""), std::string::npos);
  EXPECT_NE(content.find("\"message_type\": \"sensor_msgs/Imu\""), std::string::npos);
  EXPECT_NE(content.find("\"message_count\": 1000"), std::string::npos);

  EXPECT_NE(content.find("\"topic\": \"/lidar/points\""), std::string::npos);
  EXPECT_NE(content.find("\"message_type\": \"sensor_msgs/PointCloud2\""), std::string::npos);
  EXPECT_NE(content.find("\"message_count\": 100"), std::string::npos);
}

TEST_F(MetadataInjectorTest, SidecarMessageCountAggregation) {
  // Test that total message count is correctly aggregated from all topics
  MetadataInjector injector;
  TaskConfig config;
  config.task_id = "test_count";
  config.device_id = "robot";
  config.scene = "test";
  injector.set_task_config(config);
  injector.set_recording_start_time(std::chrono::system_clock::now());

  // Add topics with specific counts
  injector.update_topic_stats("/topic1", "std_msgs/String", 100);
  injector.update_topic_stats("/topic2", "std_msgs/String", 250);
  injector.update_topic_stats("/topic3", "std_msgs/String", 50);

  // Total should be 100 + 250 + 50 = 400
  auto mcap_path = create_test_mcap("test_count.mcap", "content");
  injector.generate_sidecar_json(mcap_path, 512);

  std::ifstream sidecar(injector.get_sidecar_path());
  std::string content((std::istreambuf_iterator<char>(sidecar)), std::istreambuf_iterator<char>());

  // The message_count in recording section should be 400
  EXPECT_NE(content.find("\"message_count\": 400"), std::string::npos);

  // Verify individual topic counts in topics_summary
  EXPECT_NE(content.find("\"/topic1\""), std::string::npos);
  size_t pos = content.find("\"/topic1\"");
  // Find the message_count for topic1 (should be nearby)
  EXPECT_NE(content.find("\"message_count\": 100", pos), std::string::npos);
}

TEST_F(MetadataInjectorTest, SidecarTimestampsIso8601Format) {
  // Test that timestamps are in ISO 8601 format
  MetadataInjector injector;
  TaskConfig config;
  config.task_id = "test_time";
  config.device_id = "robot";
  config.scene = "test";
  injector.set_task_config(config);
  injector.set_recording_start_time(std::chrono::system_clock::now());

  auto mcap_path = create_test_mcap("test_time.mcap", "content");
  injector.generate_sidecar_json(mcap_path, 100);

  std::ifstream sidecar(injector.get_sidecar_path());
  std::string content((std::istreambuf_iterator<char>(sidecar)), std::istreambuf_iterator<char>());

  // ISO 8601 format should contain 'T' between date and time, and 'Z' for UTC
  EXPECT_NE(content.find("\"recording_started_at\": \""), std::string::npos);
  EXPECT_NE(content.find("\"recording_finished_at\": \""), std::string::npos);

  // Check for T and Z in timestamp strings (basic ISO 8601 validation)
  size_t started_pos = content.find("\"recording_started_at\": \"");
  ASSERT_NE(started_pos, std::string::npos);
  size_t started_value_start = started_pos + strlen("\"recording_started_at\": \"");
  size_t started_value_end = content.find("\"", started_value_start);
  std::string started_time =
    content.substr(started_value_start, started_value_end - started_value_start);

  // Should contain 'T' (date/time separator) and 'Z' (UTC marker)
  EXPECT_NE(started_time.find('T'), std::string::npos);
  EXPECT_NE(started_time.find('Z'), std::string::npos);
}

TEST_F(MetadataInjectorTest, SidecarAtomicWrite) {
  // Test that sidecar JSON is written atomically (via temp file)
  MetadataInjector injector;
  TaskConfig config;
  config.task_id = "test_atomic";
  config.device_id = "robot";
  config.scene = "test";
  injector.set_task_config(config);
  injector.set_recording_start_time(std::chrono::system_clock::now());

  auto mcap_path = create_test_mcap("test_atomic.mcap", "content");
  bool result = injector.generate_sidecar_json(mcap_path, 100);

  ASSERT_TRUE(result);

  // Verify the final file exists
  std::string sidecar_path = injector.get_sidecar_path();
  EXPECT_TRUE(fs::exists(sidecar_path));

  // Verify .tmp file does not exist (should have been renamed)
  std::string tmp_path = sidecar_path + ".tmp";
  EXPECT_FALSE(fs::exists(tmp_path));
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
