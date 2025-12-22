/**
 * @file test_config_parser.cpp
 * @brief Unit tests for ConfigParser and RecorderConfig
 *
 * Tests YAML parsing, validation, and edge case handling for the
 * configuration system.
 */

#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#include "config_parser.hpp"

namespace fs = std::filesystem;

using namespace axon::core;

// ============================================================================
// Test Fixtures
// ============================================================================

class ConfigParserTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Create a unique temp directory for test files
    test_dir_ = fs::temp_directory_path() / ("config_test_" +
        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::create_directories(test_dir_);
  }

  void TearDown() override {
    // Clean up test directory
    if (fs::exists(test_dir_)) {
      fs::remove_all(test_dir_);
    }
  }

  std::string write_test_file(const std::string& filename, const std::string& content) {
    auto path = test_dir_ / filename;
    std::ofstream file(path);
    file << content;
    return path.string();
  }

  fs::path test_dir_;
};

// ============================================================================
// Valid YAML Parsing Tests
// ============================================================================

TEST_F(ConfigParserTest, ParseValidMinimalConfig) {
  const std::string yaml = R"(
dataset:
  path: /data/recordings
  mode: create
topics:
  - name: /camera/image
    message_type: sensor_msgs/Image
)";

  RecorderConfig config = RecorderConfig::from_yaml_string(yaml);
  EXPECT_EQ(config.dataset.path, "/data/recordings");
  EXPECT_EQ(config.dataset.mode, "create");
  ASSERT_EQ(config.topics.size(), 1);
  EXPECT_EQ(config.topics[0].name, "/camera/image");
  EXPECT_EQ(config.topics[0].message_type, "sensor_msgs/Image");
}

TEST_F(ConfigParserTest, ParseValidFullConfig) {
  const std::string yaml = R"(
dataset:
  path: /data/recordings
  mode: append
topics:
  - name: /camera/image
    message_type: sensor_msgs/Image
    batch_size: 50
    flush_interval_ms: 500
  - name: /imu/data
    message_type: sensor_msgs/Imu
    batch_size: 200
    flush_interval_ms: 100
recording:
  max_disk_usage_gb: 50.0
)";

  RecorderConfig config = RecorderConfig::from_yaml_string(yaml);
  
  EXPECT_EQ(config.dataset.path, "/data/recordings");
  EXPECT_EQ(config.dataset.mode, "append");
  
  ASSERT_EQ(config.topics.size(), 2);
  EXPECT_EQ(config.topics[0].name, "/camera/image");
  EXPECT_EQ(config.topics[0].batch_size, 50);
  EXPECT_EQ(config.topics[0].flush_interval_ms, 500);
  EXPECT_EQ(config.topics[1].name, "/imu/data");
  EXPECT_EQ(config.topics[1].batch_size, 200);
  
  EXPECT_DOUBLE_EQ(config.recording.max_disk_usage_gb, 50.0);
}

TEST_F(ConfigParserTest, ParseLoggingConfig) {
  const std::string yaml = R"(
dataset:
  path: /data
  mode: create
topics:
  - name: /test
    message_type: std_msgs/String
logging:
  console:
    enabled: true
    colors: false
    level: debug
  file:
    enabled: true
    level: info
    directory: /var/log/axon
    pattern: recorder_%Y%m%d.log
    format: json
    rotation_size_mb: 50
    max_files: 5
    rotate_at_midnight: false
)";

  RecorderConfig config = RecorderConfig::from_yaml_string(yaml);
  
  EXPECT_TRUE(config.logging.console_enabled);
  EXPECT_FALSE(config.logging.console_colors);
  EXPECT_EQ(config.logging.console_level, "debug");
  
  EXPECT_TRUE(config.logging.file_enabled);
  EXPECT_EQ(config.logging.file_level, "info");
  EXPECT_EQ(config.logging.file_directory, "/var/log/axon");
  EXPECT_EQ(config.logging.file_pattern, "recorder_%Y%m%d.log");
  EXPECT_EQ(config.logging.file_format, "json");
  EXPECT_EQ(config.logging.rotation_size_mb, 50);
  EXPECT_EQ(config.logging.max_files, 5);
  EXPECT_FALSE(config.logging.rotate_at_midnight);
}

TEST_F(ConfigParserTest, ParseUploadConfig) {
  const std::string yaml = R"(
dataset:
  path: /data
  mode: create
topics:
  - name: /test
    message_type: std_msgs/String
upload:
  enabled: true
  s3:
    endpoint_url: http://minio:9000
    bucket: test-bucket
    region: us-west-2
    use_ssl: false
    verify_ssl: false
  retry:
    max_retries: 10
    initial_delay_ms: 2000
    max_delay_ms: 600000
    exponential_base: 1.5
    jitter: false
  num_workers: 4
  state_db_path: /var/lib/axon/state.db
  delete_after_upload: false
  failed_uploads_dir: /data/failed
  warn_pending_gb: 10.0
  alert_pending_gb: 25.0
)";

  RecorderConfig config = RecorderConfig::from_yaml_string(yaml);
  
  EXPECT_TRUE(config.upload.enabled);
  
  // S3 config
  EXPECT_EQ(config.upload.s3.endpoint_url, "http://minio:9000");
  EXPECT_EQ(config.upload.s3.bucket, "test-bucket");
  EXPECT_EQ(config.upload.s3.region, "us-west-2");
  EXPECT_FALSE(config.upload.s3.use_ssl);
  EXPECT_FALSE(config.upload.s3.verify_ssl);
  
  // Retry config
  EXPECT_EQ(config.upload.retry.max_retries, 10);
  EXPECT_EQ(config.upload.retry.initial_delay_ms, 2000);
  EXPECT_EQ(config.upload.retry.max_delay_ms, 600000);
  EXPECT_DOUBLE_EQ(config.upload.retry.exponential_base, 1.5);
  EXPECT_FALSE(config.upload.retry.jitter);
  
  // Other upload settings
  EXPECT_EQ(config.upload.num_workers, 4);
  EXPECT_EQ(config.upload.state_db_path, "/var/lib/axon/state.db");
  EXPECT_FALSE(config.upload.delete_after_upload);
  EXPECT_EQ(config.upload.failed_uploads_dir, "/data/failed");
  EXPECT_DOUBLE_EQ(config.upload.warn_pending_gb, 10.0);
  EXPECT_DOUBLE_EQ(config.upload.alert_pending_gb, 25.0);
}

// ============================================================================
// Default Values Tests
// ============================================================================

TEST_F(ConfigParserTest, DefaultValuesForOptionalFields) {
  const std::string yaml = R"(
dataset:
  path: /data
topics:
  - name: /test
    message_type: std_msgs/String
)";

  RecorderConfig config = RecorderConfig::from_yaml_string(yaml);
  
  // Dataset defaults
  EXPECT_EQ(config.dataset.mode, "append");  // Default mode
  
  // Topic defaults
  EXPECT_EQ(config.topics[0].batch_size, 100);  // Default batch size
  EXPECT_EQ(config.topics[0].flush_interval_ms, 1000);  // Default flush interval
  
  // Recording defaults
  EXPECT_DOUBLE_EQ(config.recording.max_disk_usage_gb, 100.0);
  
  // Logging defaults
  EXPECT_TRUE(config.logging.console_enabled);
  EXPECT_TRUE(config.logging.console_colors);
  EXPECT_EQ(config.logging.console_level, "info");
  EXPECT_FALSE(config.logging.file_enabled);
  
  // Upload defaults
  EXPECT_FALSE(config.upload.enabled);
  EXPECT_EQ(config.upload.s3.bucket, "axon-raw-data");
  EXPECT_EQ(config.upload.s3.region, "us-east-1");
  EXPECT_TRUE(config.upload.s3.use_ssl);
  EXPECT_TRUE(config.upload.s3.verify_ssl);
  EXPECT_EQ(config.upload.retry.max_retries, 5);
  EXPECT_EQ(config.upload.num_workers, 2);
  EXPECT_TRUE(config.upload.delete_after_upload);
}

// ============================================================================
// Validation Tests
// ============================================================================

TEST_F(ConfigParserTest, ValidationFailsWithEmptyDatasetPath) {
  const std::string yaml = R"(
dataset:
  path: ""
  mode: create
topics:
  - name: /test
    message_type: std_msgs/String
)";

  RecorderConfig config = RecorderConfig::from_yaml_string(yaml);
  EXPECT_FALSE(config.validate());
}

TEST_F(ConfigParserTest, ValidationFailsWithNoTopics) {
  const std::string yaml = R"(
dataset:
  path: /data
  mode: create
topics: []
)";

  RecorderConfig config = RecorderConfig::from_yaml_string(yaml);
  EXPECT_FALSE(config.validate());
}

TEST_F(ConfigParserTest, ValidationFailsWithEmptyTopicName) {
  const std::string yaml = R"(
dataset:
  path: /data
  mode: create
topics:
  - name: ""
    message_type: std_msgs/String
)";

  RecorderConfig config = RecorderConfig::from_yaml_string(yaml);
  EXPECT_FALSE(config.validate());
}

TEST_F(ConfigParserTest, ValidationFailsWithEmptyMessageType) {
  const std::string yaml = R"(
dataset:
  path: /data
  mode: create
topics:
  - name: /test
    message_type: ""
)";

  RecorderConfig config = RecorderConfig::from_yaml_string(yaml);
  EXPECT_FALSE(config.validate());
}

TEST_F(ConfigParserTest, ValidationFailsWithZeroBatchSize) {
  const std::string yaml = R"(
dataset:
  path: /data
  mode: create
topics:
  - name: /test
    message_type: std_msgs/String
    batch_size: 0
)";

  RecorderConfig config = RecorderConfig::from_yaml_string(yaml);
  EXPECT_FALSE(config.validate());
}

TEST_F(ConfigParserTest, ValidationFailsWithInvalidMode) {
  const std::string yaml = R"(
dataset:
  path: /data
  mode: invalid
topics:
  - name: /test
    message_type: std_msgs/String
)";

  RecorderConfig config = RecorderConfig::from_yaml_string(yaml);
  EXPECT_FALSE(config.validate());
}

TEST_F(ConfigParserTest, ValidationPassesWithValidConfig) {
  const std::string yaml = R"(
dataset:
  path: /data/recordings
  mode: create
topics:
  - name: /camera/image
    message_type: sensor_msgs/Image
    batch_size: 100
)";

  RecorderConfig config = RecorderConfig::from_yaml_string(yaml);
  EXPECT_TRUE(config.validate());
}

TEST_F(ConfigParserTest, StaticValidateWithErrorMessage) {
  RecorderConfig config;
  config.dataset.path = "";  // Invalid
  
  std::string error_msg;
  EXPECT_FALSE(ConfigParser::validate(config, error_msg));
  EXPECT_FALSE(error_msg.empty());
}

// ============================================================================
// File I/O Tests
// ============================================================================

TEST_F(ConfigParserTest, LoadFromFile) {
  const std::string yaml = R"(
dataset:
  path: /data/recordings
  mode: create
topics:
  - name: /test
    message_type: std_msgs/String
)";

  auto path = write_test_file("test_config.yaml", yaml);
  
  RecorderConfig config = RecorderConfig::from_yaml(path);
  EXPECT_EQ(config.dataset.path, "/data/recordings");
  EXPECT_TRUE(config.validate());
}

TEST_F(ConfigParserTest, LoadFromNonexistentFile) {
  RecorderConfig config = RecorderConfig::from_yaml("/nonexistent/path.yaml");
  // Should return default/empty config
  EXPECT_FALSE(config.validate());
}

TEST_F(ConfigParserTest, SaveToFile) {
  RecorderConfig config;
  config.dataset.path = "/data/test";
  config.dataset.mode = "create";
  
  TopicConfig topic;
  topic.name = "/test_topic";
  topic.message_type = "std_msgs/String";
  topic.batch_size = 50;
  topic.flush_interval_ms = 500;
  config.topics.push_back(topic);
  
  config.recording.max_disk_usage_gb = 75.0;
  
  auto path = (test_dir_ / "saved_config.yaml").string();
  
  ConfigParser parser;
  EXPECT_TRUE(parser.save_to_file(path, config));
  
  // Read it back
  RecorderConfig loaded = RecorderConfig::from_yaml(path);
  EXPECT_EQ(loaded.dataset.path, "/data/test");
  EXPECT_EQ(loaded.dataset.mode, "create");
  ASSERT_EQ(loaded.topics.size(), 1);
  EXPECT_EQ(loaded.topics[0].name, "/test_topic");
  EXPECT_EQ(loaded.topics[0].batch_size, 50);
  EXPECT_DOUBLE_EQ(loaded.recording.max_disk_usage_gb, 75.0);
}

// ============================================================================
// Edge Cases and Error Handling
// ============================================================================

TEST_F(ConfigParserTest, MalformedYamlHandling) {
  const std::string yaml = R"(
dataset:
  path: /data
  mode: [invalid yaml structure
topics:
  broken
)";

  ConfigParser parser;
  RecorderConfig config;
  // Should not crash, but return false
  EXPECT_FALSE(parser.load_from_string(yaml, config));
}

TEST_F(ConfigParserTest, TopicsNotASequence) {
  const std::string yaml = R"(
dataset:
  path: /data
  mode: create
topics:
  not_a_sequence: true
)";

  ConfigParser parser;
  RecorderConfig config;
  // parse_topics returns false when not a sequence
  EXPECT_FALSE(parser.load_from_string(yaml, config));
}

TEST_F(ConfigParserTest, EmptyYaml) {
  const std::string yaml = "";
  
  ConfigParser parser;
  RecorderConfig config;
  EXPECT_FALSE(parser.load_from_string(yaml, config));
}

TEST_F(ConfigParserTest, OnlyCommentsYaml) {
  const std::string yaml = R"(
# This is a comment
# Another comment
)";
  
  ConfigParser parser;
  RecorderConfig config;
  EXPECT_FALSE(parser.load_from_string(yaml, config));
}

TEST_F(ConfigParserTest, LargeBatchSize) {
  const std::string yaml = R"(
dataset:
  path: /data
  mode: create
topics:
  - name: /test
    message_type: std_msgs/String
    batch_size: 1000000
)";

  RecorderConfig config = RecorderConfig::from_yaml_string(yaml);
  EXPECT_EQ(config.topics[0].batch_size, 1000000);
  EXPECT_TRUE(config.validate());
}

TEST_F(ConfigParserTest, NegativeFlushInterval) {
  const std::string yaml = R"(
dataset:
  path: /data
  mode: create
topics:
  - name: /test
    message_type: std_msgs/String
    flush_interval_ms: -100
)";

  RecorderConfig config = RecorderConfig::from_yaml_string(yaml);
  // Negative value is parsed but validation should still work
  // as flush_interval isn't explicitly validated
  EXPECT_EQ(config.topics[0].flush_interval_ms, -100);
}

TEST_F(ConfigParserTest, SpecialCharactersInPath) {
  const std::string yaml = R"(
dataset:
  path: /data/recordings with spaces/test!@#$%
  mode: create
topics:
  - name: /test
    message_type: std_msgs/String
)";

  RecorderConfig config = RecorderConfig::from_yaml_string(yaml);
  EXPECT_EQ(config.dataset.path, "/data/recordings with spaces/test!@#$%");
}

TEST_F(ConfigParserTest, UnicodeInTopicName) {
  const std::string yaml = R"(
dataset:
  path: /data
  mode: create
topics:
  - name: /test/日本語
    message_type: std_msgs/String
)";

  RecorderConfig config = RecorderConfig::from_yaml_string(yaml);
  EXPECT_EQ(config.topics[0].name, "/test/日本語");
}

// ============================================================================
// ToString Tests
// ============================================================================

TEST_F(ConfigParserTest, ToStringOutput) {
  RecorderConfig config;
  config.dataset.path = "/data/test";
  config.dataset.mode = "create";
  config.recording.max_disk_usage_gb = 50.0;
  
  TopicConfig topic;
  topic.name = "/camera/image";
  topic.message_type = "sensor_msgs/Image";
  topic.batch_size = 100;
  topic.flush_interval_ms = 1000;
  config.topics.push_back(topic);
  
  std::string output = config.to_string();
  
  EXPECT_NE(output.find("/data/test"), std::string::npos);
  EXPECT_NE(output.find("create"), std::string::npos);
  EXPECT_NE(output.find("/camera/image"), std::string::npos);
  EXPECT_NE(output.find("sensor_msgs/Image"), std::string::npos);
  EXPECT_NE(output.find("50"), std::string::npos);
}

// ============================================================================
// Multiple Topics Tests
// ============================================================================

TEST_F(ConfigParserTest, MultipleTopicsWithDifferentConfigs) {
  const std::string yaml = R"(
dataset:
  path: /data
  mode: create
topics:
  - name: /camera/left
    message_type: sensor_msgs/Image
    batch_size: 10
  - name: /camera/right
    message_type: sensor_msgs/Image
    batch_size: 10
  - name: /imu
    message_type: sensor_msgs/Imu
    batch_size: 500
  - name: /lidar
    message_type: sensor_msgs/PointCloud2
    batch_size: 5
    flush_interval_ms: 5000
)";

  RecorderConfig config = RecorderConfig::from_yaml_string(yaml);
  
  ASSERT_EQ(config.topics.size(), 4);
  EXPECT_EQ(config.topics[0].name, "/camera/left");
  EXPECT_EQ(config.topics[1].name, "/camera/right");
  EXPECT_EQ(config.topics[2].name, "/imu");
  EXPECT_EQ(config.topics[3].name, "/lidar");
  EXPECT_EQ(config.topics[3].flush_interval_ms, 5000);
  
  EXPECT_TRUE(config.validate());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

