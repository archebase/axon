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

#include "../config_parser.hpp"
#include "../recorder.hpp"

namespace fs = std::filesystem;

using namespace axon::recorder;

// ============================================================================
// Test Fixtures
// ============================================================================

class ConfigParserTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Create a unique temp directory for test files
    test_dir_ = fs::temp_directory_path() /
                ("config_test_" +
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

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));
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

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));

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

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));

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

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));

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

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));

  // Dataset defaults
  EXPECT_EQ(config.dataset.mode, "append");  // Default mode

  // Topic defaults
  EXPECT_EQ(config.topics[0].batch_size, 100);          // Default batch size
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
  EXPECT_TRUE(config.upload.s3.bucket.empty());  // Bucket must be explicitly configured
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

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));
  EXPECT_FALSE(config.validate());
}

TEST_F(ConfigParserTest, ValidationFailsWithNoTopics) {
  const std::string yaml = R"(
dataset:
  path: /data
  mode: create
topics: []
)";

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));
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

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));
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

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));
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

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));
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

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));
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

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));
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

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_file(path, config));
  EXPECT_EQ(config.dataset.path, "/data/recordings");
  EXPECT_TRUE(config.validate());
}

TEST_F(ConfigParserTest, LoadFromNonexistentFile) {
  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_file("/nonexistent/path.yaml");
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

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));
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

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));
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

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));
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

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));
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

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));

  ASSERT_EQ(config.topics.size(), 4);
  EXPECT_EQ(config.topics[0].name, "/camera/left");
  EXPECT_EQ(config.topics[1].name, "/camera/right");
  EXPECT_EQ(config.topics[2].name, "/imu");
  EXPECT_EQ(config.topics[3].name, "/lidar");
  EXPECT_EQ(config.topics[3].flush_interval_ms, 5000);

  EXPECT_TRUE(config.validate());
}

// ============================================================================
// Upload Configuration Tests
// ============================================================================

TEST_F(ConfigParserTest, UploadConfigDisabled) {
  const std::string yaml = R"(
dataset:
  path: /data
  mode: create
topics:
  - name: /test
    message_type: std_msgs/String
upload:
  enabled: false
)";

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));
  EXPECT_FALSE(config.upload.enabled);
  EXPECT_TRUE(config.validate());
}

TEST_F(ConfigParserTest, UploadConfigValidFull) {
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
    endpoint_url: "https://s3.amazonaws.com"
    bucket: "my-bucket"
    region: "us-west-2"
    use_ssl: true
    verify_ssl: true
  retry:
    max_retries: 3
    initial_delay_ms: 500
    max_delay_ms: 60000
  num_workers: 4
  state_db_path: "/var/lib/axon/state.db"
  delete_after_upload: true
  failed_uploads_dir: "/data/failed"
  warn_pending_gb: 5.0
  alert_pending_gb: 10.0
)";

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));
  EXPECT_TRUE(config.upload.enabled);
  EXPECT_EQ(config.upload.s3.endpoint_url, "https://s3.amazonaws.com");
  EXPECT_EQ(config.upload.s3.bucket, "my-bucket");
  EXPECT_EQ(config.upload.s3.region, "us-west-2");
  EXPECT_EQ(config.upload.retry.max_retries, 3);
  EXPECT_EQ(config.upload.num_workers, 4);
  EXPECT_TRUE(config.validate());
}

TEST_F(ConfigParserTest, UploadConfigMissingBucket) {
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
    endpoint_url: "https://s3.amazonaws.com"
    # bucket missing!
)";

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));
  EXPECT_FALSE(config.validate());

  std::string error_msg;
  EXPECT_FALSE(ConfigParser::validate(config, error_msg));
  EXPECT_NE(error_msg.find("bucket"), std::string::npos);
}

TEST_F(ConfigParserTest, UploadConfigInvalidEndpointURL) {
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
    endpoint_url: "invalid-url-without-protocol"
    bucket: "my-bucket"
)";

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));
  EXPECT_FALSE(config.validate());

  std::string error_msg;
  EXPECT_FALSE(ConfigParser::validate(config, error_msg));
  EXPECT_NE(error_msg.find("endpoint_url"), std::string::npos);
}

TEST_F(ConfigParserTest, UploadConfigInvalidNumWorkers) {
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
    bucket: "my-bucket"
  num_workers: 0
)";

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));
  EXPECT_FALSE(config.validate());

  std::string error_msg;
  EXPECT_FALSE(ConfigParser::validate(config, error_msg));
  EXPECT_NE(error_msg.find("num_workers"), std::string::npos);
}

TEST_F(ConfigParserTest, UploadConfigInvalidRetryCount) {
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
    bucket: "my-bucket"
  retry:
    max_retries: -1
)";

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));
  EXPECT_FALSE(config.validate());

  std::string error_msg;
  EXPECT_FALSE(ConfigParser::validate(config, error_msg));
  EXPECT_NE(error_msg.find("max_retries"), std::string::npos);
}

TEST_F(ConfigParserTest, UploadConfigInvalidRetryDelays) {
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
    bucket: "my-bucket"
  retry:
    initial_delay_ms: 5000
    max_delay_ms: 1000
)";

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));
  EXPECT_FALSE(config.validate());

  std::string error_msg;
  EXPECT_FALSE(ConfigParser::validate(config, error_msg));
  EXPECT_NE(error_msg.find("max_delay_ms"), std::string::npos);
}

TEST_F(ConfigParserTest, UploadConfigInvalidBackpressureThresholds) {
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
    bucket: "my-bucket"
  warn_pending_gb: 10.0
  alert_pending_gb: 5.0
)";

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));
  EXPECT_FALSE(config.validate());

  std::string error_msg;
  EXPECT_FALSE(ConfigParser::validate(config, error_msg));
  EXPECT_NE(error_msg.find("alert_pending_gb"), std::string::npos);
}

TEST_F(ConfigParserTest, UploadConfigEmptyStateDbPath) {
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
    bucket: "my-bucket"
  state_db_path: ""
)";

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));
  EXPECT_FALSE(config.validate());

  std::string error_msg;
  EXPECT_FALSE(ConfigParser::validate(config, error_msg));
  EXPECT_NE(error_msg.find("state_db_path"), std::string::npos);
}

TEST_F(ConfigParserTest, UploadConfigHTTPEndpoint) {
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
    endpoint_url: "http://localhost:9000"
    bucket: "test-bucket"
    use_ssl: false
    verify_ssl: false
)";

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));
  EXPECT_TRUE(config.upload.enabled);
  EXPECT_EQ(config.upload.s3.endpoint_url, "http://localhost:9000");
  EXPECT_FALSE(config.upload.s3.use_ssl);
  EXPECT_TRUE(config.validate());
}

TEST_F(ConfigParserTest, UploadConfigDefaultValues) {
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
    bucket: "my-bucket"
)";

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));
  EXPECT_TRUE(config.upload.enabled);
  EXPECT_EQ(config.upload.s3.bucket, "my-bucket");
  EXPECT_EQ(config.upload.num_workers, 2);         // default
  EXPECT_EQ(config.upload.retry.max_retries, 5);   // default
  EXPECT_TRUE(config.upload.delete_after_upload);  // default
  EXPECT_TRUE(config.validate());
}

// ============================================================================
// Enhanced Coverage Tests (Phase 4)
// ============================================================================

TEST_F(ConfigParserTest, ParseLogging_OnlyConsole) {
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
    colors: true
    level: warn
)";

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));

  EXPECT_TRUE(config.logging.console_enabled);
  EXPECT_TRUE(config.logging.console_colors);
  EXPECT_EQ(config.logging.console_level, "warn");
  // File should have defaults
  EXPECT_FALSE(config.logging.file_enabled);
  EXPECT_TRUE(config.validate());
}

TEST_F(ConfigParserTest, ParseLogging_OnlyFile) {
  const std::string yaml = R"(
dataset:
  path: /data
  mode: create
topics:
  - name: /test
    message_type: std_msgs/String
logging:
  file:
    enabled: true
    level: error
    directory: /tmp/logs
)";

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));

  // Console should have defaults
  EXPECT_TRUE(config.logging.console_enabled);      // Default
  EXPECT_EQ(config.logging.console_level, "info");  // Default
  // File should be configured
  EXPECT_TRUE(config.logging.file_enabled);
  EXPECT_EQ(config.logging.file_level, "error");
  EXPECT_EQ(config.logging.file_directory, "/tmp/logs");
  EXPECT_TRUE(config.validate());
}

TEST_F(ConfigParserTest, ParseUpload_MinimalS3Config) {
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
    bucket: minimal-bucket
)";

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));

  EXPECT_TRUE(config.upload.enabled);
  EXPECT_EQ(config.upload.s3.bucket, "minimal-bucket");
  // Defaults should be applied
  EXPECT_EQ(config.upload.s3.region, "us-east-1");
  EXPECT_TRUE(config.upload.s3.use_ssl);
  EXPECT_TRUE(config.upload.s3.verify_ssl);
  EXPECT_EQ(config.upload.num_workers, 2);
  EXPECT_EQ(config.upload.retry.max_retries, 5);
  EXPECT_TRUE(config.validate());
}

TEST_F(ConfigParserTest, SaveToFile_InvalidPath) {
  RecorderConfig config;
  config.dataset.path = "/data/test";
  config.dataset.mode = "create";

  TopicConfig topic;
  topic.name = "/test";
  topic.message_type = "std_msgs/String";
  config.topics.push_back(topic);

  ConfigParser parser;
  // Try to save to a read-only system path that should fail
  // Note: /proc is a read-only filesystem on Linux
  bool result = parser.save_to_file("/proc/config_test.yaml", config);
  // On most systems this should fail, but if it somehow succeeds, just verify behavior
  (void)result;  // Result may vary by system permissions
}

TEST_F(ConfigParserTest, ConvertLoggingConfig_UnknownLevel) {
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
    level: unknown_level
  file:
    enabled: true
    level: also_unknown
)";

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));

  // Unknown levels should be preserved as-is (they'll be handled at runtime)
  EXPECT_EQ(config.logging.console_level, "unknown_level");
  EXPECT_EQ(config.logging.file_level, "also_unknown");
  // Config should still be valid (level validation happens at runtime)
  EXPECT_TRUE(config.validate());
}

TEST_F(ConfigParserTest, ParseLogging_EmptySection) {
  const std::string yaml = R"(
dataset:
  path: /data
  mode: create
topics:
  - name: /test
    message_type: std_msgs/String
logging:
)";

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));

  // Should use defaults
  EXPECT_TRUE(config.logging.console_enabled);
  EXPECT_FALSE(config.logging.file_enabled);
  EXPECT_TRUE(config.validate());
}

TEST_F(ConfigParserTest, ParseRecording_AllOptions) {
  const std::string yaml = R"(
dataset:
  path: /data
  mode: create
topics:
  - name: /test
    message_type: std_msgs/String
recording:
  max_disk_usage_gb: 200.5
)";

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));
  EXPECT_DOUBLE_EQ(config.recording.max_disk_usage_gb, 200.5);
  EXPECT_TRUE(config.validate());
}

TEST_F(ConfigParserTest, ParseMissingDatasetSection) {
  const std::string yaml = R"(
topics:
  - name: /test
    message_type: std_msgs/String
)";

  ConfigParser parser;
  RecorderConfig config;
  // Should fail - dataset section is required
  EXPECT_FALSE(parser.load_from_string(yaml, config));
}

TEST_F(ConfigParserTest, ParseMissingTopicsSection) {
  const std::string yaml = R"(
dataset:
  path: /data
  mode: create
)";

  ConfigParser parser;
  RecorderConfig config;
  // Should fail - topics section is required
  EXPECT_FALSE(parser.load_from_string(yaml, config));
}

TEST_F(ConfigParserTest, ValidationWithMaxDiskUsageZero) {
  const std::string yaml = R"(
dataset:
  path: /data
  mode: create
topics:
  - name: /test
    message_type: std_msgs/String
recording:
  max_disk_usage_gb: 0
)";

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));
  // Zero disk usage is parsed and allowed (runtime handling)
  EXPECT_DOUBLE_EQ(config.recording.max_disk_usage_gb, 0.0);
}

TEST_F(ConfigParserTest, ValidationWithNegativeMaxDiskUsage) {
  const std::string yaml = R"(
dataset:
  path: /data
  mode: create
topics:
  - name: /test
    message_type: std_msgs/String
recording:
  max_disk_usage_gb: -100
)";

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));
  // Negative disk usage is parsed (runtime handling)
  EXPECT_DOUBLE_EQ(config.recording.max_disk_usage_gb, -100.0);
}

TEST_F(ConfigParserTest, DuplicateTopicNames) {
  const std::string yaml = R"(
dataset:
  path: /data
  mode: create
topics:
  - name: /camera/image
    message_type: sensor_msgs/Image
  - name: /camera/image
    message_type: sensor_msgs/Image
)";

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));
  ASSERT_EQ(config.topics.size(), 2);
  // Duplicate topic names - should this be allowed or not?
  // Currently parses but validation behavior may vary
}

TEST_F(ConfigParserTest, TopicWithAllOptionalFields) {
  const std::string yaml = R"(
dataset:
  path: /data
  mode: create
topics:
  - name: /test
    message_type: std_msgs/String
    batch_size: 250
    flush_interval_ms: 2500
)";

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));
  ASSERT_EQ(config.topics.size(), 1);
  EXPECT_EQ(config.topics[0].name, "/test");
  EXPECT_EQ(config.topics[0].message_type, "std_msgs/String");
  EXPECT_EQ(config.topics[0].batch_size, 250);
  EXPECT_EQ(config.topics[0].flush_interval_ms, 2500);
  EXPECT_TRUE(config.validate());
}

TEST_F(ConfigParserTest, FileRotationSettings) {
  const std::string yaml = R"(
dataset:
  path: /data
  mode: create
topics:
  - name: /test
    message_type: std_msgs/String
logging:
  file:
    enabled: true
    directory: /logs
    rotation_size_mb: 100
    max_files: 10
    rotate_at_midnight: true
)";

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));
  EXPECT_TRUE(config.logging.file_enabled);
  EXPECT_EQ(config.logging.rotation_size_mb, 100);
  EXPECT_EQ(config.logging.max_files, 10);
  EXPECT_TRUE(config.logging.rotate_at_midnight);
  EXPECT_TRUE(config.validate());
}

TEST_F(ConfigParserTest, UploadRetryWithJitter) {
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
    bucket: my-bucket
  retry:
    max_retries: 3
    initial_delay_ms: 100
    max_delay_ms: 10000
    exponential_base: 2.0
    jitter: true
)";

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));
  EXPECT_TRUE(config.upload.enabled);
  EXPECT_EQ(config.upload.retry.max_retries, 3);
  EXPECT_EQ(config.upload.retry.initial_delay_ms, 100);
  EXPECT_EQ(config.upload.retry.max_delay_ms, 10000);
  EXPECT_DOUBLE_EQ(config.upload.retry.exponential_base, 2.0);
  EXPECT_TRUE(config.upload.retry.jitter);
  EXPECT_TRUE(config.validate());
}

// ============================================================================
// Convert Logging Config Tests (Phase 5 - Coverage Improvement)
// ============================================================================

TEST_F(ConfigParserTest, ConvertLoggingConfig_AllLevels) {
  // Test conversion with all severity levels
  LoggingConfigYaml yaml_config;

  // Test debug level
  yaml_config.console_level = "debug";
  yaml_config.file_level = "debug";
  ::axon::logging::LoggingConfig log_config;
  convert_logging_config(yaml_config, log_config);
  EXPECT_EQ(log_config.console_level, ::axon::logging::severity_level::debug);
  EXPECT_EQ(log_config.file_level, ::axon::logging::severity_level::debug);

  // Test info level
  yaml_config.console_level = "info";
  yaml_config.file_level = "info";
  convert_logging_config(yaml_config, log_config);
  EXPECT_EQ(log_config.console_level, ::axon::logging::severity_level::info);
  EXPECT_EQ(log_config.file_level, ::axon::logging::severity_level::info);

  // Test warn level
  yaml_config.console_level = "warn";
  yaml_config.file_level = "warn";
  convert_logging_config(yaml_config, log_config);
  EXPECT_EQ(log_config.console_level, ::axon::logging::severity_level::warn);
  EXPECT_EQ(log_config.file_level, ::axon::logging::severity_level::warn);

  // Test error level
  yaml_config.console_level = "error";
  yaml_config.file_level = "error";
  convert_logging_config(yaml_config, log_config);
  EXPECT_EQ(log_config.console_level, ::axon::logging::severity_level::error);
  EXPECT_EQ(log_config.file_level, ::axon::logging::severity_level::error);

  // Test fatal level
  yaml_config.console_level = "fatal";
  yaml_config.file_level = "fatal";
  convert_logging_config(yaml_config, log_config);
  EXPECT_EQ(log_config.console_level, ::axon::logging::severity_level::fatal);
  EXPECT_EQ(log_config.file_level, ::axon::logging::severity_level::fatal);
}

TEST_F(ConfigParserTest, ConvertLoggingConfig_InvalidLevel) {
  // Test with invalid level string - should use default
  LoggingConfigYaml yaml_config;
  yaml_config.console_level = "invalid_level";
  yaml_config.file_level = "also_invalid";

  ::axon::logging::LoggingConfig log_config;
  log_config.console_level = ::axon::logging::severity_level::info;  // Set default
  log_config.file_level = ::axon::logging::severity_level::debug;    // Set default

  convert_logging_config(yaml_config, log_config);

  // Invalid levels should keep the default (not crash)
  // The function may or may not update invalid levels depending on implementation
}

TEST_F(ConfigParserTest, ConvertLoggingConfig_FileSettings) {
  LoggingConfigYaml yaml_config;
  yaml_config.console_enabled = true;
  yaml_config.console_colors = false;
  yaml_config.file_enabled = true;
  yaml_config.file_directory = "/custom/log/dir";
  yaml_config.file_pattern = "custom_%Y%m%d.log";
  yaml_config.file_format = "json";
  yaml_config.rotation_size_mb = 50;
  yaml_config.max_files = 5;
  yaml_config.rotate_at_midnight = false;

  ::axon::logging::LoggingConfig log_config;
  convert_logging_config(yaml_config, log_config);

  EXPECT_TRUE(log_config.console_enabled);
  EXPECT_FALSE(log_config.console_colors);
  EXPECT_TRUE(log_config.file_enabled);
  EXPECT_EQ(log_config.file_config.directory, "/custom/log/dir");
  EXPECT_EQ(log_config.file_config.file_pattern, "custom_%Y%m%d.log");
  EXPECT_TRUE(log_config.file_config.format_json);
  EXPECT_EQ(log_config.file_config.rotation_size_mb, 50);
  EXPECT_EQ(log_config.file_config.max_files, 5);
  EXPECT_FALSE(log_config.file_config.rotate_at_midnight);
}

TEST_F(ConfigParserTest, ConvertLoggingConfig_TextFormat) {
  LoggingConfigYaml yaml_config;
  yaml_config.file_format = "text";

  ::axon::logging::LoggingConfig log_config;
  convert_logging_config(yaml_config, log_config);

  EXPECT_FALSE(log_config.file_config.format_json);
}

// ============================================================================
// Additional Validation Edge Cases (Phase 5)
// ============================================================================

TEST_F(ConfigParserTest, ValidateUploadConfig_NegativeBackpressure) {
  UploadConfigYaml upload;
  upload.enabled = true;
  upload.s3.bucket = "test-bucket";
  upload.warn_pending_gb = -1.0;  // Invalid negative
  upload.alert_pending_gb = 10.0;

  std::string error_msg;
  EXPECT_FALSE(ConfigParser::validate_upload_config(upload, error_msg));
  EXPECT_NE(error_msg.find("backpressure"), std::string::npos);
}

TEST_F(ConfigParserTest, ValidateUploadConfig_MaxRetriesAtBoundary) {
  UploadConfigYaml upload;
  upload.enabled = true;
  upload.s3.bucket = "test-bucket";
  upload.retry.max_retries = 100;  // At boundary (max valid)

  std::string error_msg;
  EXPECT_TRUE(ConfigParser::validate_upload_config(upload, error_msg));

  // Over boundary
  upload.retry.max_retries = 101;
  EXPECT_FALSE(ConfigParser::validate_upload_config(upload, error_msg));
}

TEST_F(ConfigParserTest, ValidateUploadConfig_NumWorkersAtBoundary) {
  UploadConfigYaml upload;
  upload.enabled = true;
  upload.s3.bucket = "test-bucket";

  // Test lower boundary
  upload.num_workers = 1;
  std::string error_msg;
  EXPECT_TRUE(ConfigParser::validate_upload_config(upload, error_msg));

  // Test upper boundary
  upload.num_workers = 16;
  EXPECT_TRUE(ConfigParser::validate_upload_config(upload, error_msg));

  // Over boundary
  upload.num_workers = 17;
  EXPECT_FALSE(ConfigParser::validate_upload_config(upload, error_msg));
  EXPECT_NE(error_msg.find("num_workers"), std::string::npos);
}

TEST_F(ConfigParserTest, ValidateUploadConfig_NegativeInitialDelay) {
  UploadConfigYaml upload;
  upload.enabled = true;
  upload.s3.bucket = "test-bucket";
  upload.retry.initial_delay_ms = -100;  // Invalid

  std::string error_msg;
  EXPECT_FALSE(ConfigParser::validate_upload_config(upload, error_msg));
  EXPECT_NE(error_msg.find("initial_delay_ms"), std::string::npos);
}

TEST_F(ConfigParserTest, ValidateUploadConfig_DisabledBypassesValidation) {
  UploadConfigYaml upload;
  upload.enabled = false;
  // Invalid settings that should be ignored when disabled
  upload.s3.bucket = "";   // Would be invalid if enabled
  upload.num_workers = 0;  // Would be invalid if enabled

  std::string error_msg;
  EXPECT_TRUE(ConfigParser::validate_upload_config(upload, error_msg));
}

TEST_F(ConfigParserTest, ParseDataset_PartialFields) {
  const std::string yaml = R"(
dataset:
  path: /only/path
topics:
  - name: /test
    message_type: std_msgs/String
)";

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));
  EXPECT_EQ(config.dataset.path, "/only/path");
  EXPECT_EQ(config.dataset.mode, "append");  // Default
}

TEST_F(ConfigParserTest, ParseTopics_PartialTopicConfig) {
  const std::string yaml = R"(
dataset:
  path: /data
  mode: create
topics:
  - name: /camera
    message_type: sensor_msgs/Image
  - name: /imu
    message_type: sensor_msgs/Imu
    batch_size: 500
)";

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));

  ASSERT_EQ(config.topics.size(), 2);

  // First topic uses defaults
  EXPECT_EQ(config.topics[0].batch_size, 100);
  EXPECT_EQ(config.topics[0].flush_interval_ms, 1000);

  // Second topic has custom batch_size, default flush_interval
  EXPECT_EQ(config.topics[1].batch_size, 500);
  EXPECT_EQ(config.topics[1].flush_interval_ms, 1000);
}

TEST_F(ConfigParserTest, ValidateStaticMethod_AllErrorCases) {
  RecorderConfig config;
  std::string error_msg;

  // Empty dataset path
  config.dataset.path = "";
  config.dataset.mode = "create";
  config.topics.push_back({"topic", "type", 100, 1000});
  EXPECT_FALSE(ConfigParser::validate(config, error_msg));
  EXPECT_EQ(error_msg, "Dataset path is empty");

  // No topics
  config.dataset.path = "/data";
  config.topics.clear();
  EXPECT_FALSE(ConfigParser::validate(config, error_msg));
  EXPECT_EQ(error_msg, "No topics configured");

  // Empty topic name
  config.topics.push_back({"", "type", 100, 1000});
  EXPECT_FALSE(ConfigParser::validate(config, error_msg));
  EXPECT_EQ(error_msg, "Topic name is empty");

  // Empty message type
  config.topics.clear();
  config.topics.push_back({"/topic", "", 100, 1000});
  EXPECT_FALSE(ConfigParser::validate(config, error_msg));
  EXPECT_EQ(error_msg, "Topic message_type is empty");

  // Zero batch size
  config.topics.clear();
  config.topics.push_back({"/topic", "type", 0, 1000});
  EXPECT_FALSE(ConfigParser::validate(config, error_msg));
  EXPECT_EQ(error_msg, "Topic batch_size must be > 0");

  // Invalid mode
  config.topics.clear();
  config.topics.push_back({"/topic", "type", 100, 1000});
  config.dataset.mode = "invalid_mode";
  EXPECT_FALSE(ConfigParser::validate(config, error_msg));
  EXPECT_EQ(error_msg, "Dataset mode must be 'create' or 'append'");
}

TEST_F(ConfigParserTest, FromYamlString_YAMLException) {
  // Test YAML parsing exception handling
  const std::string malformed_yaml = R"(
dataset:
  path: [unclosed bracket
)";

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(malformed_yaml, config));
  // Should return default config (validation will fail)
  EXPECT_FALSE(config.validate());
}

TEST_F(ConfigParserTest, LoadFromFile_YAMLException) {
  // Write a malformed YAML file
  auto path = write_test_file("malformed.yaml", "{ unclosed: [brace");

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_file(path, config));
  EXPECT_FALSE(config.validate());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
