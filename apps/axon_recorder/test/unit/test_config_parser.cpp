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
subscriptions:
  - name: /camera/image
    message_type: sensor_msgs/Image
)";

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));
  EXPECT_EQ(config.dataset.path, "/data/recordings");
  EXPECT_EQ(config.dataset.mode, "create");
  ASSERT_EQ(config.subscriptions.size(), 1);
  EXPECT_EQ(config.subscriptions[0].topic_name, "/camera/image");
  EXPECT_EQ(config.subscriptions[0].message_type, "sensor_msgs/Image");
}

TEST_F(ConfigParserTest, ParseValidFullConfig) {
  const std::string yaml = R"(
plugin:
  path: /usr/lib/libaxon_ros2.so

dataset:
  path: /data/recordings
  mode: append
  stats_file_path: /tmp/stats.json

subscriptions:
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

  EXPECT_EQ(config.plugin_path, "/usr/lib/libaxon_ros2.so");

  EXPECT_EQ(config.dataset.path, "/data/recordings");
  EXPECT_EQ(config.dataset.mode, "append");
  EXPECT_EQ(config.dataset.stats_file_path, "/tmp/stats.json");

  ASSERT_EQ(config.subscriptions.size(), 2);
  EXPECT_EQ(config.subscriptions[0].topic_name, "/camera/image");
  EXPECT_EQ(config.subscriptions[0].batch_size, 50);
  EXPECT_EQ(config.subscriptions[0].flush_interval_ms, 500);
  EXPECT_EQ(config.subscriptions[1].topic_name, "/imu/data");
  EXPECT_EQ(config.subscriptions[1].batch_size, 200);

  EXPECT_DOUBLE_EQ(config.recording.max_disk_usage_gb, 50.0);

  // Logging config
  EXPECT_TRUE(config.logging.console_enabled);
  EXPECT_FALSE(config.logging.console_colors);
  EXPECT_EQ(config.logging.console_level, "debug");
  EXPECT_TRUE(config.logging.file_enabled);
  EXPECT_EQ(config.logging.file_level, "info");
  EXPECT_EQ(config.logging.file_directory, "/var/log/axon");
  EXPECT_EQ(config.logging.file_pattern, "recorder_%Y%m%d.log");
  EXPECT_EQ(config.logging.file_format, "json");

  // Upload config
  EXPECT_TRUE(config.upload.enabled);
  EXPECT_EQ(config.upload.s3.endpoint_url, "http://minio:9000");
  EXPECT_EQ(config.upload.s3.bucket, "test-bucket");
  EXPECT_EQ(config.upload.retry.max_retries, 10);
  EXPECT_EQ(config.upload.num_workers, 4);
}

// ============================================================================
// Default Values Tests
// ============================================================================

TEST_F(ConfigParserTest, DefaultValuesForOptionalFields) {
  const std::string yaml = R"(
dataset:
  path: /data
subscriptions:
  - name: /test
    message_type: std_msgs/String
)";

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));

  // Dataset defaults
  EXPECT_EQ(config.dataset.mode, "append");  // Default mode
  EXPECT_EQ(config.dataset.stats_file_path, "/data/recordings/recorder_stats.json");

  // Subscription defaults
  EXPECT_EQ(config.subscriptions[0].batch_size, 1);           // Default batch size
  EXPECT_EQ(config.subscriptions[0].flush_interval_ms, 100);  // Default flush interval

  // Recording defaults
  EXPECT_DOUBLE_EQ(config.recording.max_disk_usage_gb, 100.0);

  // Logging defaults
  EXPECT_TRUE(config.logging.console_enabled);
  EXPECT_TRUE(config.logging.console_colors);
  EXPECT_EQ(config.logging.console_level, "info");
  EXPECT_FALSE(config.logging.file_enabled);

  // Upload defaults
  EXPECT_FALSE(config.upload.enabled);
  EXPECT_TRUE(config.upload.s3.bucket.empty());
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

TEST_F(ConfigParserTest, ValidatePassesWithValidConfig) {
  RecorderConfig config;
  config.dataset.path = "/data/recordings";
  config.dataset.mode = "create";
  config.subscriptions.push_back({"/camera/image", "sensor_msgs/Image", 100, 100});

  std::string error_msg;
  EXPECT_TRUE(ConfigParser::validate(config, error_msg));
}

TEST_F(ConfigParserTest, ValidateFailsWithEmptyDatasetPath) {
  RecorderConfig config;
  config.dataset.path = "";
  config.dataset.mode = "create";
  config.subscriptions.push_back({"/test", "std_msgs/String", 100, 100});

  std::string error_msg;
  EXPECT_FALSE(ConfigParser::validate(config, error_msg));
  EXPECT_EQ(error_msg, "Dataset path is empty");
}

TEST_F(ConfigParserTest, ValidateFailsWithNoSubscriptions) {
  RecorderConfig config;
  config.dataset.path = "/data";
  config.dataset.mode = "create";

  std::string error_msg;
  EXPECT_FALSE(ConfigParser::validate(config, error_msg));
  EXPECT_EQ(error_msg, "No subscriptions configured");
}

TEST_F(ConfigParserTest, ValidateFailsWithEmptyTopicName) {
  RecorderConfig config;
  config.dataset.path = "/data";
  config.dataset.mode = "create";
  config.subscriptions.push_back({"", "std_msgs/String", 100, 100});

  std::string error_msg;
  EXPECT_FALSE(ConfigParser::validate(config, error_msg));
  EXPECT_EQ(error_msg, "Subscription topic_name is empty");
}

TEST_F(ConfigParserTest, ValidateFailsWithEmptyMessageType) {
  RecorderConfig config;
  config.dataset.path = "/data";
  config.dataset.mode = "create";
  config.subscriptions.push_back({"/test", "", 100, 100});

  std::string error_msg;
  EXPECT_FALSE(ConfigParser::validate(config, error_msg));
  EXPECT_EQ(error_msg, "Subscription message_type is empty");
}

TEST_F(ConfigParserTest, ValidateFailsWithZeroBatchSize) {
  RecorderConfig config;
  config.dataset.path = "/data";
  config.dataset.mode = "create";
  config.subscriptions.push_back({"/test", "std_msgs/String", 0, 100});

  std::string error_msg;
  EXPECT_FALSE(ConfigParser::validate(config, error_msg));
  EXPECT_EQ(error_msg, "Subscription batch_size must be > 0");
}

TEST_F(ConfigParserTest, ValidateFailsWithInvalidMode) {
  RecorderConfig config;
  config.dataset.path = "/data";
  config.dataset.mode = "invalid";
  config.subscriptions.push_back({"/test", "std_msgs/String", 100, 100});

  std::string error_msg;
  EXPECT_FALSE(ConfigParser::validate(config, error_msg));
  EXPECT_EQ(error_msg, "Dataset mode must be 'create' or 'append'");
}

// ============================================================================
// File I/O Tests
// ============================================================================

TEST_F(ConfigParserTest, LoadFromFile) {
  const std::string yaml = R"(
dataset:
  path: /data/recordings
  mode: create
subscriptions:
  - name: /test
    message_type: std_msgs/String
)";

  auto path = write_test_file("test_config.yaml", yaml);

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_file(path, config));
  EXPECT_EQ(config.dataset.path, "/data/recordings");

  std::string error_msg;
  EXPECT_TRUE(ConfigParser::validate(config, error_msg));
}

TEST_F(ConfigParserTest, LoadFromNonexistentFile) {
  ConfigParser parser;
  RecorderConfig config;
  EXPECT_FALSE(parser.load_from_file("/nonexistent/path.yaml", config));
  EXPECT_FALSE(parser.get_last_error().empty());
}

TEST_F(ConfigParserTest, SaveToFile) {
  RecorderConfig config;
  config.dataset.path = "/data/test";
  config.dataset.mode = "create";

  config.subscriptions.push_back({"/test_topic", "std_msgs/String", 50, 500});
  config.recording.max_disk_usage_gb = 75.0;

  auto path = (test_dir_ / "saved_config.yaml").string();

  ConfigParser parser;
  EXPECT_TRUE(parser.save_to_file(path, config));

  // Verify file was created
  EXPECT_TRUE(fs::exists(path));
}

// ============================================================================
// Edge Cases and Error Handling
// ============================================================================

TEST_F(ConfigParserTest, MalformedYamlHandling) {
  const std::string yaml = R"(
dataset:
  path: /data
  mode: [invalid yaml structure
subscriptions:
  broken
)";

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_FALSE(parser.load_from_string(yaml, config));
  EXPECT_FALSE(parser.get_last_error().empty());
}

TEST_F(ConfigParserTest, SubscriptionsNotASequence) {
  const std::string yaml = R"(
dataset:
  path: /data
  mode: create
subscriptions:
  not_a_sequence: true
)";

  ConfigParser parser;
  RecorderConfig config;
  // Current implementation doesn't fail on parse_subscriptions returning false
  // It just skips parsing and returns true
  EXPECT_TRUE(parser.load_from_string(yaml, config));
  // But subscriptions should be empty since parsing failed
  EXPECT_TRUE(config.subscriptions.empty());
}

TEST_F(ConfigParserTest, EmptyYaml) {
  const std::string yaml = "";

  ConfigParser parser;
  RecorderConfig config;
  // Empty YAML parses to Null node, but load_from_string returns true
  EXPECT_TRUE(parser.load_from_string(yaml, config));
  // Config should have default values
  EXPECT_TRUE(config.dataset.path.empty());
  EXPECT_TRUE(config.subscriptions.empty());
}

TEST_F(ConfigParserTest, SpecialCharactersInPath) {
  const std::string yaml = R"(
dataset:
  path: /data/recordings with spaces/test!@#$%
  mode: create
subscriptions:
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
subscriptions:
  - name: /test/日本語
    message_type: std_msgs/String
)";

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));
  EXPECT_EQ(config.subscriptions[0].topic_name, "/test/日本語");
}

// ============================================================================
// Upload Configuration Validation Tests
// ============================================================================

TEST_F(ConfigParserTest, ValidateUploadConfigDisabled) {
  UploadConfig upload;
  upload.enabled = false;

  std::string error_msg;
  EXPECT_TRUE(ConfigParser::validate_upload_config(upload, error_msg));
}

TEST_F(ConfigParserTest, ValidateUploadConfigMissingBucket) {
  UploadConfig upload;
  upload.enabled = true;
  upload.s3.bucket = "";
  upload.state_db_path = "/var/lib/axon/state.db";

  std::string error_msg;
  EXPECT_FALSE(ConfigParser::validate_upload_config(upload, error_msg));
  EXPECT_NE(error_msg.find("bucket"), std::string::npos);
}

TEST_F(ConfigParserTest, ValidateUploadConfigInvalidEndpointURL) {
  UploadConfig upload;
  upload.enabled = true;
  upload.s3.bucket = "test-bucket";
  upload.s3.endpoint_url = "invalid-url-without-protocol";
  upload.state_db_path = "/var/lib/axon/state.db";

  std::string error_msg;
  EXPECT_FALSE(ConfigParser::validate_upload_config(upload, error_msg));
  EXPECT_NE(error_msg.find("endpoint_url"), std::string::npos);
}

TEST_F(ConfigParserTest, ValidateUploadConfigHTTPEndpoint) {
  UploadConfig upload;
  upload.enabled = true;
  upload.s3.bucket = "test-bucket";
  upload.s3.endpoint_url = "http://localhost:9000";
  upload.s3.use_ssl = false;
  upload.state_db_path = "/var/lib/axon/state.db";

  std::string error_msg;
  EXPECT_TRUE(ConfigParser::validate_upload_config(upload, error_msg));
}

TEST_F(ConfigParserTest, ValidateUploadConfigHTTPSEndpoint) {
  UploadConfig upload;
  upload.enabled = true;
  upload.s3.bucket = "test-bucket";
  upload.s3.endpoint_url = "https://s3.amazonaws.com";
  upload.state_db_path = "/var/lib/axon/state.db";

  std::string error_msg;
  EXPECT_TRUE(ConfigParser::validate_upload_config(upload, error_msg));
}

TEST_F(ConfigParserTest, ValidateUploadConfigEmptyEndpointURL) {
  UploadConfig upload;
  upload.enabled = true;
  upload.s3.bucket = "test-bucket";
  upload.s3.endpoint_url = "";
  upload.state_db_path = "/var/lib/axon/state.db";

  std::string error_msg;
  EXPECT_TRUE(ConfigParser::validate_upload_config(upload, error_msg));
}

TEST_F(ConfigParserTest, ValidateUploadConfigInvalidNumWorkers) {
  UploadConfig upload;
  upload.enabled = true;
  upload.s3.bucket = "test-bucket";
  upload.num_workers = 0;
  upload.state_db_path = "/var/lib/axon/state.db";

  std::string error_msg;
  EXPECT_FALSE(ConfigParser::validate_upload_config(upload, error_msg));
  EXPECT_NE(error_msg.find("num_workers"), std::string::npos);
}

TEST_F(ConfigParserTest, ValidateUploadConfigNumWorkersBoundary) {
  UploadConfig upload;
  upload.enabled = true;
  upload.s3.bucket = "test-bucket";
  upload.state_db_path = "/var/lib/axon/state.db";

  std::string error_msg;

  // Test lower boundary
  upload.num_workers = 1;
  EXPECT_TRUE(ConfigParser::validate_upload_config(upload, error_msg));

  // Test upper boundary
  upload.num_workers = 16;
  EXPECT_TRUE(ConfigParser::validate_upload_config(upload, error_msg));

  // Over boundary
  upload.num_workers = 17;
  EXPECT_FALSE(ConfigParser::validate_upload_config(upload, error_msg));
}

TEST_F(ConfigParserTest, ValidateUploadConfigInvalidRetryCount) {
  UploadConfig upload;
  upload.enabled = true;
  upload.s3.bucket = "test-bucket";
  upload.retry.max_retries = -1;
  upload.state_db_path = "/var/lib/axon/state.db";

  std::string error_msg;
  EXPECT_FALSE(ConfigParser::validate_upload_config(upload, error_msg));
  EXPECT_NE(error_msg.find("max_retries"), std::string::npos);
}

TEST_F(ConfigParserTest, ValidateUploadConfigRetryCountBoundary) {
  UploadConfig upload;
  upload.enabled = true;
  upload.s3.bucket = "test-bucket";
  upload.state_db_path = "/var/lib/axon/state.db";

  std::string error_msg;

  upload.retry.max_retries = 0;
  EXPECT_TRUE(ConfigParser::validate_upload_config(upload, error_msg));

  upload.retry.max_retries = 100;
  EXPECT_TRUE(ConfigParser::validate_upload_config(upload, error_msg));

  upload.retry.max_retries = 101;
  EXPECT_FALSE(ConfigParser::validate_upload_config(upload, error_msg));
}

TEST_F(ConfigParserTest, ValidateUploadConfigNegativeInitialDelay) {
  UploadConfig upload;
  upload.enabled = true;
  upload.s3.bucket = "test-bucket";
  upload.retry.initial_delay_ms = -100;
  upload.state_db_path = "/var/lib/axon/state.db";

  std::string error_msg;
  EXPECT_FALSE(ConfigParser::validate_upload_config(upload, error_msg));
  EXPECT_NE(error_msg.find("initial_delay_ms"), std::string::npos);
}

TEST_F(ConfigParserTest, ValidateUploadConfigInvalidDelayOrder) {
  UploadConfig upload;
  upload.enabled = true;
  upload.s3.bucket = "test-bucket";
  upload.retry.initial_delay_ms = 5000;
  upload.retry.max_delay_ms = 1000;
  upload.state_db_path = "/var/lib/axon/state.db";

  std::string error_msg;
  EXPECT_FALSE(ConfigParser::validate_upload_config(upload, error_msg));
  EXPECT_NE(error_msg.find("max_delay_ms"), std::string::npos);
}

TEST_F(ConfigParserTest, ValidateUploadConfigInvalidBackpressure) {
  UploadConfig upload;
  upload.enabled = true;
  upload.s3.bucket = "test-bucket";
  upload.warn_pending_gb = -1.0;
  upload.alert_pending_gb = 10.0;
  upload.state_db_path = "/var/lib/axon/state.db";

  std::string error_msg;
  EXPECT_FALSE(ConfigParser::validate_upload_config(upload, error_msg));
  EXPECT_NE(error_msg.find("backpressure"), std::string::npos);
}

TEST_F(ConfigParserTest, ValidateUploadConfigInvalidBackpressureOrder) {
  UploadConfig upload;
  upload.enabled = true;
  upload.s3.bucket = "test-bucket";
  upload.warn_pending_gb = 10.0;
  upload.alert_pending_gb = 5.0;
  upload.state_db_path = "/var/lib/axon/state.db";

  std::string error_msg;
  EXPECT_FALSE(ConfigParser::validate_upload_config(upload, error_msg));
  EXPECT_NE(error_msg.find("alert_pending_gb"), std::string::npos);
}

TEST_F(ConfigParserTest, ValidateUploadConfigEmptyStateDbPath) {
  UploadConfig upload;
  upload.enabled = true;
  upload.s3.bucket = "test-bucket";
  upload.state_db_path = "";

  std::string error_msg;
  EXPECT_FALSE(ConfigParser::validate_upload_config(upload, error_msg));
  EXPECT_NE(error_msg.find("state_db_path"), std::string::npos);
}

// ============================================================================
// Parse Individual Sections Tests
// ============================================================================

TEST_F(ConfigParserTest, ParseDatasetAllFields) {
  const std::string yaml = R"(
dataset:
  path: /custom/path
  mode: create
  stats_file_path: /tmp/custom_stats.json
subscriptions:
  - name: /test
    message_type: std_msgs/String
)";

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));

  EXPECT_EQ(config.dataset.path, "/custom/path");
  EXPECT_EQ(config.dataset.mode, "create");
  EXPECT_EQ(config.dataset.stats_file_path, "/tmp/custom_stats.json");
}

TEST_F(ConfigParserTest, ParseDatasetPartialFields) {
  const std::string yaml = R"(
dataset:
  path: /only/path
subscriptions:
  - name: /test
    message_type: std_msgs/String
)";

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));

  EXPECT_EQ(config.dataset.path, "/only/path");
  EXPECT_EQ(config.dataset.mode, "append");                                           // Default
  EXPECT_EQ(config.dataset.stats_file_path, "/data/recordings/recorder_stats.json");  // Default
}

TEST_F(ConfigParserTest, ParseRecordingAllOptions) {
  const std::string yaml = R"(
dataset:
  path: /data
subscriptions:
  - name: /test
    message_type: std_msgs/String
recording:
  max_disk_usage_gb: 200.5
)";

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));
  EXPECT_DOUBLE_EQ(config.recording.max_disk_usage_gb, 200.5);
}

TEST_F(ConfigParserTest, ParseLoggingOnlyConsole) {
  const std::string yaml = R"(
dataset:
  path: /data
subscriptions:
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
  EXPECT_FALSE(config.logging.file_enabled);
}

TEST_F(ConfigParserTest, ParseLoggingOnlyFile) {
  const std::string yaml = R"(
dataset:
  path: /data
subscriptions:
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

  EXPECT_TRUE(config.logging.console_enabled);      // Default
  EXPECT_EQ(config.logging.console_level, "info");  // Default
  EXPECT_TRUE(config.logging.file_enabled);
  EXPECT_EQ(config.logging.file_level, "error");
  EXPECT_EQ(config.logging.file_directory, "/tmp/logs");
}

TEST_F(ConfigParserTest, ParseLoggingEmptySection) {
  const std::string yaml = R"(
dataset:
  path: /data
subscriptions:
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
}

TEST_F(ConfigParserTest, ParseUploadMinimalConfig) {
  const std::string yaml = R"(
dataset:
  path: /data
subscriptions:
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
  EXPECT_EQ(config.upload.s3.region, "us-east-1");  // Default
  EXPECT_TRUE(config.upload.s3.use_ssl);            // Default
  EXPECT_EQ(config.upload.num_workers, 2);          // Default
  EXPECT_EQ(config.upload.retry.max_retries, 5);    // Default
}

TEST_F(ConfigParserTest, ParseUploadFullConfig) {
  const std::string yaml = R"(
dataset:
  path: /data
subscriptions:
  - name: /test
    message_type: std_msgs/String
upload:
  enabled: true
  s3:
    endpoint_url: https://s3.amazonaws.com
    bucket: my-bucket
    region: us-west-2
    use_ssl: true
    verify_ssl: true
  retry:
    max_retries: 3
    initial_delay_ms: 500
    max_delay_ms: 60000
    exponential_base: 2.0
    jitter: true
  num_workers: 4
  state_db_path: /var/lib/axon/state.db
  delete_after_upload: true
  failed_uploads_dir: /data/failed
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
}

// ============================================================================
// Multiple Subscriptions Tests
// ============================================================================

TEST_F(ConfigParserTest, MultipleSubscriptionsWithDifferentConfigs) {
  const std::string yaml = R"(
dataset:
  path: /data
subscriptions:
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

  ASSERT_EQ(config.subscriptions.size(), 4);
  EXPECT_EQ(config.subscriptions[0].topic_name, "/camera/left");
  EXPECT_EQ(config.subscriptions[1].topic_name, "/camera/right");
  EXPECT_EQ(config.subscriptions[2].topic_name, "/imu");
  EXPECT_EQ(config.subscriptions[3].topic_name, "/lidar");
  EXPECT_EQ(config.subscriptions[3].flush_interval_ms, 5000);
}

// ============================================================================
// Plugin Configuration Tests
// ============================================================================

TEST_F(ConfigParserTest, ParsePluginPath) {
  const std::string yaml = R"(
plugin:
  path: /custom/lib/libaxon_ros1.so
dataset:
  path: /data
subscriptions:
  - name: /test
    message_type: std_msgs/String
)";

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));
  EXPECT_EQ(config.plugin_path, "/custom/lib/libaxon_ros1.so");
}

TEST_F(ConfigParserTest, ParseWithoutPluginSection) {
  const std::string yaml = R"(
dataset:
  path: /data
subscriptions:
  - name: /test
    message_type: std_msgs/String
)";

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_TRUE(parser.load_from_string(yaml, config));
  EXPECT_TRUE(config.plugin_path.empty());
}

// ============================================================================
// Error Message Tests
// ============================================================================

TEST_F(ConfigParserTest, GetErrorMessageOnFailure) {
  const std::string yaml = R"(
dataset:
  path: /data
  mode: [invalid yaml structure
subscriptions:
  broken
)";

  ConfigParser parser;
  RecorderConfig config;
  EXPECT_FALSE(parser.load_from_string(yaml, config));
  std::string error = parser.get_last_error();
  EXPECT_FALSE(error.empty());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
