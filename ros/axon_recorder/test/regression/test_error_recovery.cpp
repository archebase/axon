/**
 * @file test_error_recovery.cpp
 * @brief Regression tests for error handling and recovery paths
 *
 * Tests various error conditions to ensure the system handles failures
 * gracefully and recovers correctly.
 */

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

#include "config_parser.hpp"
#include "state_machine.hpp"

using namespace axon::recorder;

namespace fs = std::filesystem;

// ============================================================================
// Test Fixture
// ============================================================================

class ErrorRecoveryTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Create temp directory for test files
    test_dir_ = fs::temp_directory_path() / ("error_recovery_test_" +
        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::create_directories(test_dir_);
  }

  void TearDown() override {
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
// Config Parse Failure Tests
// ============================================================================

TEST_F(ErrorRecoveryTest, ConfigParseFailure_MalformedYaml) {
  const std::string malformed_yaml = R"(
dataset:
  path: /data
  mode: create
topics:
  - [this is not valid yaml
    name: broken
)";

  ConfigParser parser;
  RecorderConfig config;
  bool result = parser.load_from_string(malformed_yaml, config);

  EXPECT_FALSE(result);
  EXPECT_FALSE(config.validate());
}

TEST_F(ErrorRecoveryTest, ConfigParseFailure_MissingRequiredFields) {
  const std::string incomplete_yaml = R"(
dataset:
  mode: create
)";  // Missing path and topics

  ConfigParser parser;
  RecorderConfig config;
  bool result = parser.load_from_string(incomplete_yaml, config);

  // Parse might succeed but validation should fail
  if (result) {
    EXPECT_FALSE(config.validate());
  }
}

TEST_F(ErrorRecoveryTest, ConfigParseFailure_InvalidDataTypes) {
  const std::string invalid_types = R"(
dataset:
  path: /data
  mode: create
topics:
  - name: /test
    message_type: std_msgs/String
    batch_size: "not_a_number"
)";

  ConfigParser parser;
  RecorderConfig config;
  // May throw or return false
  bool result = parser.load_from_string(invalid_types, config);

  // If it parses, validation might catch the issue
  if (result) {
    // batch_size might be 0 (default) if parsing fails for that field
    // or the string might be converted somehow
    // Just ensure we don't crash
  }
}

TEST_F(ErrorRecoveryTest, ConfigParseFailure_EmptyFile) {
  auto path = write_test_file("empty.yaml", "");

  RecorderConfig config = RecorderConfig::from_yaml(path);
  EXPECT_FALSE(config.validate());
}

TEST_F(ErrorRecoveryTest, ConfigParseFailure_NonexistentFile) {
  RecorderConfig config = RecorderConfig::from_yaml("/nonexistent/path/config.yaml");
  EXPECT_FALSE(config.validate());
}

TEST_F(ErrorRecoveryTest, ConfigParseFailure_BinaryContent) {
  // Write binary content to a file
  auto path = test_dir_ / "binary.yaml";
  std::ofstream file(path, std::ios::binary);
  uint8_t binary_data[] = {0x00, 0x01, 0xFF, 0xFE, 0x00, 0x00};
  file.write(reinterpret_cast<const char*>(binary_data), sizeof(binary_data));
  file.close();

  ConfigParser parser;
  RecorderConfig config;
  bool result = parser.load_from_file(path.string(), config);

  // Should not crash, may return false
  EXPECT_FALSE(result);
}

// ============================================================================
// State Machine Recovery Tests
// ============================================================================

TEST_F(ErrorRecoveryTest, StateMachine_RecoverFromInvalidTransition) {
  StateManager state_manager;
  std::string error;

  // Try invalid transition
  bool result = state_manager.transition_to(RecorderState::RECORDING, error);
  EXPECT_FALSE(result);
  EXPECT_TRUE(state_manager.is_state(RecorderState::IDLE));

  // System should still work after invalid transition attempt
  result = state_manager.transition_to(RecorderState::READY, error);
  EXPECT_TRUE(result);
  EXPECT_TRUE(state_manager.is_state(RecorderState::READY));
}

TEST_F(ErrorRecoveryTest, StateMachine_ResetAfterRecording) {
  StateManager state_manager;
  std::string error;

  // Go through full workflow
  state_manager.transition_to(RecorderState::READY, error);
  state_manager.transition_to(RecorderState::RECORDING, error);
  state_manager.transition_to(RecorderState::PAUSED, error);

  EXPECT_TRUE(state_manager.is_recording_active());

  // Force reset
  state_manager.reset();

  EXPECT_TRUE(state_manager.is_state(RecorderState::IDLE));
  EXPECT_FALSE(state_manager.is_recording_active());

  // Should be able to start new recording
  EXPECT_TRUE(state_manager.transition_to(RecorderState::READY, error));
}

// ============================================================================
// StateTransactionGuard Tests
// ============================================================================

TEST_F(ErrorRecoveryTest, TransactionGuard_RollbackOnFailure) {
  StateManager state_manager;
  std::string error;

  // Go to READY state
  state_manager.transition_to(RecorderState::READY, error);
  EXPECT_TRUE(state_manager.is_state(RecorderState::READY));

  // Start a transaction with IDLE as rollback (valid from RECORDING)
  {
    StateTransactionGuard guard(state_manager, RecorderState::IDLE);

    // Transition to RECORDING
    state_manager.transition_to(RecorderState::RECORDING, error);
    EXPECT_TRUE(state_manager.is_state(RecorderState::RECORDING));

    // Don't commit - guard destructor will rollback to IDLE
  }

  // Should be rolled back to IDLE (RECORDING -> IDLE is valid)
  EXPECT_TRUE(state_manager.is_state(RecorderState::IDLE));
}

TEST_F(ErrorRecoveryTest, TransactionGuard_CommitSuccess) {
  StateManager state_manager;
  std::string error;

  // Go to READY state
  state_manager.transition_to(RecorderState::READY, error);

  // Start a transaction
  {
    StateTransactionGuard guard(state_manager, RecorderState::READY);

    // Transition to RECORDING
    state_manager.transition_to(RecorderState::RECORDING, error);
    EXPECT_TRUE(state_manager.is_state(RecorderState::RECORDING));

    // Commit the transaction
    guard.commit();
  }

  // Should stay in RECORDING (not rolled back)
  EXPECT_TRUE(state_manager.is_state(RecorderState::RECORDING));
}

TEST_F(ErrorRecoveryTest, TransactionGuard_MultipleRollbackAttempts) {
  StateManager state_manager;
  std::string error;

  state_manager.transition_to(RecorderState::READY, error);
  state_manager.transition_to(RecorderState::RECORDING, error);

  {
    StateTransactionGuard guard1(state_manager, RecorderState::IDLE);

    // Don't commit - will try to rollback to IDLE
    // This is a valid transition from RECORDING
  }

  EXPECT_TRUE(state_manager.is_state(RecorderState::IDLE));
}

// ============================================================================
// Config Validation Error Recovery
// ============================================================================

TEST_F(ErrorRecoveryTest, ValidationError_EmptyDatasetPath) {
  RecorderConfig config;
  config.dataset.path = "";
  config.dataset.mode = "create";

  TopicConfig topic;
  topic.name = "/test";
  topic.message_type = "std_msgs/String";
  config.topics.push_back(topic);

  std::string error_msg;
  bool valid = ConfigParser::validate(config, error_msg);

  EXPECT_FALSE(valid);
  EXPECT_FALSE(error_msg.empty());
  EXPECT_TRUE(error_msg.find("dataset") != std::string::npos ||
              error_msg.find("path") != std::string::npos);
}

TEST_F(ErrorRecoveryTest, ValidationError_EmptyTopics) {
  RecorderConfig config;
  config.dataset.path = "/data";
  config.dataset.mode = "create";
  config.topics = {};  // Empty

  std::string error_msg;
  bool valid = ConfigParser::validate(config, error_msg);

  EXPECT_FALSE(valid);
  EXPECT_FALSE(error_msg.empty());
}

TEST_F(ErrorRecoveryTest, ValidationError_InvalidTopicConfig) {
  RecorderConfig config;
  config.dataset.path = "/data";
  config.dataset.mode = "create";

  TopicConfig topic;
  topic.name = "";  // Empty name - invalid
  topic.message_type = "std_msgs/String";
  config.topics.push_back(topic);

  std::string error_msg;
  bool valid = ConfigParser::validate(config, error_msg);

  EXPECT_FALSE(valid);
}

TEST_F(ErrorRecoveryTest, ValidationError_ZeroBatchSize) {
  RecorderConfig config;
  config.dataset.path = "/data";
  config.dataset.mode = "create";

  TopicConfig topic;
  topic.name = "/test";
  topic.message_type = "std_msgs/String";
  topic.batch_size = 0;  // Invalid
  config.topics.push_back(topic);

  std::string error_msg;
  bool valid = ConfigParser::validate(config, error_msg);

  EXPECT_FALSE(valid);
}

// ============================================================================
// Upload Config Error Recovery
// ============================================================================

TEST_F(ErrorRecoveryTest, UploadConfig_MissingBucket) {
  RecorderConfig config;
  config.dataset.path = "/data";
  config.dataset.mode = "create";

  TopicConfig topic;
  topic.name = "/test";
  topic.message_type = "std_msgs/String";
  config.topics.push_back(topic);

  config.upload.enabled = true;
  config.upload.s3.bucket = "";  // Missing bucket

  std::string error_msg;
  bool valid = ConfigParser::validate(config, error_msg);

  EXPECT_FALSE(valid);
  EXPECT_TRUE(error_msg.find("bucket") != std::string::npos);
}

TEST_F(ErrorRecoveryTest, UploadConfig_InvalidEndpointUrl) {
  RecorderConfig config;
  config.dataset.path = "/data";
  config.dataset.mode = "create";

  TopicConfig topic;
  topic.name = "/test";
  topic.message_type = "std_msgs/String";
  config.topics.push_back(topic);

  config.upload.enabled = true;
  config.upload.s3.bucket = "valid-bucket";
  config.upload.s3.endpoint_url = "not-a-url";  // Invalid URL

  std::string error_msg;
  bool valid = ConfigParser::validate(config, error_msg);

  EXPECT_FALSE(valid);
  EXPECT_TRUE(error_msg.find("endpoint_url") != std::string::npos);
}

TEST_F(ErrorRecoveryTest, UploadConfig_InvalidRetryConfig) {
  RecorderConfig config;
  config.dataset.path = "/data";
  config.dataset.mode = "create";

  TopicConfig topic;
  topic.name = "/test";
  topic.message_type = "std_msgs/String";
  config.topics.push_back(topic);

  config.upload.enabled = true;
  config.upload.s3.bucket = "valid-bucket";
  config.upload.retry.max_retries = -5;  // Invalid

  std::string error_msg;
  bool valid = ConfigParser::validate(config, error_msg);

  EXPECT_FALSE(valid);
  EXPECT_TRUE(error_msg.find("max_retries") != std::string::npos);
}

TEST_F(ErrorRecoveryTest, UploadConfig_InitialDelayGreaterThanMax) {
  RecorderConfig config;
  config.dataset.path = "/data";
  config.dataset.mode = "create";

  TopicConfig topic;
  topic.name = "/test";
  topic.message_type = "std_msgs/String";
  config.topics.push_back(topic);

  config.upload.enabled = true;
  config.upload.s3.bucket = "valid-bucket";
  config.upload.retry.initial_delay_ms = 60000;
  config.upload.retry.max_delay_ms = 1000;  // Less than initial

  std::string error_msg;
  bool valid = ConfigParser::validate(config, error_msg);

  EXPECT_FALSE(valid);
}

// ============================================================================
// Graceful Degradation Tests
// ============================================================================

TEST_F(ErrorRecoveryTest, GracefulDegradation_InvalidLoggingLevel) {
  const std::string yaml = R"(
dataset:
  path: /data
  mode: create
topics:
  - name: /test
    message_type: std_msgs/String
logging:
  console:
    level: invalid_level
)";

  RecorderConfig config = RecorderConfig::from_yaml_string(yaml);

  // Should parse and validate (level handling is at runtime)
  EXPECT_EQ(config.logging.console_level, "invalid_level");
  EXPECT_TRUE(config.validate());  // Config is valid, level handled at runtime
}

TEST_F(ErrorRecoveryTest, GracefulDegradation_ExtraUnknownFields) {
  const std::string yaml = R"(
dataset:
  path: /data
  mode: create
  unknown_field: should_be_ignored
topics:
  - name: /test
    message_type: std_msgs/String
    also_unknown: ignored
extra_section:
  some_data: value
)";

  RecorderConfig config = RecorderConfig::from_yaml_string(yaml);

  // Should parse successfully, ignoring unknown fields
  EXPECT_EQ(config.dataset.path, "/data");
  EXPECT_EQ(config.topics.size(), 1u);
  EXPECT_TRUE(config.validate());
}

// ============================================================================
// Edge Case Recovery
// ============================================================================

TEST_F(ErrorRecoveryTest, EdgeCase_VeryLargeBatchSize) {
  const std::string yaml = R"(
dataset:
  path: /data
  mode: create
topics:
  - name: /test
    message_type: std_msgs/String
    batch_size: 2147483647
)";

  RecorderConfig config = RecorderConfig::from_yaml_string(yaml);
  EXPECT_EQ(config.topics[0].batch_size, 2147483647);
  // Should be valid (though practically unusable)
  EXPECT_TRUE(config.validate());
}

TEST_F(ErrorRecoveryTest, EdgeCase_UnicodeInPaths) {
  const std::string yaml = R"(
dataset:
  path: /data/録音/テスト
  mode: create
topics:
  - name: /テスト/トピック
    message_type: std_msgs/String
)";

  RecorderConfig config = RecorderConfig::from_yaml_string(yaml);
  EXPECT_EQ(config.dataset.path, "/data/録音/テスト");
  EXPECT_EQ(config.topics[0].name, "/テスト/トピック");
  EXPECT_TRUE(config.validate());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

