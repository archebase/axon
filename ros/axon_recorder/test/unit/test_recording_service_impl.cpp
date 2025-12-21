/**
 * @file test_recording_service_impl.cpp
 * @brief Unit tests for RecordingServiceImpl using MockRecorderContext
 *
 * These tests verify the service logic independently of RecorderNode,
 * demonstrating the testability benefits of the IRecorderContext interface.
 */

#include <gtest/gtest.h>

#include "mock_recorder_context.hpp"
#include "recording_service_impl.hpp"

namespace axon {
namespace recorder {
namespace {

class RecordingServiceImplTest : public ::testing::Test {
protected:
  void SetUp() override {
    mock_context_ = std::make_shared<testing::MockRecorderContext>();
    service_impl_ = std::make_unique<RecordingServiceImpl>(mock_context_);
  }

  void TearDown() override {
    service_impl_.reset();
    mock_context_.reset();
  }

  /**
   * Helper to cache a test configuration.
   */
  void cache_test_config() {
    bool success = false;
    std::string message;

    service_impl_->handle_cached_recording_config(
      "test_task_123",    // task_id
      "device_001",       // device_id
      "collector_001",    // data_collector_id
      "order_001",        // order_id
      "test.operator",    // operator_name
      "indoor",           // scene
      "kitchen",          // subscene
      {"cooking", "interaction"},  // skills
      "test_factory",     // factory
      {"/camera/image", "/imu/data"},  // topics
      "http://server/start",  // start_callback_url
      "http://server/finish", // finish_callback_url
      "jwt_token",        // user_token
      success, message
    );
  }

  std::shared_ptr<testing::MockRecorderContext> mock_context_;
  std::unique_ptr<RecordingServiceImpl> service_impl_;
};

// ============================================================================
// Constructor Tests
// ============================================================================

TEST_F(RecordingServiceImplTest, ConstructorRequiresContext) {
  EXPECT_THROW(RecordingServiceImpl(nullptr), std::invalid_argument);
}

// ============================================================================
// CachedRecordingConfig Service Tests
// ============================================================================

TEST_F(RecordingServiceImplTest, CacheConfigSuccess) {
  bool success = false;
  std::string message;

  bool result = service_impl_->handle_cached_recording_config(
    "task_123", "device_001", "collector_001", "", "",
    "indoor", "kitchen", {"skill1"},
    "test_factory", {"/topic1"}, "", "", "",
    success, message
  );

  EXPECT_TRUE(result);
  EXPECT_TRUE(success);
  EXPECT_NE(message.find("task_123"), std::string::npos);

  // Verify state transition to READY
  EXPECT_TRUE(mock_context_->get_state_manager().is_state(RecorderState::READY));

  // Verify config was cached
  auto config = mock_context_->get_task_config_cache().get();
  ASSERT_TRUE(config.has_value());
  EXPECT_EQ(config->task_id, "task_123");
  EXPECT_EQ(config->device_id, "device_001");
}

TEST_F(RecordingServiceImplTest, CacheConfigRequiresTaskId) {
  bool success = true;
  std::string message;

  service_impl_->handle_cached_recording_config(
    "",  // Empty task_id
    "device_001", "collector_001", "", "",
    "indoor", "kitchen", {},
    "test_factory", {}, "", "", "",
    success, message
  );

  EXPECT_FALSE(success);
  EXPECT_NE(message.find("task_id"), std::string::npos);
}

TEST_F(RecordingServiceImplTest, CacheConfigOnlyInIdleState) {
  // First, cache a config to get to READY state
  cache_test_config();
  EXPECT_TRUE(mock_context_->get_state_manager().is_state(RecorderState::READY));

  // Try to cache another config - should fail (already in READY state)
  bool success = true;
  std::string message;

  service_impl_->handle_cached_recording_config(
    "task_456", "device_002", "collector_002", "", "",
    "outdoor", "garden", {},
    "test_factory", {}, "", "", "",
    success, message
  );

  EXPECT_FALSE(success);
  // Error message indicates the current state blocking the operation ("ready")
  EXPECT_NE(message.find("ready"), std::string::npos);
}

// ============================================================================
// IsRecordingReady Service Tests
// ============================================================================

TEST_F(RecordingServiceImplTest, IsRecordingReadyWhenIdle) {
  bool success, is_configured, is_recording;
  std::string message, task_id, device_id, order_id, operator_name, scene, subscene, factory, data_collector_id;
  std::vector<std::string> skills, topics;

  service_impl_->handle_is_recording_ready(
    success, message, is_configured, is_recording,
    task_id, device_id, order_id, operator_name, scene, subscene, skills, factory,
    data_collector_id, topics
  );

  EXPECT_TRUE(success);
  EXPECT_FALSE(is_configured);
  EXPECT_FALSE(is_recording);
}

TEST_F(RecordingServiceImplTest, IsRecordingReadyWhenReady) {
  cache_test_config();

  bool success, is_configured, is_recording;
  std::string message, task_id, device_id, order_id, operator_name, scene, subscene, factory, data_collector_id;
  std::vector<std::string> skills, topics;

  service_impl_->handle_is_recording_ready(
    success, message, is_configured, is_recording,
    task_id, device_id, order_id, operator_name, scene, subscene, skills, factory,
    data_collector_id, topics
  );

  EXPECT_TRUE(success);
  EXPECT_TRUE(is_configured);
  EXPECT_FALSE(is_recording);
  EXPECT_EQ(task_id, "test_task_123");
  EXPECT_EQ(device_id, "device_001");
  EXPECT_EQ(order_id, "order_001");
  EXPECT_EQ(operator_name, "test.operator");
}

// ============================================================================
// RecordingControl Service Tests
// ============================================================================

TEST_F(RecordingServiceImplTest, StartCommand) {
  cache_test_config();

  bool success = false;
  std::string message, task_id_response;

  service_impl_->handle_recording_control(
    "start", "", success, message, task_id_response
  );

  EXPECT_TRUE(success);
  EXPECT_EQ(task_id_response, "test_task_123");
  EXPECT_TRUE(mock_context_->was_configure_called());
  EXPECT_TRUE(mock_context_->was_start_recording_called());
  EXPECT_TRUE(mock_context_->get_state_manager().is_state(RecorderState::RECORDING));
}

TEST_F(RecordingServiceImplTest, StartCommandFailsWhenNotReady) {
  // Don't cache config - state is IDLE

  bool success = true;
  std::string message, task_id_response;

  service_impl_->handle_recording_control(
    "start", "", success, message, task_id_response
  );

  EXPECT_FALSE(success);
  // Error message indicates the current state blocking the operation ("idle")
  EXPECT_NE(message.find("idle"), std::string::npos);
}

TEST_F(RecordingServiceImplTest, PauseCommand) {
  cache_test_config();

  // Start recording first
  bool success = false;
  std::string message, task_id_response;
  service_impl_->handle_recording_control("start", "", success, message, task_id_response);
  ASSERT_TRUE(success);

  mock_context_->reset_call_flags();

  // Now pause
  service_impl_->handle_recording_control(
    "pause", "test_task_123", success, message, task_id_response
  );

  EXPECT_TRUE(success);
  EXPECT_TRUE(mock_context_->was_pause_recording_called());
  EXPECT_TRUE(mock_context_->get_state_manager().is_state(RecorderState::PAUSED));
}

TEST_F(RecordingServiceImplTest, ResumeCommand) {
  cache_test_config();

  bool success = false;
  std::string message, task_id_response;

  // Start then pause
  service_impl_->handle_recording_control("start", "", success, message, task_id_response);
  service_impl_->handle_recording_control("pause", "test_task_123", success, message, task_id_response);
  ASSERT_TRUE(mock_context_->get_state_manager().is_state(RecorderState::PAUSED));

  mock_context_->reset_call_flags();

  // Now resume
  service_impl_->handle_recording_control(
    "resume", "test_task_123", success, message, task_id_response
  );

  EXPECT_TRUE(success);
  EXPECT_TRUE(mock_context_->was_resume_recording_called());
  EXPECT_TRUE(mock_context_->get_state_manager().is_state(RecorderState::RECORDING));
}

TEST_F(RecordingServiceImplTest, FinishCommand) {
  cache_test_config();

  bool success = false;
  std::string message, task_id_response;

  // Start recording
  service_impl_->handle_recording_control("start", "", success, message, task_id_response);
  ASSERT_TRUE(success);

  mock_context_->reset_call_flags();

  // Finish
  service_impl_->handle_recording_control(
    "finish", "test_task_123", success, message, task_id_response
  );

  EXPECT_TRUE(success);
  EXPECT_TRUE(mock_context_->was_stop_recording_called());
  EXPECT_TRUE(mock_context_->get_state_manager().is_state(RecorderState::IDLE));
}

TEST_F(RecordingServiceImplTest, CancelCommand) {
  cache_test_config();

  bool success = false;
  std::string message, task_id_response;

  // Start recording
  service_impl_->handle_recording_control("start", "", success, message, task_id_response);
  ASSERT_TRUE(success);

  mock_context_->reset_call_flags();

  // Cancel
  service_impl_->handle_recording_control(
    "cancel", "test_task_123", success, message, task_id_response
  );

  EXPECT_TRUE(success);
  EXPECT_TRUE(mock_context_->was_cancel_recording_called());
  EXPECT_TRUE(mock_context_->get_state_manager().is_state(RecorderState::IDLE));
}

TEST_F(RecordingServiceImplTest, ClearCommand) {
  cache_test_config();
  ASSERT_TRUE(mock_context_->get_state_manager().is_state(RecorderState::READY));

  bool success = false;
  std::string message, task_id_response;

  service_impl_->handle_recording_control(
    "clear", "", success, message, task_id_response
  );

  EXPECT_TRUE(success);
  EXPECT_TRUE(mock_context_->get_state_manager().is_state(RecorderState::IDLE));
  EXPECT_FALSE(mock_context_->get_task_config_cache().get().has_value());
}

TEST_F(RecordingServiceImplTest, UnknownCommand) {
  bool success = true;
  std::string message, task_id_response;

  service_impl_->handle_recording_control(
    "unknown_command", "", success, message, task_id_response
  );

  EXPECT_FALSE(success);
  EXPECT_NE(message.find("Unknown command"), std::string::npos);
}

TEST_F(RecordingServiceImplTest, TaskIdValidation) {
  cache_test_config();

  bool success = false;
  std::string message, task_id_response;

  // Start recording
  service_impl_->handle_recording_control("start", "", success, message, task_id_response);
  ASSERT_TRUE(success);

  // Try to pause with wrong task_id
  service_impl_->handle_recording_control(
    "pause", "wrong_task_id", success, message, task_id_response
  );

  EXPECT_FALSE(success);
  EXPECT_NE(message.find("does not match"), std::string::npos);
}

// ============================================================================
// RecordingStatus Service Tests
// ============================================================================

TEST_F(RecordingServiceImplTest, StatusWhenIdle) {
  bool success;
  std::string message, status, task_id, device_id, data_collector_id, order_id, operator_name;
  std::string scene, subscene, factory, output_path, last_error;
  std::vector<std::string> skills, active_topics;
  double disk_usage_gb, duration_sec, throughput_mb_sec;
  int64_t message_count;

  service_impl_->handle_recording_status(
    "", success, message, status, task_id, device_id, data_collector_id,
    order_id, operator_name, scene, subscene, skills, factory, active_topics, output_path,
    disk_usage_gb, duration_sec, message_count, throughput_mb_sec, last_error
  );

  EXPECT_TRUE(success);
  EXPECT_EQ(status, "idle");
  EXPECT_TRUE(task_id.empty());
}

TEST_F(RecordingServiceImplTest, StatusWhenRecording) {
  cache_test_config();

  bool success = false;
  std::string message, task_id_response;
  service_impl_->handle_recording_control("start", "", success, message, task_id_response);
  ASSERT_TRUE(success);

  // Set some stats
  RecordingStats stats;
  stats.messages_written = 100;
  mock_context_->set_stats(stats);
  mock_context_->set_duration_sec(10.5);

  std::string status, task_id, device_id, data_collector_id, order_id, operator_name;
  std::string scene, subscene, factory, output_path, last_error;
  std::vector<std::string> skills, active_topics;
  double disk_usage_gb, duration_sec, throughput_mb_sec;
  int64_t message_count;

  service_impl_->handle_recording_status(
    "test_task_123", success, message, status, task_id, device_id, data_collector_id,
    order_id, operator_name, scene, subscene, skills, factory, active_topics, output_path,
    disk_usage_gb, duration_sec, message_count, throughput_mb_sec, last_error
  );

  EXPECT_TRUE(success);
  EXPECT_EQ(status, "recording");
  EXPECT_EQ(task_id, "test_task_123");
  EXPECT_EQ(device_id, "device_001");
  EXPECT_EQ(order_id, "order_001");
  EXPECT_EQ(operator_name, "test.operator");
  EXPECT_EQ(message_count, 100);
  EXPECT_DOUBLE_EQ(duration_sec, 10.5);
}

// ============================================================================
// Error Handling Tests
// ============================================================================

TEST_F(RecordingServiceImplTest, StartRecordingFailure) {
  cache_test_config();

  // Configure mock to fail start_recording
  mock_context_->set_configure_result(false);

  bool success = true;
  std::string message, task_id_response;

  service_impl_->handle_recording_control(
    "start", "", success, message, task_id_response
  );

  EXPECT_FALSE(success);
  // State should NOT have transitioned to RECORDING due to configure failure
}

}  // namespace
}  // namespace recorder
}  // namespace axon

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

