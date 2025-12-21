/**
 * @file test_recorder_context_interface.cpp
 * @brief Unit tests for IRecorderContext interface and MockRecorderContext
 *
 * These tests verify that:
 * 1. The IRecorderContext interface is correctly defined
 * 2. MockRecorderContext properly implements the interface
 * 3. Dependency injection pattern works correctly
 */

#include <gtest/gtest.h>

#include <memory>

#include "mock_recorder_context.hpp"
#include "recorder_context.hpp"

namespace axon {
namespace recorder {
namespace {

class RecorderContextInterfaceTest : public ::testing::Test {
protected:
  void SetUp() override {
    mock_ = std::make_shared<testing::MockRecorderContext>();
  }

  std::shared_ptr<testing::MockRecorderContext> mock_;
};

// ============================================================================
// Interface Verification Tests
// ============================================================================

TEST_F(RecorderContextInterfaceTest, MockImplementsInterface) {
  // Verify MockRecorderContext can be used as IRecorderContext
  std::shared_ptr<IRecorderContext> context = mock_;
  EXPECT_NE(context, nullptr);
}

TEST_F(RecorderContextInterfaceTest, StateManagerAccess) {
  std::shared_ptr<IRecorderContext> context = mock_;

  // Access state manager through interface
  auto& state_manager = context->get_state_manager();
  EXPECT_TRUE(state_manager.is_state(RecorderState::IDLE));

  // Modify state
  std::string error;
  EXPECT_TRUE(state_manager.transition_to(RecorderState::READY, error));
  EXPECT_TRUE(state_manager.is_state(RecorderState::READY));
}

TEST_F(RecorderContextInterfaceTest, TaskConfigCacheAccess) {
  std::shared_ptr<IRecorderContext> context = mock_;

  auto& cache = context->get_task_config_cache();

  // Initially empty
  EXPECT_FALSE(cache.get().has_value());

  // Cache a config
  TaskConfig config;
  config.task_id = "test_123";
  cache.cache(config);

  // Verify cached
  auto cached = cache.get();
  ASSERT_TRUE(cached.has_value());
  EXPECT_EQ(cached->task_id, "test_123");
}

TEST_F(RecorderContextInterfaceTest, HttpCallbackClientAccess) {
  std::shared_ptr<IRecorderContext> context = mock_;

  auto client = context->get_http_callback_client();
  EXPECT_NE(client, nullptr);
}

// ============================================================================
// Recording Operations Tests
// ============================================================================

TEST_F(RecorderContextInterfaceTest, StartRecording) {
  std::shared_ptr<IRecorderContext> context = mock_;

  EXPECT_FALSE(mock_->was_start_recording_called());
  EXPECT_TRUE(context->start_recording());
  EXPECT_TRUE(mock_->was_start_recording_called());
}

TEST_F(RecorderContextInterfaceTest, StopRecording) {
  std::shared_ptr<IRecorderContext> context = mock_;

  EXPECT_FALSE(mock_->was_stop_recording_called());
  context->stop_recording();
  EXPECT_TRUE(mock_->was_stop_recording_called());
}

TEST_F(RecorderContextInterfaceTest, PauseRecording) {
  std::shared_ptr<IRecorderContext> context = mock_;

  EXPECT_FALSE(mock_->was_pause_recording_called());
  context->pause_recording();
  EXPECT_TRUE(mock_->was_pause_recording_called());
}

TEST_F(RecorderContextInterfaceTest, ResumeRecording) {
  std::shared_ptr<IRecorderContext> context = mock_;

  EXPECT_FALSE(mock_->was_resume_recording_called());
  context->resume_recording();
  EXPECT_TRUE(mock_->was_resume_recording_called());
}

TEST_F(RecorderContextInterfaceTest, CancelRecording) {
  std::shared_ptr<IRecorderContext> context = mock_;

  EXPECT_FALSE(mock_->was_cancel_recording_called());
  context->cancel_recording();
  EXPECT_TRUE(mock_->was_cancel_recording_called());
}

TEST_F(RecorderContextInterfaceTest, ConfigureFromTaskConfig) {
  std::shared_ptr<IRecorderContext> context = mock_;

  TaskConfig config;
  config.task_id = "test";

  EXPECT_FALSE(mock_->was_configure_called());
  EXPECT_TRUE(context->configure_from_task_config(config));
  EXPECT_TRUE(mock_->was_configure_called());
}

// ============================================================================
// Status and Configuration Tests
// ============================================================================

TEST_F(RecorderContextInterfaceTest, GetOutputPath) {
  std::shared_ptr<IRecorderContext> context = mock_;

  mock_->set_output_path("/custom/path.mcap");
  EXPECT_EQ(context->get_output_path(), "/custom/path.mcap");
}

TEST_F(RecorderContextInterfaceTest, GetStats) {
  std::shared_ptr<IRecorderContext> context = mock_;

  RecordingStats stats;
  stats.messages_received = 100;
  stats.messages_written = 95;
  stats.messages_dropped = 5;
  stats.drop_rate_percent = 5.0;
  mock_->set_stats(stats);

  auto returned_stats = context->get_stats();
  EXPECT_EQ(returned_stats.messages_received, 100);
  EXPECT_EQ(returned_stats.messages_written, 95);
  EXPECT_EQ(returned_stats.messages_dropped, 5);
  EXPECT_DOUBLE_EQ(returned_stats.drop_rate_percent, 5.0);
}

TEST_F(RecorderContextInterfaceTest, GetRecordingDuration) {
  std::shared_ptr<IRecorderContext> context = mock_;

  mock_->set_duration_sec(123.456);
  EXPECT_DOUBLE_EQ(context->get_recording_duration_sec(), 123.456);
}

// ============================================================================
// Logging Tests
// ============================================================================

TEST_F(RecorderContextInterfaceTest, Logging) {
  std::shared_ptr<IRecorderContext> context = mock_;

  context->log_info("Info message");
  context->log_warn("Warning message");
  context->log_error("Error message");

  auto logs = mock_->get_log_messages();
  ASSERT_EQ(logs.size(), 3);
  EXPECT_EQ(logs[0], "INFO: Info message");
  EXPECT_EQ(logs[1], "WARN: Warning message");
  EXPECT_EQ(logs[2], "ERROR: Error message");
}

// ============================================================================
// Configurable Behavior Tests
// ============================================================================

TEST_F(RecorderContextInterfaceTest, ConfigurableStartRecordingResult) {
  std::shared_ptr<IRecorderContext> context = mock_;

  // Default is true
  EXPECT_TRUE(context->start_recording());

  // Configure to return false
  mock_->reset_call_flags();
  mock_->set_start_recording_result(false);
  EXPECT_FALSE(context->start_recording());
}

TEST_F(RecorderContextInterfaceTest, ConfigurableConfigureResult) {
  std::shared_ptr<IRecorderContext> context = mock_;

  TaskConfig config;

  // Default is true
  EXPECT_TRUE(context->configure_from_task_config(config));

  // Configure to return false
  mock_->reset_call_flags();
  mock_->set_configure_result(false);
  EXPECT_FALSE(context->configure_from_task_config(config));
}

// ============================================================================
// Reset Tests
// ============================================================================

TEST_F(RecorderContextInterfaceTest, ResetCallFlags) {
  std::shared_ptr<IRecorderContext> context = mock_;

  // Call all operations
  context->start_recording();
  context->stop_recording();
  context->pause_recording();
  context->resume_recording();
  context->cancel_recording();
  context->configure_from_task_config(TaskConfig{});
  context->log_info("test");

  // Verify all flags are set
  EXPECT_TRUE(mock_->was_start_recording_called());
  EXPECT_TRUE(mock_->was_stop_recording_called());
  EXPECT_TRUE(mock_->was_pause_recording_called());
  EXPECT_TRUE(mock_->was_resume_recording_called());
  EXPECT_TRUE(mock_->was_cancel_recording_called());
  EXPECT_TRUE(mock_->was_configure_called());
  EXPECT_FALSE(mock_->get_log_messages().empty());

  // Reset
  mock_->reset_call_flags();

  // Verify all flags are reset
  EXPECT_FALSE(mock_->was_start_recording_called());
  EXPECT_FALSE(mock_->was_stop_recording_called());
  EXPECT_FALSE(mock_->was_pause_recording_called());
  EXPECT_FALSE(mock_->was_resume_recording_called());
  EXPECT_FALSE(mock_->was_cancel_recording_called());
  EXPECT_FALSE(mock_->was_configure_called());
  EXPECT_TRUE(mock_->get_log_messages().empty());
}

// ============================================================================
// Dependency Injection Pattern Tests
// ============================================================================

/**
 * Simulates a service that depends on IRecorderContext.
 */
class TestService {
public:
  explicit TestService(std::shared_ptr<IRecorderContext> context)
      : context_(std::move(context)) {
    if (!context_) {
      throw std::invalid_argument("context cannot be null");
    }
  }

  bool start() {
    context_->log_info("Starting...");
    return context_->start_recording();
  }

  void stop() {
    context_->log_info("Stopping...");
    context_->stop_recording();
  }

  std::string get_status() {
    return context_->get_state_manager().get_state_string();
  }

private:
  std::shared_ptr<IRecorderContext> context_;
};

TEST_F(RecorderContextInterfaceTest, DependencyInjectionPattern) {
  // Create service with mock context
  TestService service(mock_);

  // Use service
  EXPECT_TRUE(service.start());
  EXPECT_TRUE(mock_->was_start_recording_called());

  service.stop();
  EXPECT_TRUE(mock_->was_stop_recording_called());

  EXPECT_EQ(service.get_status(), "idle");

  // Verify logging was called
  auto logs = mock_->get_log_messages();
  ASSERT_GE(logs.size(), 2);
  EXPECT_NE(logs[0].find("Starting"), std::string::npos);
  EXPECT_NE(logs[1].find("Stopping"), std::string::npos);
}

TEST_F(RecorderContextInterfaceTest, ServiceRejectsNullContext) {
  EXPECT_THROW(TestService(nullptr), std::invalid_argument);
}

}  // namespace
}  // namespace recorder
}  // namespace axon

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

