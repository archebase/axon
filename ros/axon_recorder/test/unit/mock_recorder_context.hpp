/**
 * @file mock_recorder_context.hpp
 * @brief Mock implementation of IRecorderContext for testing
 */

#ifndef AXON_RECORDER_MOCK_RECORDER_CONTEXT_HPP
#define AXON_RECORDER_MOCK_RECORDER_CONTEXT_HPP

#include <memory>
#include <string>
#include <vector>

#include "http_callback_client.hpp"
#include "recorder_context.hpp"
#include "state_machine.hpp"
#include "task_config.hpp"

namespace axon {
namespace recorder {
namespace testing {

/**
 * Mock implementation of IRecorderContext for unit testing.
 *
 * This allows testing RecordingServiceImpl and ServiceAdapter
 * without needing a full RecorderNode instance.
 */
class MockRecorderContext : public IRecorderContext,
                            public std::enable_shared_from_this<MockRecorderContext> {
public:
  MockRecorderContext()
      : http_callback_client_(std::make_shared<HttpCallbackClient>()) {}

  // =========================================================================
  // State Management
  // =========================================================================

  StateManager& get_state_manager() override {
    return state_manager_;
  }

  TaskConfigCache& get_task_config_cache() override {
    return task_config_cache_;
  }

  std::shared_ptr<HttpCallbackClient> get_http_callback_client() override {
    return http_callback_client_;
  }

  // =========================================================================
  // Recording Operations
  // =========================================================================

  bool start_recording() override {
    start_recording_called_ = true;
    return start_recording_result_;
  }

  void stop_recording() override {
    stop_recording_called_ = true;
  }

  void pause_recording() override {
    pause_recording_called_ = true;
  }

  void resume_recording() override {
    resume_recording_called_ = true;
  }

  void cancel_recording() override {
    cancel_recording_called_ = true;
  }

  // =========================================================================
  // Configuration and Status
  // =========================================================================

  bool configure_from_task_config(const TaskConfig& /* config */) override {
    configure_called_ = true;
    return configure_result_;
  }

  std::string get_output_path() const override {
    return output_path_;
  }

  RecordingStats get_stats() const override {
    return stats_;
  }

  double get_recording_duration_sec() const override {
    return duration_sec_;
  }

  // =========================================================================
  // Logging
  // =========================================================================

  void log_info(const std::string& msg) override {
    log_messages_.push_back("INFO: " + msg);
  }

  void log_warn(const std::string& msg) override {
    log_messages_.push_back("WARN: " + msg);
  }

  void log_error(const std::string& msg) override {
    log_messages_.push_back("ERROR: " + msg);
  }

  // =========================================================================
  // Test Configuration
  // =========================================================================

  void set_start_recording_result(bool result) {
    start_recording_result_ = result;
  }

  void set_configure_result(bool result) {
    configure_result_ = result;
  }

  void set_output_path(const std::string& path) {
    output_path_ = path;
  }

  void set_stats(const RecordingStats& stats) {
    stats_ = stats;
  }

  void set_duration_sec(double duration) {
    duration_sec_ = duration;
  }

  // =========================================================================
  // Test Verification
  // =========================================================================

  bool was_start_recording_called() const { return start_recording_called_; }
  bool was_stop_recording_called() const { return stop_recording_called_; }
  bool was_pause_recording_called() const { return pause_recording_called_; }
  bool was_resume_recording_called() const { return resume_recording_called_; }
  bool was_cancel_recording_called() const { return cancel_recording_called_; }
  bool was_configure_called() const { return configure_called_; }

  const std::vector<std::string>& get_log_messages() const { return log_messages_; }

  void reset_call_flags() {
    start_recording_called_ = false;
    stop_recording_called_ = false;
    pause_recording_called_ = false;
    resume_recording_called_ = false;
    cancel_recording_called_ = false;
    configure_called_ = false;
    log_messages_.clear();
  }

private:
  StateManager state_manager_;
  TaskConfigCache task_config_cache_;
  std::shared_ptr<HttpCallbackClient> http_callback_client_;

  // Configurable results
  bool start_recording_result_ = true;
  bool configure_result_ = true;
  std::string output_path_ = "/test/output.mcap";
  RecordingStats stats_;
  double duration_sec_ = 0.0;

  // Call tracking
  bool start_recording_called_ = false;
  bool stop_recording_called_ = false;
  bool pause_recording_called_ = false;
  bool resume_recording_called_ = false;
  bool cancel_recording_called_ = false;
  bool configure_called_ = false;
  std::vector<std::string> log_messages_;
};

}  // namespace testing
}  // namespace recorder
}  // namespace axon

#endif  // AXON_RECORDER_MOCK_RECORDER_CONTEXT_HPP

