/**
 * Integration tests for the Recording Service workflow
 *
 * These tests verify the complete workflow from service calls to state transitions
 * without requiring ROS to be running.
 *
 * Note: This test uses MockRecorderNode and inline service implementation
 * to avoid dependencies on the full recorder infrastructure.
 */

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "http_callback_client.hpp"
#include "state_machine.hpp"
#include "task_config.hpp"

using namespace axon::recorder;

// ============================================================================
// Mock RecorderNode for testing
// ============================================================================

/**
 * MockRecorderNode provides minimal implementation for testing service logic.
 * It tracks method calls and state for verification.
 */
class MockRecorderNode {
public:
  MockRecorderNode() = default;

  StateManager& get_state_manager() {
    return state_manager_;
  }
  TaskConfigCache& get_task_config_cache() {
    return task_config_cache_;
  }
  HttpCallbackClient& get_http_callback_client() {
    return http_callback_client_;
  }

  bool configure_from_task_config(const TaskConfig& config) {
    configure_called_ = true;
    last_configured_task_id_ = config.task_id;
    output_path_ = "/data/recordings/" + config.generate_output_filename();
    return true;
  }

  void start_recording() {
    start_called_ = true;
    recording_ = true;
    paused_ = false;
  }

  void pause_recording() {
    pause_called_ = true;
    paused_ = true;
  }

  void resume_recording() {
    resume_called_ = true;
    paused_ = false;
  }

  void stop_recording() {
    stop_called_ = true;
    recording_ = false;
    paused_ = false;
  }

  void cancel_recording() {
    cancel_called_ = true;
    recording_ = false;
    paused_ = false;
    task_config_cache_.clear();
  }

  std::string get_output_path() const {
    return output_path_;
  }

  struct RecordingStats {
    uint64_t messages_received = 0;
    uint64_t messages_dropped = 0;
    uint64_t messages_written = 100;
    double drop_rate_percent = 0.0;
  };

  RecordingStats get_stats() const {
    return stats_;
  }

  // Verification methods
  bool was_configure_called() const {
    return configure_called_;
  }
  bool was_start_called() const {
    return start_called_;
  }
  bool was_pause_called() const {
    return pause_called_;
  }
  bool was_resume_called() const {
    return resume_called_;
  }
  bool was_stop_called() const {
    return stop_called_;
  }
  bool was_cancel_called() const {
    return cancel_called_;
  }

  std::string get_last_configured_task_id() const {
    return last_configured_task_id_;
  }

  void reset_tracking() {
    configure_called_ = false;
    start_called_ = false;
    pause_called_ = false;
    resume_called_ = false;
    stop_called_ = false;
    cancel_called_ = false;
    last_configured_task_id_.clear();
  }

private:
  StateManager state_manager_;
  TaskConfigCache task_config_cache_;
  HttpCallbackClient http_callback_client_;

  std::string output_path_;
  RecordingStats stats_;

  bool recording_ = false;
  bool paused_ = false;

  // Tracking
  bool configure_called_ = false;
  bool start_called_ = false;
  bool pause_called_ = false;
  bool resume_called_ = false;
  bool stop_called_ = false;
  bool cancel_called_ = false;
  std::string last_configured_task_id_;
};

// ============================================================================
// Mock Service Implementation (inline for testing)
// ============================================================================

/**
 * MockRecordingServiceImpl - inline implementation for testing
 */
class MockRecordingServiceImpl {
public:
  explicit MockRecordingServiceImpl(MockRecorderNode* node)
      : node_(node) {}

  bool handle_cached_recording_config(
    const std::string& task_id, const std::string& device_id, const std::string& data_collector_id,
    const std::string& scene, const std::string& subscene, const std::vector<std::string>& skills,
    const std::string& organization, const std::vector<std::string>& topics,
    const std::string& start_callback_url, const std::string& finish_callback_url,
    const std::string& user_token,
    bool& success, std::string& message
  ) {
    success = false;

    if (task_id.empty()) {
      message = "ERR_INVALID_COMMAND: task_id is required";
      return true;
    }

    auto& state_manager = node_->get_state_manager();
    if (!state_manager.is_state(RecorderState::IDLE)) {
      message = "ERR_INVALID_STATE: Cannot cache config in state " + state_manager.get_state_string();
      return true;
    }

    TaskConfig config;
    config.task_id = task_id;
    config.device_id = device_id;
    config.data_collector_id = data_collector_id;
    config.scene = scene;
    config.subscene = subscene;
    config.skills = skills;
    config.organization = organization;
    config.topics = topics;
    config.start_callback_url = start_callback_url;
    config.finish_callback_url = finish_callback_url;
    config.user_token = user_token;

    node_->get_task_config_cache().cache(config);

    std::string error_msg;
    if (!state_manager.transition_to(RecorderState::READY, error_msg)) {
      node_->get_task_config_cache().clear();
      message = error_msg;
      return true;
    }

    success = true;
    message = "Config cached for task: " + task_id;
    return true;
  }

  bool handle_is_recording_ready(
    bool& success, std::string& message, bool& is_configured, bool& is_recording,
    std::string& task_id, std::string& device_id, std::string& scene, std::string& subscene,
    std::vector<std::string>& skills, std::string& organization, std::string& data_collector_id,
    std::vector<std::string>& topics
  ) {
    success = true;
    message = "OK";

    auto& state_manager = node_->get_state_manager();
    auto& task_cache = node_->get_task_config_cache();

    RecorderState state = state_manager.get_state();
    is_configured = (state == RecorderState::READY || state == RecorderState::RECORDING ||
                     state == RecorderState::PAUSED);
    is_recording = (state == RecorderState::RECORDING || state == RecorderState::PAUSED);

    auto config_opt = task_cache.get();
    if (config_opt) {
      task_id = config_opt->task_id;
      device_id = config_opt->device_id;
      scene = config_opt->scene;
      subscene = config_opt->subscene;
      skills = config_opt->skills;
      organization = config_opt->organization;
      data_collector_id = config_opt->data_collector_id;
      topics = config_opt->topics;
    } else {
      task_id.clear();
      device_id.clear();
      scene.clear();
      subscene.clear();
      skills.clear();
      organization.clear();
      data_collector_id.clear();
      topics.clear();
    }

    return true;
  }

  bool handle_recording_control(
    const std::string& command, const std::string& task_id_request, bool& success, std::string& message,
    std::string& task_id_response
  ) {
    success = false;

    auto config_opt = node_->get_task_config_cache().get();
    task_id_response = config_opt ? config_opt->task_id : "";

    if (command == "start") {
      success = handle_start_command(message, task_id_response);
    } else if (command == "pause") {
      success = handle_pause_command(task_id_request, message);
    } else if (command == "resume") {
      success = handle_resume_command(task_id_request, message);
    } else if (command == "cancel") {
      success = handle_cancel_command(task_id_request, message);
    } else if (command == "finish") {
      success = handle_finish_command(task_id_request, message);
    } else if (command == "clear") {
      success = handle_clear_command(message);
    } else {
      message = "ERR_INVALID_COMMAND: Unknown command: " + command;
    }

    return true;
  }

  bool handle_recording_status(
    const std::string& task_id_request, bool& success, std::string& message, std::string& status,
    std::string& task_id, std::string& device_id, std::string& data_collector_id, std::string& scene,
    std::string& subscene, std::vector<std::string>& skills, std::string& organization,
    std::vector<std::string>& active_topics, std::string& output_path, double& disk_usage_gb,
    double& duration_sec, int64_t& message_count, double& throughput_mb_sec, std::string& last_error
  ) {
    success = true;
    message = "OK";

    auto& state_manager = node_->get_state_manager();
    status = state_manager.get_state_string();

    auto config_opt = node_->get_task_config_cache().get();
    if (config_opt) {
      task_id = config_opt->task_id;
      device_id = config_opt->device_id;
      data_collector_id = config_opt->data_collector_id;
      scene = config_opt->scene;
      subscene = config_opt->subscene;
      skills = config_opt->skills;
      organization = config_opt->organization;
      active_topics = config_opt->topics;
    } else {
      task_id.clear();
      device_id.clear();
      data_collector_id.clear();
      scene.clear();
      subscene.clear();
      skills.clear();
      organization.clear();
      active_topics.clear();
    }

    auto stats = node_->get_stats();
    output_path = node_->get_output_path();
    message_count = static_cast<int64_t>(stats.messages_written);
    last_error = "";
    duration_sec = 0.0;
    disk_usage_gb = 0.0;
    throughput_mb_sec = 0.0;

    return true;
  }

private:
  bool handle_start_command(std::string& message, std::string& task_id_response) {
    auto& state_manager = node_->get_state_manager();

    if (!state_manager.is_state(RecorderState::READY)) {
      message = "ERR_INVALID_STATE: Cannot start recording in state " + state_manager.get_state_string();
      return false;
    }

    auto config_opt = node_->get_task_config_cache().get();
    if (!config_opt) {
      message = "ERR_CONFIG_NOT_FOUND: No configuration cached";
      return false;
    }

    task_id_response = config_opt->task_id;

    if (!node_->configure_from_task_config(*config_opt)) {
      message = "ERR_INVALID_STATE: Failed to configure from task config";
      return false;
    }

    std::string error_msg;
    if (!state_manager.transition_to(RecorderState::RECORDING, error_msg)) {
      message = error_msg;
      return false;
    }

    node_->start_recording();
    message = "Recording started for task: " + task_id_response;
    return true;
  }

  bool handle_pause_command(const std::string& task_id, std::string& message) {
    auto& state_manager = node_->get_state_manager();

    std::string error_msg;
    if (!validate_task_id(task_id, error_msg)) {
      message = error_msg;
      return false;
    }

    if (!state_manager.is_state(RecorderState::RECORDING)) {
      message = "ERR_INVALID_STATE: Cannot pause in state " + state_manager.get_state_string();
      return false;
    }

    if (!state_manager.transition_to(RecorderState::PAUSED, error_msg)) {
      message = error_msg;
      return false;
    }

    node_->pause_recording();
    message = "Recording paused";
    return true;
  }

  bool handle_resume_command(const std::string& task_id, std::string& message) {
    auto& state_manager = node_->get_state_manager();

    std::string error_msg;
    if (!validate_task_id(task_id, error_msg)) {
      message = error_msg;
      return false;
    }

    if (!state_manager.is_state(RecorderState::PAUSED)) {
      message = "ERR_INVALID_STATE: Cannot resume in state " + state_manager.get_state_string();
      return false;
    }

    if (!state_manager.transition_to(RecorderState::RECORDING, error_msg)) {
      message = error_msg;
      return false;
    }

    node_->resume_recording();
    message = "Recording resumed";
    return true;
  }

  bool handle_cancel_command(const std::string& task_id, std::string& message) {
    auto& state_manager = node_->get_state_manager();

    std::string error_msg;
    if (!validate_task_id(task_id, error_msg)) {
      message = error_msg;
      return false;
    }

    if (!state_manager.is_recording_active()) {
      message = "ERR_INVALID_STATE: Cannot cancel in state " + state_manager.get_state_string();
      return false;
    }

    node_->cancel_recording();

    if (!state_manager.transition_to(RecorderState::IDLE, error_msg)) {
      message = error_msg;
      return false;
    }

    message = "Recording cancelled";
    return true;
  }

  bool handle_finish_command(const std::string& task_id, std::string& message) {
    auto& state_manager = node_->get_state_manager();

    std::string error_msg;
    if (!validate_task_id(task_id, error_msg)) {
      message = error_msg;
      return false;
    }

    if (!state_manager.is_recording_active()) {
      message = "ERR_INVALID_STATE: Cannot finish in state " + state_manager.get_state_string();
      return false;
    }

    node_->stop_recording();

    if (!state_manager.transition_to(RecorderState::IDLE, error_msg)) {
      message = error_msg;
      return false;
    }

    message = "Recording finished";
    return true;
  }

  bool handle_clear_command(std::string& message) {
    auto& state_manager = node_->get_state_manager();

    if (!state_manager.is_state(RecorderState::READY)) {
      message = "ERR_INVALID_STATE: Cannot clear in state " + state_manager.get_state_string();
      return false;
    }

    node_->get_task_config_cache().clear();

    std::string error_msg;
    if (!state_manager.transition_to(RecorderState::IDLE, error_msg)) {
      message = error_msg;
      return false;
    }

    message = "Config cleared";
    return true;
  }

  bool validate_task_id(const std::string& task_id, std::string& error_msg) {
    if (task_id.empty()) {
      error_msg = "ERR_INVALID_COMMAND: task_id is required";
      return false;
    }

    auto& task_cache = node_->get_task_config_cache();
    if (!task_cache.matches_task_id(task_id)) {
      error_msg = "ERR_RECORDING_NOT_FOUND: task_id does not match current task";
      return false;
    }

    return true;
  }

  MockRecorderNode* node_;
};

// ============================================================================
// Test Fixtures
// ============================================================================

class RecordingServiceImplTest : public ::testing::Test {
protected:
  void SetUp() override {
    node_ = std::make_unique<MockRecorderNode>();
    service_ = std::make_unique<MockRecordingServiceImpl>(node_.get());
  }

  std::unique_ptr<MockRecorderNode> node_;
  std::unique_ptr<MockRecordingServiceImpl> service_;
};

// ============================================================================
// CachedRecordingConfig Tests
// ============================================================================

TEST_F(RecordingServiceImplTest, CachedRecordingConfig_Success) {
  bool success;
  std::string message;

  bool result = service_->handle_cached_recording_config(
    "task_001",       // task_id
    "robot_01",       // device_id
    "collector_01",   // data_collector_id
    "warehouse",      // scene
    "picking",        // subscene
    {"nav", "manip"}, // skills
    "test_org",       // organization
    {"/camera"},      // topics
    "",               // start_callback_url
    "",               // finish_callback_url
    "",               // user_token
    success, message
  );

  EXPECT_TRUE(result);
  EXPECT_TRUE(success);
  EXPECT_FALSE(message.empty());

  EXPECT_EQ(node_->get_state_manager().get_state(), RecorderState::READY);
  EXPECT_TRUE(node_->get_task_config_cache().has_config());
  auto config = node_->get_task_config_cache().get();
  ASSERT_TRUE(config.has_value());
  EXPECT_EQ(config->task_id, "task_001");
  EXPECT_EQ(config->scene, "warehouse");
  EXPECT_EQ(config->subscene, "picking");
}

TEST_F(RecordingServiceImplTest, CachedRecordingConfig_EmptyTaskId) {
  bool success;
  std::string message;

  bool result = service_->handle_cached_recording_config(
    "",               // Empty task_id
    "robot_01", "collector_01", "warehouse", "picking", {}, "test_org",
    {}, "", "", "", success, message
  );

  EXPECT_TRUE(result);
  EXPECT_FALSE(success);
  EXPECT_TRUE(message.find("ERR_INVALID_COMMAND") != std::string::npos);
  EXPECT_EQ(node_->get_state_manager().get_state(), RecorderState::IDLE);
}

TEST_F(RecordingServiceImplTest, CachedRecordingConfig_NotInIdleState) {
  bool success;
  std::string message;

  service_->handle_cached_recording_config(
    "task_001", "robot_01", "collector_01", "warehouse", "picking", {},
    "test_org", {}, "", "", "", success, message
  );
  ASSERT_TRUE(success);

  bool result = service_->handle_cached_recording_config(
    "task_002", "robot_01", "collector_01", "warehouse", "picking", {},
    "test_org", {}, "", "", "", success, message
  );

  EXPECT_TRUE(result);
  EXPECT_FALSE(success);
  EXPECT_TRUE(message.find("ERR_INVALID_STATE") != std::string::npos);
}

// ============================================================================
// IsRecordingReady Tests
// ============================================================================

TEST_F(RecordingServiceImplTest, IsRecordingReady_NotConfigured) {
  bool success, is_configured, is_recording;
  std::string message, task_id, device_id, scene, subscene, org, data_collector;
  std::vector<std::string> skills, topics;

  bool result = service_->handle_is_recording_ready(
    success, message, is_configured, is_recording, task_id, device_id,
    scene, subscene, skills, org, data_collector, topics
  );

  EXPECT_TRUE(result);
  EXPECT_TRUE(success);
  EXPECT_FALSE(is_configured);
  EXPECT_FALSE(is_recording);
  EXPECT_TRUE(task_id.empty());
}

TEST_F(RecordingServiceImplTest, IsRecordingReady_Configured) {
  bool success;
  std::string message;
  service_->handle_cached_recording_config(
    "task_001", "robot_01", "collector_01", "warehouse", "picking",
    {"skill1", "skill2"}, "test_org", {"/camera", "/lidar"}, "", "", "",
    success, message
  );
  ASSERT_TRUE(success);

  bool is_configured, is_recording;
  std::string task_id, device_id, scene, subscene, org, data_collector;
  std::vector<std::string> skills, topics;

  bool result = service_->handle_is_recording_ready(
    success, message, is_configured, is_recording, task_id, device_id,
    scene, subscene, skills, org, data_collector, topics
  );

  EXPECT_TRUE(result);
  EXPECT_TRUE(success);
  EXPECT_TRUE(is_configured);
  EXPECT_FALSE(is_recording);
  EXPECT_EQ(task_id, "task_001");
  EXPECT_EQ(device_id, "robot_01");
  EXPECT_EQ(scene, "warehouse");
  EXPECT_EQ(subscene, "picking");
  EXPECT_EQ(skills.size(), 2);
  EXPECT_EQ(topics.size(), 2);
}

// ============================================================================
// RecordingControl Tests
// ============================================================================

TEST_F(RecordingServiceImplTest, RecordingControl_Start_Success) {
  bool success;
  std::string message, task_id_response;

  service_->handle_cached_recording_config(
    "task_001", "robot_01", "collector_01", "warehouse", "picking", {},
    "test_org", {}, "", "", "", success, message
  );
  ASSERT_TRUE(success);

  node_->reset_tracking();

  bool result = service_->handle_recording_control(
    "start", "", success, message, task_id_response
  );

  EXPECT_TRUE(result);
  EXPECT_TRUE(success);
  EXPECT_EQ(task_id_response, "task_001");
  EXPECT_EQ(node_->get_state_manager().get_state(), RecorderState::RECORDING);
  EXPECT_TRUE(node_->was_configure_called());
  EXPECT_TRUE(node_->was_start_called());
}

TEST_F(RecordingServiceImplTest, RecordingControl_Start_NotReady) {
  bool success;
  std::string message, task_id_response;

  bool result = service_->handle_recording_control(
    "start", "", success, message, task_id_response
  );

  EXPECT_TRUE(result);
  EXPECT_FALSE(success);
  EXPECT_TRUE(message.find("ERR_INVALID_STATE") != std::string::npos);
}

TEST_F(RecordingServiceImplTest, RecordingControl_Pause_Success) {
  bool success;
  std::string message, task_id_response;

  service_->handle_cached_recording_config(
    "task_001", "robot_01", "collector_01", "warehouse", "picking", {},
    "test_org", {}, "", "", "", success, message
  );
  service_->handle_recording_control("start", "", success, message, task_id_response);
  ASSERT_TRUE(success);

  node_->reset_tracking();

  bool result = service_->handle_recording_control(
    "pause", "task_001", success, message, task_id_response
  );

  EXPECT_TRUE(result);
  EXPECT_TRUE(success);
  EXPECT_EQ(node_->get_state_manager().get_state(), RecorderState::PAUSED);
  EXPECT_TRUE(node_->was_pause_called());
}

TEST_F(RecordingServiceImplTest, RecordingControl_Pause_WrongTaskId) {
  bool success;
  std::string message, task_id_response;

  service_->handle_cached_recording_config(
    "task_001", "robot_01", "collector_01", "warehouse", "picking", {},
    "test_org", {}, "", "", "", success, message
  );
  service_->handle_recording_control("start", "", success, message, task_id_response);
  ASSERT_TRUE(success);

  bool result = service_->handle_recording_control(
    "pause", "wrong_task_id", success, message, task_id_response
  );

  EXPECT_TRUE(result);
  EXPECT_FALSE(success);
  EXPECT_TRUE(message.find("ERR_RECORDING_NOT_FOUND") != std::string::npos);
}

TEST_F(RecordingServiceImplTest, RecordingControl_Resume_Success) {
  bool success;
  std::string message, task_id_response;

  service_->handle_cached_recording_config(
    "task_001", "robot_01", "collector_01", "warehouse", "picking", {},
    "test_org", {}, "", "", "", success, message
  );
  service_->handle_recording_control("start", "", success, message, task_id_response);
  service_->handle_recording_control("pause", "task_001", success, message, task_id_response);
  ASSERT_TRUE(success);

  node_->reset_tracking();

  bool result = service_->handle_recording_control(
    "resume", "task_001", success, message, task_id_response
  );

  EXPECT_TRUE(result);
  EXPECT_TRUE(success);
  EXPECT_EQ(node_->get_state_manager().get_state(), RecorderState::RECORDING);
  EXPECT_TRUE(node_->was_resume_called());
}

TEST_F(RecordingServiceImplTest, RecordingControl_Finish_Success) {
  bool success;
  std::string message, task_id_response;

  service_->handle_cached_recording_config(
    "task_001", "robot_01", "collector_01", "warehouse", "picking", {},
    "test_org", {}, "", "", "", success, message
  );
  service_->handle_recording_control("start", "", success, message, task_id_response);
  ASSERT_TRUE(success);

  node_->reset_tracking();

  bool result = service_->handle_recording_control(
    "finish", "task_001", success, message, task_id_response
  );

  EXPECT_TRUE(result);
  EXPECT_TRUE(success);
  EXPECT_EQ(node_->get_state_manager().get_state(), RecorderState::IDLE);
  EXPECT_TRUE(node_->was_stop_called());
}

TEST_F(RecordingServiceImplTest, RecordingControl_Cancel_Success) {
  bool success;
  std::string message, task_id_response;

  service_->handle_cached_recording_config(
    "task_001", "robot_01", "collector_01", "warehouse", "picking", {},
    "test_org", {}, "", "", "", success, message
  );
  service_->handle_recording_control("start", "", success, message, task_id_response);
  ASSERT_TRUE(success);

  node_->reset_tracking();

  bool result = service_->handle_recording_control(
    "cancel", "task_001", success, message, task_id_response
  );

  EXPECT_TRUE(result);
  EXPECT_TRUE(success);
  EXPECT_EQ(node_->get_state_manager().get_state(), RecorderState::IDLE);
  EXPECT_TRUE(node_->was_cancel_called());
}

TEST_F(RecordingServiceImplTest, RecordingControl_Clear_Success) {
  bool success;
  std::string message, task_id_response;

  service_->handle_cached_recording_config(
    "task_001", "robot_01", "collector_01", "warehouse", "picking", {},
    "test_org", {}, "", "", "", success, message
  );
  ASSERT_TRUE(success);
  ASSERT_EQ(node_->get_state_manager().get_state(), RecorderState::READY);

  bool result = service_->handle_recording_control(
    "clear", "", success, message, task_id_response
  );

  EXPECT_TRUE(result);
  EXPECT_TRUE(success);
  EXPECT_EQ(node_->get_state_manager().get_state(), RecorderState::IDLE);
  EXPECT_FALSE(node_->get_task_config_cache().has_config());
}

TEST_F(RecordingServiceImplTest, RecordingControl_UnknownCommand) {
  bool success;
  std::string message, task_id_response;

  bool result = service_->handle_recording_control(
    "unknown_command", "", success, message, task_id_response
  );

  EXPECT_TRUE(result);
  EXPECT_FALSE(success);
  EXPECT_TRUE(message.find("ERR_INVALID_COMMAND") != std::string::npos);
}

// ============================================================================
// RecordingStatus Tests
// ============================================================================

TEST_F(RecordingServiceImplTest, RecordingStatus_Idle) {
  bool success;
  std::string message, status, task_id, device_id, data_collector, scene, subscene, org;
  std::string output_path, last_error;
  std::vector<std::string> skills, active_topics;
  double disk_usage, duration, throughput;
  int64_t message_count;

  bool result = service_->handle_recording_status(
    "", success, message, status, task_id, device_id, data_collector,
    scene, subscene, skills, org, active_topics, output_path,
    disk_usage, duration, message_count, throughput, last_error
  );

  EXPECT_TRUE(result);
  EXPECT_TRUE(success);
  EXPECT_EQ(status, "idle");
  EXPECT_TRUE(task_id.empty());
}

TEST_F(RecordingServiceImplTest, RecordingStatus_Recording) {
  bool success;
  std::string message, task_id_response;

  service_->handle_cached_recording_config(
    "task_001", "robot_01", "collector_01", "warehouse", "picking",
    {"skill1"}, "test_org", {"/camera"}, "", "", "",
    success, message
  );
  service_->handle_recording_control("start", "", success, message, task_id_response);
  ASSERT_TRUE(success);

  std::string status, task_id, device_id, data_collector, scene, subscene, org;
  std::string output_path, last_error;
  std::vector<std::string> skills, active_topics;
  double disk_usage, duration, throughput;
  int64_t message_count;

  bool result = service_->handle_recording_status(
    "", success, message, status, task_id, device_id, data_collector,
    scene, subscene, skills, org, active_topics, output_path,
    disk_usage, duration, message_count, throughput, last_error
  );

  EXPECT_TRUE(result);
  EXPECT_TRUE(success);
  EXPECT_EQ(status, "recording");
  EXPECT_EQ(task_id, "task_001");
  EXPECT_EQ(device_id, "robot_01");
  EXPECT_EQ(scene, "warehouse");
}

// ============================================================================
// Complete Workflow Integration Tests
// ============================================================================

TEST_F(RecordingServiceImplTest, FullRecordingWorkflow) {
  bool success;
  std::string message, task_id_response;

  // 1. Cache config
  bool result = service_->handle_cached_recording_config(
    "task_workflow_001", "robot_01", "collector_01", "warehouse", "picking",
    {"navigation", "manipulation"}, "test_org",
    {"/camera/image", "/lidar/scan", "/imu/data"},
    "http://server/start", "http://server/finish", "jwt_token_123",
    success, message
  );
  ASSERT_TRUE(result);
  ASSERT_TRUE(success);
  EXPECT_EQ(node_->get_state_manager().get_state(), RecorderState::READY);

  // 2. Verify ready
  bool is_configured, is_recording;
  std::string task_id, device_id, scene, subscene, org, data_collector;
  std::vector<std::string> skills, topics;
  service_->handle_is_recording_ready(
    success, message, is_configured, is_recording, task_id, device_id,
    scene, subscene, skills, org, data_collector, topics
  );
  EXPECT_TRUE(is_configured);
  EXPECT_FALSE(is_recording);
  EXPECT_EQ(task_id, "task_workflow_001");

  // 3. Start recording
  service_->handle_recording_control("start", "", success, message, task_id_response);
  ASSERT_TRUE(success);
  EXPECT_EQ(task_id_response, "task_workflow_001");
  EXPECT_EQ(node_->get_state_manager().get_state(), RecorderState::RECORDING);

  // 4. Check status while recording
  std::string status;
  std::string output_path, last_error;
  std::vector<std::string> active_topics;
  double disk_usage, duration, throughput;
  int64_t message_count;
  service_->handle_recording_status(
    "", success, message, status, task_id, device_id, data_collector,
    scene, subscene, skills, org, active_topics, output_path,
    disk_usage, duration, message_count, throughput, last_error
  );
  EXPECT_EQ(status, "recording");

  // 5. Pause recording
  service_->handle_recording_control(
    "pause", "task_workflow_001", success, message, task_id_response);
  ASSERT_TRUE(success);
  EXPECT_EQ(node_->get_state_manager().get_state(), RecorderState::PAUSED);

  // 6. Resume recording
  service_->handle_recording_control(
    "resume", "task_workflow_001", success, message, task_id_response);
  ASSERT_TRUE(success);
  EXPECT_EQ(node_->get_state_manager().get_state(), RecorderState::RECORDING);

  // 7. Finish recording
  service_->handle_recording_control(
    "finish", "task_workflow_001", success, message, task_id_response);
  ASSERT_TRUE(success);
  EXPECT_EQ(node_->get_state_manager().get_state(), RecorderState::IDLE);

  // 8. Verify back to idle
  service_->handle_recording_status(
    "", success, message, status, task_id, device_id, data_collector,
    scene, subscene, skills, org, active_topics, output_path,
    disk_usage, duration, message_count, throughput, last_error
  );
  EXPECT_EQ(status, "idle");
}

TEST_F(RecordingServiceImplTest, CancelWorkflow) {
  bool success;
  std::string message, task_id_response;

  service_->handle_cached_recording_config(
    "task_cancel_001", "robot_01", "collector_01", "warehouse", "picking", {},
    "test_org", {}, "", "", "", success, message
  );
  service_->handle_recording_control("start", "", success, message, task_id_response);
  ASSERT_TRUE(success);

  service_->handle_recording_control(
    "cancel", "task_cancel_001", success, message, task_id_response);
  ASSERT_TRUE(success);
  EXPECT_EQ(node_->get_state_manager().get_state(), RecorderState::IDLE);
  EXPECT_TRUE(node_->was_cancel_called());
}

TEST_F(RecordingServiceImplTest, ClearConfigWorkflow) {
  bool success;
  std::string message, task_id_response;

  service_->handle_cached_recording_config(
    "task_clear_001", "robot_01", "collector_01", "warehouse", "picking", {},
    "test_org", {}, "", "", "", success, message
  );
  ASSERT_TRUE(success);
  EXPECT_TRUE(node_->get_task_config_cache().has_config());

  service_->handle_recording_control("clear", "", success, message, task_id_response);
  ASSERT_TRUE(success);
  EXPECT_EQ(node_->get_state_manager().get_state(), RecorderState::IDLE);
  EXPECT_FALSE(node_->get_task_config_cache().has_config());
}

TEST_F(RecordingServiceImplTest, MultipleTasksSequential) {
  bool success;
  std::string message, task_id_response;

  // First task
  service_->handle_cached_recording_config(
    "task_seq_001", "robot_01", "collector_01", "warehouse", "picking", {},
    "test_org", {}, "", "", "", success, message
  );
  service_->handle_recording_control("start", "", success, message, task_id_response);
  service_->handle_recording_control("finish", "task_seq_001", success, message, task_id_response);
  EXPECT_EQ(node_->get_state_manager().get_state(), RecorderState::IDLE);

  // Second task
  service_->handle_cached_recording_config(
    "task_seq_002", "robot_01", "collector_01", "loading_dock", "unloading", {},
    "test_org", {}, "", "", "", success, message
  );
  ASSERT_TRUE(success);
  EXPECT_EQ(node_->get_task_config_cache().get_task_id(), "task_seq_002");

  service_->handle_recording_control("start", "", success, message, task_id_response);
  ASSERT_TRUE(success);
  EXPECT_EQ(task_id_response, "task_seq_002");

  service_->handle_recording_control("finish", "task_seq_002", success, message, task_id_response);
  ASSERT_TRUE(success);
  EXPECT_EQ(node_->get_state_manager().get_state(), RecorderState::IDLE);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
