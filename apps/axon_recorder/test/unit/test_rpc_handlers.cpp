// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

/**
 * Unit tests for RPC Handlers
 */

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <atomic>
#include <chrono>
#include <optional>
#include <string>
#include <thread>

#include "../../src/config/task_config.hpp"
#include "../../src/http/rpc_handlers.hpp"

using namespace axon::recorder;

// ============================================================================
// Test Fixtures
// ============================================================================

class RpcHandlersTest : public ::testing::Test {
protected:
  RpcCallbacks create_mock_callbacks() {
    RpcCallbacks callbacks;

    // Set up state callback
    callbacks.get_state = [this]() -> std::string {
      return current_state_;
    };

    // Set up stats callback
    callbacks.get_stats = [this]() -> nlohmann::json {
      return test_stats_;
    };

    // Set up task config callback
    callbacks.get_task_config = [this]() -> const TaskConfig* {
      return current_config_.has_value() ? &current_config_.value() : nullptr;
    };

    return callbacks;
  }

  void SetUp() override {
    current_state_ = "idle";
    test_stats_ = {{"messages_received", 1000}, {"messages_written", 995}, {"messages_dropped", 5}};
    begin_recording_called = false;
    finish_recording_called = false;
    pause_recording_called = false;
    resume_recording_called = false;
    cancel_recording_called = false;
    clear_config_called = false;
    quit_called = false;
    last_task_id.clear();
  }

  std::string current_state_;
  nlohmann::json test_stats_;
  std::optional<TaskConfig> current_config_;

  // Track callback invocations
  std::atomic<bool> begin_recording_called;
  std::atomic<bool> finish_recording_called;
  std::atomic<bool> pause_recording_called;
  std::atomic<bool> resume_recording_called;
  std::atomic<bool> cancel_recording_called;
  std::atomic<bool> clear_config_called;
  std::atomic<bool> quit_called;
  std::string last_task_id;
};

// ============================================================================
// handle_rpc_begin Tests
// ============================================================================

TEST_F(RpcHandlersTest, BeginSuccess) {
  RpcCallbacks callbacks = create_mock_callbacks();
  callbacks.begin_recording = [&](const std::string& task_id) -> bool {
    begin_recording_called = true;
    last_task_id = task_id;
    current_state_ = "recording";
    return true;
  };

  nlohmann::json params;
  params["task_id"] = "test_task_123";

  RpcResponse response = handle_rpc_begin(callbacks, params);

  EXPECT_TRUE(response.success);
  EXPECT_EQ("Recording started successfully", response.message);
  EXPECT_EQ("recording", response.data["state"]);
  EXPECT_EQ("test_task_123", response.data["task_id"]);
  EXPECT_TRUE(begin_recording_called);
  EXPECT_EQ("test_task_123", last_task_id);
}

TEST_F(RpcHandlersTest, BeginMissingTaskId) {
  RpcCallbacks callbacks = create_mock_callbacks();
  callbacks.begin_recording = [&](const std::string&) -> bool {
    return true;
  };

  nlohmann::json params;

  RpcResponse response = handle_rpc_begin(callbacks, params);

  EXPECT_FALSE(response.success);
  EXPECT_EQ("Missing required parameter: task_id", response.message);
  EXPECT_EQ("idle", response.data["state"]);
  EXPECT_FALSE(begin_recording_called);
}

TEST_F(RpcHandlersTest, BeginInvalidTaskIdType) {
  RpcCallbacks callbacks = create_mock_callbacks();
  callbacks.begin_recording = [&](const std::string&) -> bool {
    return true;
  };

  nlohmann::json params;
  params["task_id"] = 12345;  // Number instead of string

  RpcResponse response = handle_rpc_begin(callbacks, params);

  EXPECT_FALSE(response.success);
  EXPECT_EQ("task_id must be a string", response.message);
  EXPECT_FALSE(begin_recording_called);
}

TEST_F(RpcHandlersTest, BeginCallbackNotRegistered) {
  RpcCallbacks callbacks = create_mock_callbacks();
  // Don't set begin_recording callback

  nlohmann::json params;
  params["task_id"] = "test_task_123";

  RpcResponse response = handle_rpc_begin(callbacks, params);

  EXPECT_FALSE(response.success);
  EXPECT_EQ("Begin recording callback not registered", response.message);
}

TEST_F(RpcHandlersTest, BeginCallbackFails) {
  RpcCallbacks callbacks = create_mock_callbacks();
  callbacks.begin_recording = [&](const std::string& task_id) -> bool {
    begin_recording_called = true;
    last_task_id = task_id;
    return false;  // Simulate failure
  };

  nlohmann::json params;
  params["task_id"] = "test_task_123";

  RpcResponse response = handle_rpc_begin(callbacks, params);

  EXPECT_FALSE(response.success);
  EXPECT_EQ("Failed to start recording", response.message);
  EXPECT_EQ("idle", response.data["state"]);
  EXPECT_TRUE(begin_recording_called);
}

// ============================================================================
// handle_rpc_finish Tests
// ============================================================================

TEST_F(RpcHandlersTest, FinishSuccess) {
  current_state_ = "recording";
  RpcCallbacks callbacks = create_mock_callbacks();
  callbacks.finish_recording = [&](const std::string& task_id) -> bool {
    finish_recording_called = true;
    last_task_id = task_id;
    current_state_ = "idle";
    return true;
  };

  nlohmann::json params;
  params["task_id"] = "test_task_123";

  RpcResponse response = handle_rpc_finish(callbacks, params);

  EXPECT_TRUE(response.success);
  EXPECT_EQ("Recording finished successfully", response.message);
  EXPECT_EQ("idle", response.data["state"]);
  EXPECT_EQ("test_task_123", response.data["task_id"]);
  EXPECT_TRUE(finish_recording_called);
}

TEST_F(RpcHandlersTest, FinishMissingTaskId) {
  RpcCallbacks callbacks = create_mock_callbacks();
  callbacks.finish_recording = [&](const std::string&) -> bool {
    return true;
  };

  nlohmann::json params;

  RpcResponse response = handle_rpc_finish(callbacks, params);

  EXPECT_FALSE(response.success);
  EXPECT_EQ("Missing required parameter: task_id", response.message);
  EXPECT_FALSE(finish_recording_called);
}

TEST_F(RpcHandlersTest, FinishInvalidTaskIdType) {
  RpcCallbacks callbacks = create_mock_callbacks();
  callbacks.finish_recording = [&](const std::string&) -> bool {
    return true;
  };

  nlohmann::json params;
  params["task_id"] = 12345;

  RpcResponse response = handle_rpc_finish(callbacks, params);

  EXPECT_FALSE(response.success);
  EXPECT_EQ("task_id must be a string", response.message);
  EXPECT_FALSE(finish_recording_called);
}

// ============================================================================
// handle_rpc_pause Tests
// ============================================================================

TEST_F(RpcHandlersTest, PauseSuccess) {
  current_state_ = "recording";
  RpcCallbacks callbacks = create_mock_callbacks();
  callbacks.pause_recording = [&]() -> bool {
    pause_recording_called = true;
    current_state_ = "paused";
    return true;
  };

  nlohmann::json params;

  RpcResponse response = handle_rpc_pause(callbacks, params);

  EXPECT_TRUE(response.success);
  EXPECT_EQ("Recording paused successfully", response.message);
  EXPECT_EQ("paused", response.data["state"]);
  EXPECT_TRUE(pause_recording_called);
}

TEST_F(RpcHandlersTest, PauseCallbackFails) {
  current_state_ = "recording";
  RpcCallbacks callbacks = create_mock_callbacks();
  callbacks.pause_recording = [&]() -> bool {
    pause_recording_called = true;
    return false;
  };

  nlohmann::json params;

  RpcResponse response = handle_rpc_pause(callbacks, params);

  EXPECT_FALSE(response.success);
  EXPECT_EQ("Failed to pause recording", response.message);
  EXPECT_EQ("recording", response.data["state"]);
}

TEST_F(RpcHandlersTest, PauseCallbackNotRegistered) {
  RpcCallbacks callbacks = create_mock_callbacks();
  // Don't set pause_recording callback

  nlohmann::json params;

  RpcResponse response = handle_rpc_pause(callbacks, params);

  EXPECT_FALSE(response.success);
  EXPECT_EQ("Pause recording callback not registered", response.message);
}

// ============================================================================
// handle_rpc_resume Tests
// ============================================================================

TEST_F(RpcHandlersTest, ResumeSuccess) {
  current_state_ = "paused";
  RpcCallbacks callbacks = create_mock_callbacks();
  callbacks.resume_recording = [&]() -> bool {
    resume_recording_called = true;
    current_state_ = "recording";
    return true;
  };

  nlohmann::json params;

  RpcResponse response = handle_rpc_resume(callbacks, params);

  EXPECT_TRUE(response.success);
  EXPECT_EQ("Recording resumed successfully", response.message);
  EXPECT_EQ("recording", response.data["state"]);
  EXPECT_TRUE(resume_recording_called);
}

TEST_F(RpcHandlersTest, ResumeCallbackFails) {
  current_state_ = "paused";
  RpcCallbacks callbacks = create_mock_callbacks();
  callbacks.resume_recording = [&]() -> bool {
    resume_recording_called = true;
    return false;
  };

  nlohmann::json params;

  RpcResponse response = handle_rpc_resume(callbacks, params);

  EXPECT_FALSE(response.success);
  EXPECT_EQ("Failed to resume recording", response.message);
}

// ============================================================================
// handle_rpc_cancel Tests
// ============================================================================

TEST_F(RpcHandlersTest, CancelSuccess) {
  current_state_ = "recording";
  RpcCallbacks callbacks = create_mock_callbacks();
  callbacks.cancel_recording = [&]() -> bool {
    cancel_recording_called = true;
    current_state_ = "idle";
    return true;
  };

  nlohmann::json params;

  RpcResponse response = handle_rpc_cancel(callbacks, params);

  EXPECT_TRUE(response.success);
  EXPECT_EQ("Recording cancelled successfully", response.message);
  EXPECT_EQ("idle", response.data["state"]);
  EXPECT_TRUE(cancel_recording_called);
}

TEST_F(RpcHandlersTest, CancelCallbackFails) {
  current_state_ = "recording";
  RpcCallbacks callbacks = create_mock_callbacks();
  callbacks.cancel_recording = [&]() -> bool {
    cancel_recording_called = true;
    return false;
  };

  nlohmann::json params;

  RpcResponse response = handle_rpc_cancel(callbacks, params);

  EXPECT_FALSE(response.success);
  EXPECT_EQ("Failed to cancel recording", response.message);
}

// ============================================================================
// handle_rpc_clear Tests
// ============================================================================

TEST_F(RpcHandlersTest, ClearSuccess) {
  current_state_ = "ready";
  current_config_ = TaskConfig{};
  current_config_->task_id = "test_task";
  RpcCallbacks callbacks = create_mock_callbacks();
  callbacks.clear_config = [&]() -> bool {
    clear_config_called = true;
    current_config_.reset();
    current_state_ = "idle";
    return true;
  };

  nlohmann::json params;

  RpcResponse response = handle_rpc_clear(callbacks, params);

  EXPECT_TRUE(response.success);
  EXPECT_EQ("Configuration cleared successfully", response.message);
  EXPECT_EQ("idle", response.data["state"]);
  EXPECT_TRUE(clear_config_called);
}

// ============================================================================
// handle_rpc_config Tests
// ============================================================================

TEST_F(RpcHandlersTest, ConfigSuccess) {
  current_state_ = "idle";
  RpcCallbacks callbacks = create_mock_callbacks();
  callbacks.set_config = [&](const std::string& task_id, const nlohmann::json& config) -> bool {
    current_config_ = TaskConfig{};
    current_config_->task_id = task_id;
    current_config_->device_id = config.value("device_id", "");
    current_config_->topics = config.value("topics", std::vector<std::string>{});
    current_state_ = "ready";
    return true;
  };

  nlohmann::json params;
  params["task_config"] = {
    {"task_id", "new_task_123"},
    {"device_id", "robot_01"},
    {"topics", {"/camera/image", "/lidar/scan"}}
  };

  RpcResponse response = handle_rpc_config(callbacks, params);

  EXPECT_TRUE(response.success);
  EXPECT_EQ("Task configuration set successfully", response.message);
  EXPECT_EQ("ready", response.data["state"]);
  EXPECT_EQ("new_task_123", response.data["task_id"]);
}

TEST_F(RpcHandlersTest, ConfigMissingTaskConfig) {
  RpcCallbacks callbacks = create_mock_callbacks();
  callbacks.set_config = [&](const std::string&, const nlohmann::json&) -> bool {
    return true;
  };

  nlohmann::json params;

  RpcResponse response = handle_rpc_config(callbacks, params);

  EXPECT_FALSE(response.success);
  EXPECT_EQ("Missing 'task_config' in request parameters", response.message);
}

TEST_F(RpcHandlersTest, ConfigInvalidTaskIdType) {
  RpcCallbacks callbacks = create_mock_callbacks();
  callbacks.set_config = [&](const std::string&, const nlohmann::json&) -> bool {
    return true;
  };

  nlohmann::json params;
  params["task_config"] = {
    {"task_id", 12345}  // Number instead of string
  };

  RpcResponse response = handle_rpc_config(callbacks, params);

  EXPECT_FALSE(response.success);
  EXPECT_EQ("task_id must be a string", response.message);
}

TEST_F(RpcHandlersTest, ConfigCallbackFails) {
  RpcCallbacks callbacks = create_mock_callbacks();
  callbacks.set_config = [&](const std::string&, const nlohmann::json&) -> bool {
    return false;
  };

  nlohmann::json params;
  params["task_config"] = {{"task_id", "new_task_123"}};

  RpcResponse response = handle_rpc_config(callbacks, params);

  EXPECT_FALSE(response.success);
  EXPECT_EQ("Failed to set task configuration", response.message);
}

TEST_F(RpcHandlersTest, ConfigParseException) {
  RpcCallbacks callbacks = create_mock_callbacks();
  callbacks.set_config = [&](const std::string&, const nlohmann::json&) -> bool {
    throw std::runtime_error("Parse error");
  };

  nlohmann::json params;
  params["task_config"] = {{"task_id", "new_task_123"}};

  RpcResponse response = handle_rpc_config(callbacks, params);

  EXPECT_FALSE(response.success);
  EXPECT_TRUE(response.message.find("Failed to parse task config") != std::string::npos);
}

// ============================================================================
// handle_rpc_quit Tests
// ============================================================================

TEST_F(RpcHandlersTest, QuitSuccess) {
  RpcCallbacks callbacks = create_mock_callbacks();
  callbacks.quit = [&]() -> void {
    quit_called = true;
  };

  nlohmann::json params;

  RpcResponse response = handle_rpc_quit(callbacks, params);

  EXPECT_TRUE(response.success);
  EXPECT_EQ("Program quitting. Recording stopped and data saved.", response.message);
  EXPECT_EQ("idle", response.data["state"]);
  EXPECT_TRUE(quit_called);
}

TEST_F(RpcHandlersTest, QuitWithoutCallback) {
  RpcCallbacks callbacks = create_mock_callbacks();
  // Don't set quit callback

  nlohmann::json params;

  // Should not crash, just succeed without calling anything
  RpcResponse response = handle_rpc_quit(callbacks, params);

  EXPECT_TRUE(response.success);
}

// ============================================================================
// handle_rpc_get_state Tests
// ============================================================================

TEST_F(RpcHandlersTest, GetStateSuccess) {
  current_state_ = "recording";
  current_config_ = TaskConfig{};
  current_config_->task_id = "test_task_123";
  current_config_->device_id = "robot_01";
  current_config_->scene = "warehouse";
  current_config_->topics = {"/camera/image", "/lidar/scan"};

  RpcCallbacks callbacks = create_mock_callbacks();

  nlohmann::json params;

  RpcResponse response = handle_rpc_get_state(callbacks, params);

  EXPECT_TRUE(response.success);
  EXPECT_EQ("State retrieved successfully", response.message);
  EXPECT_EQ("recording", response.data["state"]);
  EXPECT_EQ("test_task_123", response.data["task_config"]["task_id"]);
  EXPECT_EQ("robot_01", response.data["task_config"]["device_id"]);
  EXPECT_EQ("warehouse", response.data["task_config"]["scene"]);
  EXPECT_TRUE(response.data.contains("version"));
}

TEST_F(RpcHandlersTest, GetStateNoConfig) {
  current_state_ = "idle";
  current_config_.reset();

  RpcCallbacks callbacks = create_mock_callbacks();

  nlohmann::json params;

  RpcResponse response = handle_rpc_get_state(callbacks, params);

  EXPECT_TRUE(response.success);
  EXPECT_EQ("idle", response.data["state"]);
  EXPECT_FALSE(response.data.contains("task_config"));
}

TEST_F(RpcHandlersTest, GetStateWithStatsRunning) {
  current_state_ = "recording";
  test_stats_ = {{"messages_written", 1000}, {"messages_received", 1000}};

  RpcCallbacks callbacks = create_mock_callbacks();

  nlohmann::json params;

  RpcResponse response = handle_rpc_get_state(callbacks, params);

  EXPECT_TRUE(response.success);
  EXPECT_TRUE(response.data["running"]);
}

TEST_F(RpcHandlersTest, GetStateNotRunning) {
  current_state_ = "ready";
  test_stats_ = {{"messages_written", 0}, {"messages_received", 0}};

  RpcCallbacks callbacks = create_mock_callbacks();

  nlohmann::json params;

  RpcResponse response = handle_rpc_get_state(callbacks, params);

  EXPECT_TRUE(response.success);
  EXPECT_FALSE(response.data["running"]);
}

TEST_F(RpcHandlersTest, GetStateStatsException) {
  current_state_ = "ready";
  RpcCallbacks callbacks = create_mock_callbacks();
  callbacks.get_stats = [&]() -> nlohmann::json {
    throw std::runtime_error("Stats error");
  };

  nlohmann::json params;

  RpcResponse response = handle_rpc_get_state(callbacks, params);

  EXPECT_TRUE(response.success);
  EXPECT_FALSE(response.data["running"]);
}

// ============================================================================
// handle_rpc_get_stats Tests
// ============================================================================

TEST_F(RpcHandlersTest, GetStatsSuccess) {
  test_stats_ = {
    {"messages_received", 5000},
    {"messages_written", 4950},
    {"messages_dropped", 50},
    {"bytes_written", 1024000}
  };

  RpcCallbacks callbacks = create_mock_callbacks();

  nlohmann::json params;

  RpcResponse response = handle_rpc_get_stats(callbacks, params);

  EXPECT_TRUE(response.success);
  EXPECT_EQ("Statistics retrieved successfully", response.message);
  EXPECT_EQ(5000, response.data["messages_received"]);
  EXPECT_EQ(4950, response.data["messages_written"]);
  EXPECT_EQ(50, response.data["messages_dropped"]);
  EXPECT_EQ(1024000, response.data["bytes_written"]);
}

TEST_F(RpcHandlersTest, GetStatsCallbackNotRegistered) {
  RpcCallbacks callbacks = create_mock_callbacks();
  // Clear the get_stats callback to simulate "not registered"
  callbacks.get_stats = nullptr;

  nlohmann::json params;

  RpcResponse response = handle_rpc_get_stats(callbacks, params);

  EXPECT_FALSE(response.success);
  EXPECT_EQ("Get stats callback not registered", response.message);
}

TEST_F(RpcHandlersTest, GetStatsCallbackException) {
  RpcCallbacks callbacks = create_mock_callbacks();
  callbacks.get_stats = [&]() -> nlohmann::json {
    throw std::runtime_error("Stats collection failed");
  };

  nlohmann::json params;

  RpcResponse response = handle_rpc_get_stats(callbacks, params);

  EXPECT_FALSE(response.success);
  EXPECT_TRUE(response.message.find("Failed to get statistics") != std::string::npos);
}

// ============================================================================
// RpcResponse::to_json Tests
// ============================================================================

TEST(RpcResponseToJson, SuccessWithAllFields) {
  RpcResponse response;
  response.success = true;
  response.message = "Operation successful";
  response.data = {{"key1", "value1"}, {"key2", 42}};

  nlohmann::json json = response.to_json();

  EXPECT_TRUE(json["success"]);
  EXPECT_EQ("Operation successful", json["message"]);
  EXPECT_EQ("value1", json["data"]["key1"]);
  EXPECT_EQ(42, json["data"]["key2"]);
}

TEST(RpcResponseToJson, SuccessWithNullData) {
  RpcResponse response;
  response.success = true;
  response.message = "OK";
  // data is null by default

  nlohmann::json json = response.to_json();

  EXPECT_TRUE(json["success"]);
  EXPECT_EQ("OK", json["message"]);
  EXPECT_FALSE(json.contains("data"));
}

TEST(RpcResponseToJson, ErrorWithMessage) {
  RpcResponse response;
  response.success = false;
  response.message = "Operation failed";

  nlohmann::json json = response.to_json();

  EXPECT_FALSE(json["success"]);
  EXPECT_EQ("Operation failed", json["message"]);
}
