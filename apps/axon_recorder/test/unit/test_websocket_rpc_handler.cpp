// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

/**
 * Unit tests for WebSocket RPC Handler
 */

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "../../src/config/task_config.hpp"
#include "../../src/http/websocket_rpc_handler.hpp"

using namespace axon::recorder;

// ============================================================================
// Test Fixtures
// ============================================================================

class WebSocketRpcHandlerTest : public ::testing::Test {
protected:
  WebSocketRpcHandlerTest()
      : response_received_(false)
      , last_client_id_()
      , last_response_() {}

  void SetUp() override {
    current_state_ = "idle";

    // Set up mock callbacks
    callbacks_.get_state = [this]() -> std::string {
      return current_state_;
    };

    callbacks_.get_stats = [this]() -> nlohmann::json {
      return test_stats_;
    };

    callbacks_.get_task_config = [this]() -> const TaskConfig* {
      return current_config_.has_value() ? &current_config_.value() : nullptr;
    };

    callbacks_.begin_recording = [&](const std::string& task_id) -> bool {
      begin_called_ = true;
      last_task_id_ = task_id;
      current_state_ = "recording";
      return begin_return_value_;
    };

    callbacks_.finish_recording = [&](const std::string& task_id) -> bool {
      finish_called_ = true;
      last_task_id_ = task_id;
      current_state_ = "idle";
      return true;
    };

    callbacks_.pause_recording = [&]() -> bool {
      pause_called_ = true;
      current_state_ = "paused";
      return true;
    };

    callbacks_.resume_recording = [&]() -> bool {
      resume_called_ = true;
      current_state_ = "recording";
      return true;
    };

    callbacks_.cancel_recording = [&]() -> bool {
      cancel_called_ = true;
      current_state_ = "idle";
      return true;
    };

    callbacks_.clear_config = [&]() -> bool {
      clear_called_ = true;
      current_config_.reset();
      current_state_ = "idle";
      return true;
    };

    callbacks_.set_config = [&](const std::string& task_id, const nlohmann::json& config) -> bool {
      config_called_ = true;
      current_config_ = TaskConfig{};
      current_config_->task_id = task_id;
      current_config_->device_id = config.value("device_id", "");
      current_config_->topics = config.value("topics", std::vector<std::string>{});
      current_state_ = "ready";
      return true;
    };

    callbacks_.get_log_levels = [this]() -> nlohmann::json {
      return {{"console_level", console_level_}, {"file_level", file_level_}};
    };

    callbacks_.set_log_levels = [this](const nlohmann::json& params, nlohmann::json& result) {
      log_levels_called_ = true;
      if (params.contains("console_level")) {
        console_level_ = params["console_level"];
      }
      if (params.contains("file_level")) {
        file_level_ = params["file_level"];
      }
      result = {{"console_level", console_level_}, {"file_level", file_level_}};
      return true;
    };

    callbacks_.quit = [&]() -> void {
      quit_called_ = true;
    };

    // Create response sender mock
    response_sender_ = [&](const std::string& client_id, const nlohmann::json& response) {
      std::lock_guard<std::mutex> lock(mutex_);
      response_received_ = true;
      last_client_id_ = client_id;
      last_response_ = response;
      cv_.notify_one();
    };

    // Create handler
    handler_ = std::make_unique<WebSocketRpcHandler>(callbacks_, response_sender_);

    // Reset flags
    begin_called_ = false;
    finish_called_ = false;
    pause_called_ = false;
    resume_called_ = false;
    cancel_called_ = false;
    clear_called_ = false;
    config_called_ = false;
    log_levels_called_ = false;
    quit_called_ = false;
    begin_return_value_ = true;
    console_level_ = "info";
    file_level_ = "debug";
  }

  // Wait for response with timeout
  bool wait_for_response(std::chrono::milliseconds timeout = std::chrono::milliseconds(100)) {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, timeout, [this] {
      return response_received_.load();
    });
  }

  void reset_response_state() {
    std::lock_guard<std::mutex> lock(mutex_);
    response_received_ = false;
    last_client_id_.clear();
    last_response_ = nullptr;
  }

  std::string current_state_;
  nlohmann::json test_stats_;
  std::optional<TaskConfig> current_config_;

  RpcCallbacks callbacks_;
  WebSocketRpcHandler::ResponseSender response_sender_;
  std::unique_ptr<WebSocketRpcHandler> handler_;

  // Response tracking
  std::atomic<bool> response_received_;
  std::string last_client_id_;
  nlohmann::json last_response_;
  std::mutex mutex_;
  std::condition_variable cv_;

  // Callback tracking
  std::atomic<bool> begin_called_;
  std::atomic<bool> finish_called_;
  std::atomic<bool> pause_called_;
  std::atomic<bool> resume_called_;
  std::atomic<bool> cancel_called_;
  std::atomic<bool> clear_called_;
  std::atomic<bool> config_called_;
  std::atomic<bool> log_levels_called_;
  std::atomic<bool> quit_called_;
  std::string last_task_id_;
  bool begin_return_value_;
  std::string console_level_;
  std::string file_level_;
};

// ============================================================================
// Message Handling Tests
// ============================================================================

TEST_F(WebSocketRpcHandlerTest, HandleValidBeginMessage) {
  std::string message = R"({
    "action": "begin",
    "request_id": "req_123",
    "task_id": "test_task_456"
  })";

  handler_->handle_message("client_001", message);

  EXPECT_TRUE(wait_for_response());
  EXPECT_EQ("client_001", last_client_id_);
  EXPECT_TRUE(last_response_.contains("type"));
  EXPECT_EQ("rpc_response", last_response_["type"]);
  EXPECT_EQ("req_123", last_response_["request_id"]);
  EXPECT_TRUE(last_response_["success"]);
  EXPECT_TRUE(begin_called_);
  EXPECT_EQ("test_task_456", last_task_id_);
}

TEST_F(WebSocketRpcHandlerTest, HandleValidFinishMessage) {
  current_state_ = "recording";

  std::string message = R"({
    "action": "finish",
    "request_id": "req_456",
    "task_id": "test_task_456"
  })";

  handler_->handle_message("client_002", message);

  EXPECT_TRUE(wait_for_response());
  EXPECT_TRUE(last_response_["success"]);
  EXPECT_TRUE(finish_called_);
}

TEST_F(WebSocketRpcHandlerTest, HandleValidPauseMessage) {
  current_state_ = "recording";

  std::string message = R"({
    "action": "pause",
    "request_id": "req_789"
  })";

  handler_->handle_message("client_003", message);

  EXPECT_TRUE(wait_for_response());
  EXPECT_TRUE(last_response_["success"]);
  EXPECT_TRUE(pause_called_);
}

TEST_F(WebSocketRpcHandlerTest, HandleValidResumeMessage) {
  current_state_ = "paused";

  std::string message = R"({
    "action": "resume",
    "request_id": "req_abc"
  })";

  handler_->handle_message("client_004", message);

  EXPECT_TRUE(wait_for_response());
  EXPECT_TRUE(last_response_["success"]);
  EXPECT_TRUE(resume_called_);
}

TEST_F(WebSocketRpcHandlerTest, HandleValidCancelMessage) {
  current_state_ = "recording";

  std::string message = R"({
    "action": "cancel",
    "request_id": "req_def"
  })";

  handler_->handle_message("client_005", message);

  EXPECT_TRUE(wait_for_response());
  EXPECT_TRUE(last_response_["success"]);
  EXPECT_TRUE(cancel_called_);
}

TEST_F(WebSocketRpcHandlerTest, HandleValidClearMessage) {
  current_state_ = "ready";
  current_config_ = TaskConfig{};

  std::string message = R"({
    "action": "clear",
    "request_id": "req_ghi"
  })";

  handler_->handle_message("client_006", message);

  EXPECT_TRUE(wait_for_response());
  EXPECT_TRUE(last_response_["success"]);
  EXPECT_TRUE(clear_called_);
}

TEST_F(WebSocketRpcHandlerTest, HandleValidConfigMessage) {
  std::string message = R"({
    "action": "config",
    "request_id": "req_jkl",
    "task_config": {
      "task_id": "new_task_789",
      "device_id": "robot_02",
      "topics": ["/camera", "/lidar"]
    }
  })";

  handler_->handle_message("client_007", message);

  EXPECT_TRUE(wait_for_response());
  EXPECT_TRUE(last_response_["success"]);
  EXPECT_TRUE(config_called_);
  EXPECT_EQ("new_task_789", last_response_["data"]["task_id"]);
}

TEST_F(WebSocketRpcHandlerTest, HandleValidQuitMessage) {
  std::string message = R"({
    "action": "quit",
    "request_id": "req_mno"
  })";

  handler_->handle_message("client_008", message);

  EXPECT_TRUE(wait_for_response());
  EXPECT_TRUE(last_response_["success"]);
  EXPECT_TRUE(quit_called_);
}

TEST_F(WebSocketRpcHandlerTest, HandleValidGetStateMessage) {
  current_state_ = "recording";
  current_config_ = TaskConfig{};
  current_config_->task_id = "test_task";

  std::string message = R"({
    "action": "get_state",
    "request_id": "req_pqr"
  })";

  handler_->handle_message("client_009", message);

  EXPECT_TRUE(wait_for_response());
  EXPECT_TRUE(last_response_["success"]);
  EXPECT_EQ("recording", last_response_["data"]["state"]);
  EXPECT_EQ("req_pqr", last_response_["request_id"]);
}

TEST_F(WebSocketRpcHandlerTest, HandleValidGetStatsMessage) {
  test_stats_ = {{"messages_received", 1000}, {"messages_written", 995}};

  std::string message = R"({
    "action": "get_stats",
    "request_id": "req_stu"
  })";

  handler_->handle_message("client_010", message);

  EXPECT_TRUE(wait_for_response());
  EXPECT_TRUE(last_response_["success"]);
  EXPECT_EQ(1000, last_response_["data"]["messages_received"]);
  EXPECT_EQ(995, last_response_["data"]["messages_written"]);
}

TEST_F(WebSocketRpcHandlerTest, HandleValidGetStatusMessage) {
  test_stats_ = {
    {"state", "recording"},
    {"messages_written", 995},
    {"disk_usage", {{"state", "warn"}, {"total_used_bytes", 4096}}}
  };

  std::string message = R"({
    "action": "get_status",
    "request_id": "req_status"
  })";

  handler_->handle_message("client_status", message);

  EXPECT_TRUE(wait_for_response());
  EXPECT_TRUE(last_response_["success"]);
  EXPECT_EQ("Status retrieved successfully", last_response_["message"]);
  EXPECT_EQ("warn", last_response_["data"]["disk_usage"]["state"]);
}

TEST_F(WebSocketRpcHandlerTest, HandleValidListTasksMessage) {
  current_state_ = "ready";
  current_config_ = TaskConfig{};
  current_config_->task_id = "task_ws_001";
  current_config_->device_id = "robot_ws";
  test_stats_ = {{"messages_written", 7}, {"duration_sec", 1.25}};

  std::string message = R"({
    "action": "list_tasks",
    "request_id": "req_tasks"
  })";

  handler_->handle_message("client_tasks", message);

  EXPECT_TRUE(wait_for_response());
  EXPECT_TRUE(last_response_["success"]);
  ASSERT_EQ(1, last_response_["data"]["tasks"].size());
  EXPECT_EQ("task_ws_001", last_response_["data"]["tasks"][0]["task_id"]);
}

TEST_F(WebSocketRpcHandlerTest, HandleValidGetTaskStatusMessage) {
  current_state_ = "recording";
  current_config_ = TaskConfig{};
  current_config_->task_id = "task_ws_status";
  current_config_->device_id = "robot_ws";

  std::string message = R"({
    "action": "get_task_status",
    "request_id": "req_task_status",
    "task_id": "task_ws_status"
  })";

  handler_->handle_message("client_task_status", message);

  EXPECT_TRUE(wait_for_response());
  EXPECT_TRUE(last_response_["success"]);
  EXPECT_TRUE(last_response_["data"]["found"]);
  EXPECT_EQ("recording", last_response_["data"]["task"]["state"]);
}

TEST_F(WebSocketRpcHandlerTest, HandleValidBatchBeginMessage) {
  std::string message = R"({
    "action": "begin_batch",
    "request_id": "req_batch_begin",
    "task_ids": ["batch_task"]
  })";

  handler_->handle_message("client_batch", message);

  EXPECT_TRUE(wait_for_response());
  EXPECT_TRUE(last_response_["success"]);
  EXPECT_TRUE(begin_called_);
  EXPECT_EQ("batch_task", last_task_id_);
  EXPECT_EQ(1, last_response_["data"]["succeeded"]);
}

TEST_F(WebSocketRpcHandlerTest, HandleValidLogLevelMessages) {
  std::string get_message = R"({
    "action": "get_log_levels",
    "request_id": "req_get_logs"
  })";

  handler_->handle_message("client_logs", get_message);
  EXPECT_TRUE(wait_for_response());
  EXPECT_TRUE(last_response_["success"]);
  EXPECT_EQ("info", last_response_["data"]["console_level"]);

  reset_response_state();

  std::string set_message = R"({
    "action": "set_log_levels",
    "request_id": "req_set_logs",
    "console_level": "warn",
    "file_level": "error"
  })";

  handler_->handle_message("client_logs", set_message);
  EXPECT_TRUE(wait_for_response());
  EXPECT_TRUE(last_response_["success"]);
  EXPECT_TRUE(log_levels_called_);
  EXPECT_EQ("warn", last_response_["data"]["console_level"]);
  EXPECT_EQ("error", last_response_["data"]["file_level"]);
}

// ============================================================================
// Error Handling Tests
// ============================================================================

TEST_F(WebSocketRpcHandlerTest, HandleMissingActionField) {
  std::string message = R"({
    "task_id": "test_task"
  })";

  handler_->handle_message("client_011", message);

  EXPECT_TRUE(wait_for_response());
  EXPECT_FALSE(last_response_["success"]);
  EXPECT_EQ("Missing 'action' field in request", last_response_["message"]);
}

TEST_F(WebSocketRpcHandlerTest, HandleUnknownAction) {
  std::string message = R"({
    "action": "unknown_action",
    "request_id": "req_xyz"
  })";

  handler_->handle_message("client_012", message);

  EXPECT_TRUE(wait_for_response());
  EXPECT_FALSE(last_response_["success"]);
  EXPECT_TRUE(
    last_response_["message"].get<std::string>().find("Unknown RPC action") != std::string::npos
  );
}

TEST_F(WebSocketRpcHandlerTest, HandleInvalidJson) {
  std::string message = "invalid json {";

  handler_->handle_message("client_013", message);

  EXPECT_TRUE(wait_for_response());
  EXPECT_FALSE(last_response_["success"]);
  EXPECT_TRUE(
    last_response_["message"].get<std::string>().find("JSON parse error") != std::string::npos
  );
}

TEST_F(WebSocketRpcHandlerTest, HandleBeginWithMissingTaskId) {
  std::string message = R"({
    "action": "begin",
    "request_id": "req_fail1"
  })";

  handler_->handle_message("client_014", message);

  EXPECT_TRUE(wait_for_response());
  EXPECT_FALSE(last_response_["success"]);
  EXPECT_EQ("Missing required parameter: task_id", last_response_["message"]);
  EXPECT_EQ("req_fail1", last_response_["request_id"]);
}

TEST_F(WebSocketRpcHandlerTest, HandleBeginCallbackFailure) {
  begin_return_value_ = false;

  std::string message = R"({
    "action": "begin",
    "request_id": "req_fail2",
    "task_id": "test_task"
  })";

  handler_->handle_message("client_015", message);

  EXPECT_TRUE(wait_for_response());
  EXPECT_FALSE(last_response_["success"]);
  EXPECT_EQ("Failed to start recording", last_response_["message"]);
}

TEST_F(WebSocketRpcHandlerTest, HandleConfigWithMissingTaskConfig) {
  std::string message = R"({
    "action": "config",
    "request_id": "req_fail3"
  })";

  handler_->handle_message("client_016", message);

  EXPECT_TRUE(wait_for_response());
  EXPECT_FALSE(last_response_["success"]);
  EXPECT_EQ("Missing 'task_config' in request parameters", last_response_["message"]);
}

TEST_F(WebSocketRpcHandlerTest, HandleConfigValidationErrors) {
  std::string message = R"({
    "action": "config",
    "request_id": "req_config_validation",
    "task_config": {
      "task_id": "missing_device"
    }
  })";

  handler_->handle_message("client_validation", message);

  EXPECT_TRUE(wait_for_response());
  EXPECT_FALSE(last_response_["success"]);
  EXPECT_EQ("Task configuration validation failed", last_response_["message"]);
  ASSERT_TRUE(last_response_["data"].contains("validation_errors"));
}

// ============================================================================
// Request ID Correlation Tests
// ============================================================================

TEST_F(WebSocketRpcHandlerTest, CorrelateRequestIdInResponse) {
  std::string message1 = R"({
    "action": "pause",
    "request_id": "client_req_001"
  })";

  handler_->handle_message("client_a", message1);

  EXPECT_TRUE(wait_for_response());
  EXPECT_EQ("client_req_001", last_response_["request_id"]);
  EXPECT_EQ("rpc_response", last_response_["type"]);

  reset_response_state();

  // Send another request with different request_id
  std::string message2 = R"({
    "action": "resume",
    "request_id": "client_req_002"
  })";

  handler_->handle_message("client_a", message2);

  EXPECT_TRUE(wait_for_response());
  EXPECT_EQ("client_req_002", last_response_["request_id"]);
}

TEST_F(WebSocketRpcHandlerTest, HandleNullRequestId) {
  std::string message = R"({
    "action": "get_state"
  })";

  handler_->handle_message("client_017", message);

  EXPECT_TRUE(wait_for_response());
  EXPECT_TRUE(last_response_["success"]);
  // request_id should be null or missing
  EXPECT_TRUE(last_response_["request_id"].is_null() || !last_response_.contains("request_id"));
}

// ============================================================================
// Client Isolation Tests
// ============================================================================

TEST_F(WebSocketRpcHandlerTest, MultipleClientsIsolated) {
  std::string message1 = R"({
    "action": "get_state",
    "request_id": "req_client1"
  })";

  std::string message2 = R"({
    "action": "pause",
    "request_id": "req_client2"
  })";

  handler_->handle_message("client_alice", message1);
  EXPECT_TRUE(wait_for_response());
  EXPECT_EQ("client_alice", last_client_id_);
  EXPECT_EQ("req_client1", last_response_["request_id"]);

  reset_response_state();

  handler_->handle_message("client_bob", message2);
  EXPECT_TRUE(wait_for_response());
  EXPECT_EQ("client_bob", last_client_id_);
  EXPECT_EQ("req_client2", last_response_["request_id"]);
}

// ============================================================================
// Response Format Tests
// ============================================================================

TEST_F(WebSocketRpcHandlerTest, ResponseFormatSuccess) {
  std::string message = R"({
    "action": "get_state",
    "request_id": "req_format"
  })";

  handler_->handle_message("client_format", message);

  EXPECT_TRUE(wait_for_response());
  EXPECT_TRUE(last_response_.is_object());
  EXPECT_TRUE(last_response_.contains("type"));
  EXPECT_TRUE(last_response_.contains("request_id"));
  EXPECT_TRUE(last_response_.contains("success"));
  EXPECT_TRUE(last_response_.contains("message"));
  EXPECT_TRUE(last_response_.contains("data"));
}

TEST_F(WebSocketRpcHandlerTest, ResponseFormatError) {
  std::string message = R"({
    "action": "unknown_action",
    "request_id": "req_error"
  })";

  handler_->handle_message("client_error", message);

  EXPECT_TRUE(wait_for_response());
  EXPECT_FALSE(last_response_["success"]);
  EXPECT_TRUE(last_response_.contains("message"));
  EXPECT_EQ("Unknown RPC action: unknown_action", last_response_["message"]);
}

// ============================================================================
// Concurrent Request Handling Tests
// ============================================================================

TEST_F(WebSocketRpcHandlerTest, HandleMultipleRapidRequests) {
  std::vector<std::string> request_ids = {"req_001", "req_002", "req_003", "req_004", "req_005"};
  std::vector<std::string> actions = {"get_state", "pause", "resume", "get_stats", "get_state"};

  // Send multiple requests rapidly
  for (size_t i = 0; i < request_ids.size(); ++i) {
    std::string message =
      "{"
      "\"action\": \"" +
      actions[i] +
      "\","
      "\"request_id\": \"" +
      request_ids[i] +
      "\""
      "}";
    handler_->handle_message("client_concurrent", message);
  }

  // Wait for all responses (in practice, they'd be collected asynchronously)
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // Verify we received responses (last one will be in last_response_)
  EXPECT_TRUE(response_received_);
}

// ============================================================================
// Special Action Tests
// ============================================================================

TEST_F(WebSocketRpcHandlerTest, GetStateNoConfig) {
  current_config_.reset();
  current_state_ = "idle";

  std::string message = R"({
    "action": "get_state",
    "request_id": "req_noconfig"
  })";

  handler_->handle_message("client_nocfg", message);

  EXPECT_TRUE(wait_for_response());
  EXPECT_TRUE(last_response_["success"]);
  EXPECT_EQ("idle", last_response_["data"]["state"]);
  EXPECT_FALSE(last_response_["data"].contains("task_config"));
}

TEST_F(WebSocketRpcHandlerTest, GetStatsEmptyStats) {
  test_stats_ = nlohmann::json::object();

  std::string message = R"({
    "action": "get_stats",
    "request_id": "req_empty"
  })";

  handler_->handle_message("client_empty", message);

  EXPECT_TRUE(wait_for_response());
  EXPECT_TRUE(last_response_["success"]);
  EXPECT_TRUE(last_response_["data"].is_object());
}

// ============================================================================
// Params Field Tests
// ============================================================================

TEST_F(WebSocketRpcHandlerTest, HandleMessageWithExtraParams) {
  std::string message = R"({
    "action": "get_state",
    "request_id": "req_extra",
    "extra_param": "should_be_ignored",
    "another_param": 12345
  })";

  handler_->handle_message("client_extra", message);

  EXPECT_TRUE(wait_for_response());
  EXPECT_TRUE(last_response_["success"]);
  // Extra params should be ignored
}
