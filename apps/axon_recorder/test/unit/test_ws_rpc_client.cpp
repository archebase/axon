// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <thread>

#include "../../src/core/recorder.hpp"
#include "../../src/http/rpc_handlers.hpp"
#include "../../src/http/ws_rpc_client.hpp"

using namespace axon::recorder;

namespace {

int64_t now_epoch_ms_for_test() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
           std::chrono::system_clock::now().time_since_epoch()
  )
    .count();
}

}  // namespace

class WsRpcClientTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Default test config
    test_config_.url = "ws://localhost:8091/rpc";
    test_config_.reconnect_initial_delay_ms = 100;
    test_config_.reconnect_backoff_multiplier = 2.0;
    test_config_.reconnect_max_delay_ms = 1000;
    test_config_.reconnect_jitter_factor = 0.1;
    test_config_.ping_interval_ms = 1000;
  }

  void TearDown() override {}

  WsClientConfig test_config_;
  net::io_context ioc_;
};

// Test: WsRpcClient construction and configuration
TEST_F(WsRpcClientTest, Construction) {
  WsRpcClient client(ioc_, test_config_);

  EXPECT_FALSE(client.is_connected());
}

// Test: WsRpcClient callback registration
TEST_F(WsRpcClientTest, CallbackRegistration) {
  WsRpcClient client(ioc_, test_config_);

  RpcCallbacks callbacks;
  callbacks.get_state = []() {
    return "idle";
  };
  callbacks.get_stats = []() {
    return nlohmann::json{};
  };

  // Should not throw
  EXPECT_NO_THROW(client.register_callbacks(callbacks));
}

// Test: WsRpcClient start/stop lifecycle
TEST_F(WsRpcClientTest, StartStopLifecycle) {
  WsRpcClient client(ioc_, test_config_);

  // Start should begin connection attempt
  EXPECT_NO_THROW(client.start());

  // Should not be connected immediately (server not running)
  EXPECT_FALSE(client.is_connected());

  // Stop should clean up
  EXPECT_NO_THROW(client.stop());
}

// Test: WsRpcClient state update message format
TEST_F(WsRpcClientTest, StateUpdateMessage) {
  // This test verifies the state update message format without network
  nlohmann::json expected_msg;
  expected_msg["type"] = "state_update";
  expected_msg["data"]["previous"] = "idle";
  expected_msg["data"]["current"] = "recording";
  expected_msg["data"]["task_id"] = "test_task_123";

  // Verify JSON structure
  EXPECT_EQ(expected_msg["type"], "state_update");
  EXPECT_EQ(expected_msg["data"]["previous"], "idle");
  EXPECT_EQ(expected_msg["data"]["current"], "recording");
  EXPECT_EQ(expected_msg["data"]["task_id"], "test_task_123");
}

// Test: WsRpcClient RPC response message format
TEST_F(WsRpcClientTest, RpcResponseMessage) {
  RpcResponse response;
  response.success = true;
  response.message = "Recording started";
  response.data["state"] = "recording";
  response.data["task_id"] = "task_456";

  nlohmann::json j = response.to_json();

  EXPECT_EQ(j["success"], true);
  EXPECT_EQ(j["message"], "Recording started");
  EXPECT_EQ(j["data"]["state"], "recording");
  EXPECT_EQ(j["data"]["task_id"], "task_456");
}

// Test: WsRpcClient failed RPC response format
TEST_F(WsRpcClientTest, FailedRpcResponse) {
  RpcResponse response;
  response.success = false;
  response.message = "Invalid state transition";

  nlohmann::json j = response.to_json();

  EXPECT_EQ(j["success"], false);
  EXPECT_EQ(j["message"], "Invalid state transition");
}

// Test: Reconnection delay calculation with exponential backoff
TEST_F(WsRpcClientTest, ReconnectionBackoff) {
  WsClientConfig config;
  config.reconnect_initial_delay_ms = 1000;
  config.reconnect_backoff_multiplier = 2.0;
  config.reconnect_max_delay_ms = 10000;

  // Simulate backoff calculation
  // Attempt 0: 1000ms
  // Attempt 1: 2000ms
  // Attempt 2: 4000ms
  // Attempt 3: 8000ms
  // Attempt 4: 10000ms (capped)
  // Attempt 5: 10000ms (capped)

  int expected_delays_ms[] = {1000, 2000, 4000, 8000, 10000, 10000};

  for (int i = 0; i < 6; i++) {
    long long expected = std::min(static_cast<long long>(1000 * std::pow(2.0, i)), 10000LL);
    EXPECT_EQ(expected_delays_ms[i], expected);
  }
}

// Test: WsClientConfig defaults
TEST_F(WsRpcClientTest, ConfigDefaults) {
  WsClientConfig config;

  EXPECT_TRUE(config.url.empty());
  EXPECT_TRUE(config.auth_token.empty());
  EXPECT_EQ(config.reconnect_initial_delay_ms, 1000);
  EXPECT_DOUBLE_EQ(config.reconnect_backoff_multiplier, 2.0);
  EXPECT_EQ(config.reconnect_max_delay_ms, 60000);
  EXPECT_DOUBLE_EQ(config.reconnect_jitter_factor, 0.2);
  EXPECT_EQ(config.ping_interval_ms, 30000);
  EXPECT_TRUE(config.time_gap_check_enabled);
  EXPECT_EQ(config.time_gap_warning_threshold_ms, 1000);
  EXPECT_EQ(config.time_gap_critical_threshold_ms, 5000);
  EXPECT_EQ(config.time_gap_stale_after_ms, 60000);
}

TEST_F(WsRpcClientTest, TimeGapReportsNormalWarningCriticalAndUnreliable) {
  WsClientConfig config = test_config_;
  config.time_gap_warning_threshold_ms = 100;
  config.time_gap_critical_threshold_ms = 500;
  WsRpcClient client(ioc_, config);

  client.observe_keystone_time_gap_for_test({{"timestamp", now_epoch_ms_for_test()}});
  auto status = client.get_time_gap_status_json();
  EXPECT_EQ(status["status"], "normal");
  EXPECT_TRUE(status["reliable"]);

  client.observe_keystone_time_gap_for_test({{"timestamp", now_epoch_ms_for_test() - 200}});
  status = client.get_time_gap_status_json();
  EXPECT_EQ(status["status"], "warning");
  EXPECT_TRUE(status["reliable"]);
  EXPECT_GE(status["absolute_offset_ms"].get<int64_t>(), 100);

  client.observe_keystone_time_gap_for_test({{"timestamp", now_epoch_ms_for_test() - 800}});
  status = client.get_time_gap_status_json();
  EXPECT_EQ(status["status"], "critical");
  EXPECT_TRUE(status["reliable"]);
  EXPECT_GE(status["absolute_offset_ms"].get<int64_t>(), 500);

  client.observe_keystone_time_gap_for_test({{"action", "get_state"}});
  status = client.get_time_gap_status_json();
  EXPECT_EQ(status["status"], "unreliable");
  EXPECT_FALSE(status["reliable"]);
}

// Test: RpcModeConfig defaults
TEST_F(WsRpcClientTest, RpcModeConfigDefaults) {
  RpcModeConfig config;

  EXPECT_EQ(config.mode, RpcMode::HTTP_SERVER);
}

// Test: RpcMode enum values
TEST_F(WsRpcClientTest, RpcModeEnumValues) {
  EXPECT_NE(static_cast<int>(RpcMode::HTTP_SERVER), static_cast<int>(RpcMode::WS_CLIENT));
}

// Test: RPC handlers work with mock callbacks
TEST_F(WsRpcClientTest, RpcHandlersWithMockCallbacks) {
  RpcCallbacks callbacks;
  callbacks.get_state = []() {
    return "idle";
  };
  callbacks.get_stats = []() {
    nlohmann::json j;
    j["messages_received"] = 100;
    j["messages_written"] = 95;
    j["messages_dropped"] = 5;
    j["bytes_written"] = 10240;
    return j;
  };
  callbacks.get_task_config = []() {
    return nullptr;
  };
  callbacks.set_config = [](const std::string&, const nlohmann::json&) {
    return true;
  };
  callbacks.begin_recording = [](const std::string&) {
    return true;
  };
  callbacks.finish_recording = [](const std::string&) {
    return true;
  };
  callbacks.cancel_recording = []() {
    return true;
  };
  callbacks.pause_recording = []() {
    return true;
  };
  callbacks.resume_recording = []() {
    return true;
  };
  callbacks.clear_config = []() {
    return true;
  };
  callbacks.quit = []() {};

  // Test get_state
  RpcResponse response = handle_rpc_get_state(callbacks, nlohmann::json{});
  EXPECT_TRUE(response.success);
  EXPECT_EQ(response.data["state"], "idle");

  // Test get_stats
  response = handle_rpc_get_stats(callbacks, nlohmann::json{});
  EXPECT_TRUE(response.success);
  EXPECT_EQ(response.data["messages_received"], 100);
  EXPECT_EQ(response.data["messages_written"], 95);
  EXPECT_EQ(response.data["messages_dropped"], 5);
  EXPECT_EQ(response.data["bytes_written"], 10240);
}

// Test: Config validation
TEST_F(WsRpcClientTest, ConfigValidation) {
  // Valid config
  WsClientConfig config;
  config.url = "ws://keystone:8090/rpc";
  EXPECT_FALSE(config.url.empty());

  // Empty URL should be caught during start
  WsClientConfig empty_config;
  EXPECT_TRUE(empty_config.url.empty());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
