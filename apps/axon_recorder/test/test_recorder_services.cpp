/**
 * @file test_recorder_services.cpp
 * @brief Integration tests for axon_recorder services
 *
 * These tests verify the service adapter plugin functionality
 */

#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "axon_utils/common_types.hpp"
#include "axon_utils/state_machine.hpp"

using namespace axon::utils;

// ============================================================================
// State Machine Tests
// ============================================================================

enum class TestState { IDLE, READY, RECORDING, PAUSED };

class StateMachineTest : public ::testing::Test {
protected:
  void SetUp() override {
    transitions = {
      {TestState::IDLE, {TestState::READY}},
      {TestState::READY, {TestState::IDLE, TestState::RECORDING}},
      {TestState::RECORDING, {TestState::PAUSED, TestState::IDLE}},
      {TestState::PAUSED, {TestState::RECORDING, TestState::IDLE}}};
    state_machine = std::make_unique<StateMachine<TestState>>(TestState::IDLE, transitions);
  }

  std::unordered_map<TestState, std::vector<TestState>> transitions;
  std::unique_ptr<StateMachine<TestState>> state_machine;
};

TEST_F(StateMachineTest, InitialState) {
  EXPECT_EQ(state_machine->get_state(), TestState::IDLE);
}

TEST_F(StateMachineTest, ValidTransition) {
  std::string error;
  ASSERT_TRUE(state_machine->transition_to(TestState::READY, error));
  EXPECT_EQ(state_machine->get_state(), TestState::READY);
  EXPECT_TRUE(error.empty());
}

TEST_F(StateMachineTest, InvalidTransition) {
  std::string error;
  // Cannot transition from IDLE to RECORDING directly
  ASSERT_FALSE(state_machine->transition_to(TestState::RECORDING, error));
  EXPECT_EQ(state_machine->get_state(), TestState::IDLE);
  EXPECT_FALSE(error.empty());
}

TEST_F(StateMachineTest, StateRollbackOnFailure) {
  std::string error;
  ASSERT_TRUE(state_machine->transition_to(TestState::READY, error));

  // Attempt invalid transition
  ASSERT_FALSE(state_machine->transition_to(TestState::PAUSED, error));

  // State should remain READY
  EXPECT_EQ(state_machine->get_state(), TestState::READY);
}

TEST_F(StateMachineTest, CompleteRecordingCycle) {
  std::string error;

  // IDLE -> READY
  ASSERT_TRUE(state_machine->transition_to(TestState::READY, error));
  EXPECT_EQ(state_machine->get_state(), TestState::READY);

  // READY -> RECORDING
  ASSERT_TRUE(state_machine->transition_to(TestState::RECORDING, error));
  EXPECT_EQ(state_machine->get_state(), TestState::RECORDING);

  // RECORDING -> PAUSED
  ASSERT_TRUE(state_machine->transition_to(TestState::PAUSED, error));
  EXPECT_EQ(state_machine->get_state(), TestState::PAUSED);

  // PAUSED -> RECORDING
  ASSERT_TRUE(state_machine->transition_to(TestState::RECORDING, error));
  EXPECT_EQ(state_machine->get_state(), TestState::RECORDING);

  // RECORDING -> IDLE
  ASSERT_TRUE(state_machine->transition_to(TestState::IDLE, error));
  EXPECT_EQ(state_machine->get_state(), TestState::IDLE);
}

TEST_F(StateMachineTest, IsStateIn) {
  EXPECT_TRUE(state_machine->is_state_in({TestState::IDLE}));
  EXPECT_TRUE(state_machine->is_state_in({TestState::READY, TestState::IDLE}));
  EXPECT_FALSE(state_machine->is_state_in({TestState::READY, TestState::RECORDING}));
}

TEST_F(StateMachineTest, GetStateString) {
  auto state_str = state_machine->get_state_string();
  EXPECT_FALSE(state_str.empty());
  // For enum class, this will return the enum name or custom string
}

// ============================================================================
// TaskConfig Tests
// ============================================================================

class TaskConfigIntegrationTest : public ::testing::Test {
protected:
  TaskConfig create_valid_config() {
    TaskConfig config;
    config.task_id = "test_task_123";
    config.device_id = "test_device_001";
    config.data_collector_id = "test_collector_001";
    config.order_id = "test_order_001";
    config.operator_name = "test.operator";
    config.scene = "indoor";
    config.subscene = "kitchen";
    config.skills = {"cooking", "navigation"};
    config.factory = "test_factory";
    config.topics = {"/camera/image", "/imu/data"};
    config.start_callback_url = "http://localhost:8080/api/recording/start";
    config.finish_callback_url = "http://localhost:8080/api/recording/finish";
    config.user_token = "test_jwt_token";
    config.cached_at = std::chrono::system_clock::now();
    return config;
  }
};

TEST_F(TaskConfigIntegrationTest, IsValidWithValidConfig) {
  TaskConfig config = create_valid_config();
  EXPECT_TRUE(config.is_valid());
}

TEST_F(TaskConfigIntegrationTest, IsInvalidWithMissingTaskId) {
  TaskConfig config = create_valid_config();
  config.task_id = "";
  EXPECT_FALSE(config.is_valid());
}

TEST_F(TaskConfigIntegrationTest, HasCallbacksWhenBothSet) {
  TaskConfig config = create_valid_config();
  EXPECT_TRUE(config.has_callbacks());
}

TEST_F(TaskConfigIntegrationTest, HasCallbacksWhenOnlyStartSet) {
  TaskConfig config = create_valid_config();
  config.finish_callback_url = "";
  EXPECT_TRUE(config.has_callbacks());
}

TEST_F(TaskConfigIntegrationTest, HasCallbacksWhenNoneSet) {
  TaskConfig config = create_valid_config();
  config.start_callback_url = "";
  config.finish_callback_url = "";
  EXPECT_FALSE(config.has_callbacks());
}

TEST_F(TaskConfigIntegrationTest, GenerateOutputFilename) {
  TaskConfig config = create_valid_config();
  config.task_id = "my_recording_123";
  EXPECT_EQ(config.generate_output_filename(), "my_recording_123.mcap");
}

// ============================================================================
// TaskConfigCache Tests
// ============================================================================

class TaskConfigCacheIntegrationTest : public ::testing::Test {
protected:
  TaskConfig create_test_config(const std::string& task_id = "test_task") {
    TaskConfig config;
    config.task_id = task_id;
    config.device_id = "test_device";
    config.topics = {"/test_topic"};
    config.cached_at = std::chrono::system_clock::now();
    return config;
  }
};

TEST_F(TaskConfigCacheIntegrationTest, CacheAndRetrieve) {
  TaskConfigCache cache;
  TaskConfig config = create_test_config();

  cache.cache(config);

  EXPECT_TRUE(cache.has_config());
  EXPECT_EQ(cache.get_task_id(), "test_task");

  auto retrieved = cache.get();
  ASSERT_TRUE(retrieved.has_value());
  EXPECT_EQ(retrieved->task_id, "test_task");
  EXPECT_EQ(retrieved->device_id, "test_device");
}

TEST_F(TaskConfigCacheIntegrationTest, ClearCache) {
  TaskConfigCache cache;
  cache.cache(create_test_config());

  EXPECT_TRUE(cache.has_config());

  cache.clear();

  EXPECT_FALSE(cache.has_config());
  EXPECT_TRUE(cache.get_task_id().empty());
}

TEST_F(TaskConfigCacheIntegrationTest, OverwriteCache) {
  TaskConfigCache cache;

  cache.cache(create_test_config("task_001"));
  EXPECT_EQ(cache.get_task_id(), "task_001");

  cache.cache(create_test_config("task_002"));
  EXPECT_EQ(cache.get_task_id(), "task_002");
}

TEST_F(TaskConfigCacheIntegrationTest, MatchesTaskId) {
  TaskConfigCache cache;
  cache.cache(create_test_config("specific_task"));

  EXPECT_TRUE(cache.matches_task_id("specific_task"));
  EXPECT_FALSE(cache.matches_task_id("other_task"));
  EXPECT_FALSE(cache.matches_task_id(""));
}

TEST_F(TaskConfigCacheIntegrationTest, ThreadSafety) {
  TaskConfigCache cache;
  const int num_threads = 4;
  const int iterations = 100;

  std::vector<std::thread> threads;
  std::atomic<int> success_count{0};

  for (int t = 0; t < num_threads; ++t) {
    threads.emplace_back([&, t]() {
      for (int i = 0; i < iterations; ++i) {
        auto config = create_test_config("task_" + std::to_string(t) + "_" + std::to_string(i));
        cache.cache(config);
        auto retrieved = cache.get();
        cache.has_config();
        cache.get_task_id();
        success_count++;
      }
    });
  }

  for (auto& thread : threads) {
    thread.join();
  }

  EXPECT_EQ(success_count.load(), num_threads * iterations);
}

// ============================================================================
// Service Callback Payload Tests
// ============================================================================

class CallbackPayloadTest : public ::testing::Test {
protected:
  StartCallbackPayload create_start_payload() {
    StartCallbackPayload payload;
    payload.task_id = "task_123";
    payload.device_id = "device_001";
    payload.status = "started";
    payload.started_at = "2025-01-08T12:00:00Z";
    payload.topics = {"/camera", "/imu"};
    return payload;
  }

  FinishCallbackPayload create_finish_payload() {
    FinishCallbackPayload payload;
    payload.task_id = "task_123";
    payload.device_id = "device_001";
    payload.status = "completed";
    payload.started_at = "2025-01-08T12:00:00Z";
    payload.finished_at = "2025-01-08T12:05:00Z";
    payload.duration_sec = 300.0;
    payload.message_count = 1000;
    payload.file_size_bytes = 1024000;
    payload.output_path = "/data/recordings/task_123.mcap";
    payload.sidecar_path = "/data/recordings/task_123.json";
    payload.topics = {"/camera", "/imu"};

    CallbackMetadata metadata;
    metadata.scene = "indoor";
    metadata.subscene = "kitchen";
    metadata.skills = {"cooking"};
    metadata.factory = "test_factory";
    payload.metadata = metadata;

    return payload;
  }
};

TEST_F(CallbackPayloadTest, StartPayloadToJson) {
  StartCallbackPayload payload = create_start_payload();
  std::string json = payload.to_json();

  EXPECT_FALSE(json.empty());
  EXPECT_NE(json.find("task_123"), std::string::npos);
  EXPECT_NE(json.find("device_001"), std::string::npos);
  EXPECT_NE(json.find("/camera"), std::string::npos);
}

TEST_F(CallbackPayloadTest, FinishPayloadToJson) {
  FinishCallbackPayload payload = create_finish_payload();
  std::string json = payload.to_json();

  EXPECT_FALSE(json.empty());
  EXPECT_NE(json.find("task_123"), std::string::npos);
  EXPECT_NE(json.find("300.0"), std::string::npos);
  EXPECT_NE(json.find("indoor"), std::string::npos);
}

TEST_F(CallbackPayloadTest, FinishPayloadWithError) {
  FinishCallbackPayload payload = create_finish_payload();
  payload.error = "Recording failed: disk full";

  std::string json = payload.to_json();

  EXPECT_NE(json.find("Recording failed"), std::string::npos);
}

TEST_F(CallbackPayloadTest, CallbackMetadataToJson) {
  CallbackMetadata metadata;
  metadata.scene = "warehouse";
  metadata.subscene = "picking";
  metadata.skills = {"navigation", "manipulation"};
  metadata.factory = "factory_01";

  std::string json = metadata.to_json();

  EXPECT_NE(json.find("warehouse"), std::string::npos);
  EXPECT_NE(json.find("navigation"), std::string::npos);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
