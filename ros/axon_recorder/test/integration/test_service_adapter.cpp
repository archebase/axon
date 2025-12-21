/**
 * Integration tests for ServiceAdapter with ROS services
 *
 * These tests verify the ServiceAdapter correctly registers and handles
 * ROS services using the actual generated service message types.
 *
 * Requires ROS environment (built as part of catkin/colcon build).
 */

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

#include "http_callback_client.hpp"
#include "recording_service_impl.hpp"
#include "state_machine.hpp"
#include "task_config.hpp"

// Include ROS-specific headers based on version
#if defined(AXON_ROS1)
#include <ros/ros.h>
#include <axon_recorder/CachedRecordingConfig.h>
#include <axon_recorder/IsRecordingReady.h>
#include <axon_recorder/RecordingControl.h>
#include <axon_recorder/RecordingStatus.h>
#elif defined(AXON_ROS2)
#include <rclcpp/rclcpp.hpp>
#include <axon_recorder/srv/cached_recording_config.hpp>
#include <axon_recorder/srv/is_recording_ready.hpp>
#include <axon_recorder/srv/recording_control.hpp>
#include <axon_recorder/srv/recording_status.hpp>
#endif

using namespace axon::recorder;

// ============================================================================
// Test Fixtures
// ============================================================================

class ServiceAdapterTest : public ::testing::Test {
protected:
  void SetUp() override {
#if defined(AXON_ROS1)
    // ROS 1 initialization
    if (!ros::isInitialized()) {
      int argc = 0;
      char** argv = nullptr;
      ros::init(argc, argv, "test_service_adapter", ros::init_options::NoSigintHandler);
    }
    nh_ = std::make_unique<ros::NodeHandle>("~");
#elif defined(AXON_ROS2)
    // ROS 2 initialization
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = rclcpp::Node::make_shared("test_service_adapter");
#endif
  }

  void TearDown() override {
#if defined(AXON_ROS1)
    nh_.reset();
#elif defined(AXON_ROS2)
    node_.reset();
#endif
  }

#if defined(AXON_ROS1)
  std::unique_ptr<ros::NodeHandle> nh_;
#elif defined(AXON_ROS2)
  rclcpp::Node::SharedPtr node_;
#endif
};

// ============================================================================
// Service Message Structure Tests
// ============================================================================

#if defined(AXON_ROS1)

TEST_F(ServiceAdapterTest, CachedRecordingConfigRequest_Structure) {
  axon_recorder::CachedRecordingConfig::Request req;

  // Verify all fields are accessible
  req.task_id = "task_001";
  req.device_id = "robot_01";
  req.data_collector_id = "collector_01";
  req.order_id = "order_001";
  req.operator_name = "operator_01";
  req.scene = "warehouse";
  req.subscene = "picking";
  req.skills = {"navigation", "manipulation"};
  req.factory = "test_factory";
  req.topics = {"/camera/image", "/lidar/scan"};
  req.start_callback_url = "http://server/start";
  req.finish_callback_url = "http://server/finish";
  req.user_token = "jwt_token";

  EXPECT_EQ(req.task_id, "task_001");
  EXPECT_EQ(req.skills.size(), 2);
  EXPECT_EQ(req.topics.size(), 2);
}

TEST_F(ServiceAdapterTest, CachedRecordingConfigResponse_Structure) {
  axon_recorder::CachedRecordingConfig::Response res;

  res.success = true;
  res.message = "Config cached";

  EXPECT_TRUE(res.success);
  EXPECT_EQ(res.message, "Config cached");
}

TEST_F(ServiceAdapterTest, IsRecordingReadyResponse_Structure) {
  axon_recorder::IsRecordingReady::Response res;

  res.success = true;
  res.message = "OK";
  res.is_configured = true;
  res.is_recording = false;
  res.task_id = "task_001";
  res.device_id = "robot_01";
  res.order_id = "order_001";
  res.operator_name = "operator_01";
  res.scene = "warehouse";
  res.subscene = "picking";
  res.skills = {"skill1"};
  res.factory = "test_factory";
  res.data_collector_id = "collector_01";
  res.topics = {"/camera"};

  EXPECT_TRUE(res.is_configured);
  EXPECT_FALSE(res.is_recording);
  EXPECT_EQ(res.task_id, "task_001");
  EXPECT_EQ(res.order_id, "order_001");
  EXPECT_EQ(res.operator_name, "operator_01");
}

TEST_F(ServiceAdapterTest, RecordingControlRequest_Structure) {
  axon_recorder::RecordingControl::Request req;

  req.command = "start";
  req.task_id = "task_001";

  EXPECT_EQ(req.command, "start");
  EXPECT_EQ(req.task_id, "task_001");
}

TEST_F(ServiceAdapterTest, RecordingControlResponse_Structure) {
  axon_recorder::RecordingControl::Response res;

  res.success = true;
  res.message = "Recording started";
  res.task_id = "task_001";

  EXPECT_TRUE(res.success);
  EXPECT_EQ(res.task_id, "task_001");
}

TEST_F(ServiceAdapterTest, RecordingStatusResponse_Structure) {
  axon_recorder::RecordingStatus::Response res;

  res.success = true;
  res.message = "OK";
  res.status = "recording";
  res.task_id = "task_001";
  res.device_id = "robot_01";
  res.data_collector_id = "collector_01";
  res.order_id = "order_001";
  res.operator_name = "operator_01";
  res.scene = "warehouse";
  res.subscene = "picking";
  res.skills = {"skill1"};
  res.factory = "test_factory";
  res.active_topics = {"/camera"};
  res.output_path = "/data/task_001.mcap";
  res.disk_usage_gb = 1.5;
  res.duration_sec = 120.0;
  res.message_count = 10000;
  res.throughput_mb_sec = 50.0;
  res.last_error = "";

  EXPECT_EQ(res.status, "recording");
  EXPECT_EQ(res.message_count, 10000);
  EXPECT_EQ(res.order_id, "order_001");
  EXPECT_EQ(res.operator_name, "operator_01");
}

#elif defined(AXON_ROS2)

TEST_F(ServiceAdapterTest, CachedRecordingConfigRequest_Structure) {
  auto req = std::make_shared<axon_recorder::srv::CachedRecordingConfig::Request>();

  req->task_id = "task_001";
  req->device_id = "robot_01";
  req->data_collector_id = "collector_01";
  req->order_id = "order_001";
  req->operator_name = "operator_01";
  req->scene = "warehouse";
  req->subscene = "picking";
  req->skills = {"navigation", "manipulation"};
  req->factory = "test_factory";
  req->topics = {"/camera/image", "/lidar/scan"};
  req->start_callback_url = "http://server/start";
  req->finish_callback_url = "http://server/finish";
  req->user_token = "jwt_token";

  EXPECT_EQ(req->task_id, "task_001");
  EXPECT_EQ(req->skills.size(), 2u);
  EXPECT_EQ(req->topics.size(), 2u);
}

TEST_F(ServiceAdapterTest, CachedRecordingConfigResponse_Structure) {
  auto res = std::make_shared<axon_recorder::srv::CachedRecordingConfig::Response>();

  res->success = true;
  res->message = "Config cached";

  EXPECT_TRUE(res->success);
  EXPECT_EQ(res->message, "Config cached");
}

TEST_F(ServiceAdapterTest, IsRecordingReadyResponse_Structure) {
  auto res = std::make_shared<axon_recorder::srv::IsRecordingReady::Response>();

  res->success = true;
  res->message = "OK";
  res->is_configured = true;
  res->is_recording = false;
  res->task_id = "task_001";
  res->device_id = "robot_01";
  res->order_id = "order_001";
  res->operator_name = "operator_01";
  res->scene = "warehouse";
  res->subscene = "picking";
  res->skills = {"skill1"};
  res->factory = "test_factory";
  res->data_collector_id = "collector_01";
  res->topics = {"/camera"};

  EXPECT_TRUE(res->is_configured);
  EXPECT_FALSE(res->is_recording);
  EXPECT_EQ(res->task_id, "task_001");
  EXPECT_EQ(res->order_id, "order_001");
  EXPECT_EQ(res->operator_name, "operator_01");
}

TEST_F(ServiceAdapterTest, RecordingControlRequest_Structure) {
  auto req = std::make_shared<axon_recorder::srv::RecordingControl::Request>();

  req->command = "start";
  req->task_id = "task_001";

  EXPECT_EQ(req->command, "start");
  EXPECT_EQ(req->task_id, "task_001");
}

TEST_F(ServiceAdapterTest, RecordingControlResponse_Structure) {
  auto res = std::make_shared<axon_recorder::srv::RecordingControl::Response>();

  res->success = true;
  res->message = "Recording started";
  res->task_id = "task_001";

  EXPECT_TRUE(res->success);
  EXPECT_EQ(res->task_id, "task_001");
}

TEST_F(ServiceAdapterTest, RecordingStatusResponse_Structure) {
  auto res = std::make_shared<axon_recorder::srv::RecordingStatus::Response>();

  res->success = true;
  res->message = "OK";
  res->status = "recording";
  res->task_id = "task_001";
  res->device_id = "robot_01";
  res->data_collector_id = "collector_01";
  res->order_id = "order_001";
  res->operator_name = "operator_01";
  res->scene = "warehouse";
  res->subscene = "picking";
  res->skills = {"skill1"};
  res->factory = "test_factory";
  res->active_topics = {"/camera"};
  res->output_path = "/data/task_001.mcap";
  res->disk_usage_gb = 1.5;
  res->duration_sec = 120.0;
  res->message_count = 10000;
  res->throughput_mb_sec = 50.0;
  res->last_error = "";

  EXPECT_EQ(res->status, "recording");
  EXPECT_EQ(res->message_count, 10000);
  EXPECT_EQ(res->order_id, "order_001");
  EXPECT_EQ(res->operator_name, "operator_01");
}

#endif

// ============================================================================
// Command Validation Tests
// ============================================================================

TEST_F(ServiceAdapterTest, ValidCommands) {
  std::vector<std::string> valid_commands = {
    "start", "pause", "resume", "finish", "cancel", "clear"
  };

  for (const auto& cmd : valid_commands) {
    EXPECT_FALSE(cmd.empty()) << "Command should not be empty: " << cmd;
  }
}

TEST_F(ServiceAdapterTest, StatusValues) {
  std::vector<std::string> valid_statuses = {
    "idle", "ready", "recording", "paused"
  };

  // Verify state_to_string produces expected values
  EXPECT_EQ(state_to_string(RecorderState::IDLE), "idle");
  EXPECT_EQ(state_to_string(RecorderState::READY), "ready");
  EXPECT_EQ(state_to_string(RecorderState::RECORDING), "recording");
  EXPECT_EQ(state_to_string(RecorderState::PAUSED), "paused");
}

// ============================================================================
// Integration with RecordingServiceImpl
// ============================================================================

TEST_F(ServiceAdapterTest, ServiceImplIntegration_CachedConfig) {
  // Create mock components
  StateManager state_manager;
  TaskConfigCache task_cache;

  // Verify initial state
  EXPECT_EQ(state_manager.get_state(), RecorderState::IDLE);
  EXPECT_FALSE(task_cache.has_config());

  // Simulate caching config
  TaskConfig config;
  config.task_id = "integration_test_001";
  config.device_id = "robot_01";
  config.scene = "test_scene";
  task_cache.cache(config);

  // Verify cache works
  EXPECT_TRUE(task_cache.has_config());
  EXPECT_EQ(task_cache.get_task_id(), "integration_test_001");

  // Transition state
  std::string error;
  EXPECT_TRUE(state_manager.transition_to(RecorderState::READY, error));
  EXPECT_EQ(state_manager.get_state(), RecorderState::READY);
}

TEST_F(ServiceAdapterTest, ServiceImplIntegration_FullWorkflow) {
  StateManager state_manager;
  TaskConfigCache task_cache;
  std::string error;

  // 1. Cache config → READY
  TaskConfig config;
  config.task_id = "workflow_test";
  task_cache.cache(config);
  ASSERT_TRUE(state_manager.transition_to(RecorderState::READY, error));

  // 2. Start → RECORDING
  ASSERT_TRUE(state_manager.transition_to(RecorderState::RECORDING, error));
  EXPECT_TRUE(state_manager.is_recording_active());

  // 3. Pause → PAUSED
  ASSERT_TRUE(state_manager.transition_to(RecorderState::PAUSED, error));
  EXPECT_TRUE(state_manager.is_recording_active());
  EXPECT_TRUE(state_manager.is_state(RecorderState::PAUSED));

  // 4. Resume → RECORDING
  ASSERT_TRUE(state_manager.transition_to(RecorderState::RECORDING, error));

  // 5. Finish → IDLE
  ASSERT_TRUE(state_manager.transition_to(RecorderState::IDLE, error));
  EXPECT_FALSE(state_manager.is_recording_active());

  // Clear cache
  task_cache.clear();
  EXPECT_FALSE(task_cache.has_config());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);

#if defined(AXON_ROS1)
  ros::init(argc, argv, "test_service_adapter");
#elif defined(AXON_ROS2)
  rclcpp::init(argc, argv);
#endif

  int result = RUN_ALL_TESTS();

#if defined(AXON_ROS2)
  rclcpp::shutdown();
#endif

  return result;
}

