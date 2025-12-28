/**
 * Integration tests for ServiceAdapter using REAL RecorderNode
 *
 * These tests exercise the actual service registration and callback paths
 * using real RecorderNode instances (not mocks).
 *
 * Coverage targets:
 * - service_adapter.cpp: register_services(), handle_*() callbacks
 * - recorder_node.cpp: start_recording(), stop_recording(), pause_recording(), etc.
 *
 * Requires: ROS environment with axon_recorder service types generated
 */

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>

#include "config_parser.hpp"
#include "recorder_node.hpp"

#if defined(AXON_ROS2)
#include <rclcpp/rclcpp.hpp>
#include <axon_recorder/srv/cached_recording_config.hpp>
#include <axon_recorder/srv/is_recording_ready.hpp>
#include <axon_recorder/srv/recording_control.hpp>
#include <axon_recorder/srv/recording_status.hpp>
#elif defined(AXON_ROS1)
#include <ros/ros.h>
#include <axon_recorder/CachedRecordingConfig.h>
#include <axon_recorder/IsRecordingReady.h>
#include <axon_recorder/RecordingControl.h>
#include <axon_recorder/RecordingStatus.h>
#endif

using namespace axon::recorder;

// ============================================================================
// Helper: Create injectable config for testing
// ============================================================================

static RecorderConfig create_test_config(const std::string& test_name) {
  RecorderConfig config;
  
  // Dataset configuration - use temp directory
  config.dataset.path = "/tmp/axon_service_tests/" + test_name;
  config.dataset.mode = "create";
  
  // Create output directory
  std::filesystem::create_directories(config.dataset.path);
  
  // Logging configuration
  config.logging.console_enabled = true;
  config.logging.console_level = "debug";
  config.logging.file_enabled = false;
  
  // Upload disabled for tests
  config.upload.enabled = false;
  
  // Add a minimal topic for validation (RecorderConfig requires at least one topic)
  TopicConfig topic;
  topic.name = "/test_topic";
  topic.message_type = "std_msgs/msg/String";
  topic.batch_size = 100;
  config.topics.push_back(topic);
  
  return config;
}

// ============================================================================
// Test Fixture - ROS 2
// ============================================================================

#if defined(AXON_ROS2)

class ServiceAdapterTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Create config for this test
    auto config = create_test_config(::testing::UnitTest::GetInstance()->current_test_info()->name());
    
    // Create and initialize real RecorderNode with injected config
    recorder_node_ = RecorderNode::create();
    ASSERT_NE(recorder_node_, nullptr);
    
    // Initialize with injected config (bypasses file lookup)
    ASSERT_TRUE(recorder_node_->initialize(0, nullptr, config));
    
    // Create client node for calling services
    client_node_ = rclcpp::Node::make_shared("test_service_client");
    
    // Start spinning the recorder node in a background thread
    spin_thread_running_ = true;
    spin_thread_ = std::thread([this]() {
      while (spin_thread_running_.load(std::memory_order_acquire) && rclcpp::ok()) {
        auto* ros_interface = recorder_node_->get_ros_interface();
        if (ros_interface) {
          ros_interface->spin_once();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    });
    
    // Give services time to register
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  void TearDown() override {
    // Stop spin thread - use memory_order_release to ensure visibility
    spin_thread_running_.store(false, std::memory_order_release);
    
    // Wait for thread to exit the loop and finish any in-flight spin_once() calls
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    
    // Small delay to ensure any executor operations have fully completed
    // This prevents race conditions where executor is accessed during shutdown
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    // Shutdown recorder node
    if (recorder_node_) {
      recorder_node_->shutdown();
    }
    recorder_node_.reset();
    client_node_.reset();
    
    // Clean up test files
    std::filesystem::remove_all("/tmp/axon_service_tests");
  }

  std::shared_ptr<RecorderNode> recorder_node_;
  rclcpp::Node::SharedPtr client_node_;
  std::thread spin_thread_;
  std::atomic<bool> spin_thread_running_{false};
};

// ============================================================================
// Service Call Tests (Real ROS Communication with Real RecorderNode)
// ============================================================================

TEST_F(ServiceAdapterTest, CachedRecordingConfigService) {
  // Create service client
  auto client = client_node_->create_client<axon_recorder::srv::CachedRecordingConfig>(
    "/axon_recorder/cached_recording_config");
  
  // Wait for service to be available
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)))
    << "CachedRecordingConfig service not available";
  
  // Create request
  auto request = std::make_shared<axon_recorder::srv::CachedRecordingConfig::Request>();
  request->task_id = "cache_test_task";
  request->device_id = "test_robot";
  request->data_collector_id = "collector_01";
  request->order_id = "order_001";
  request->operator_name = "test_operator";
  request->scene = "warehouse";
  request->subscene = "aisle_1";
  request->skills = {"navigation", "mapping"};
  request->factory = "test_factory";
  request->topics = {"/camera/image", "/lidar/scan"};
  request->start_callback_url = "http://test/start";
  request->finish_callback_url = "http://test/finish";
  request->user_token = "test_token";
  
  // Send request
  auto future = client->async_send_request(request);
  
  // Spin to process
  auto result = rclcpp::spin_until_future_complete(client_node_, future, std::chrono::seconds(5));
  ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
  
  // Check response
  auto response = future.get();
  EXPECT_TRUE(response->success);
  EXPECT_FALSE(response->message.empty());
  
  // Verify config was cached in the real RecorderNode
  EXPECT_TRUE(recorder_node_->get_task_config_cache().has_config());
  EXPECT_EQ(recorder_node_->get_task_config_cache().get_task_id(), "cache_test_task");
  
  // Verify state transitioned to READY
  EXPECT_EQ(recorder_node_->get_state_manager().get_state(), RecorderState::READY);
}

TEST_F(ServiceAdapterTest, IsRecordingReadyService_NotConfigured) {
  // Create service client
  auto client = client_node_->create_client<axon_recorder::srv::IsRecordingReady>(
    "/axon_recorder/is_recording_ready");
  
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
  
  // Call without caching config first - should show IDLE state
  auto request = std::make_shared<axon_recorder::srv::IsRecordingReady::Request>();
  auto future = client->async_send_request(request);
  
  auto result = rclcpp::spin_until_future_complete(client_node_, future, std::chrono::seconds(5));
  ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
  
  auto response = future.get();
  EXPECT_TRUE(response->success);
  // No task config cached yet (via CachedRecordingConfig service), so is_configured should be false
  // Note: is_configured reflects TaskConfigCache state (READY/RECORDING/PAUSED), not RecorderConfig injection
  EXPECT_FALSE(response->is_configured);
  EXPECT_FALSE(response->is_recording);
}

TEST_F(ServiceAdapterTest, IsRecordingReadyService_Configured) {
  // First cache a config via service
  {
    auto client = client_node_->create_client<axon_recorder::srv::CachedRecordingConfig>(
      "/axon_recorder/cached_recording_config");
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    auto request = std::make_shared<axon_recorder::srv::CachedRecordingConfig::Request>();
    request->task_id = "ready_test";
    request->device_id = "robot_01";
    request->scene = "test_scene";
    
    auto future = client->async_send_request(request);
    auto result = rclcpp::spin_until_future_complete(client_node_, future, std::chrono::seconds(5));
    ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
    ASSERT_TRUE(future.get()->success);
  }
  
  // Now check IsRecordingReady
  auto client = client_node_->create_client<axon_recorder::srv::IsRecordingReady>(
    "/axon_recorder/is_recording_ready");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
  
  auto request = std::make_shared<axon_recorder::srv::IsRecordingReady::Request>();
  auto future = client->async_send_request(request);
  
  auto result = rclcpp::spin_until_future_complete(client_node_, future, std::chrono::seconds(5));
  ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
  
  auto response = future.get();
  EXPECT_TRUE(response->success);
  EXPECT_TRUE(response->is_configured);
  EXPECT_EQ(response->task_id, "ready_test");
}

TEST_F(ServiceAdapterTest, RecordingControlService_StartRecording) {
  // First cache a config via service
  {
    auto client = client_node_->create_client<axon_recorder::srv::CachedRecordingConfig>(
      "/axon_recorder/cached_recording_config");
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    auto request = std::make_shared<axon_recorder::srv::CachedRecordingConfig::Request>();
    request->task_id = "start_test";
    request->device_id = "robot_01";
    
    auto future = client->async_send_request(request);
    auto result = rclcpp::spin_until_future_complete(client_node_, future, std::chrono::seconds(5));
    ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
    ASSERT_TRUE(future.get()->success);
  }
  
  // Verify state is READY
  EXPECT_EQ(recorder_node_->get_state_manager().get_state(), RecorderState::READY);
  
  // Create RecordingControl client and start
  auto client = client_node_->create_client<axon_recorder::srv::RecordingControl>(
    "/axon_recorder/recording_control");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
  
  auto request = std::make_shared<axon_recorder::srv::RecordingControl::Request>();
  request->command = "start";
  request->task_id = "start_test";
  
  auto future = client->async_send_request(request);
  auto result = rclcpp::spin_until_future_complete(client_node_, future, std::chrono::seconds(5));
  ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
  
  auto response = future.get();
  EXPECT_TRUE(response->success);
  EXPECT_EQ(response->task_id, "start_test");
  
  // Verify real RecorderNode is now recording
  EXPECT_TRUE(recorder_node_->is_recording());
  EXPECT_EQ(recorder_node_->get_state_manager().get_state(), RecorderState::RECORDING);
}

TEST_F(ServiceAdapterTest, RecordingControlService_FullWorkflow) {
  // Cache config first
  {
    auto client = client_node_->create_client<axon_recorder::srv::CachedRecordingConfig>(
      "/axon_recorder/cached_recording_config");
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    auto request = std::make_shared<axon_recorder::srv::CachedRecordingConfig::Request>();
    request->task_id = "workflow_test";
    request->device_id = "robot_01";
    
    auto future = client->async_send_request(request);
    auto result = rclcpp::spin_until_future_complete(client_node_, future, std::chrono::seconds(5));
    ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
    ASSERT_TRUE(future.get()->success);
  }
  
  auto client = client_node_->create_client<axon_recorder::srv::RecordingControl>(
    "/axon_recorder/recording_control");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
  
  auto send_command = [&](const std::string& cmd) -> bool {
    auto request = std::make_shared<axon_recorder::srv::RecordingControl::Request>();
    request->command = cmd;
    request->task_id = "workflow_test";
    
    auto future = client->async_send_request(request);
    auto result = rclcpp::spin_until_future_complete(client_node_, future, std::chrono::seconds(5));
    if (result != rclcpp::FutureReturnCode::SUCCESS) return false;
    
    auto response = future.get();
    return response->success;
  };
  
  // Start recording
  EXPECT_TRUE(send_command("start"));
  EXPECT_TRUE(recorder_node_->is_recording());
  EXPECT_EQ(recorder_node_->get_state_manager().get_state(), RecorderState::RECORDING);
  
  // Pause
  EXPECT_TRUE(send_command("pause"));
  EXPECT_TRUE(recorder_node_->is_paused());
  EXPECT_EQ(recorder_node_->get_state_manager().get_state(), RecorderState::PAUSED);
  
  // Resume
  EXPECT_TRUE(send_command("resume"));
  EXPECT_TRUE(recorder_node_->is_actively_recording());
  EXPECT_EQ(recorder_node_->get_state_manager().get_state(), RecorderState::RECORDING);
  
  // Finish
  EXPECT_TRUE(send_command("finish"));
  EXPECT_FALSE(recorder_node_->is_recording());
  EXPECT_EQ(recorder_node_->get_state_manager().get_state(), RecorderState::IDLE);
}

TEST_F(ServiceAdapterTest, RecordingStatusService) {
  // Cache config and start recording
  {
    auto client = client_node_->create_client<axon_recorder::srv::CachedRecordingConfig>(
      "/axon_recorder/cached_recording_config");
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    auto request = std::make_shared<axon_recorder::srv::CachedRecordingConfig::Request>();
    request->task_id = "status_test";
    request->device_id = "robot_01";
    request->scene = "status_scene";
    request->subscene = "status_subscene";
    request->factory = "status_factory";
    
    auto future = client->async_send_request(request);
    auto result = rclcpp::spin_until_future_complete(client_node_, future, std::chrono::seconds(5));
    ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
    ASSERT_TRUE(future.get()->success);
  }
  
  {
    auto client = client_node_->create_client<axon_recorder::srv::RecordingControl>(
      "/axon_recorder/recording_control");
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    auto request = std::make_shared<axon_recorder::srv::RecordingControl::Request>();
    request->command = "start";
    request->task_id = "status_test";
    
    auto future = client->async_send_request(request);
    auto result = rclcpp::spin_until_future_complete(client_node_, future, std::chrono::seconds(5));
    ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
    ASSERT_TRUE(future.get()->success);
  }
  
  // Query status
  auto client = client_node_->create_client<axon_recorder::srv::RecordingStatus>(
    "/axon_recorder/recording_status");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
  
  auto request = std::make_shared<axon_recorder::srv::RecordingStatus::Request>();
  request->task_id = "status_test";
  
  auto future = client->async_send_request(request);
  auto result = rclcpp::spin_until_future_complete(client_node_, future, std::chrono::seconds(5));
  ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
  
  auto response = future.get();
  EXPECT_TRUE(response->success);
  EXPECT_EQ(response->status, "recording");
  EXPECT_EQ(response->task_id, "status_test");
  EXPECT_EQ(response->device_id, "robot_01");
  EXPECT_EQ(response->scene, "status_scene");
}

TEST_F(ServiceAdapterTest, InvalidCommand) {
  // Cache config first
  {
    auto client = client_node_->create_client<axon_recorder::srv::CachedRecordingConfig>(
      "/axon_recorder/cached_recording_config");
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    auto request = std::make_shared<axon_recorder::srv::CachedRecordingConfig::Request>();
    request->task_id = "invalid_cmd_test";
    
    auto future = client->async_send_request(request);
    auto result = rclcpp::spin_until_future_complete(client_node_, future, std::chrono::seconds(5));
    ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
  }
  
  auto client = client_node_->create_client<axon_recorder::srv::RecordingControl>(
    "/axon_recorder/recording_control");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
  
  auto request = std::make_shared<axon_recorder::srv::RecordingControl::Request>();
  request->command = "invalid_command";
  request->task_id = "invalid_cmd_test";
  
  auto future = client->async_send_request(request);
  auto result = rclcpp::spin_until_future_complete(client_node_, future, std::chrono::seconds(5));
  ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
  
  auto response = future.get();
  EXPECT_FALSE(response->success);
  EXPECT_FALSE(response->message.empty());
}

TEST_F(ServiceAdapterTest, TaskIdMismatch) {
  // Cache config and start recording
  {
    auto client = client_node_->create_client<axon_recorder::srv::CachedRecordingConfig>(
      "/axon_recorder/cached_recording_config");
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    auto request = std::make_shared<axon_recorder::srv::CachedRecordingConfig::Request>();
    request->task_id = "correct_task";
    
    auto future = client->async_send_request(request);
    auto result = rclcpp::spin_until_future_complete(client_node_, future, std::chrono::seconds(5));
    ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
  }
  
  {
    auto client = client_node_->create_client<axon_recorder::srv::RecordingControl>(
      "/axon_recorder/recording_control");
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    auto request = std::make_shared<axon_recorder::srv::RecordingControl::Request>();
    request->command = "start";
    request->task_id = "correct_task";
    
    auto future = client->async_send_request(request);
    auto result = rclcpp::spin_until_future_complete(client_node_, future, std::chrono::seconds(5));
    ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
    ASSERT_TRUE(future.get()->success);
  }
  
  EXPECT_TRUE(recorder_node_->is_recording());
  
  // Try to control with wrong task_id
  auto client = client_node_->create_client<axon_recorder::srv::RecordingControl>(
    "/axon_recorder/recording_control");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
  
  auto request = std::make_shared<axon_recorder::srv::RecordingControl::Request>();
  request->command = "finish";
  request->task_id = "wrong_task_id";
  
  auto future = client->async_send_request(request);
  auto result = rclcpp::spin_until_future_complete(client_node_, future, std::chrono::seconds(5));
  ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
  
  auto response = future.get();
  EXPECT_FALSE(response->success);
  
  // Recording should still be active since command was rejected
  EXPECT_TRUE(recorder_node_->is_recording());
}

TEST_F(ServiceAdapterTest, ClearCommand) {
  // Cache config first
  {
    auto client = client_node_->create_client<axon_recorder::srv::CachedRecordingConfig>(
      "/axon_recorder/cached_recording_config");
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    auto request = std::make_shared<axon_recorder::srv::CachedRecordingConfig::Request>();
    request->task_id = "clear_test";
    
    auto future = client->async_send_request(request);
    auto result = rclcpp::spin_until_future_complete(client_node_, future, std::chrono::seconds(5));
    ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
  }
  
  EXPECT_TRUE(recorder_node_->get_task_config_cache().has_config());
  EXPECT_EQ(recorder_node_->get_state_manager().get_state(), RecorderState::READY);
  
  // Send clear command
  auto client = client_node_->create_client<axon_recorder::srv::RecordingControl>(
    "/axon_recorder/recording_control");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
  
  auto request = std::make_shared<axon_recorder::srv::RecordingControl::Request>();
  request->command = "clear";
  request->task_id = "clear_test";
  
  auto future = client->async_send_request(request);
  auto result = rclcpp::spin_until_future_complete(client_node_, future, std::chrono::seconds(5));
  ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
  
  auto response = future.get();
  EXPECT_TRUE(response->success);
  
  // Config should be cleared and state back to IDLE
  EXPECT_FALSE(recorder_node_->get_task_config_cache().has_config());
  EXPECT_EQ(recorder_node_->get_state_manager().get_state(), RecorderState::IDLE);
}

TEST_F(ServiceAdapterTest, CancelRecording) {
  // Cache config and start recording
  {
    auto client = client_node_->create_client<axon_recorder::srv::CachedRecordingConfig>(
      "/axon_recorder/cached_recording_config");
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    auto request = std::make_shared<axon_recorder::srv::CachedRecordingConfig::Request>();
    request->task_id = "cancel_test";
    
    auto future = client->async_send_request(request);
    auto result = rclcpp::spin_until_future_complete(client_node_, future, std::chrono::seconds(5));
    ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
  }
  
  {
    auto client = client_node_->create_client<axon_recorder::srv::RecordingControl>(
      "/axon_recorder/recording_control");
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
    
    auto request = std::make_shared<axon_recorder::srv::RecordingControl::Request>();
    request->command = "start";
    request->task_id = "cancel_test";
    
    auto future = client->async_send_request(request);
    auto result = rclcpp::spin_until_future_complete(client_node_, future, std::chrono::seconds(5));
    ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
    ASSERT_TRUE(future.get()->success);
  }
  
  EXPECT_TRUE(recorder_node_->is_recording());
  
  // Cancel recording
  auto client = client_node_->create_client<axon_recorder::srv::RecordingControl>(
    "/axon_recorder/recording_control");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
  
  auto request = std::make_shared<axon_recorder::srv::RecordingControl::Request>();
  request->command = "cancel";
  request->task_id = "cancel_test";
  
  auto future = client->async_send_request(request);
  auto result = rclcpp::spin_until_future_complete(client_node_, future, std::chrono::seconds(5));
  ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
  
  auto response = future.get();
  EXPECT_TRUE(response->success);
  EXPECT_FALSE(recorder_node_->is_recording());
  EXPECT_EQ(recorder_node_->get_state_manager().get_state(), RecorderState::IDLE);
}

TEST_F(ServiceAdapterTest, StartWithoutConfig) {
  // Try to start recording without caching config first
  // Note: The initial config was injected in SetUp(), so we need to clear it
  recorder_node_->get_task_config_cache().clear();
  std::string error;
  recorder_node_->get_state_manager().transition_to(RecorderState::IDLE, error);
  
  auto client = client_node_->create_client<axon_recorder::srv::RecordingControl>(
    "/axon_recorder/recording_control");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
  
  auto request = std::make_shared<axon_recorder::srv::RecordingControl::Request>();
  request->command = "start";
  request->task_id = "no_config_test";
  
  auto future = client->async_send_request(request);
  auto result = rclcpp::spin_until_future_complete(client_node_, future, std::chrono::seconds(5));
  ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
  
  auto response = future.get();
  EXPECT_FALSE(response->success);
  EXPECT_FALSE(response->message.empty());
  EXPECT_FALSE(recorder_node_->is_recording());
}

#endif  // AXON_ROS2

// ============================================================================
// Test Fixture - ROS 1
// ============================================================================

#if defined(AXON_ROS1)

class ServiceAdapterTest : public ::testing::Test {
protected:
  void SetUp() override {
    if (!ros::isInitialized()) {
      int argc = 0;
      char** argv = nullptr;
      ros::init(argc, argv, "test_service_adapter", ros::init_options::NoSigintHandler);
    }
    
    nh_ = std::make_unique<ros::NodeHandle>("~");
    
    // Create config for this test
    auto config = create_test_config(::testing::UnitTest::GetInstance()->current_test_info()->name());
    
    // Create and initialize real RecorderNode with injected config
    recorder_node_ = RecorderNode::create();
    ASSERT_NE(recorder_node_, nullptr);
    
    // Initialize with injected config (bypasses file lookup)
    ASSERT_TRUE(recorder_node_->initialize(0, nullptr, config));
    
    // Give services time to register
    ros::Duration(0.1).sleep();
  }

  void TearDown() override {
    if (recorder_node_) {
      recorder_node_->shutdown();
    }
    recorder_node_.reset();
    nh_.reset();
    
    // Clean up test files
    std::filesystem::remove_all("/tmp/axon_service_tests");
  }

  std::unique_ptr<ros::NodeHandle> nh_;
  std::shared_ptr<RecorderNode> recorder_node_;
};

TEST_F(ServiceAdapterTest, CachedRecordingConfigServiceROS1) {
  ros::ServiceClient client = 
    nh_->serviceClient<axon_recorder::CachedRecordingConfig>("/axon_recorder/cached_recording_config");
  
  ASSERT_TRUE(client.waitForExistence(ros::Duration(5.0)));
  
  axon_recorder::CachedRecordingConfig srv;
  srv.request.task_id = "ros1_test_task";
  srv.request.device_id = "ros1_robot";
  srv.request.scene = "ros1_scene";
  
  ASSERT_TRUE(client.call(srv));
  EXPECT_TRUE(srv.response.success);
  
  // Verify config was cached in real RecorderNode
  EXPECT_TRUE(recorder_node_->get_task_config_cache().has_config());
  EXPECT_EQ(recorder_node_->get_task_config_cache().get_task_id(), "ros1_test_task");
  EXPECT_EQ(recorder_node_->get_state_manager().get_state(), RecorderState::READY);
}

TEST_F(ServiceAdapterTest, RecordingControlServiceROS1) {
  // Cache config first
  {
    ros::ServiceClient client = 
      nh_->serviceClient<axon_recorder::CachedRecordingConfig>("/axon_recorder/cached_recording_config");
    ASSERT_TRUE(client.waitForExistence(ros::Duration(5.0)));
    
    axon_recorder::CachedRecordingConfig srv;
    srv.request.task_id = "ros1_control_test";
    srv.request.device_id = "robot_01";
    
    ASSERT_TRUE(client.call(srv));
    ASSERT_TRUE(srv.response.success);
  }
  
  ros::ServiceClient client = 
    nh_->serviceClient<axon_recorder::RecordingControl>("/axon_recorder/recording_control");
  ASSERT_TRUE(client.waitForExistence(ros::Duration(5.0)));
  
  // Start
  {
    axon_recorder::RecordingControl srv;
    srv.request.command = "start";
    srv.request.task_id = "ros1_control_test";
    
    ASSERT_TRUE(client.call(srv));
    EXPECT_TRUE(srv.response.success);
    EXPECT_TRUE(recorder_node_->is_recording());
    EXPECT_EQ(recorder_node_->get_state_manager().get_state(), RecorderState::RECORDING);
  }
  
  // Pause
  {
    axon_recorder::RecordingControl srv;
    srv.request.command = "pause";
    srv.request.task_id = "ros1_control_test";
    
    ASSERT_TRUE(client.call(srv));
    EXPECT_TRUE(srv.response.success);
    EXPECT_TRUE(recorder_node_->is_paused());
  }
  
  // Resume
  {
    axon_recorder::RecordingControl srv;
    srv.request.command = "resume";
    srv.request.task_id = "ros1_control_test";
    
    ASSERT_TRUE(client.call(srv));
    EXPECT_TRUE(srv.response.success);
    EXPECT_TRUE(recorder_node_->is_actively_recording());
  }
  
  // Finish
  {
    axon_recorder::RecordingControl srv;
    srv.request.command = "finish";
    srv.request.task_id = "ros1_control_test";
    
    ASSERT_TRUE(client.call(srv));
    EXPECT_TRUE(srv.response.success);
    EXPECT_FALSE(recorder_node_->is_recording());
    EXPECT_EQ(recorder_node_->get_state_manager().get_state(), RecorderState::IDLE);
  }
}

TEST_F(ServiceAdapterTest, RecordingStatusServiceROS1) {
  // Cache config and start recording
  {
    ros::ServiceClient client = 
      nh_->serviceClient<axon_recorder::CachedRecordingConfig>("/axon_recorder/cached_recording_config");
    ASSERT_TRUE(client.waitForExistence(ros::Duration(5.0)));
    
    axon_recorder::CachedRecordingConfig srv;
    srv.request.task_id = "ros1_status_test";
    srv.request.device_id = "ros1_robot";
    srv.request.scene = "ros1_scene";
    
    ASSERT_TRUE(client.call(srv));
    ASSERT_TRUE(srv.response.success);
  }
  
  {
    ros::ServiceClient client = 
      nh_->serviceClient<axon_recorder::RecordingControl>("/axon_recorder/recording_control");
    ASSERT_TRUE(client.waitForExistence(ros::Duration(5.0)));
    
    axon_recorder::RecordingControl srv;
    srv.request.command = "start";
    srv.request.task_id = "ros1_status_test";
    
    ASSERT_TRUE(client.call(srv));
    ASSERT_TRUE(srv.response.success);
  }
  
  // Query status
  ros::ServiceClient client = 
    nh_->serviceClient<axon_recorder::RecordingStatus>("/axon_recorder/recording_status");
  ASSERT_TRUE(client.waitForExistence(ros::Duration(5.0)));
  
  axon_recorder::RecordingStatus srv;
  srv.request.task_id = "ros1_status_test";
  
  ASSERT_TRUE(client.call(srv));
  EXPECT_TRUE(srv.response.success);
  EXPECT_EQ(srv.response.status, "recording");
  EXPECT_EQ(srv.response.task_id, "ros1_status_test");
  EXPECT_EQ(srv.response.device_id, "ros1_robot");
}

TEST_F(ServiceAdapterTest, CancelRecordingROS1) {
  // Cache config and start recording
  {
    ros::ServiceClient client = 
      nh_->serviceClient<axon_recorder::CachedRecordingConfig>("/axon_recorder/cached_recording_config");
    ASSERT_TRUE(client.waitForExistence(ros::Duration(5.0)));
    
    axon_recorder::CachedRecordingConfig srv;
    srv.request.task_id = "ros1_cancel_test";
    
    ASSERT_TRUE(client.call(srv));
  }
  
  {
    ros::ServiceClient client = 
      nh_->serviceClient<axon_recorder::RecordingControl>("/axon_recorder/recording_control");
    ASSERT_TRUE(client.waitForExistence(ros::Duration(5.0)));
    
    axon_recorder::RecordingControl srv;
    srv.request.command = "start";
    srv.request.task_id = "ros1_cancel_test";
    
    ASSERT_TRUE(client.call(srv));
    ASSERT_TRUE(srv.response.success);
  }
  
  EXPECT_TRUE(recorder_node_->is_recording());
  
  // Cancel
  ros::ServiceClient client = 
    nh_->serviceClient<axon_recorder::RecordingControl>("/axon_recorder/recording_control");
  
  axon_recorder::RecordingControl srv;
  srv.request.command = "cancel";
  srv.request.task_id = "ros1_cancel_test";
  
  ASSERT_TRUE(client.call(srv));
  EXPECT_TRUE(srv.response.success);
  EXPECT_FALSE(recorder_node_->is_recording());
  EXPECT_EQ(recorder_node_->get_state_manager().get_state(), RecorderState::IDLE);
}

#endif  // AXON_ROS1

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  
#if defined(AXON_ROS2)
  rclcpp::init(argc, argv);
#elif defined(AXON_ROS1)
  ros::init(argc, argv, "test_service_adapter", ros::init_options::NoSigintHandler);
#endif
  
  int result = RUN_ALL_TESTS();
  
#if defined(AXON_ROS2)
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
#elif defined(AXON_ROS1)
  if (ros::ok()) {
    ros::shutdown();
  }
#endif
  
  return result;
}

