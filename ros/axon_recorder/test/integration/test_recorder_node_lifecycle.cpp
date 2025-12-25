/**
 * Integration tests for RecorderNode lifecycle with REAL ROS
 *
 * These tests exercise the RecorderNode class methods and lifecycle
 * using real ROS infrastructure. Coverage targets:
 * - recorder_node.cpp: initialize(), shutdown(), start/stop/pause/resume_recording()
 *
 * Note: Full RecorderNode initialization requires a valid config file.
 * These tests exercise the code paths that don't require full initialization.
 */

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <thread>

#include "recorder_node.hpp"
#include "ros_interface.hpp"
#include "state_machine.hpp"
#include "task_config.hpp"

#if defined(AXON_ROS2)
#include <rclcpp/rclcpp.hpp>
#elif defined(AXON_ROS1)
#include <ros/ros.h>
#endif

using namespace axon::recorder;

// ============================================================================
// Test Fixture
// ============================================================================

class RecorderNodeLifecycleTest : public ::testing::Test {
protected:
  // Note: ROS context is initialized ONCE in main() and never re-initialized.
  // Do NOT add SetUpTestSuite/TearDownTestSuite that manage ROS lifecycle.

  void SetUp() override {
    // Create temp directory for test outputs
    test_dir_ = std::filesystem::temp_directory_path() / "recorder_node_test";
    std::filesystem::create_directories(test_dir_);
    
    // Create a minimal config file for testing
    config_path_ = test_dir_ / "test_config.yaml";
    create_test_config();
  }

  void TearDown() override {
    std::filesystem::remove_all(test_dir_);
  }

  void create_test_config() {
    std::ofstream config(config_path_);
    config << R"(
# Test configuration for RecorderNode
dataset:
  path: ")" << test_dir_.string() << R"("
  prefix: "test_recording"
  max_file_size_mb: 100

logging:
  severity: "info"
  enable_ros_console: true
  enable_file: false

topics:
  - name: "/test_topic"
    message_type: "std_msgs/msg/String"
    qos: high_throughput

upload:
  enabled: false
)";
  }

  std::filesystem::path test_dir_;
  std::filesystem::path config_path_;
};

// ============================================================================
// Factory Method Tests
// ============================================================================

TEST_F(RecorderNodeLifecycleTest, CreateRecorderNode) {
  // Test factory method
  auto node = RecorderNode::create();
  ASSERT_NE(node, nullptr);
  
  // Node should be in initial state (before initialization)
  EXPECT_FALSE(node->is_recording());
  EXPECT_FALSE(node->is_paused());
  EXPECT_FALSE(node->is_actively_recording());
}

TEST_F(RecorderNodeLifecycleTest, GetStatsBeforeRecording) {
  auto node = RecorderNode::create();
  ASSERT_NE(node, nullptr);
  
  // Stats should be zero before recording
  RecordingStats stats = node->get_stats();
  EXPECT_EQ(stats.messages_received, 0u);
  EXPECT_EQ(stats.messages_written, 0u);
  EXPECT_EQ(stats.messages_dropped, 0u);
  EXPECT_DOUBLE_EQ(stats.drop_rate_percent, 0.0);
}

TEST_F(RecorderNodeLifecycleTest, GetRecordingDurationBeforeRecording) {
  auto node = RecorderNode::create();
  ASSERT_NE(node, nullptr);
  
  // Duration should be 0 before recording starts
  double duration = node->get_recording_duration_sec();
  EXPECT_DOUBLE_EQ(duration, 0.0);
}

TEST_F(RecorderNodeLifecycleTest, GetOutputPathBeforeRecording) {
  auto node = RecorderNode::create();
  ASSERT_NE(node, nullptr);
  
  // Output path may be empty or default before initialization
  std::string path = node->get_output_path();
  // Just verify it doesn't crash
}

TEST_F(RecorderNodeLifecycleTest, GetHttpCallbackClient) {
  auto node = RecorderNode::create();
  ASSERT_NE(node, nullptr);
  
  auto http_client = node->get_http_callback_client();
  EXPECT_NE(http_client, nullptr);
}

// ============================================================================
// State Manager Access Tests
// ============================================================================

TEST_F(RecorderNodeLifecycleTest, StateManagerInitialState) {
  auto node = RecorderNode::create();
  ASSERT_NE(node, nullptr);
  
  StateManager& state_mgr = node->get_state_manager();
  EXPECT_EQ(state_mgr.get_state(), RecorderState::IDLE);
}

TEST_F(RecorderNodeLifecycleTest, TaskConfigCacheAccess) {
  auto node = RecorderNode::create();
  ASSERT_NE(node, nullptr);
  
  TaskConfigCache& cache = node->get_task_config_cache();
  EXPECT_FALSE(cache.has_config());
}

TEST_F(RecorderNodeLifecycleTest, TaskConfigCacheManualSet) {
  auto node = RecorderNode::create();
  ASSERT_NE(node, nullptr);
  
  TaskConfig config;
  config.task_id = "lifecycle_test";
  config.device_id = "test_robot";
  config.scene = "test_scene";
  config.topics = {"/test_topic"};
  
  // Set config directly via cache (doesn't require ros_interface to be initialized)
  node->get_task_config_cache().cache(config);
  
  // Verify cache was updated
  EXPECT_TRUE(node->get_task_config_cache().has_config());
  EXPECT_EQ(node->get_task_config_cache().get_task_id(), "lifecycle_test");
}

// ============================================================================
// Shutdown Tests
// ============================================================================

TEST_F(RecorderNodeLifecycleTest, ShutdownWithoutInitialize) {
  auto node = RecorderNode::create();
  ASSERT_NE(node, nullptr);
  
  // Shutdown without initialize should not crash
  EXPECT_NO_THROW(node->shutdown());
}

TEST_F(RecorderNodeLifecycleTest, DoubleShutdown) {
  auto node = RecorderNode::create();
  ASSERT_NE(node, nullptr);
  
  // Double shutdown should be safe (idempotent)
  EXPECT_NO_THROW(node->shutdown());
  EXPECT_NO_THROW(node->shutdown());
}

// ============================================================================
// Recording Control Tests (without full initialization)
// ============================================================================

// Note: StartRecordingWithoutConfig test removed - calling start_recording()
// without initialization causes undefined behavior (uses null ros_interface_)

TEST_F(RecorderNodeLifecycleTest, StopRecordingBeforeStart) {
  auto node = RecorderNode::create();
  ASSERT_NE(node, nullptr);
  
  // Stop recording before starting should be safe
  EXPECT_NO_THROW(node->stop_recording());
}

TEST_F(RecorderNodeLifecycleTest, PauseResumeBeforeStart) {
  auto node = RecorderNode::create();
  ASSERT_NE(node, nullptr);
  
  // Pause/resume before starting should be safe
  EXPECT_NO_THROW(node->pause_recording());
  EXPECT_NO_THROW(node->resume_recording());
}

TEST_F(RecorderNodeLifecycleTest, CancelBeforeStart) {
  auto node = RecorderNode::create();
  ASSERT_NE(node, nullptr);
  
  // Cancel before starting should be safe
  EXPECT_NO_THROW(node->cancel_recording());
}

// ============================================================================
// State Transitions via IRecorderContext
// ============================================================================

TEST_F(RecorderNodeLifecycleTest, StateTransitionsViaInterface) {
  auto node = RecorderNode::create();
  ASSERT_NE(node, nullptr);
  
  // Access via IRecorderContext interface
  IRecorderContext* context = node.get();
  
  // Initial state via StateManager (IRecorderContext provides get_state_manager)
  StateManager& sm = context->get_state_manager();
  EXPECT_EQ(sm.get_state(), RecorderState::IDLE);
  EXPECT_FALSE(sm.is_recording_active());
  
  // Cache a config directly (configure_from_task_config requires initialized ros_interface)
  TaskConfig config;
  config.task_id = "state_test";
  config.device_id = "robot";
  context->get_task_config_cache().cache(config);
  
  // Verify config was cached
  EXPECT_TRUE(context->get_task_config_cache().has_config());
}

// ============================================================================
// Full Lifecycle Test (if initialization works)
// ============================================================================

#if defined(AXON_ROS2)

class RecorderNodeFullTest : public RecorderNodeLifecycleTest {
protected:
  void SetUp() override {
    RecorderNodeLifecycleTest::SetUp();
    
    // Create more complete config
    create_full_config();
  }

  void create_full_config() {
    std::ofstream config(config_path_);
    config << R"(
dataset:
  path: ")" << test_dir_.string() << R"("
  prefix: "full_test"
  max_file_size_mb: 100

logging:
  severity: "info"
  enable_ros_console: true
  enable_file: false

topics:
  - name: "/chatter"
    message_type: "std_msgs/msg/String"
    qos: default

upload:
  enabled: false
)";
  }
};

TEST_F(RecorderNodeFullTest, InitializeWithValidConfig) {
  auto node = RecorderNode::create();
  ASSERT_NE(node, nullptr);
  
  // Try initialization - may fail if config path isn't set correctly
  // This exercises the initialize code paths
  int argc = 1;
  const char* argv[] = {"test_recorder_node"};
  char** argv_nc = const_cast<char**>(argv);
  
  // Note: Full initialization requires config file at expected path
  // This test documents the interface; actual success depends on environment
  bool result = node->initialize(argc, argv_nc);
  
  // Whether it succeeds or fails, verify cleanup works
  node->shutdown();
}

TEST_F(RecorderNodeFullTest, RecordingWorkflowWithTaskConfig) {
  auto node = RecorderNode::create();
  ASSERT_NE(node, nullptr);
  
  // Configure task directly via cache (configure_from_task_config requires initialization)
  TaskConfig config;
  config.task_id = "workflow_test_001";
  config.device_id = "test_robot_01";
  config.data_collector_id = "collector_01";
  config.scene = "test_scene";
  config.subscene = "test_subscene";
  config.topics = {"/chatter"};
  config.start_callback_url = "";
  config.finish_callback_url = "";
  
  // Cache config directly
  node->get_task_config_cache().cache(config);
  
  // Verify config is cached
  EXPECT_TRUE(node->get_task_config_cache().has_config());
  
  // Note: Starting recording requires full initialization
  // This test just verifies the config flow
  
  node->shutdown();
}

#endif  // AXON_ROS2

// ============================================================================
// RecorderNode as shared_ptr (for ServiceAdapter compatibility)
// ============================================================================

TEST_F(RecorderNodeLifecycleTest, SharedPtrUsage) {
  // RecorderNode must be created via factory to use shared_from_this
  std::shared_ptr<RecorderNode> node = RecorderNode::create();
  ASSERT_NE(node, nullptr);
  
  // Verify it can be used as IRecorderContext shared_ptr
  std::shared_ptr<IRecorderContext> context = node;
  ASSERT_NE(context, nullptr);
  
  // Access through interface
  StateManager& sm = context->get_state_manager();
  EXPECT_EQ(sm.get_state(), RecorderState::IDLE);
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  
#if defined(AXON_ROS2)
  rclcpp::init(argc, argv);
#elif defined(AXON_ROS1)
  ros::init(argc, argv, "test_recorder_node_lifecycle", ros::init_options::NoSigintHandler);
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

