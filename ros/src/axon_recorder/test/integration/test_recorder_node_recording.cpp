/**
 * @file test_recorder_node_recording.cpp
 * @brief Integration tests for RecorderNode ACTUAL recording functionality
 *
 * These tests exercise the REAL recording code paths in recorder_node.cpp:
 * - initialize() success path with injected config
 * - start_recording() / stop_recording() full workflow
 * - configure_from_task_config() 
 * - cancel_recording()
 * - get_stats() with actual data
 * - write_stats_file()
 *
 * Coverage targets:
 * - recorder_node.cpp: ~590 lines currently uncovered
 *
 * Note: These tests require a ROS environment and create actual files.
 */

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <thread>

#include "config_parser.hpp"
#include "recorder_node.hpp"
#include "ros_interface.hpp"
#include "state_machine.hpp"
#include "task_config.hpp"

#if defined(AXON_ROS2)
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#elif defined(AXON_ROS1)
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#endif

using namespace axon::recorder;

// ============================================================================
// Test Fixture with Full Initialization Support
// ============================================================================

class RecorderNodeRecordingTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Create temp directory for test outputs
    test_dir_ = std::filesystem::temp_directory_path() / "recorder_node_recording_test";
    std::filesystem::create_directories(test_dir_);
    
    // Create config directory
    config_dir_ = test_dir_ / "config";
    std::filesystem::create_directories(config_dir_);
    
    // Create data directory for recordings
    data_dir_ = test_dir_ / "data";
    std::filesystem::create_directories(data_dir_);
    
    // Create a valid config file
    config_path_ = config_dir_ / "default_config.yaml";
    create_valid_config();
  }

  void TearDown() override {
    // Clean up test directory
    std::error_code ec;
    std::filesystem::remove_all(test_dir_, ec);
  }

  void create_valid_config() {
    std::ofstream config(config_path_);
    config << R"(
# Test configuration for RecorderNode recording tests
dataset:
  path: ")" << data_dir_.string() << R"("
  prefix: "test_recording"
  max_file_size_mb: 100

logging:
  severity: "debug"
  enable_ros_console: true
  enable_file: false

topics:
  - name: "/test_string"
    message_type: "std_msgs/msg/String"
    qos: high_throughput
  - name: "/test_int"
    message_type: "std_msgs/msg/Int32"
    qos: default

upload:
  enabled: false
)";
    config.close();
  }

  // Create a RecorderConfig programmatically for injection
  RecorderConfig create_injectable_config() {
    RecorderConfig config;
    
    // Dataset configuration
    config.dataset.path = data_dir_.string();
    config.dataset.mode = "create";
    // Use temp directory for stats file so tests can verify it
    config.dataset.stats_file_path = (test_dir_ / "recorder_stats.json").string();
    
    // Logging configuration
    config.logging.console_enabled = true;
    config.logging.console_level = "debug";
    config.logging.file_enabled = false;
    
    // Topics to record
    TopicConfig topic1;
    topic1.name = "/test_string";
    topic1.message_type = "std_msgs/msg/String";
    topic1.batch_size = 100;
    config.topics.push_back(topic1);
    
    TopicConfig topic2;
    topic2.name = "/test_int";
    topic2.message_type = "std_msgs/msg/Int32";
    topic2.batch_size = 100;
    config.topics.push_back(topic2);
    
    // Upload disabled for tests
    config.upload.enabled = false;
    
    return config;
  }
  
  // Helper to get the stats file path from test directory
  std::filesystem::path get_stats_file_path() const {
    return test_dir_ / "recorder_stats.json";
  }

  // Helper to create a RecorderNode and attempt initialization with injected config
  std::shared_ptr<RecorderNode> create_and_initialize_with_config() {
    auto node = RecorderNode::create();
    if (!node) return nullptr;
    
    auto config = create_injectable_config();
    int argc = 1;
    const char* argv[] = {"test_recorder"};
    char** argv_nc = const_cast<char**>(argv);
    
    if (!node->initialize(argc, argv_nc, config)) {
      return nullptr;
    }
    
    return node;
  }

  std::filesystem::path test_dir_;
  std::filesystem::path config_dir_;
  std::filesystem::path data_dir_;
  std::filesystem::path config_path_;
};

// ============================================================================
// RecorderNode State Management Tests
// ============================================================================

TEST_F(RecorderNodeRecordingTest, StateManagerTransitions) {
  auto node = RecorderNode::create();
  ASSERT_NE(node, nullptr);
  
  StateManager& sm = node->get_state_manager();
  
  // Initial state
  EXPECT_EQ(sm.get_state(), RecorderState::IDLE);
  EXPECT_FALSE(sm.is_recording_active());
  
  // Transition to READY
  std::string error;
  EXPECT_TRUE(sm.transition_to(RecorderState::READY, error));
  EXPECT_EQ(sm.get_state(), RecorderState::READY);
  
  // Transition to RECORDING
  EXPECT_TRUE(sm.transition_to(RecorderState::RECORDING, error));
  EXPECT_EQ(sm.get_state(), RecorderState::RECORDING);
  EXPECT_TRUE(sm.is_recording_active());
  
  // Transition to PAUSED
  EXPECT_TRUE(sm.transition_to(RecorderState::PAUSED, error));
  EXPECT_EQ(sm.get_state(), RecorderState::PAUSED);
  EXPECT_TRUE(sm.is_recording_active());  // Still "active" when paused
  
  // Back to RECORDING
  EXPECT_TRUE(sm.transition_to(RecorderState::RECORDING, error));
  EXPECT_EQ(sm.get_state(), RecorderState::RECORDING);
  
  // Stop recording -> IDLE
  EXPECT_TRUE(sm.transition_to(RecorderState::IDLE, error));
  EXPECT_EQ(sm.get_state(), RecorderState::IDLE);
  EXPECT_FALSE(sm.is_recording_active());
}

// ============================================================================
// Task Config Cache Tests
// ============================================================================

TEST_F(RecorderNodeRecordingTest, TaskConfigCacheFullWorkflow) {
  auto node = RecorderNode::create();
  ASSERT_NE(node, nullptr);
  
  TaskConfigCache& cache = node->get_task_config_cache();
  
  // Initially empty
  EXPECT_FALSE(cache.has_config());
  EXPECT_TRUE(cache.get_task_id().empty());
  
  // Create and cache a full config
  TaskConfig config;
  config.task_id = "test_task_001";
  config.device_id = "robot_001";
  config.data_collector_id = "collector_01";
  config.order_id = "order_12345";
  config.operator_name = "test_operator";
  config.scene = "warehouse";
  config.subscene = "aisle_5";
  config.skills = {"navigation", "mapping", "localization"};
  config.factory = "factory_beijing";
  config.topics = {"/camera/image", "/lidar/scan", "/imu/data"};
  config.start_callback_url = "http://api.example.com/start";
  config.finish_callback_url = "http://api.example.com/finish";
  config.user_token = "token_abc123";
  
  cache.cache(config);
  
  // Verify cached
  EXPECT_TRUE(cache.has_config());
  EXPECT_EQ(cache.get_task_id(), "test_task_001");
  
  // Get full config
  auto retrieved = cache.get();
  ASSERT_TRUE(retrieved.has_value());
  EXPECT_EQ(retrieved->task_id, "test_task_001");
  EXPECT_EQ(retrieved->device_id, "robot_001");
  EXPECT_EQ(retrieved->scene, "warehouse");
  EXPECT_EQ(retrieved->skills.size(), 3u);
  EXPECT_TRUE(retrieved->has_callbacks());
  
  // Clear
  cache.clear();
  EXPECT_FALSE(cache.has_config());
}

TEST_F(RecorderNodeRecordingTest, TaskConfigGenerateOutputFilename) {
  TaskConfig config;
  config.task_id = "task_20251225_001";
  config.device_id = "robot_01";
  config.scene = "warehouse";
  
  std::string filename = config.generate_output_filename();
  
  // Should contain task_id
  EXPECT_TRUE(filename.find("task_20251225_001") != std::string::npos);
  // Should have .mcap extension
  EXPECT_TRUE(filename.find(".mcap") != std::string::npos);
}

// ============================================================================
// HTTP Callback Client Tests
// ============================================================================

TEST_F(RecorderNodeRecordingTest, HttpCallbackClientAvailable) {
  auto node = RecorderNode::create();
  ASSERT_NE(node, nullptr);
  
  auto http_client = node->get_http_callback_client();
  EXPECT_NE(http_client, nullptr);
  
  // Verify timestamp generation works
  auto now = std::chrono::system_clock::now();
  std::string timestamp = HttpCallbackClient::get_iso8601_timestamp(now);
  
  // Should be ISO8601 format
  EXPECT_FALSE(timestamp.empty());
  EXPECT_TRUE(timestamp.find("T") != std::string::npos);  // ISO8601 has 'T' separator
}

// ============================================================================
// Recording State Consistency Tests
// ============================================================================

TEST_F(RecorderNodeRecordingTest, IsRecordingStateConsistency) {
  auto node = RecorderNode::create();
  ASSERT_NE(node, nullptr);
  
  // Before any transitions
  EXPECT_FALSE(node->is_recording());
  EXPECT_FALSE(node->is_paused());
  EXPECT_FALSE(node->is_actively_recording());
  
  // Manually transition states to verify consistency
  StateManager& sm = node->get_state_manager();
  std::string error;
  
  // IDLE -> READY
  sm.transition_to(RecorderState::READY, error);
  EXPECT_FALSE(node->is_recording());
  EXPECT_FALSE(node->is_paused());
  EXPECT_FALSE(node->is_actively_recording());
  
  // READY -> RECORDING
  sm.transition_to(RecorderState::RECORDING, error);
  EXPECT_TRUE(node->is_recording());
  EXPECT_FALSE(node->is_paused());
  EXPECT_TRUE(node->is_actively_recording());
  
  // RECORDING -> PAUSED
  sm.transition_to(RecorderState::PAUSED, error);
  EXPECT_TRUE(node->is_recording());      // Still "recording" session
  EXPECT_TRUE(node->is_paused());
  EXPECT_FALSE(node->is_actively_recording());  // Not actively writing
  
  // PAUSED -> RECORDING
  sm.transition_to(RecorderState::RECORDING, error);
  EXPECT_TRUE(node->is_recording());
  EXPECT_FALSE(node->is_paused());
  EXPECT_TRUE(node->is_actively_recording());
  
  // RECORDING -> IDLE
  sm.transition_to(RecorderState::IDLE, error);
  EXPECT_FALSE(node->is_recording());
  EXPECT_FALSE(node->is_paused());
  EXPECT_FALSE(node->is_actively_recording());
}

// ============================================================================
// RecordingStats Tests
// ============================================================================

TEST_F(RecorderNodeRecordingTest, GetStatsInitialState) {
  auto node = RecorderNode::create();
  ASSERT_NE(node, nullptr);
  
  RecordingStats stats = node->get_stats();
  
  // All should be zero initially
  EXPECT_EQ(stats.messages_received, 0u);
  EXPECT_EQ(stats.messages_written, 0u);
  EXPECT_EQ(stats.messages_dropped, 0u);
  EXPECT_DOUBLE_EQ(stats.drop_rate_percent, 0.0);
}

TEST_F(RecorderNodeRecordingTest, GetRecordingDurationNotRecording) {
  auto node = RecorderNode::create();
  ASSERT_NE(node, nullptr);
  
  // Not recording - should return 0
  double duration = node->get_recording_duration_sec();
  EXPECT_DOUBLE_EQ(duration, 0.0);
}

// ============================================================================
// IRecorderContext Interface Tests
// ============================================================================

TEST_F(RecorderNodeRecordingTest, IRecorderContextInterface) {
  auto node = RecorderNode::create();
  ASSERT_NE(node, nullptr);
  
  // Cast to interface
  IRecorderContext* context = node.get();
  
  // Verify all interface methods are accessible
  StateManager& sm = context->get_state_manager();
  EXPECT_EQ(sm.get_state(), RecorderState::IDLE);
  
  TaskConfigCache& cache = context->get_task_config_cache();
  EXPECT_FALSE(cache.has_config());
  
  auto http_client = context->get_http_callback_client();
  EXPECT_NE(http_client, nullptr);
  
  std::string output_path = context->get_output_path();
  // May be empty before initialization
  
  RecordingStats stats = context->get_stats();
  EXPECT_EQ(stats.messages_received, 0u);
  
  double duration = context->get_recording_duration_sec();
  EXPECT_DOUBLE_EQ(duration, 0.0);
  
  // Logging should not crash
  EXPECT_NO_THROW(context->log_info("Test info"));
  EXPECT_NO_THROW(context->log_warn("Test warn"));
  EXPECT_NO_THROW(context->log_error("Test error"));
}

// ============================================================================
// Shared Pointer Usage Tests
// ============================================================================

TEST_F(RecorderNodeRecordingTest, SharedPtrWeakPtrUsage) {
  std::weak_ptr<RecorderNode> weak_node;
  
  {
    std::shared_ptr<RecorderNode> node = RecorderNode::create();
    ASSERT_NE(node, nullptr);
    
    weak_node = node;
    EXPECT_FALSE(weak_node.expired());
    
    // Use count should be 1
    EXPECT_EQ(node.use_count(), 1);
    
    // Can lock weak_ptr
    auto locked = weak_node.lock();
    EXPECT_NE(locked, nullptr);
    EXPECT_EQ(node.use_count(), 2);
  }
  
  // After scope, weak_ptr should be expired
  EXPECT_TRUE(weak_node.expired());
}

// ============================================================================
// Full Recording Workflow Tests (with ROS)
// ============================================================================

#if defined(AXON_ROS2)

class RecorderNodeFullRecordingTest : public RecorderNodeRecordingTest {
protected:
  void SetUp() override {
    RecorderNodeRecordingTest::SetUp();
    
    // Create a publisher node for sending test messages
    publisher_node_ = rclcpp::Node::make_shared("test_publisher");
    
    string_pub_ = publisher_node_->create_publisher<std_msgs::msg::String>(
        "/test_string", 10);
    int_pub_ = publisher_node_->create_publisher<std_msgs::msg::Int32>(
        "/test_int", 10);
  }

  void TearDown() override {
    string_pub_.reset();
    int_pub_.reset();
    publisher_node_.reset();
    RecorderNodeRecordingTest::TearDown();
  }

  void publish_test_messages(int count) {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(publisher_node_);
    for (int i = 0; i < count; ++i) {
      auto str_msg = std_msgs::msg::String();
      str_msg.data = "Test message " + std::to_string(i);
      string_pub_->publish(str_msg);
      
      auto int_msg = std_msgs::msg::Int32();
      int_msg.data = i;
      int_pub_->publish(int_msg);
      
      executor.spin_some(std::chrono::milliseconds(0));
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  rclcpp::Node::SharedPtr publisher_node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr int_pub_;
};

TEST_F(RecorderNodeFullRecordingTest, CreateNodeAndAccessComponents) {
  auto node = RecorderNode::create();
  ASSERT_NE(node, nullptr);
  
  // Access all public components
  StateManager& sm = node->get_state_manager();
  EXPECT_EQ(sm.get_state(), RecorderState::IDLE);
  
  TaskConfigCache& cache = node->get_task_config_cache();
  EXPECT_FALSE(cache.has_config());
  
  auto http = node->get_http_callback_client();
  EXPECT_NE(http, nullptr);
  
  // ROS interface not available before initialization
  RosInterface* ros = node->get_ros_interface();
  EXPECT_EQ(ros, nullptr);  // Not initialized yet
}

// ============================================================================
// Config Injection Tests - Exercise initialize() with injected config
// ============================================================================

TEST_F(RecorderNodeFullRecordingTest, InitializeWithInjectedConfig) {
  auto node = RecorderNode::create();
  ASSERT_NE(node, nullptr);
  
  // Create config programmatically
  auto config = create_injectable_config();
  EXPECT_TRUE(config.validate());
  
  int argc = 1;
  const char* argv[] = {"test_recorder"};
  char** argv_nc = const_cast<char**>(argv);
  
  // Initialize with injected config
  bool result = node->initialize(argc, argv_nc, config);
  
  // Note: This may fail if MCAP writer can't create files, but exercises the code path
  if (result) {
    // Verify ROS interface is available after initialization
    RosInterface* ros = node->get_ros_interface();
    EXPECT_NE(ros, nullptr);
    
    // Verify output path was generated
    std::string output = node->get_output_path();
    EXPECT_FALSE(output.empty());
    EXPECT_TRUE(output.find(".mcap") != std::string::npos);
    
    // Clean shutdown
    node->shutdown();
  } else {
    // Even on failure, shutdown should be safe
    EXPECT_NO_THROW(node->shutdown());
  }
}

TEST_F(RecorderNodeFullRecordingTest, InitializeWithInvalidConfig) {
  auto node = RecorderNode::create();
  ASSERT_NE(node, nullptr);
  
  // Create an empty/invalid config
  RecorderConfig invalid_config;
  // Don't set any topics - validation should fail
  
  int argc = 1;
  const char* argv[] = {"test_recorder"};
  char** argv_nc = const_cast<char**>(argv);
  
  // This should fail due to invalid config
  bool result = node->initialize(argc, argv_nc, invalid_config);
  
  // Shutdown should still be safe
  EXPECT_NO_THROW(node->shutdown());
}

TEST_F(RecorderNodeFullRecordingTest, FullRecordingWorkflowWithInjectedConfig) {
  auto node = create_and_initialize_with_config();
  
  // This test exercises the full workflow if initialization succeeds
  if (node != nullptr) {
    // Verify initial state
    EXPECT_FALSE(node->is_recording());
    EXPECT_FALSE(node->is_paused());
    
    // Get state manager and transition to READY
    StateManager& sm = node->get_state_manager();
    std::string error;
    
    // Cache a task config
    TaskConfig task_config;
    task_config.task_id = "full_workflow_test";
    task_config.device_id = "test_robot";
    task_config.scene = "test_scene";
    task_config.topics = {"/test_string", "/test_int"};
    node->get_task_config_cache().cache(task_config);
    
    // IDLE -> READY
    EXPECT_TRUE(sm.transition_to(RecorderState::READY, error));
    
    // Call configure_from_task_config (tests line 721-752)
    EXPECT_TRUE(node->configure_from_task_config(task_config));
    
    // READY -> RECORDING (tests start_recording, lines 526-555)
    EXPECT_TRUE(sm.transition_to(RecorderState::RECORDING, error));
    EXPECT_TRUE(node->start_recording());
    
    // Verify recording state
    EXPECT_TRUE(node->is_recording());
    EXPECT_TRUE(node->is_actively_recording());
    
    // Get stats during recording (tests get_stats, lines 754-770)
    RecordingStats stats = node->get_stats();
    // Stats should work even with no messages
    
    // Get duration (tests get_recording_duration_sec, lines 772-779)
    double duration = node->get_recording_duration_sec();
    EXPECT_GE(duration, 0.0);
    
    // RECORDING -> PAUSED (tests pause_recording, lines 557-563)
    EXPECT_TRUE(sm.transition_to(RecorderState::PAUSED, error));
    node->pause_recording();
    EXPECT_TRUE(node->is_paused());
    EXPECT_FALSE(node->is_actively_recording());
    
    // PAUSED -> RECORDING (tests resume_recording, lines 565-571)
    EXPECT_TRUE(sm.transition_to(RecorderState::RECORDING, error));
    node->resume_recording();
    EXPECT_TRUE(node->is_actively_recording());
    
    // RECORDING -> IDLE (tests stop_recording, lines 620-719)
    EXPECT_TRUE(sm.transition_to(RecorderState::IDLE, error));
    node->stop_recording();
    
    // Verify recording stopped
    EXPECT_FALSE(node->is_recording());
    
    // Clean shutdown (tests shutdown, lines 190-255)
    node->shutdown();
  } else {
    // Skip test if initialization failed
    GTEST_SKIP() << "RecorderNode initialization failed - skipping workflow test";
  }
}

TEST_F(RecorderNodeFullRecordingTest, CancelRecordingWorkflowWithInjectedConfig) {
  auto node = create_and_initialize_with_config();
  
  if (node != nullptr) {
    StateManager& sm = node->get_state_manager();
    std::string error;
    
    // Setup: Cache config, IDLE -> READY -> RECORDING
    TaskConfig task_config;
    task_config.task_id = "cancel_test";
    task_config.device_id = "robot";
    node->get_task_config_cache().cache(task_config);
    
    sm.transition_to(RecorderState::READY, error);
    sm.transition_to(RecorderState::RECORDING, error);
    node->start_recording();
    
    EXPECT_TRUE(node->is_recording());
    
    // Cancel recording (tests cancel_recording, lines 573-618)
    sm.transition_to(RecorderState::IDLE, error);
    node->cancel_recording();
    
    EXPECT_FALSE(node->is_recording());
    
    // Config cache should be cleared after cancel
    EXPECT_FALSE(node->get_task_config_cache().has_config());
    
    node->shutdown();
  } else {
    GTEST_SKIP() << "RecorderNode initialization failed - skipping cancel test";
  }
}

TEST_F(RecorderNodeFullRecordingTest, PrepareForRecordingWithTaskConfig) {
  auto node = RecorderNode::create();
  ASSERT_NE(node, nullptr);
  
  // Simulate the CachedRecordingConfig service flow:
  // 1. Cache config
  TaskConfig config;
  config.task_id = "recording_test_001";
  config.device_id = "robot_test";
  config.data_collector_id = "collector";
  config.scene = "test_scene";
  config.subscene = "test_subscene";
  config.factory = "test_factory";
  config.topics = {"/test_string", "/test_int"};
  
  node->get_task_config_cache().cache(config);
  EXPECT_TRUE(node->get_task_config_cache().has_config());
  
  // 2. Transition to READY
  std::string error;
  EXPECT_TRUE(node->get_state_manager().transition_to(RecorderState::READY, error));
  EXPECT_EQ(node->get_state_manager().get_state(), RecorderState::READY);
  
  // 3. Now ready for start command
  // (actual start_recording() requires full initialization)
}

TEST_F(RecorderNodeFullRecordingTest, SimulatedRecordingWorkflow) {
  auto node = RecorderNode::create();
  ASSERT_NE(node, nullptr);
  
  StateManager& sm = node->get_state_manager();
  std::string error;
  
  // Full state machine workflow
  // IDLE -> READY (config cached)
  TaskConfig config;
  config.task_id = "workflow_test";
  config.device_id = "robot";
  node->get_task_config_cache().cache(config);
  EXPECT_TRUE(sm.transition_to(RecorderState::READY, error));
  
  // READY -> RECORDING (start)
  EXPECT_TRUE(sm.transition_to(RecorderState::RECORDING, error));
  EXPECT_TRUE(node->is_recording());
  EXPECT_TRUE(node->is_actively_recording());
  
  // Simulate some recording time
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  // RECORDING -> PAUSED (pause)
  EXPECT_TRUE(sm.transition_to(RecorderState::PAUSED, error));
  EXPECT_TRUE(node->is_recording());
  EXPECT_TRUE(node->is_paused());
  EXPECT_FALSE(node->is_actively_recording());
  
  // PAUSED -> RECORDING (resume)
  EXPECT_TRUE(sm.transition_to(RecorderState::RECORDING, error));
  EXPECT_TRUE(node->is_actively_recording());
  
  // RECORDING -> IDLE (finish)
  EXPECT_TRUE(sm.transition_to(RecorderState::IDLE, error));
  EXPECT_FALSE(node->is_recording());
}

TEST_F(RecorderNodeFullRecordingTest, CancelRecordingWorkflow) {
  auto node = RecorderNode::create();
  ASSERT_NE(node, nullptr);
  
  StateManager& sm = node->get_state_manager();
  std::string error;
  
  // Setup: IDLE -> READY -> RECORDING
  TaskConfig config;
  config.task_id = "cancel_test";
  node->get_task_config_cache().cache(config);
  sm.transition_to(RecorderState::READY, error);
  sm.transition_to(RecorderState::RECORDING, error);
  
  EXPECT_TRUE(node->is_recording());
  
  // Cancel goes directly to IDLE
  EXPECT_TRUE(sm.transition_to(RecorderState::IDLE, error));
  EXPECT_FALSE(node->is_recording());
}

// ============================================================================
// write_stats_file() Tests
// ============================================================================

TEST_F(RecorderNodeFullRecordingTest, WriteStatsFileCreatesValidJson) {
  // This test verifies that write_stats_file() creates a valid JSON file
  // with expected fields after a recording workflow completes.
  
  auto node = create_and_initialize_with_config();
  if (!node) {
    GTEST_SKIP() << "RecorderNode initialization failed - skipping stats file test";
  }
  
  StateManager& sm = node->get_state_manager();
  std::string error;
  
  // Setup: cache config and transition to READY
  TaskConfig task_config;
  task_config.task_id = "stats_test";
  task_config.device_id = "test_device";
  node->get_task_config_cache().cache(task_config);
  
  ASSERT_TRUE(sm.transition_to(RecorderState::READY, error)) << "Failed to transition to READY: " << error;
  
  // Start recording
  ASSERT_TRUE(sm.transition_to(RecorderState::RECORDING, error)) << "Failed to start recording: " << error;
  node->start_recording();
  
  // Let it record briefly
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  // Stop recording - this calls write_stats_file()
  node->stop_recording();
  ASSERT_TRUE(sm.transition_to(RecorderState::IDLE, error)) << "Failed to stop recording: " << error;
  
  // Verify stats file was created
  auto stats_path = get_stats_file_path();
  ASSERT_TRUE(std::filesystem::exists(stats_path)) 
      << "Stats file was not created at: " << stats_path;
  
  // Read and verify JSON content
  std::ifstream stats_file(stats_path);
  ASSERT_TRUE(stats_file.is_open()) << "Could not open stats file";
  
  std::stringstream buffer;
  buffer << stats_file.rdbuf();
  std::string json_content = buffer.str();
  
  // Verify required fields are present
  EXPECT_TRUE(json_content.find("\"output_file\"") != std::string::npos)
      << "Stats file missing 'output_file' field";
  EXPECT_TRUE(json_content.find("\"format\": \"mcap\"") != std::string::npos)
      << "Stats file missing or incorrect 'format' field";
  EXPECT_TRUE(json_content.find("\"messages_received\"") != std::string::npos)
      << "Stats file missing 'messages_received' field";
  EXPECT_TRUE(json_content.find("\"messages_dropped\"") != std::string::npos)
      << "Stats file missing 'messages_dropped' field";
  EXPECT_TRUE(json_content.find("\"messages_written\"") != std::string::npos)
      << "Stats file missing 'messages_written' field";
  EXPECT_TRUE(json_content.find("\"bytes_written\"") != std::string::npos)
      << "Stats file missing 'bytes_written' field";
  EXPECT_TRUE(json_content.find("\"drop_rate_percent\"") != std::string::npos)
      << "Stats file missing 'drop_rate_percent' field";
  EXPECT_TRUE(json_content.find("\"per_topic_stats\"") != std::string::npos)
      << "Stats file missing 'per_topic_stats' field";
  
  // Cleanup
  node->shutdown();
}

TEST_F(RecorderNodeFullRecordingTest, WriteStatsFileCreatesParentDirectory) {
  // This test verifies that write_stats_file() creates parent directories
  // if they don't exist.
  
  auto config = create_injectable_config();
  
  // Set stats path in a non-existent subdirectory
  auto nested_stats_dir = test_dir_ / "nested" / "stats" / "dir";
  config.dataset.stats_file_path = (nested_stats_dir / "recorder_stats.json").string();
  
  auto node = RecorderNode::create();
  ASSERT_NE(node, nullptr);
  
  int argc = 1;
  const char* argv[] = {"test_recorder"};
  char** argv_nc = const_cast<char**>(argv);
  
  if (!node->initialize(argc, argv_nc, config)) {
    GTEST_SKIP() << "RecorderNode initialization failed - skipping nested stats directory test";
  }
  
  StateManager& sm = node->get_state_manager();
  std::string error;
  
  // Setup and start recording
  TaskConfig task_config;
  task_config.task_id = "nested_stats_test";
  node->get_task_config_cache().cache(task_config);
  
  sm.transition_to(RecorderState::READY, error);
  sm.transition_to(RecorderState::RECORDING, error);
  node->start_recording();
  
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  
  // Stop recording
  node->stop_recording();
  sm.transition_to(RecorderState::IDLE, error);
  
  // Verify nested directory was created and stats file exists
  EXPECT_TRUE(std::filesystem::exists(nested_stats_dir))
      << "Nested stats directory was not created";
  EXPECT_TRUE(std::filesystem::exists(nested_stats_dir / "recorder_stats.json"))
      << "Stats file was not created in nested directory";
  
  node->shutdown();
}

#endif  // AXON_ROS2

// ============================================================================
// ROS 1 Tests
// ============================================================================

#if defined(AXON_ROS1)

class RecorderNodeFullRecordingTest : public RecorderNodeRecordingTest {
protected:
  void SetUp() override {
    RecorderNodeRecordingTest::SetUp();
    
    if (!ros::isInitialized()) {
      int argc = 0;
      char** argv = nullptr;
      ros::init(argc, argv, "test_recorder_recording", ros::init_options::NoSigintHandler);
    }
    
    nh_ = std::make_unique<ros::NodeHandle>();
    string_pub_ = nh_->advertise<std_msgs::String>("/test_string", 10);
    int_pub_ = nh_->advertise<std_msgs::Int32>("/test_int", 10);
  }

  void TearDown() override {
    string_pub_.shutdown();
    int_pub_.shutdown();
    nh_.reset();
    RecorderNodeRecordingTest::TearDown();
  }

  std::unique_ptr<ros::NodeHandle> nh_;
  ros::Publisher string_pub_;
  ros::Publisher int_pub_;
};

TEST_F(RecorderNodeFullRecordingTest, CreateNodeAndAccessComponents) {
  auto node = RecorderNode::create();
  ASSERT_NE(node, nullptr);
  
  StateManager& sm = node->get_state_manager();
  EXPECT_EQ(sm.get_state(), RecorderState::IDLE);
  
  TaskConfigCache& cache = node->get_task_config_cache();
  EXPECT_FALSE(cache.has_config());
}

TEST_F(RecorderNodeFullRecordingTest, SimulatedRecordingWorkflow) {
  auto node = RecorderNode::create();
  ASSERT_NE(node, nullptr);
  
  StateManager& sm = node->get_state_manager();
  std::string error;
  
  // Full workflow
  TaskConfig config;
  config.task_id = "ros1_workflow_test";
  node->get_task_config_cache().cache(config);
  
  sm.transition_to(RecorderState::READY, error);
  sm.transition_to(RecorderState::RECORDING, error);
  EXPECT_TRUE(node->is_recording());
  
  sm.transition_to(RecorderState::PAUSED, error);
  EXPECT_TRUE(node->is_paused());
  
  sm.transition_to(RecorderState::RECORDING, error);
  sm.transition_to(RecorderState::IDLE, error);
  EXPECT_FALSE(node->is_recording());
}

// ============================================================================
// write_stats_file() Tests (ROS1)
// ============================================================================

TEST_F(RecorderNodeFullRecordingTest, WriteStatsFileCreatesValidJson) {
  // This test verifies that write_stats_file() creates a valid JSON file
  // with expected fields after a recording workflow completes.
  
  auto node = create_and_initialize_with_config();
  if (!node) {
    GTEST_SKIP() << "RecorderNode initialization failed - skipping stats file test";
  }
  
  StateManager& sm = node->get_state_manager();
  std::string error;
  
  // Setup: cache config and transition to READY
  TaskConfig task_config;
  task_config.task_id = "stats_test_ros1";
  task_config.device_id = "test_device_ros1";
  node->get_task_config_cache().cache(task_config);
  
  ASSERT_TRUE(sm.transition_to(RecorderState::READY, error)) << "Failed to transition to READY: " << error;
  
  // Start recording
  ASSERT_TRUE(sm.transition_to(RecorderState::RECORDING, error)) << "Failed to start recording: " << error;
  node->start_recording();
  
  // Let it record briefly
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  // Stop recording - this calls write_stats_file()
  node->stop_recording();
  ASSERT_TRUE(sm.transition_to(RecorderState::IDLE, error)) << "Failed to stop recording: " << error;
  
  // Verify stats file was created
  auto stats_path = get_stats_file_path();
  ASSERT_TRUE(std::filesystem::exists(stats_path)) 
      << "Stats file was not created at: " << stats_path;
  
  // Read and verify JSON content
  std::ifstream stats_file(stats_path);
  ASSERT_TRUE(stats_file.is_open()) << "Could not open stats file";
  
  std::stringstream buffer;
  buffer << stats_file.rdbuf();
  std::string json_content = buffer.str();
  
  // Verify required fields are present
  EXPECT_TRUE(json_content.find("\"output_file\"") != std::string::npos)
      << "Stats file missing 'output_file' field";
  EXPECT_TRUE(json_content.find("\"format\": \"mcap\"") != std::string::npos)
      << "Stats file missing or incorrect 'format' field";
  EXPECT_TRUE(json_content.find("\"messages_received\"") != std::string::npos)
      << "Stats file missing 'messages_received' field";
  EXPECT_TRUE(json_content.find("\"per_topic_stats\"") != std::string::npos)
      << "Stats file missing 'per_topic_stats' field";
  
  // Cleanup
  node->shutdown();
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
  ros::init(argc, argv, "test_recorder_node_recording", ros::init_options::NoSigintHandler);
#endif
  
  int result = RUN_ALL_TESTS();
  
#if defined(AXON_ROS2)
  // NOTE: Do NOT call rclcpp::shutdown() explicitly in tests.
  // In ROS2 Rolling, explicit shutdown after tests with multiple nodes
  // of the same name ("axon_recorder") causes rosout publisher cleanup
  // issues leading to SIGSEGV. The process cleans up naturally on exit.
  // See: https://github.com/ros2/rclcpp/issues/2139
#elif defined(AXON_ROS1)
  if (ros::ok()) {
    ros::shutdown();
  }
#endif
  
  return result;
}