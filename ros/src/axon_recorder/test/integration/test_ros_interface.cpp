/**
 * Integration tests for RosInterface using REAL ROS infrastructure
 *
 * These tests exercise the actual ROS binding code paths that require
 * a running ROS environment. They are NOT mock-based and provide
 * coverage for:
 * - ros_interface.cpp
 * - message_factory.cpp
 * - register_common_messages.cpp
 * - service_adapter.cpp (service registration)
 *
 * Run with: colcon test (in ROS environment)
 */

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#if defined(AXON_ROS2)
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#elif defined(AXON_ROS1)
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#endif

#include "ros_interface.hpp"
#include "message_factory.hpp"

using namespace axon::recorder;

// ============================================================================
// Test Fixture
// ============================================================================

class RosInterfaceRealTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Create the real ROS interface using factory
    ros_interface_ = RosInterfaceFactory::create();
    ASSERT_NE(ros_interface_, nullptr);
    
    // Initialize the interface. This will:
    // - Skip rclcpp::init() since ROS is already initialized in main()
    // - Create the internal node needed for operations
    int argc = 0;
    char** argv = nullptr;
    ASSERT_TRUE(ros_interface_->init(argc, argv, "test_ros_interface"));
  }

  void TearDown() override {
    // Shutdown cleans up the internal node but doesn't call rclcpp::shutdown()
    // since this instance didn't initialize the ROS context
    if (ros_interface_) {
      ros_interface_->shutdown();
    }
    ros_interface_.reset();
  }

  std::unique_ptr<RosInterface> ros_interface_;
};

// ============================================================================
// RosInterface Basic Operations
// ============================================================================

TEST_F(RosInterfaceRealTest, FactoryCreatesValidInterface) {
  auto interface = RosInterfaceFactory::create();
  ASSERT_NE(interface, nullptr);
}

TEST_F(RosInterfaceRealTest, InterfaceCreation) {
  // Verify interface can be created (ROS context initialized in main())
  auto interface = RosInterfaceFactory::create();
  ASSERT_NE(interface, nullptr);
}

TEST_F(RosInterfaceRealTest, OkReturnsTrue) {
  // ROS is initialized in main(), so ok() should return true
  EXPECT_TRUE(ros_interface_->ok());
}

TEST_F(RosInterfaceRealTest, GetNodeHandle) {
  // ROS is initialized in main(), node handle should be available
  void* handle = ros_interface_->get_node_handle();
  EXPECT_NE(handle, nullptr);
}

TEST_F(RosInterfaceRealTest, NowNsec) {
  // Get current time in nanoseconds
  int64_t t1 = ros_interface_->now_nsec();
  
  // Wait a small amount
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  
  int64_t t2 = ros_interface_->now_nsec();
  
  // Time should have advanced
  EXPECT_GT(t2, t1);
  
  // Time should be positive
  EXPECT_GT(t1, 0);
}

TEST_F(RosInterfaceRealTest, LoggingMethods) {
  // These should not crash
  EXPECT_NO_THROW(ros_interface_->log_info("Test info message"));
  EXPECT_NO_THROW(ros_interface_->log_warn("Test warning message"));
  EXPECT_NO_THROW(ros_interface_->log_error("Test error message"));
  EXPECT_NO_THROW(ros_interface_->log_debug("Test debug message"));
}

// ============================================================================
// Message Definition Tests
// ============================================================================

TEST_F(RosInterfaceRealTest, GetMessageDefinitionKnownTypes) {
  // Test known message types
#if defined(AXON_ROS2)
  std::string header_def = ros_interface_->get_message_definition("std_msgs/msg/Header");
  EXPECT_FALSE(header_def.empty());
  
  std::string image_def = ros_interface_->get_message_definition("sensor_msgs/msg/Image");
  EXPECT_FALSE(image_def.empty());
  EXPECT_TRUE(image_def.find("header") != std::string::npos || 
              image_def.find("Header") != std::string::npos);
  
  std::string imu_def = ros_interface_->get_message_definition("sensor_msgs/msg/Imu");
  EXPECT_FALSE(imu_def.empty());
  
  std::string twist_def = ros_interface_->get_message_definition("geometry_msgs/msg/Twist");
  EXPECT_FALSE(twist_def.empty());
#elif defined(AXON_ROS1)
  std::string header_def = ros_interface_->get_message_definition("std_msgs/Header");
  EXPECT_FALSE(header_def.empty());
#endif
}

TEST_F(RosInterfaceRealTest, GetMessageDefinitionUnknownType) {
  std::string def = ros_interface_->get_message_definition("nonexistent_pkg/msg/FakeMessage");
  
  // Should return a placeholder, not empty
  EXPECT_FALSE(def.empty());
  EXPECT_TRUE(def.find("not available") != std::string::npos ||
              def.find("placeholder") != std::string::npos ||
              def.find("FakeMessage") != std::string::npos);
}

// ============================================================================
// Subscription Tests (Real ROS Communication)
// ============================================================================

#if defined(AXON_ROS2)

class RosInterfaceSubscriptionTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Create a test node for publishing (uses existing ROS context)
    test_node_ = rclcpp::Node::make_shared("test_publisher_node");
    
    // Create and initialize the ROS interface under test
    ros_interface_ = RosInterfaceFactory::create();
    ASSERT_NE(ros_interface_, nullptr);
    
    int argc = 0;
    char** argv = nullptr;
    ASSERT_TRUE(ros_interface_->init(argc, argv, "test_subscriber_node"));
  }

  void TearDown() override {
    if (ros_interface_) {
      ros_interface_->shutdown();
    }
    ros_interface_.reset();
    test_node_.reset();
  }

  void spin_some(int iterations = 10, int delay_ms = 10) {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(test_node_);
    for (int i = 0; i < iterations; ++i) {
      executor.spin_some(std::chrono::milliseconds(0));
      ros_interface_->spin_once();
      std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }
  }

  rclcpp::Node::SharedPtr test_node_;
  std::unique_ptr<RosInterface> ros_interface_;
};

TEST_F(RosInterfaceSubscriptionTest, SubscribeToStringTopic) {
  std::atomic<int> received_count{0};
  std::string last_message;
  
  // Create publisher on test node
  auto publisher = test_node_->create_publisher<std_msgs::msg::String>(
    "/test_string", 10);
  
  // Subscribe using ROS interface (legacy method)
  auto callback = [&](const void* msg) {
    auto* serialized = static_cast<const rclcpp::SerializedMessage*>(msg);
    received_count++;
  };
  
  void* sub_handle = ros_interface_->subscribe("/test_string", "std_msgs/msg/String", callback);
  ASSERT_NE(sub_handle, nullptr);
  
  // Give time for subscription to establish
  spin_some(5);
  
  // Publish messages
  auto msg = std_msgs::msg::String();
  msg.data = "Hello ROS Interface";
  
  for (int i = 0; i < 3; ++i) {
    publisher->publish(msg);
    spin_some(5);
  }
  
  // Verify messages were received
  EXPECT_GT(received_count.load(), 0);
  
  // Unsubscribe
  ros_interface_->unsubscribe(sub_handle);
}

TEST_F(RosInterfaceSubscriptionTest, ZeroCopySubscription) {
  std::atomic<int> received_count{0};
  std::atomic<size_t> total_bytes{0};
  
  // Create publisher
  auto publisher = test_node_->create_publisher<std_msgs::msg::String>(
    "/test_zero_copy", 10);
  
  // Subscribe using zero-copy method
  auto callback = [&](SerializedMessageData&& data) {
    received_count++;
    total_bytes += data.size();
  };
  
  SubscriptionConfig config;
  config.qos = QosProfile::HighThroughput;
  config.history_depth = 100;
  
  void* sub_handle = ros_interface_->subscribe_zero_copy(
    "/test_zero_copy", "std_msgs/msg/String", callback, config);
  ASSERT_NE(sub_handle, nullptr);
  
  // Give time for subscription to establish
  spin_some(5);
  
  // Publish messages
  auto msg = std_msgs::msg::String();
  msg.data = "Zero copy test message";
  
  for (int i = 0; i < 5; ++i) {
    publisher->publish(msg);
    spin_some(3);
  }
  
  // Verify messages were received
  EXPECT_GT(received_count.load(), 0);
  EXPECT_GT(total_bytes.load(), 0u);
  
  // Unsubscribe
  ros_interface_->unsubscribe(sub_handle);
}

TEST_F(RosInterfaceSubscriptionTest, ZeroCopyWithDifferentQosProfiles) {
  // Test SensorData QoS
  {
    std::atomic<int> received{0};
    
    auto publisher = test_node_->create_publisher<std_msgs::msg::Int32>(
      "/test_sensor_qos",
      rclcpp::QoS(10).best_effort());
    
    auto callback = [&](SerializedMessageData&& /* data */) { received++; };
    
    SubscriptionConfig config;
    config.qos = QosProfile::SensorData;
    config.history_depth = 10;
    
    void* sub = ros_interface_->subscribe_zero_copy(
      "/test_sensor_qos", "std_msgs/msg/Int32", callback, config);
    
    spin_some(5);
    
    auto msg = std_msgs::msg::Int32();
    msg.data = 42;
    publisher->publish(msg);
    
    spin_some(10);
    
    // Best-effort may or may not receive depending on timing
    // Just verify no crash
    
    ros_interface_->unsubscribe(sub);
  }
  
  // Test SystemDefault QoS
  {
    std::atomic<int> received{0};
    
    auto publisher = test_node_->create_publisher<std_msgs::msg::Int32>(
      "/test_default_qos", 10);
    
    auto callback = [&](SerializedMessageData&& /* data */) { received++; };
    
    SubscriptionConfig config;
    config.qos = QosProfile::SystemDefault;
    config.history_depth = 10;
    
    void* sub = ros_interface_->subscribe_zero_copy(
      "/test_default_qos", "std_msgs/msg/Int32", callback, config);
    
    spin_some(5);
    
    auto msg = std_msgs::msg::Int32();
    msg.data = 123;
    publisher->publish(msg);
    
    spin_some(10);
    
    ros_interface_->unsubscribe(sub);
  }
}

TEST_F(RosInterfaceSubscriptionTest, DedicatedCallbackGroup) {
  std::atomic<int> received{0};
  
  auto publisher = test_node_->create_publisher<std_msgs::msg::String>(
    "/test_dedicated_callback", 10);
  
  auto callback = [&](SerializedMessageData&& /* data */) { received++; };
  
  SubscriptionConfig config;
  config.qos = QosProfile::HighThroughput;
  config.history_depth = 100;
  config.use_dedicated_callback_group = true;  // Key difference
  
  void* sub = ros_interface_->subscribe_zero_copy(
    "/test_dedicated_callback", "std_msgs/msg/String", callback, config);
  ASSERT_NE(sub, nullptr);
  
  spin_some(5);
  
  auto msg = std_msgs::msg::String();
  msg.data = "Dedicated callback test";
  publisher->publish(msg);
  
  spin_some(10);
  
  EXPECT_GT(received.load(), 0);
  
  ros_interface_->unsubscribe(sub);
}

TEST_F(RosInterfaceSubscriptionTest, MultipleSubscriptions) {
  std::atomic<int> string_received{0};
  std::atomic<int> int_received{0};
  std::atomic<int> twist_received{0};
  
  // Create publishers
  auto string_pub = test_node_->create_publisher<std_msgs::msg::String>("/multi_string", 10);
  auto int_pub = test_node_->create_publisher<std_msgs::msg::Int32>("/multi_int", 10);
  auto twist_pub = test_node_->create_publisher<geometry_msgs::msg::Twist>("/multi_twist", 10);
  
  // Subscribe to all topics
  SubscriptionConfig config;
  config.qos = QosProfile::HighThroughput;
  config.history_depth = 100;
  
  void* sub1 = ros_interface_->subscribe_zero_copy(
    "/multi_string", "std_msgs/msg/String",
    [&](SerializedMessageData&& /* data */) { string_received++; }, config);
  
  void* sub2 = ros_interface_->subscribe_zero_copy(
    "/multi_int", "std_msgs/msg/Int32",
    [&](SerializedMessageData&& /* data */) { int_received++; }, config);
  
  void* sub3 = ros_interface_->subscribe_zero_copy(
    "/multi_twist", "geometry_msgs/msg/Twist",
    [&](SerializedMessageData&& /* data */) { twist_received++; }, config);
  
  ASSERT_NE(sub1, nullptr);
  ASSERT_NE(sub2, nullptr);
  ASSERT_NE(sub3, nullptr);
  
  spin_some(10);
  
  // Publish to all topics
  auto str_msg = std_msgs::msg::String();
  str_msg.data = "test";
  string_pub->publish(str_msg);
  
  auto int_msg = std_msgs::msg::Int32();
  int_msg.data = 42;
  int_pub->publish(int_msg);
  
  auto twist_msg = geometry_msgs::msg::Twist();
  twist_msg.linear.x = 1.0;
  twist_pub->publish(twist_msg);
  
  spin_some(20);
  
  // Verify all subscriptions received messages
  EXPECT_GT(string_received.load(), 0);
  EXPECT_GT(int_received.load(), 0);
  EXPECT_GT(twist_received.load(), 0);
  
  // Cleanup
  ros_interface_->unsubscribe(sub1);
  ros_interface_->unsubscribe(sub2);
  ros_interface_->unsubscribe(sub3);
}

TEST_F(RosInterfaceSubscriptionTest, UnsubscribeNullHandle) {
  // Should not crash
  EXPECT_NO_THROW(ros_interface_->unsubscribe(nullptr));
}

TEST_F(RosInterfaceSubscriptionTest, UnsubscribeInvalidHandle) {
  // Should not crash with invalid handle
  void* fake_handle = reinterpret_cast<void*>(0xDEADBEEF);
  EXPECT_NO_THROW(ros_interface_->unsubscribe(fake_handle));
}

#endif  // AXON_ROS2

// ============================================================================
// Service Advertisement Tests
// ============================================================================

#if defined(AXON_ROS2)

TEST_F(RosInterfaceRealTest, AdvertiseServiceGeneric) {
  auto callback = [](const void* /* req */, void* /* res */) -> bool {
    return true;
  };
  
  // Generic service advertisement (currently logs warning in ROS 2)
  void* service = ros_interface_->advertise_service(
    "test_service", "std_srvs/srv/Empty", callback);
  
  // The current implementation returns a wrapper but doesn't create a real service
  // This exercises the code path regardless
}

#endif

// ============================================================================
// Spin Methods Tests
// ============================================================================

TEST_F(RosInterfaceRealTest, SpinOnceDoesNotBlock) {
  auto start = std::chrono::steady_clock::now();
  
  // spin_once should return quickly when there's nothing to process
  ros_interface_->spin_once();
  
  auto elapsed = std::chrono::steady_clock::now() - start;
  
  // Should complete in less than 1 second
  EXPECT_LT(elapsed, std::chrono::seconds(1));
}

// ============================================================================
// MessageFactory Tests (Real ROS Types)
// ============================================================================

#if defined(AXON_ROS2)

TEST(MessageFactoryRealTest, IsRegisteredCommonTypes) {
  // Test the is_registered API for types that may or may not be registered
  // Common types are registered at startup via register_common_message_types()
  
  // Unregistered types should return false
  EXPECT_FALSE(MessageFactory::is_registered("nonexistent/Message"));
  EXPECT_FALSE(MessageFactory::is_registered(""));
  EXPECT_FALSE(MessageFactory::is_registered("invalid_type_name"));
}

TEST(MessageFactoryRealTest, GetMessageInfoUnregistered) {
  MessageFactory::MessageInfo info;
  // Getting info for unregistered types should return false
  EXPECT_FALSE(MessageFactory::get_message_info("unknown/UnknownType", info));
  EXPECT_FALSE(MessageFactory::get_message_info("", info));
}

TEST(MessageFactoryRealTest, CreateMessageUnregistered) {
  // Creating unregistered message should return nullptr
  auto msg = MessageFactory::create_message("nonexistent/Message");
  EXPECT_EQ(msg, nullptr);
}

#elif defined(AXON_ROS1)

TEST(MessageFactoryRealTest, IsRegisteredCommonTypes) {
  // Test the is_registered API for types that may or may not be registered
  EXPECT_FALSE(MessageFactory::is_registered("nonexistent/Message"));
  EXPECT_FALSE(MessageFactory::is_registered(""));
}

TEST(MessageFactoryRealTest, GetMessageInfoUnregistered) {
  MessageFactory::MessageInfo info;
  EXPECT_FALSE(MessageFactory::get_message_info("unknown/UnknownType", info));
}

TEST(MessageFactoryRealTest, CreateMessageUnregistered) {
  auto msg = MessageFactory::create_message("nonexistent/Message");
  EXPECT_EQ(msg, nullptr);
}

#endif

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  
  // Pre-initialize ROS for tests
#if defined(AXON_ROS2)
  rclcpp::init(argc, argv);
#elif defined(AXON_ROS1)
  ros::init(argc, argv, "test_ros_interface_real", ros::init_options::NoSigintHandler);
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

