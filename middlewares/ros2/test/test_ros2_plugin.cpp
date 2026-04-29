// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <thread>
#include <vector>

#include "ros2_plugin.hpp"
#include "ros2_subscription_wrapper.hpp"

using namespace ros2_plugin;

extern "C" {

using AxonMessageCallback = void (*)(
  const char*, const uint8_t*, size_t, const char*, uint64_t, void*
);

struct AxonPluginVtable {
  int32_t (*init)(const char*);
  int32_t (*start)(void);
  int32_t (*stop)(void);
  int32_t (*subscribe)(const char*, const char*, const char*, AxonMessageCallback, void*);
  int32_t (*publish)(const char*, const uint8_t*, size_t, const char*);
  void* reserved[9];
};

struct AxonPluginDescriptor {
  uint32_t abi_version_major;
  uint32_t abi_version_minor;
  const char* middleware_name;
  const char* middleware_version;
  const char* plugin_version;
  AxonPluginVtable* vtable;
  void* reserved[16];
};

const AxonPluginDescriptor* axon_get_plugin_descriptor(void);

}  // extern "C"

/**
 * @brief Test fixture for ROS2 Plugin tests
 *
 * Sets up and tears down ROS2 context for each test
 */
class Ros2PluginTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Initialize ROS2 if not already initialized
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void TearDown() override {
    // Shutdown ROS2 if we initialized it
    // Note: We don't shutdown here to allow other tests to run
  }
};

/**
 * @brief Test default constructor initializes state correctly
 */
TEST_F(Ros2PluginTest, ConstructorInitialState) {
  Ros2Plugin plugin;

  EXPECT_FALSE(plugin.is_initialized());
  EXPECT_FALSE(plugin.is_running());
  EXPECT_EQ(plugin.get_node(), nullptr);
}

/**
 * @brief Test initialization with valid JSON config
 */
TEST_F(Ros2PluginTest, InitWithValidConfig) {
  Ros2Plugin plugin;

  const char* config_json = R"({
    "node_name": "test_plugin_node",
    "namespace": "test_ns"
  })";

  bool result = plugin.init(config_json);

  EXPECT_TRUE(result);
  EXPECT_TRUE(plugin.is_initialized());
  EXPECT_NE(plugin.get_node(), nullptr);

  // Clean up
  EXPECT_TRUE(plugin.stop());
}

/**
 * @brief Test initialization with empty config (default values)
 */
TEST_F(Ros2PluginTest, InitWithEmptyConfig) {
  Ros2Plugin plugin;

  bool result = plugin.init(nullptr);
  EXPECT_TRUE(result);
  EXPECT_TRUE(plugin.is_initialized());

  result = plugin.init("");
  EXPECT_FALSE(result);  // Should fail if already initialized

  // Clean up
  EXPECT_TRUE(plugin.stop());
}

/**
 * @brief Test initialization with minimal config
 */
TEST_F(Ros2PluginTest, InitWithMinimalConfig) {
  Ros2Plugin plugin;

  bool result = plugin.init("{}");
  EXPECT_TRUE(result);
  EXPECT_TRUE(plugin.is_initialized());

  // Clean up
  EXPECT_TRUE(plugin.stop());
}

/**
 * @brief Test that double initialization fails
 */
TEST_F(Ros2PluginTest, DoubleInitFails) {
  Ros2Plugin plugin;

  EXPECT_TRUE(plugin.init("{}"));

  // Second init should fail
  EXPECT_FALSE(plugin.init("{}"));

  // Clean up
  EXPECT_TRUE(plugin.stop());
}

/**
 * @brief Test start without initialization fails
 */
TEST_F(Ros2PluginTest, StartWithoutInitFails) {
  Ros2Plugin plugin;

  bool result = plugin.start();
  EXPECT_FALSE(result);
  EXPECT_FALSE(plugin.is_running());
}

/**
 * @brief Test start after successful initialization
 */
TEST_F(Ros2PluginTest, StartAfterInitSucceeds) {
  Ros2Plugin plugin;

  ASSERT_TRUE(plugin.init("{}"));

  bool result = plugin.start();
  EXPECT_TRUE(result);
  EXPECT_TRUE(plugin.is_running());

  // Clean up
  EXPECT_TRUE(plugin.stop());
}

/**
 * @brief Test double start fails
 */
TEST_F(Ros2PluginTest, DoubleStartFails) {
  Ros2Plugin plugin;

  ASSERT_TRUE(plugin.init("{}"));
  ASSERT_TRUE(plugin.start());

  // Second start should fail
  EXPECT_FALSE(plugin.start());

  // Clean up
  EXPECT_TRUE(plugin.stop());
}

/**
 * @brief Test stop without initialization succeeds (no-op)
 */
TEST_F(Ros2PluginTest, StopWithoutInitSucceeds) {
  Ros2Plugin plugin;

  bool result = plugin.stop();
  EXPECT_TRUE(result);
  EXPECT_FALSE(plugin.is_initialized());
  EXPECT_FALSE(plugin.is_running());
}

/**
 * @brief Test start and stop lifecycle
 */
TEST_F(Ros2PluginTest, StartStopLifecycle) {
  Ros2Plugin plugin;

  ASSERT_TRUE(plugin.init("{}"));

  // Start
  EXPECT_TRUE(plugin.start());
  EXPECT_TRUE(plugin.is_running());

  // Stop
  EXPECT_TRUE(plugin.stop());
  EXPECT_FALSE(plugin.is_initialized());
  EXPECT_FALSE(plugin.is_running());
}

/**
 * @brief Test ABI v1.3 session stop preserves initialized plugin state
 */
TEST_F(Ros2PluginTest, SessionStopPreservesInitialization) {
  Ros2Plugin plugin;

  ASSERT_TRUE(plugin.init("{}"));

  auto callback =
    [](const std::string&, const std::string&, const std::vector<uint8_t>&, rclcpp::Time) {};
  ASSERT_TRUE(plugin.subscribe("/session_topic", "std_msgs/msg/String", callback));

  ASSERT_TRUE(plugin.start());
  ASSERT_TRUE(plugin.is_running());

  EXPECT_TRUE(plugin.stop_session());
  EXPECT_TRUE(plugin.is_initialized());
  EXPECT_FALSE(plugin.is_running());
  EXPECT_NE(plugin.get_node(), nullptr);
  EXPECT_TRUE(plugin.get_subscribed_topics().empty());

  // Subscriptions can be recreated for a later recording session.
  EXPECT_TRUE(plugin.subscribe("/session_topic", "std_msgs/msg/String", callback));
  EXPECT_EQ(plugin.get_subscribed_topics().size(), 1);

  EXPECT_TRUE(plugin.stop());
  EXPECT_FALSE(plugin.is_initialized());
}

/**
 * @brief Test exported descriptor advertises ABI v1.3 capability slots
 */
TEST(Ros2PluginExportTest, DescriptorAdvertisesAbi13) {
  const auto* descriptor = axon_get_plugin_descriptor();

  ASSERT_NE(descriptor, nullptr);
  ASSERT_NE(descriptor->vtable, nullptr);
  EXPECT_EQ(descriptor->abi_version_major, 1U);
  EXPECT_EQ(descriptor->abi_version_minor, 3U);
  EXPECT_NE(descriptor->vtable->reserved[0], nullptr);
  EXPECT_NE(descriptor->vtable->reserved[1], nullptr);
}

/**
 * @brief Test stop without explicit start (cleanup in destructor)
 */
TEST_F(Ros2PluginTest, DestructorStopsPlugin) {
  auto plugin = std::make_unique<Ros2Plugin>();

  ASSERT_TRUE(plugin->init("{}"));
  ASSERT_TRUE(plugin->start());

  // Destructor should clean up
  plugin.reset();

  // If we get here without crash, destructor worked
  SUCCEED();
}

/**
 * @brief Test subscribe without initialization fails
 */
TEST_F(Ros2PluginTest, SubscribeWithoutInitFails) {
  Ros2Plugin plugin;

  int callback_count = 0;
  auto callback = [&callback_count](
                    const std::string& topic,
                    const std::string& type,
                    const std::vector<uint8_t>& data,
                    rclcpp::Time timestamp
                  ) {
    callback_count++;
  };

  bool result = plugin.subscribe("/test/topic", "std_msgs/msg/String", callback);
  EXPECT_FALSE(result);
  EXPECT_EQ(callback_count, 0);
}

/**
 * @brief Test unsubscribe without initialization fails
 */
TEST_F(Ros2PluginTest, UnsubscribeWithoutInitFails) {
  Ros2Plugin plugin;

  bool result = plugin.unsubscribe("/test/topic");
  EXPECT_FALSE(result);
}

/**
 * @brief Test get_subscribed_topics without initialization returns empty
 */
TEST_F(Ros2PluginTest, GetTopicsWithoutInitReturnsEmpty) {
  Ros2Plugin plugin;

  auto topics = plugin.get_subscribed_topics();
  EXPECT_TRUE(topics.empty());
}

/**
 * @brief Test subscribe with valid parameters
 */
TEST_F(Ros2PluginTest, SubscribeValidTopic) {
  Ros2Plugin plugin;

  ASSERT_TRUE(plugin.init("{}"));

  int callback_count = 0;
  auto callback = [&callback_count](
                    const std::string& topic,
                    const std::string& type,
                    const std::vector<uint8_t>& data,
                    rclcpp::Time timestamp
                  ) {
    callback_count++;
  };

  bool result = plugin.subscribe("/test_topic", "std_msgs/msg/String", callback);
  // This may succeed even if no publisher exists (ROS2 allows this)
  // The subscription is created but won't receive messages

  // Clean up
  EXPECT_TRUE(plugin.stop());
}

/**
 * @brief Test unsubscribe from subscribed topic
 */
TEST_F(Ros2PluginTest, UnsubscribeFromTopic) {
  Ros2Plugin plugin;

  ASSERT_TRUE(plugin.init("{}"));

  auto callback =
    [](const std::string&, const std::string&, const std::vector<uint8_t>&, rclcpp::Time) {};

  ASSERT_TRUE(plugin.subscribe("/test_topic", "std_msgs/msg/String", callback));
  EXPECT_TRUE(plugin.unsubscribe("/test_topic"));

  // Unsubscribe again should fail (not subscribed)
  EXPECT_FALSE(plugin.unsubscribe("/test_topic"));

  // Clean up
  EXPECT_TRUE(plugin.stop());
}

/**
 * @brief Test get_subscribed_topics returns subscribed topics
 */
TEST_F(Ros2PluginTest, GetSubscribedTopics) {
  Ros2Plugin plugin;

  ASSERT_TRUE(plugin.init("{}"));

  auto callback =
    [](const std::string&, const std::string&, const std::vector<uint8_t>&, rclcpp::Time) {};

  plugin.subscribe("/topic1", "std_msgs/msg/String", callback);
  plugin.subscribe("/topic2", "std_msgs/msg/Int32", callback);
  plugin.subscribe("/topic3", "sensor_msgs/msg/Image", callback);

  auto topics = plugin.get_subscribed_topics();

  // Note: Topic order is not guaranteed
  EXPECT_EQ(topics.size(), 3);

  std::vector<std::string> expected_topics = {"/topic1", "/topic2", "/topic3"};
  for (const auto& expected : expected_topics) {
    bool found = std::find(topics.begin(), topics.end(), expected) != topics.end();
    EXPECT_TRUE(found) << "Topic " << expected << " not found in subscribed topics";
  }

  // Clean up
  EXPECT_TRUE(plugin.stop());
}

/**
 * @brief Test plugin with executor running
 */
TEST_F(Ros2PluginTest, PluginWithExecutor) {
  Ros2Plugin plugin;

  ASSERT_TRUE(plugin.init("{}"));

  int callback_count = 0;
  auto callback = [&callback_count](
                    const std::string& topic,
                    const std::string& type,
                    const std::vector<uint8_t>& data,
                    rclcpp::Time timestamp
                  ) {
    callback_count++;
  };

  ASSERT_TRUE(plugin.subscribe("/test_topic", "std_msgs/msg/String", callback));

  // Start the executor
  ASSERT_TRUE(plugin.start());
  EXPECT_TRUE(plugin.is_running());

  // Let executor run for a short time
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Clean up
  EXPECT_TRUE(plugin.stop());
}

/**
 * @brief Test multiple initialization cycles
 */
TEST_F(Ros2PluginTest, MultipleInitCycles) {
  // First cycle
  {
    Ros2Plugin plugin1;
    EXPECT_TRUE(plugin1.init("{}"));
    EXPECT_TRUE(plugin1.start());
    EXPECT_TRUE(plugin1.stop());
  }

  // Second cycle (should work cleanly)
  {
    Ros2Plugin plugin2;
    EXPECT_TRUE(plugin2.init("{}"));
    EXPECT_TRUE(plugin2.start());
    EXPECT_TRUE(plugin2.stop());
  }
}

/**
 * @brief Test initialization with invalid JSON fails gracefully
 */
TEST_F(Ros2PluginTest, InitWithInvalidJson) {
  Ros2Plugin plugin;

  // Invalid JSON
  bool result = plugin.init("{invalid json}");
  EXPECT_FALSE(result);
  EXPECT_FALSE(plugin.is_initialized());
}

/**
 * @brief Test configuration parsing with namespace
 */
TEST_F(Ros2PluginTest, InitWithNamespace) {
  Ros2Plugin plugin;

  const char* config_json = R"({
    "node_name": "test_node",
    "namespace": "/my_namespace"
  })";

  bool result = plugin.init(config_json);
  EXPECT_TRUE(result);
  EXPECT_TRUE(plugin.is_initialized());

  auto node = plugin.get_node();
  ASSERT_NE(node, nullptr);

  // Verify namespace is set
  std::string namespace_str = node->get_namespace();
  EXPECT_EQ(namespace_str, "/my_namespace");

  // Clean up
  EXPECT_TRUE(plugin.stop());
}

/**
 * @brief Test configuration parsing with custom node name
 */
TEST_F(Ros2PluginTest, InitWithCustomNodeName) {
  Ros2Plugin plugin;

  const char* config_json = R"({
    "node_name": "custom_axon_plugin"
  })";

  bool result = plugin.init(config_json);
  EXPECT_TRUE(result);
  EXPECT_TRUE(plugin.is_initialized());

  auto node = plugin.get_node();
  ASSERT_NE(node, nullptr);

  // Verify node name
  std::string node_name = node->get_name();
  EXPECT_EQ(node_name, "custom_axon_plugin");

  // Clean up
  EXPECT_TRUE(plugin.stop());
}

// =============================================================================
// SubscriptionManager Tests
// =============================================================================

/**
 * @brief Test fixture for SubscriptionManager tests
 */
class SubscriptionManagerTest : public ::testing::Test {
protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    // Create a test node
    node_ = rclcpp::Node::make_shared("test_subscription_manager");
    manager_ = std::make_unique<SubscriptionManager>(node_);
  }

  void TearDown() override {
    manager_.reset();
    node_.reset();
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<SubscriptionManager> manager_;
};

/**
 * @brief Test constructor creates valid manager
 */
TEST_F(SubscriptionManagerTest, ConstructorCreatesValidManager) {
  ASSERT_NE(manager_, nullptr);
  EXPECT_TRUE(manager_->get_subscribed_topics().empty());
}

/**
 * @brief Test subscribe to a topic
 */
TEST_F(SubscriptionManagerTest, SubscribeToTopic) {
  int callback_count = 0;
  auto callback = [&callback_count](
                    const std::string& topic,
                    const std::string& type,
                    const std::vector<uint8_t>& data,
                    rclcpp::Time timestamp
                  ) {
    callback_count++;
  };

  ros2_plugin::SubscribeOptions options;
  options.qos = rclcpp::QoS(10);
  bool result = manager_->subscribe("/test_topic", "std_msgs/msg/String", options, callback);

  // Subscribe should succeed (creates subscription even if no publisher)
  EXPECT_TRUE(result);

  auto topics = manager_->get_subscribed_topics();
  EXPECT_EQ(topics.size(), 1);
  EXPECT_EQ(topics[0], "/test_topic");
}

/**
 * @brief Test unsubscribe from a topic
 */
TEST_F(SubscriptionManagerTest, UnsubscribeFromTopic) {
  auto callback =
    [](const std::string&, const std::string&, const std::vector<uint8_t>&, rclcpp::Time) {};

  ros2_plugin::SubscribeOptions options;
  options.qos = rclcpp::QoS(10);
  ASSERT_TRUE(manager_->subscribe("/test_topic", "std_msgs/msg/String", options, callback));

  // Unsubscribe should succeed
  EXPECT_TRUE(manager_->unsubscribe("/test_topic"));

  // Topic should be removed
  auto topics = manager_->get_subscribed_topics();
  EXPECT_TRUE(topics.empty());

  // Unsubscribe again should fail
  EXPECT_FALSE(manager_->unsubscribe("/test_topic"));
}

/**
 * @brief Test double subscribe to same topic
 */
TEST_F(SubscriptionManagerTest, DoubleSubscribeSameTopic) {
  auto callback =
    [](const std::string&, const std::string&, const std::vector<uint8_t>&, rclcpp::Time) {};

  ros2_plugin::SubscribeOptions options;
  options.qos = rclcpp::QoS(10);
  ASSERT_TRUE(manager_->subscribe("/test_topic", "std_msgs/msg/String", options, callback));

  // Second subscribe should return true (already subscribed, but not an error)
  ros2_plugin::SubscribeOptions options2;
  options2.qos = rclcpp::QoS(10);
  bool result = manager_->subscribe("/test_topic", "std_msgs/msg/String", options2, callback);
  EXPECT_TRUE(result);

  // Should still only have one subscription
  auto topics = manager_->get_subscribed_topics();
  EXPECT_EQ(topics.size(), 1);
}

/**
 * @brief Test subscribe to multiple topics
 */
TEST_F(SubscriptionManagerTest, SubscribeToMultipleTopics) {
  auto callback =
    [](const std::string&, const std::string&, const std::vector<uint8_t>&, rclcpp::Time) {};

  rclcpp::QoS qos(10);

  ros2_plugin::SubscribeOptions options;
  options.qos = rclcpp::QoS(10);
  EXPECT_TRUE(manager_->subscribe("/topic1", "std_msgs/msg/String", options, callback));
  options.qos = rclcpp::QoS(10);
  EXPECT_TRUE(manager_->subscribe("/topic2", "std_msgs/msg/Int32", options, callback));
  options.qos = rclcpp::QoS(10);
  EXPECT_TRUE(manager_->subscribe("/topic3", "sensor_msgs/msg/Image", options, callback));

  auto topics = manager_->get_subscribed_topics();
  EXPECT_EQ(topics.size(), 3);
}

/**
 * @brief Test get_subscribed_topics returns correct topics
 */
TEST_F(SubscriptionManagerTest, GetSubscribedTopicsReturnsCorrectTopics) {
  auto callback =
    [](const std::string&, const std::string&, const std::vector<uint8_t>&, rclcpp::Time) {};

  ros2_plugin::SubscribeOptions options;
  options.qos = rclcpp::QoS(10);

  manager_->subscribe("/topic1", "std_msgs/msg/String", options, callback);
  options.qos = rclcpp::QoS(10);
  manager_->subscribe("/topic2", "std_msgs/msg/Int32", options, callback);

  auto topics = manager_->get_subscribed_topics();
  EXPECT_EQ(topics.size(), 2);

  // Check topics are present (order may vary)
  bool has_topic1 = std::find(topics.begin(), topics.end(), "/topic1") != topics.end();
  bool has_topic2 = std::find(topics.begin(), topics.end(), "/topic2") != topics.end();

  EXPECT_TRUE(has_topic1);
  EXPECT_TRUE(has_topic2);
}

/**
 * @brief Test callback is invoked for subscription
 * Note: This test doesn't publish actual messages, just verifies callback storage
 */
TEST_F(SubscriptionManagerTest, CallbackIsStored) {
  int callback_count = 0;
  auto callback = [&callback_count](
                    const std::string& topic,
                    const std::string& type,
                    const std::vector<uint8_t>& data,
                    rclcpp::Time timestamp
                  ) {
    callback_count++;
  };

  ros2_plugin::SubscribeOptions options;
  options.qos = rclcpp::QoS(10);
  bool result = manager_->subscribe("/test_topic", "std_msgs/msg/String", options, callback);

  EXPECT_TRUE(result);
  // We can't verify callback is actually called without publishing,
  // but we can verify the subscription was created
}

/**
 * @brief Test destructor cleans up subscriptions
 */
TEST_F(SubscriptionManagerTest, DestructorCleansUpSubscriptions) {
  auto callback =
    [](const std::string&, const std::string&, const std::vector<uint8_t>&, rclcpp::Time) {};

  ros2_plugin::SubscribeOptions options;
  options.qos = rclcpp::QoS(10);
  ASSERT_TRUE(manager_->subscribe("/test_topic", "std_msgs/msg/String", options, callback));

  // Destroy manager
  manager_.reset();

  // If we get here without crash, destructor worked
  SUCCEED();
}

// =============================================================================
// Main
// =============================================================================

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
