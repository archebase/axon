// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include <gtest/gtest.h>

#include <chrono>
#include <thread>
#include <vector>

#include "ros1_plugin.hpp"
#include "ros1_subscription_wrapper.hpp"

using namespace ros1_plugin;

/**
 * @brief Test fixture for ROS1 Plugin tests
 *
 * Sets up and tears down ROS1 context for each test
 */
class Ros1PluginTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Initialize ROS1 if not already initialized
    if (!ros::isInitialized()) {
      int argc = 0;
      ros::init(argc, nullptr, "test_ros1_plugin", ros::init_options::AnonymousName);
    }
  }

  void TearDown() override {
    // Note: We don't shutdown here to allow other tests to run
  }
};

/**
 * @brief Test default constructor initializes state correctly
 */
TEST_F(Ros1PluginTest, ConstructorInitialState) {
  Ros1Plugin plugin;

  EXPECT_FALSE(plugin.is_initialized());
  EXPECT_FALSE(plugin.is_running());
  EXPECT_EQ(plugin.get_node_handle(), nullptr);
}

/**
 * @brief Test initialization with valid JSON config
 */
TEST_F(Ros1PluginTest, InitWithValidConfig) {
  Ros1Plugin plugin;

  const char* config_json = R"({
    "node_name": "test_plugin_node"
  })";

  bool result = plugin.init(config_json);

  EXPECT_TRUE(result);
  EXPECT_TRUE(plugin.is_initialized());
  EXPECT_NE(plugin.get_node_handle(), nullptr);

  // Clean up
  EXPECT_TRUE(plugin.stop());
}

/**
 * @brief Test initialization with empty config (default values)
 */
TEST_F(Ros1PluginTest, InitWithEmptyConfig) {
  Ros1Plugin plugin;

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
TEST_F(Ros1PluginTest, InitWithMinimalConfig) {
  Ros1Plugin plugin;

  bool result = plugin.init("{}");
  EXPECT_TRUE(result);
  EXPECT_TRUE(plugin.is_initialized());

  // Clean up
  EXPECT_TRUE(plugin.stop());
}

/**
 * @brief Test that double initialization fails
 */
TEST_F(Ros1PluginTest, DoubleInitFails) {
  Ros1Plugin plugin;

  EXPECT_TRUE(plugin.init("{}"));

  // Second init should fail
  EXPECT_FALSE(plugin.init("{}"));

  // Clean up
  EXPECT_TRUE(plugin.stop());
}

/**
 * @brief Test start without initialization fails
 */
TEST_F(Ros1PluginTest, StartWithoutInitFails) {
  Ros1Plugin plugin;

  bool result = plugin.start();
  EXPECT_FALSE(result);
  EXPECT_FALSE(plugin.is_running());
}

/**
 * @brief Test start after successful initialization
 */
TEST_F(Ros1PluginTest, StartAfterInitSucceeds) {
  Ros1Plugin plugin;

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
TEST_F(Ros1PluginTest, DoubleStartFails) {
  Ros1Plugin plugin;

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
TEST_F(Ros1PluginTest, StopWithoutInitSucceeds) {
  Ros1Plugin plugin;

  bool result = plugin.stop();
  EXPECT_TRUE(result);
  EXPECT_FALSE(plugin.is_initialized());
  EXPECT_FALSE(plugin.is_running());
}

/**
 * @brief Test start and stop lifecycle
 */
TEST_F(Ros1PluginTest, StartStopLifecycle) {
  Ros1Plugin plugin;

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
 * @brief Test stop without explicit start (cleanup in destructor)
 */
TEST_F(Ros1PluginTest, DestructorStopsPlugin) {
  auto plugin = std::make_unique<Ros1Plugin>();

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
TEST_F(Ros1PluginTest, SubscribeWithoutInitFails) {
  Ros1Plugin plugin;

  int callback_count = 0;
  auto callback = [&callback_count](
                    const std::string& topic,
                    const std::string& type,
                    const std::vector<uint8_t>& data,
                    uint64_t timestamp
                  ) {
    callback_count++;
  };

  bool result = plugin.subscribe("/test/topic", "std_msgs/String", callback);
  EXPECT_FALSE(result);
  EXPECT_EQ(callback_count, 0);
}

/**
 * @brief Test unsubscribe without initialization fails
 */
TEST_F(Ros1PluginTest, UnsubscribeWithoutInitFails) {
  Ros1Plugin plugin;

  bool result = plugin.unsubscribe("/test/topic");
  EXPECT_FALSE(result);
}

/**
 * @brief Test get_subscribed_topics without initialization returns empty
 */
TEST_F(Ros1PluginTest, GetTopicsWithoutInitReturnsEmpty) {
  Ros1Plugin plugin;

  auto topics = plugin.get_subscribed_topics();
  EXPECT_TRUE(topics.empty());
}

/**
 * @brief Test subscribe with valid parameters
 */
TEST_F(Ros1PluginTest, SubscribeValidTopic) {
  Ros1Plugin plugin;

  ASSERT_TRUE(plugin.init("{}"));

  int callback_count = 0;
  auto callback = [&callback_count](
                    const std::string& topic,
                    const std::string& type,
                    const std::vector<uint8_t>& data,
                    uint64_t timestamp
                  ) {
    callback_count++;
  };

  bool result = plugin.subscribe("/test_topic", "std_msgs/String", callback);
  // This may succeed even if no publisher exists (ROS1 allows this)
  // The subscription is created but won't receive messages

  // Clean up
  EXPECT_TRUE(plugin.stop());
}

/**
 * @brief Test unsubscribe from subscribed topic
 */
TEST_F(Ros1PluginTest, UnsubscribeFromTopic) {
  Ros1Plugin plugin;

  ASSERT_TRUE(plugin.init("{}"));

  auto callback =
    [](const std::string&, const std::string&, const std::vector<uint8_t>&, uint64_t) {};

  ASSERT_TRUE(plugin.subscribe("/test_topic", "std_msgs/String", callback));
  EXPECT_TRUE(plugin.unsubscribe("/test_topic"));

  // Unsubscribe again should fail (not subscribed)
  EXPECT_FALSE(plugin.unsubscribe("/test_topic"));

  // Clean up
  EXPECT_TRUE(plugin.stop());
}

/**
 * @brief Test get_subscribed_topics returns subscribed topics
 */
TEST_F(Ros1PluginTest, GetSubscribedTopics) {
  Ros1Plugin plugin;

  ASSERT_TRUE(plugin.init("{}"));

  auto callback =
    [](const std::string&, const std::string&, const std::vector<uint8_t>&, uint64_t) {};

  plugin.subscribe("/topic1", "std_msgs/String", callback);
  plugin.subscribe("/topic2", "std_msgs/Int32", callback);
  plugin.subscribe("/topic3", "sensor_msgs/Image", callback);

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
 * @brief Test plugin with async spinner running
 */
TEST_F(Ros1PluginTest, PluginWithAsyncSpinner) {
  Ros1Plugin plugin;

  ASSERT_TRUE(plugin.init("{}"));

  int callback_count = 0;
  auto callback = [&callback_count](
                    const std::string& topic,
                    const std::string& type,
                    const std::vector<uint8_t>& data,
                    uint64_t timestamp
                  ) {
    callback_count++;
  };

  ASSERT_TRUE(plugin.subscribe("/test_topic", "std_msgs/String", callback));

  // Start the async spinner
  ASSERT_TRUE(plugin.start());
  EXPECT_TRUE(plugin.is_running());

  // Let spinner run for a short time
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Clean up
  EXPECT_TRUE(plugin.stop());
}

/**
 * @brief Test multiple initialization cycles
 */
TEST_F(Ros1PluginTest, MultipleInitCycles) {
  // First cycle
  {
    Ros1Plugin plugin1;
    EXPECT_TRUE(plugin1.init("{}"));
    EXPECT_TRUE(plugin1.start());
    EXPECT_TRUE(plugin1.stop());
  }

  // Second cycle (should work cleanly)
  {
    Ros1Plugin plugin2;
    EXPECT_TRUE(plugin2.init("{}"));
    EXPECT_TRUE(plugin2.start());
    EXPECT_TRUE(plugin2.stop());
  }
}

/**
 * @brief Test initialization with invalid JSON fails gracefully
 */
TEST_F(Ros1PluginTest, InitWithInvalidJson) {
  Ros1Plugin plugin;

  // Invalid JSON
  bool result = plugin.init("{invalid json}");
  EXPECT_FALSE(result);
  EXPECT_FALSE(plugin.is_initialized());
}

/**
 * @brief Test configuration parsing with custom node name
 */
TEST_F(Ros1PluginTest, InitWithCustomNodeName) {
  Ros1Plugin plugin;

  const char* config_json = R"({
    "node_name": "custom_axon_plugin"
  })";

  bool result = plugin.init(config_json);
  EXPECT_TRUE(result);
  EXPECT_TRUE(plugin.is_initialized());

  auto node_handle = plugin.get_node_handle();
  ASSERT_NE(node_handle, nullptr);

  // Verify node name is set (ROS1 doesn't provide easy way to get node name from NodeHandle)
  // but we can verify the NodeHandle is valid

  // Clean up
  EXPECT_TRUE(plugin.stop());
}

/**
 * @brief Test configuration parsing with namespace
 */
TEST_F(Ros1PluginTest, InitWithNamespace) {
  Ros1Plugin plugin;

  const char* config_json = R"({
    "namespace": "/my_namespace"
  })";

  bool result = plugin.init(config_json);
  EXPECT_TRUE(result);
  EXPECT_TRUE(plugin.is_initialized());

  auto node_handle = plugin.get_node_handle();
  ASSERT_NE(node_handle, nullptr);

  // Verify namespace is set by checking the namespace
  std::string ns = node_handle->getNamespace();
  EXPECT_EQ(ns, "/my_namespace");

  // Clean up
  EXPECT_TRUE(plugin.stop());
}

/**
 * @brief Test configuration parsing with both node_name and namespace
 */
TEST_F(Ros1PluginTest, InitWithNodeNameAndNamespace) {
  Ros1Plugin plugin;

  const char* config_json = R"({
    "node_name": "custom_node",
    "namespace": "/custom_ns"
  })";

  bool result = plugin.init(config_json);
  EXPECT_TRUE(result);
  EXPECT_TRUE(plugin.is_initialized());

  auto node_handle = plugin.get_node_handle();
  ASSERT_NE(node_handle, nullptr);

  // Verify both node name and namespace are set
  std::string ns = node_handle->getNamespace();
  EXPECT_EQ(ns, "/custom_ns");

  // Clean up
  EXPECT_TRUE(plugin.stop());
}

/**
 * @brief Test configuration parsing with empty namespace (default behavior)
 */
TEST_F(Ros1PluginTest, InitWithEmptyNamespace) {
  Ros1Plugin plugin;

  const char* config_json = R"({
    "namespace": ""
  })";

  bool result = plugin.init(config_json);
  EXPECT_TRUE(result);
  EXPECT_TRUE(plugin.is_initialized());

  auto node_handle = plugin.get_node_handle();
  ASSERT_NE(node_handle, nullptr);

  // Verify namespace is global namespace
  std::string ns = node_handle->getNamespace();
  EXPECT_EQ(ns, "/");

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
    if (!ros::isInitialized()) {
      int argc = 0;
      ros::init(argc, nullptr, "test_subscription_manager", ros::init_options::AnonymousName);
    }

    // Create a test node handle
    node_handle_ = boost::make_shared<ros::NodeHandle>();
    manager_ = std::make_unique<SubscriptionManager>(node_handle_);
  }

  void TearDown() override {
    manager_.reset();
    node_handle_.reset();
  }

  ros::NodeHandlePtr node_handle_;
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
                    uint64_t timestamp
                  ) {
    callback_count++;
  };

  uint32_t queue_size = 10;
  bool result = manager_->subscribe("/test_topic", "std_msgs/String", queue_size, callback);

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
    [](const std::string&, const std::string&, const std::vector<uint8_t>&, uint64_t) {};

  uint32_t queue_size = 10;
  ASSERT_TRUE(manager_->subscribe("/test_topic", "std_msgs/String", queue_size, callback));

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
    [](const std::string&, const std::string&, const std::vector<uint8_t>&, uint64_t) {};

  uint32_t queue_size = 10;
  ASSERT_TRUE(manager_->subscribe("/test_topic", "std_msgs/String", queue_size, callback));

  // Second subscribe should return true (already subscribed, but not an error)
  bool result = manager_->subscribe("/test_topic", "std_msgs/String", queue_size, callback);
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
    [](const std::string&, const std::string&, const std::vector<uint8_t>&, uint64_t) {};

  uint32_t queue_size = 10;

  EXPECT_TRUE(manager_->subscribe("/topic1", "std_msgs/String", queue_size, callback));
  EXPECT_TRUE(manager_->subscribe("/topic2", "std_msgs/Int32", queue_size, callback));
  EXPECT_TRUE(manager_->subscribe("/topic3", "sensor_msgs/Image", queue_size, callback));

  auto topics = manager_->get_subscribed_topics();
  EXPECT_EQ(topics.size(), 3);
}

/**
 * @brief Test get_subscribed_topics returns correct topics
 */
TEST_F(SubscriptionManagerTest, GetSubscribedTopicsReturnsCorrectTopics) {
  auto callback =
    [](const std::string&, const std::string&, const std::vector<uint8_t>&, uint64_t) {};

  uint32_t queue_size = 10;

  manager_->subscribe("/topic1", "std_msgs/String", queue_size, callback);
  manager_->subscribe("/topic2", "std_msgs/Int32", queue_size, callback);

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
                    uint64_t timestamp
                  ) {
    callback_count++;
  };

  uint32_t queue_size = 10;
  bool result = manager_->subscribe("/test_topic", "std_msgs/String", queue_size, callback);

  EXPECT_TRUE(result);
  // We can't verify callback is actually called without publishing,
  // but we can verify the subscription was created
}

/**
 * @brief Test destructor cleans up subscriptions
 */
TEST_F(SubscriptionManagerTest, DestructorCleansUpSubscriptions) {
  auto callback =
    [](const std::string&, const std::string&, const std::vector<uint8_t>&, uint64_t) {};

  uint32_t queue_size = 10;
  ASSERT_TRUE(manager_->subscribe("/test_topic", "std_msgs/String", queue_size, callback));

  // Destroy manager
  manager_.reset();

  // If we get here without crash, destructor worked
  SUCCEED();
}

/**
 * @brief Test different queue sizes
 */
TEST_F(SubscriptionManagerTest, DifferentQueueSizes) {
  auto callback =
    [](const std::string&, const std::string&, const std::vector<uint8_t>&, uint64_t) {};

  // Test with different queue sizes
  EXPECT_TRUE(manager_->subscribe("/topic1", "std_msgs/String", 1, callback));
  EXPECT_TRUE(manager_->subscribe("/topic2", "std_msgs/String", 10, callback));
  EXPECT_TRUE(manager_->subscribe("/topic3", "std_msgs/String", 100, callback));

  auto topics = manager_->get_subscribed_topics();
  EXPECT_EQ(topics.size(), 3);
}

// =============================================================================
// Main
// =============================================================================

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
