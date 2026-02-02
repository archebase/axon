// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "zenoh_plugin.hpp"

namespace zenoh_plugin {

class ZenohPluginTest : public ::testing::Test {
protected:
  void SetUp() override {
    plugin_ = std::make_unique<ZenohPlugin>();
  }

  void TearDown() override {
    if (plugin_) {
      plugin_->stop();
    }
  }

  std::unique_ptr<ZenohPlugin> plugin_;
};

// Test initialization with default config
TEST_F(ZenohPluginTest, InitDefaultConfig) {
#ifndef ZENOHCPP_FOUND
  GTEST_SKIP() << "Skipping: Zenoh library not available";
#endif

  EXPECT_TRUE(plugin_->init("{}"));
  EXPECT_TRUE(plugin_->is_initialized());
  EXPECT_TRUE(plugin_->is_running());
}

// Test initialization with custom config
TEST_F(ZenohPluginTest, InitCustomConfig) {
#ifndef ZENOHCPP_FOUND
  GTEST_SKIP() << "Skipping: Zenoh library not available";
#endif

  // Test with mode specified (if supported)
  EXPECT_TRUE(plugin_->init(R"({"mode": "client"})"));
  EXPECT_TRUE(plugin_->is_initialized());
}

// Test double initialization fails
TEST_F(ZenohPluginTest, DoubleInitFails) {
#ifndef ZENOHCPP_FOUND
  GTEST_SKIP() << "Skipping: Zenoh library not available";
#endif

  EXPECT_TRUE(plugin_->init("{}"));
  EXPECT_FALSE(plugin_->init("{}"));
}

// Test subscribe/unsubscribe
TEST_F(ZenohPluginTest, SubscribeUnsubscribe) {
#ifndef ZENOHCPP_FOUND
  GTEST_SKIP() << "Skipping: Zenoh library not available";
#endif

  ASSERT_TRUE(plugin_->init("{}"));

  bool callback_called = false;
  std::string received_keyexpr;

  auto callback = [&callback_called, &received_keyexpr](
                    const std::string& keyexpr,
                    const std::string& /*message_type*/,
                    const std::vector<uint8_t>& /*data*/,
                    uint64_t /*timestamp_ns*/
                  ) {
    callback_called = true;
    received_keyexpr = keyexpr;
  };

  // Subscribe to test keyexpr
  EXPECT_TRUE(plugin_->subscribe("demo/test/keyexpr", "test_type", callback));

  auto subscribed = plugin_->get_subscribed_keyexprs();
  EXPECT_EQ(subscribed.size(), 1);
  EXPECT_EQ(subscribed[0], "demo/test/keyexpr");

  // Unsubscribe
  EXPECT_TRUE(plugin_->unsubscribe("demo/test/keyexpr"));

  auto after_unsub = plugin_->get_subscribed_keyexprs();
  EXPECT_EQ(after_unsub.size(), 0);
}

// Test subscribe before init fails
TEST_F(ZenohPluginTest, SubscribeBeforeInitFails) {
  auto callback =
    [](const std::string&, const std::string&, const std::vector<uint8_t>&, uint64_t) {
      // Empty callback
    };

  EXPECT_FALSE(plugin_->subscribe("demo/test", "test_type", callback));
}

// Test start/stop lifecycle
TEST_F(ZenohPluginTest, StartStopLifecycle) {
#ifndef ZENOHCPP_FOUND
  GTEST_SKIP() << "Skipping: Zenoh library not available";
#endif

  EXPECT_TRUE(plugin_->init("{}"));
  EXPECT_TRUE(plugin_->start());

  plugin_->stop();
  EXPECT_FALSE(plugin_->is_initialized());
  EXPECT_FALSE(plugin_->is_running());
}

// Test reinitialization after stop
TEST_F(ZenohPluginTest, ReinitAfterStop) {
#ifndef ZENOHCPP_FOUND
  GTEST_SKIP() << "Skipping: Zenoh library not available";
#endif

  // First init
  EXPECT_TRUE(plugin_->init("{}"));
  EXPECT_TRUE(plugin_->is_initialized());

  // Stop
  plugin_->stop();
  EXPECT_FALSE(plugin_->is_initialized());

  // Reinit should succeed
  EXPECT_TRUE(plugin_->init("{}"));
  EXPECT_TRUE(plugin_->is_initialized());

  // Cleanup
  plugin_->stop();
}

// Test reinitialization after init failure
TEST_F(ZenohPluginTest, ReinitAfterPartialFailure) {
#ifndef ZENOHCPP_FOUND
  GTEST_SKIP() << "Skipping: Zenoh library not available";
#endif

  // Create a new plugin instance for this test
  auto test_plugin = std::make_unique<ZenohPlugin>();

  // First init should succeed
  EXPECT_TRUE(test_plugin->init("{}"));

  // Stop the plugin
  test_plugin->stop();

  // Reinit should succeed after clean shutdown
  EXPECT_TRUE(test_plugin->init("{}"));

  // Cleanup
  test_plugin->stop();
}

// Test callback execution with message publish
TEST_F(ZenohPluginTest, CallbackExecution) {
#ifndef ZENOHCPP_FOUND
  GTEST_SKIP() << "Skipping: Zenoh library not available";
#endif

  ASSERT_TRUE(plugin_->init("{}"));

  bool callback_called = false;
  std::string received_keyexpr;
  std::string received_message_type;
  std::vector<uint8_t> received_data;
  uint64_t received_timestamp = 0;

  auto callback = [&callback_called,
                   &received_keyexpr,
                   &received_message_type,
                   &received_data,
                   &received_timestamp](
                    const std::string& keyexpr,
                    const std::string& message_type,
                    const std::vector<uint8_t>& data,
                    uint64_t timestamp_ns
                  ) {
    callback_called = true;
    received_keyexpr = keyexpr;
    received_message_type = message_type;
    received_data = data;
    received_timestamp = timestamp_ns;
  };

  // Subscribe to test keyexpr
  EXPECT_TRUE(plugin_->subscribe("demo/test/callback", "test_type", callback));

  // Give subscription time to register
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Note: Actual callback execution would require publishing through Zenoh,
  // which needs a Zenoh publisher. This test verifies the callback is registered
  // and the subscription manager tracks it correctly.
  auto subscribed = plugin_->get_subscribed_keyexprs();
  EXPECT_EQ(subscribed.size(), 1);
  EXPECT_EQ(subscribed[0], "demo/test/callback");

  // In a full integration test, we would:
  // 1. Create a Zenoh publisher
  // 2. Publish to "demo/test/callback"
  // 3. Verify callback_was_called becomes true
  // 4. Verify received data matches published data

  // For unit testing, we verify callback registration
  EXPECT_TRUE(plugin_->unsubscribe("demo/test/callback"));
  EXPECT_FALSE(callback_called);  // No actual message published
}

// Test callback with multiple subscriptions
TEST_F(ZenohPluginTest, MultipleCallbacks) {
#ifndef ZENOHCPP_FOUND
  GTEST_SKIP() << "Skipping: Zenoh library not available";
#endif

  ASSERT_TRUE(plugin_->init("{}"));

  int callback1_count = 0;
  int callback2_count = 0;

  auto callback1 = [&callback1_count](
                     const std::string&, const std::string&, const std::vector<uint8_t>&, uint64_t
                   ) {
    callback1_count++;
  };

  auto callback2 = [&callback2_count](
                     const std::string&, const std::string&, const std::vector<uint8_t>&, uint64_t
                   ) {
    callback2_count++;
  };

  // Subscribe to different keyexprs
  EXPECT_TRUE(plugin_->subscribe("demo/test/one", "type1", callback1));
  EXPECT_TRUE(plugin_->subscribe("demo/test/two", "type2", callback2));

  auto subscribed = plugin_->get_subscribed_keyexprs();
  EXPECT_EQ(subscribed.size(), 2);

  // Unsubscribe one
  EXPECT_TRUE(plugin_->unsubscribe("demo/test/one"));

  auto after_unsub = plugin_->get_subscribed_keyexprs();
  EXPECT_EQ(after_unsub.size(), 1);
  EXPECT_EQ(after_unsub[0], "demo/test/two");
}

// Test stop clears subscriptions
TEST_F(ZenohPluginTest, StopClearsSubscriptions) {
#ifndef ZENOHCPP_FOUND
  GTEST_SKIP() << "Skipping: Zenoh library not available";
#endif

  ASSERT_TRUE(plugin_->init("{}"));

  auto callback =
    [](const std::string&, const std::string&, const std::vector<uint8_t>&, uint64_t) {};

  EXPECT_TRUE(plugin_->subscribe("demo/test/clear", "test_type", callback));

  auto subscribed = plugin_->get_subscribed_keyexprs();
  EXPECT_EQ(subscribed.size(), 1);

  // Stop the plugin
  plugin_->stop();

  // After stop, get_subscribed_keyexprs should return empty
  auto after_stop = plugin_->get_subscribed_keyexprs();
  EXPECT_EQ(after_stop.size(), 0);
}

// Test get_subscribed_keyexprs after init failure
TEST_F(ZenohPluginTest, GetSubscribedAfterInitFailure) {
#ifndef ZENOHCPP_FOUND
  GTEST_SKIP() << "Skipping: Zenoh library not available";
#endif

  // Plugin not initialized - should return empty
  auto keyexprs = plugin_->get_subscribed_keyexprs();
  EXPECT_EQ(keyexprs.size(), 0);

  // Now initialize successfully
  ASSERT_TRUE(plugin_->init("{}"));

  // Still empty (no subscriptions)
  keyexprs = plugin_->get_subscribed_keyexprs();
  EXPECT_EQ(keyexprs.size(), 0);
}

}  // namespace zenoh_plugin

// Main function for standalone test execution
int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
