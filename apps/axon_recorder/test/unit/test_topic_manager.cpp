// Copyright (c) 2026 ArcheBase
// Axon is licensed under Mulan PSL v2.
// You can use this software according to the terms and conditions of the Mulan PSL v2.
// You may obtain a copy of Mulan PSL v2 at:
//          http://license.coscl.org.cn/MulanPSL2
// THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
// EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
// MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
// See the Mulan PSL v2 for more details.

/**
 * @file test_topic_manager.cpp
 * @brief Unit tests for TopicManager class
 */

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <unordered_set>
#include <vector>

#include "topic_manager.hpp"

namespace axon {
namespace recorder {
namespace {

/**
 * Mock RosInterface for testing.
 * Simulates ROS subscription without requiring actual ROS.
 */
class MockRosInterface : public RosInterface {
public:
  bool init(int /* argc */, char** /* argv */, const std::string& /* node_name */) override {
    return true;
  }

  void shutdown() override {}

  bool ok() const override {
    return true;
  }

  void* get_node_handle() override {
    return nullptr;
  }

  void* subscribe(
    const std::string& /* topic */, const std::string& /* message_type */,
    std::function<void(const void*)> /* callback */
  ) override {
    return nullptr;
  }

  void* subscribe_zero_copy(
    const std::string& topic, const std::string& message_type,
    std::function<void(SerializedMessageData&&)> callback,
    const SubscriptionConfig& /* config */ = SubscriptionConfig::high_throughput()
  ) override {
    // Store the callback for later invocation
    auto* sub = new MockSubscription{topic, message_type, std::move(callback)};
    std::lock_guard<std::mutex> lock(mutex_);
    subscriptions_.push_back(sub);
    return sub;
  }

  void unsubscribe(void* subscription_handle) override {
    auto* sub = static_cast<MockSubscription*>(subscription_handle);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto it = std::find(subscriptions_.begin(), subscriptions_.end(), sub);
      if (it != subscriptions_.end()) {
        subscriptions_.erase(it);
      }
    }
    delete sub;
  }

  void* advertise_service(
    const std::string& /* service_name */, const std::string& /* service_type */,
    std::function<bool(const void*, void*)> /* callback */
  ) override {
    return nullptr;
  }

  void spin_once() override {}
  void spin() override {}
  int64_t now_nsec() const override {
    return 0;
  }

  void log_info(const std::string& /* message */) const override {}
  void log_warn(const std::string& /* message */) const override {}
  void log_error(const std::string& /* message */) const override {}
  void log_debug(const std::string& /* message */) const override {}

  std::string get_message_definition(const std::string& /* message_type */) const override {
    return "";
  }

  // Test helper: simulate message arrival
  void simulate_message(const std::string& topic, std::vector<uint8_t>&& data, int64_t timestamp) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto* sub : subscriptions_) {
      if (sub->topic == topic) {
        SerializedMessageData msg(std::move(data), timestamp);
        sub->callback(std::move(msg));
        break;
      }
    }
  }

  size_t subscription_count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return subscriptions_.size();
  }

private:
  struct MockSubscription {
    std::string topic;
    std::string message_type;
    std::function<void(SerializedMessageData&&)> callback;
  };

  mutable std::mutex mutex_;
  std::vector<MockSubscription*> subscriptions_;
};

class TopicManagerTest : public ::testing::Test {
protected:
  void SetUp() override {
    mock_ros_ = std::make_unique<MockRosInterface>();
    topic_manager_ = std::make_unique<TopicManager>(mock_ros_.get());
  }

  void TearDown() override {
    topic_manager_.reset();
    mock_ros_.reset();
  }

  std::unique_ptr<MockRosInterface> mock_ros_;
  std::unique_ptr<TopicManager> topic_manager_;
};

TEST_F(TopicManagerTest, ConstructorRequiresRosInterface) {
  EXPECT_THROW(TopicManager(nullptr), std::invalid_argument);
}

TEST_F(TopicManagerTest, InitialState) {
  EXPECT_EQ(topic_manager_->topic_count(), 0);
  EXPECT_TRUE(topic_manager_->get_topics().empty());
}

TEST_F(TopicManagerTest, SubscribeUnsubscribe) {
  bool callback_invoked = false;
  auto callback =
    [&](const std::string& /* topic */, int64_t /* ts */, std::vector<uint8_t>&& /* data */) {
      callback_invoked = true;
    };

  EXPECT_TRUE(topic_manager_->subscribe("/test_topic", "std_msgs/String", callback));
  EXPECT_EQ(topic_manager_->topic_count(), 1);
  EXPECT_TRUE(topic_manager_->is_subscribed("/test_topic"));

  // Verify mock received the subscription
  EXPECT_EQ(mock_ros_->subscription_count(), 1);

  // Unsubscribe
  topic_manager_->unsubscribe("/test_topic");
  EXPECT_EQ(topic_manager_->topic_count(), 0);
  EXPECT_FALSE(topic_manager_->is_subscribed("/test_topic"));
}

TEST_F(TopicManagerTest, CannotSubscribeTwice) {
  auto callback = [](const std::string&, int64_t, std::vector<uint8_t>&&) {};

  EXPECT_TRUE(topic_manager_->subscribe("/test", "std_msgs/String", callback));
  EXPECT_FALSE(topic_manager_->subscribe("/test", "std_msgs/String", callback));
  EXPECT_EQ(topic_manager_->topic_count(), 1);
}

TEST_F(TopicManagerTest, CannotSubscribeEmptyTopic) {
  auto callback = [](const std::string&, int64_t, std::vector<uint8_t>&&) {};
  EXPECT_FALSE(topic_manager_->subscribe("", "std_msgs/String", callback));
}

TEST_F(TopicManagerTest, GetTopics) {
  auto callback = [](const std::string&, int64_t, std::vector<uint8_t>&&) {};

  topic_manager_->subscribe("/topic1", "Type1", callback);
  topic_manager_->subscribe("/topic2", "Type2", callback);
  topic_manager_->subscribe("/topic3", "Type3", callback);

  auto topics = topic_manager_->get_topics();
  EXPECT_EQ(topics.size(), 3);

  // Sort for consistent comparison
  std::sort(topics.begin(), topics.end());
  EXPECT_EQ(topics[0], "/topic1");
  EXPECT_EQ(topics[1], "/topic2");
  EXPECT_EQ(topics[2], "/topic3");
}

TEST_F(TopicManagerTest, GetTopicInfo) {
  auto callback = [](const std::string&, int64_t, std::vector<uint8_t>&&) {};

  topic_manager_->subscribe("/test", "std_msgs/String", callback);

  auto info = topic_manager_->get_topic_info();
  ASSERT_EQ(info.size(), 1);
  EXPECT_EQ(info[0].topic, "/test");
  EXPECT_EQ(info[0].message_type, "std_msgs/String");
}

TEST_F(TopicManagerTest, GetMessageType) {
  auto callback = [](const std::string&, int64_t, std::vector<uint8_t>&&) {};

  topic_manager_->subscribe("/test", "sensor_msgs/Image", callback);

  EXPECT_EQ(topic_manager_->get_message_type("/test"), "sensor_msgs/Image");
  EXPECT_EQ(topic_manager_->get_message_type("/nonexistent"), "");
}

TEST_F(TopicManagerTest, UnsubscribeAll) {
  auto callback = [](const std::string&, int64_t, std::vector<uint8_t>&&) {};

  topic_manager_->subscribe("/topic1", "Type1", callback);
  topic_manager_->subscribe("/topic2", "Type2", callback);
  topic_manager_->subscribe("/topic3", "Type3", callback);

  EXPECT_EQ(topic_manager_->topic_count(), 3);
  EXPECT_EQ(mock_ros_->subscription_count(), 3);

  topic_manager_->unsubscribe_all();

  EXPECT_EQ(topic_manager_->topic_count(), 0);
  EXPECT_EQ(mock_ros_->subscription_count(), 0);
}

TEST_F(TopicManagerTest, CallbackInvocation) {
  std::atomic<int> callback_count{0};
  std::string received_topic;
  int64_t received_timestamp = 0;
  std::vector<uint8_t> received_data;

  auto callback = [&](const std::string& topic, int64_t ts, std::vector<uint8_t>&& data) {
    received_topic = topic;
    received_timestamp = ts;
    received_data = std::move(data);
    callback_count.fetch_add(1);
  };

  ASSERT_TRUE(topic_manager_->subscribe("/test", "std_msgs/String", callback));

  // Simulate message arrival
  std::vector<uint8_t> test_data = {0x01, 0x02, 0x03};
  mock_ros_->simulate_message("/test", std::move(test_data), 12345);

  EXPECT_EQ(callback_count.load(), 1);
  EXPECT_EQ(received_topic, "/test");
  EXPECT_EQ(received_timestamp, 12345);
  EXPECT_EQ(received_data, std::vector<uint8_t>({0x01, 0x02, 0x03}));
}

TEST_F(TopicManagerTest, DestructorCleansUp) {
  auto callback = [](const std::string&, int64_t, std::vector<uint8_t>&&) {};

  topic_manager_->subscribe("/topic1", "Type1", callback);
  topic_manager_->subscribe("/topic2", "Type2", callback);

  EXPECT_EQ(mock_ros_->subscription_count(), 2);

  // Reset topic manager (calls destructor)
  topic_manager_.reset();

  // Subscriptions should be cleaned up
  EXPECT_EQ(mock_ros_->subscription_count(), 0);
}

// ============================================================================
// Enhanced Coverage Tests (Phase 5)
// ============================================================================

TEST_F(TopicManagerTest, UnsubscribeNonExistentTopic) {
  // Unsubscribe a topic that was never subscribed - should not crash
  EXPECT_NO_THROW(topic_manager_->unsubscribe("/nonexistent"));

  // Subscribe then unsubscribe twice
  auto callback = [](const std::string&, int64_t, std::vector<uint8_t>&&) {};
  topic_manager_->subscribe("/test", "std_msgs/String", callback);
  topic_manager_->unsubscribe("/test");

  // Second unsubscribe should be a no-op
  EXPECT_NO_THROW(topic_manager_->unsubscribe("/test"));
}

TEST_F(TopicManagerTest, GetMessageTypeForNonExistentTopic) {
  EXPECT_EQ(topic_manager_->get_message_type("/nonexistent"), "");
}

TEST_F(TopicManagerTest, GetTopicInfoEmpty) {
  auto info = topic_manager_->get_topic_info();
  EXPECT_TRUE(info.empty());
}

TEST_F(TopicManagerTest, GetTopicsEmpty) {
  auto topics = topic_manager_->get_topics();
  EXPECT_TRUE(topics.empty());
}

TEST_F(TopicManagerTest, IsSubscribedFalseForNonExistent) {
  EXPECT_FALSE(topic_manager_->is_subscribed("/nonexistent"));
}

TEST_F(TopicManagerTest, TopicCountAfterOperations) {
  auto callback = [](const std::string&, int64_t, std::vector<uint8_t>&&) {};

  EXPECT_EQ(topic_manager_->topic_count(), 0);

  topic_manager_->subscribe("/topic1", "Type1", callback);
  EXPECT_EQ(topic_manager_->topic_count(), 1);

  topic_manager_->subscribe("/topic2", "Type2", callback);
  EXPECT_EQ(topic_manager_->topic_count(), 2);

  topic_manager_->unsubscribe("/topic1");
  EXPECT_EQ(topic_manager_->topic_count(), 1);

  topic_manager_->unsubscribe_all();
  EXPECT_EQ(topic_manager_->topic_count(), 0);
}

TEST_F(TopicManagerTest, MultipleMessageCallbacks) {
  std::vector<std::string> received_topics;
  std::vector<int64_t> received_timestamps;

  auto callback = [&](const std::string& topic, int64_t ts, std::vector<uint8_t>&& /* data */) {
    received_topics.push_back(topic);
    received_timestamps.push_back(ts);
  };

  topic_manager_->subscribe("/topic1", "Type1", callback);
  topic_manager_->subscribe("/topic2", "Type2", callback);

  // Simulate messages on both topics
  mock_ros_->simulate_message("/topic1", {0x01}, 100);
  mock_ros_->simulate_message("/topic2", {0x02}, 200);
  mock_ros_->simulate_message("/topic1", {0x03}, 300);

  ASSERT_EQ(received_topics.size(), 3);
  EXPECT_EQ(received_topics[0], "/topic1");
  EXPECT_EQ(received_topics[1], "/topic2");
  EXPECT_EQ(received_topics[2], "/topic1");

  EXPECT_EQ(received_timestamps[0], 100);
  EXPECT_EQ(received_timestamps[1], 200);
  EXPECT_EQ(received_timestamps[2], 300);
}

TEST_F(TopicManagerTest, GetTopicInfoMultiple) {
  auto callback = [](const std::string&, int64_t, std::vector<uint8_t>&&) {};

  topic_manager_->subscribe("/camera/image", "sensor_msgs/Image", callback);
  topic_manager_->subscribe("/imu/data", "sensor_msgs/Imu", callback);
  topic_manager_->subscribe("/lidar/scan", "sensor_msgs/LaserScan", callback);

  auto info_list = topic_manager_->get_topic_info();
  ASSERT_EQ(info_list.size(), 3);

  // Check that all topics are present (order may vary)
  std::unordered_set<std::string> topics;
  for (const auto& info : info_list) {
    topics.insert(info.topic);
    // Verify message type is set
    if (info.topic == "/camera/image") {
      EXPECT_EQ(info.message_type, "sensor_msgs/Image");
    } else if (info.topic == "/imu/data") {
      EXPECT_EQ(info.message_type, "sensor_msgs/Imu");
    } else if (info.topic == "/lidar/scan") {
      EXPECT_EQ(info.message_type, "sensor_msgs/LaserScan");
    }
  }

  EXPECT_TRUE(topics.count("/camera/image"));
  EXPECT_TRUE(topics.count("/imu/data"));
  EXPECT_TRUE(topics.count("/lidar/scan"));
}

TEST_F(TopicManagerTest, ConcurrentSubscribeUnsubscribe) {
  // 4 threads covers typical multi-core contention without excessive test time
  constexpr int kNumThreads = 4;
  // 50 iterations per thread - lower than state machine test due to heavier operations
  constexpr int kIterations = 50;

  std::atomic<int> subscribe_count{0};
  std::atomic<int> unsubscribe_count{0};

  std::vector<std::thread> threads;

  for (int t = 0; t < kNumThreads; ++t) {
    threads.emplace_back([&, t]() {
      auto callback = [](const std::string&, int64_t, std::vector<uint8_t>&&) {};

      for (int i = 0; i < kIterations; ++i) {
        std::string topic = "/topic_" + std::to_string(t) + "_" + std::to_string(i);

        if (topic_manager_->subscribe(topic, "std_msgs/String", callback)) {
          subscribe_count.fetch_add(1);
        }

        topic_manager_->unsubscribe(topic);
        unsubscribe_count.fetch_add(1);
      }
    });
  }

  for (auto& thread : threads) {
    thread.join();
  }

  // All subscribes should have succeeded (unique topics)
  EXPECT_EQ(subscribe_count.load(), kNumThreads * kIterations);
  EXPECT_EQ(unsubscribe_count.load(), kNumThreads * kIterations);

  // Final count should be 0
  EXPECT_EQ(topic_manager_->topic_count(), 0);
}

TEST_F(TopicManagerTest, ConcurrentReads) {
  auto callback = [](const std::string&, int64_t, std::vector<uint8_t>&&) {};

  // Subscribe to some topics
  topic_manager_->subscribe("/topic1", "Type1", callback);
  topic_manager_->subscribe("/topic2", "Type2", callback);
  topic_manager_->subscribe("/topic3", "Type3", callback);

  // 4 threads for concurrent read stress testing
  constexpr int kNumThreads = 4;
  // 100 iterations - reads are lightweight, so more iterations
  constexpr int kIterations = 100;

  std::atomic<int> read_count{0};
  std::vector<std::thread> threads;

  for (int t = 0; t < kNumThreads; ++t) {
    threads.emplace_back([&]() {
      for (int i = 0; i < kIterations; ++i) {
        topic_manager_->topic_count();
        topic_manager_->get_topics();
        topic_manager_->get_topic_info();
        topic_manager_->is_subscribed("/topic1");
        topic_manager_->get_message_type("/topic2");
        read_count.fetch_add(5);
      }
    });
  }

  for (auto& thread : threads) {
    thread.join();
  }

  EXPECT_EQ(read_count.load(), kNumThreads * kIterations * 5);

  // Verify state is still consistent after concurrent reads
  EXPECT_EQ(topic_manager_->topic_count(), 3);
}

TEST_F(TopicManagerTest, SubscribeWithConfig) {
  auto callback = [](const std::string&, int64_t, std::vector<uint8_t>&&) {};

  SubscriptionConfig config;
  config.history_depth = 100;
  config.qos = QosProfile::HighThroughput;

  EXPECT_TRUE(topic_manager_->subscribe("/configured_topic", "std_msgs/String", callback, config));
  EXPECT_TRUE(topic_manager_->is_subscribed("/configured_topic"));
}

TEST_F(TopicManagerTest, UnsubscribeAllWhenEmpty) {
  // Should not crash when called on empty manager
  EXPECT_NO_THROW(topic_manager_->unsubscribe_all());
  EXPECT_EQ(topic_manager_->topic_count(), 0);
}

TEST_F(TopicManagerTest, SubscribeAfterUnsubscribeAll) {
  auto callback = [](const std::string&, int64_t, std::vector<uint8_t>&&) {};

  topic_manager_->subscribe("/topic1", "Type1", callback);
  topic_manager_->subscribe("/topic2", "Type2", callback);

  topic_manager_->unsubscribe_all();
  EXPECT_EQ(topic_manager_->topic_count(), 0);

  // Should be able to subscribe again
  EXPECT_TRUE(topic_manager_->subscribe("/topic1", "Type1", callback));
  EXPECT_EQ(topic_manager_->topic_count(), 1);
}

}  // namespace
}  // namespace recorder
}  // namespace axon

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
