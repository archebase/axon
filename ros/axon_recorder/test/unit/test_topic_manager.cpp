/**
 * @file test_topic_manager.cpp
 * @brief Unit tests for TopicManager class
 */

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>
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

  bool ok() const override { return true; }

  void* get_node_handle() override { return nullptr; }

  void* subscribe(const std::string& /* topic */, const std::string& /* message_type */,
                  std::function<void(const void*)> /* callback */) override {
    return nullptr;
  }

  void* subscribe_zero_copy(
    const std::string& topic, const std::string& message_type,
    std::function<void(SerializedMessageData&&)> callback,
    const SubscriptionConfig& /* config */ = SubscriptionConfig::high_throughput()
  ) override {
    // Store the callback for later invocation
    auto* sub = new MockSubscription{topic, message_type, std::move(callback)};
    subscriptions_.push_back(sub);
    return sub;
  }

  void unsubscribe(void* subscription_handle) override {
    auto* sub = static_cast<MockSubscription*>(subscription_handle);
    auto it = std::find(subscriptions_.begin(), subscriptions_.end(), sub);
    if (it != subscriptions_.end()) {
      subscriptions_.erase(it);
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
  int64_t now_nsec() const override { return 0; }

  void log_info(const std::string& /* message */) const override {}
  void log_warn(const std::string& /* message */) const override {}
  void log_error(const std::string& /* message */) const override {}
  void log_debug(const std::string& /* message */) const override {}

  std::string get_message_definition(const std::string& /* message_type */) const override {
    return "";
  }

  // Test helper: simulate message arrival
  void simulate_message(const std::string& topic, std::vector<uint8_t>&& data, int64_t timestamp) {
    for (auto* sub : subscriptions_) {
      if (sub->topic == topic) {
        SerializedMessageData msg(std::move(data), timestamp);
        sub->callback(std::move(msg));
        break;
      }
    }
  }

  size_t subscription_count() const { return subscriptions_.size(); }

private:
  struct MockSubscription {
    std::string topic;
    std::string message_type;
    std::function<void(SerializedMessageData&&)> callback;
  };

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
  auto callback = [&](const std::string& /* topic */, int64_t /* ts */,
                      std::vector<uint8_t>&& /* data */) {
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

}  // namespace
}  // namespace recorder
}  // namespace axon

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

