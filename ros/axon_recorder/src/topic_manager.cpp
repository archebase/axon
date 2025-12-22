#include "topic_manager.hpp"

#include <stdexcept>

// Logging infrastructure
#define AXON_LOG_COMPONENT "topic_manager"
#include <axon_log_macros.hpp>

namespace axon {
namespace recorder {

TopicManager::TopicManager(RosInterface* ros_interface)
    : ros_interface_(ros_interface) {
  if (!ros_interface_) {
    throw std::invalid_argument("TopicManager: ros_interface cannot be null");
  }
}

TopicManager::~TopicManager() {
  unsubscribe_all();
}

bool TopicManager::subscribe(const std::string& topic, const std::string& message_type,
                             MessageCallback callback, const SubscriptionConfig& config) {
  if (!ros_interface_) {
    AXON_LOG_ERROR("ROS interface not available");
    return false;
  }

  if (topic.empty()) {
    AXON_LOG_ERROR("Cannot subscribe to empty topic");
    return false;
  }

  // Check if already subscribed
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (subscriptions_.find(topic) != subscriptions_.end()) {
      AXON_LOG_WARN("Already subscribed to topic" << axon::logging::kv("topic", topic));
      return false;
    }
  }

  // Create subscription with zero-copy callback
  // Wrap the user callback to route messages
  auto wrapped_callback = [topic, callback](SerializedMessageData&& msg) {
    callback(topic, msg.receive_time_ns, std::move(msg.data));
  };

  void* handle = ros_interface_->subscribe_zero_copy(topic, message_type, wrapped_callback, config);
  if (!handle) {
    AXON_LOG_ERROR("Failed to subscribe to topic" << axon::logging::kv("topic", topic));
    return false;
  }

  // Store subscription info
  {
    std::lock_guard<std::mutex> lock(mutex_);
    TopicInfo info;
    info.topic = topic;
    info.message_type = message_type;
    info.subscription_handle = handle;
    subscriptions_[topic] = info;
  }

  return true;
}

void TopicManager::unsubscribe(const std::string& topic) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = subscriptions_.find(topic);
  if (it == subscriptions_.end()) {
    return;
  }

  if (ros_interface_ && it->second.subscription_handle) {
    ros_interface_->unsubscribe(it->second.subscription_handle);
  }

  subscriptions_.erase(it);
}

void TopicManager::unsubscribe_all() {
  std::lock_guard<std::mutex> lock(mutex_);

  for (auto& [topic, info] : subscriptions_) {
    if (ros_interface_ && info.subscription_handle) {
      ros_interface_->unsubscribe(info.subscription_handle);
    }
  }

  subscriptions_.clear();
}

bool TopicManager::is_subscribed(const std::string& topic) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return subscriptions_.find(topic) != subscriptions_.end();
}

size_t TopicManager::topic_count() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return subscriptions_.size();
}

std::vector<std::string> TopicManager::get_topics() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> topics;
  topics.reserve(subscriptions_.size());

  for (const auto& [topic, info] : subscriptions_) {
    topics.push_back(topic);
  }

  return topics;
}

std::vector<TopicManager::TopicInfo> TopicManager::get_topic_info() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<TopicInfo> result;
  result.reserve(subscriptions_.size());

  for (const auto& [topic, info] : subscriptions_) {
    result.push_back(info);
  }

  return result;
}

std::string TopicManager::get_message_type(const std::string& topic) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = subscriptions_.find(topic);
  return (it != subscriptions_.end()) ? it->second.message_type : "";
}

}  // namespace recorder
}  // namespace axon

