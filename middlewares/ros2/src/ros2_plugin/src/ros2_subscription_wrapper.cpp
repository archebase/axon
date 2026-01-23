// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "ros2_subscription_wrapper.hpp"

#include <rcutils/logging_macros.h>

namespace ros2_plugin {

// =============================================================================
// SubscriptionManager Implementation
// =============================================================================

SubscriptionManager::SubscriptionManager(rclcpp::Node::SharedPtr node)
    : node_(node) {}

SubscriptionManager::~SubscriptionManager() {
  std::lock_guard<std::mutex> lock(mutex_);
  subscriptions_.clear();
}

bool SubscriptionManager::subscribe(
  const std::string& topic_name, const std::string& message_type, const rclcpp::QoS& qos,
  MessageCallback callback
) {
  std::lock_guard<std::mutex> lock(mutex_);

  // Check if already subscribed
  if (subscriptions_.find(topic_name) != subscriptions_.end()) {
    RCUTILS_LOG_WARN("Already subscribed to topic: %s", topic_name.c_str());
    return true;
  }

  try {
    // Create generic subscription using ROS2's built-in API
    // Store callback by value to be used in lambda
    auto subscription = node_->create_generic_subscription(
      topic_name,
      message_type,
      qos,
      [topic_name, message_type, callback](std::shared_ptr<rclcpp::SerializedMessage> msg) {
        if (!callback) {
          return;
        }

        try {
          // Extract raw bytes from serialized message
          std::vector<uint8_t> data(
            msg->get_rcl_serialized_message().buffer,
            msg->get_rcl_serialized_message().buffer +
              msg->get_rcl_serialized_message().buffer_length
          );

          // Get current time as timestamp
          rclcpp::Time timestamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();

          // Invoke callback with serialized data
          callback(topic_name, message_type, data, timestamp);

        } catch (const std::exception& e) {
          RCUTILS_LOG_ERROR(
            "Failed to handle message on topic %s: %s", topic_name.c_str(), e.what()
          );
        }
      }
    );

    if (!subscription) {
      RCUTILS_LOG_ERROR("Failed to create subscription for: %s", topic_name.c_str());
      return false;
    }

    // Store subscription info with callback
    SubscriptionInfo info;
    info.subscription = subscription;
    info.callback = callback;
    subscriptions_[topic_name] = std::move(info);

    RCUTILS_LOG_INFO("Subscribed to topic: %s (%s)", topic_name.c_str(), message_type.c_str());

    return true;

  } catch (const std::exception& e) {
    RCUTILS_LOG_ERROR("Exception subscribing to %s: %s", topic_name.c_str(), e.what());
    return false;
  }
}

bool SubscriptionManager::unsubscribe(const std::string& topic_name) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = subscriptions_.find(topic_name);
  if (it == subscriptions_.end()) {
    RCUTILS_LOG_WARN("Not subscribed to topic: %s", topic_name.c_str());
    return false;
  }

  subscriptions_.erase(it);
  RCUTILS_LOG_INFO("Unsubscribed from topic: %s", topic_name.c_str());
  return true;
}

std::vector<std::string> SubscriptionManager::get_subscribed_topics() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> topics;
  topics.reserve(subscriptions_.size());
  for (const auto& [topic, _] : subscriptions_) {
    topics.push_back(topic);
  }
  return topics;
}

}  // namespace ros2_plugin
