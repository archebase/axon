// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "ros1_subscription_wrapper.hpp"

#include <ros/console.h>
#include <ros/serialization.h>
#include <topic_tools/shape_shifter.h>

namespace ros1_plugin {

// =============================================================================
// SubscriptionManager Implementation
// =============================================================================

SubscriptionManager::SubscriptionManager(ros::NodeHandlePtr node_handle)
    : node_handle_(node_handle) {}

SubscriptionManager::~SubscriptionManager() {
  std::lock_guard<std::mutex> lock(mutex_);
  subscriptions_.clear();
}

bool SubscriptionManager::subscribe(
  const std::string& topic_name, const std::string& message_type, uint32_t queue_size,
  MessageCallback callback
) {
  std::lock_guard<std::mutex> lock(mutex_);

  // Check if already subscribed
  if (subscriptions_.find(topic_name) != subscriptions_.end()) {
    ROS_WARN("Already subscribed to topic: %s", topic_name.c_str());
    return true;
  }

  try {
    // Use topic_tools::ShapeShifter to subscribe to any message type
    // This is the ROS1 equivalent of ROS2's GenericSubscription
    auto subscriber = node_handle_->subscribe<topic_tools::ShapeShifter>(
      topic_name,
      queue_size,
      [topic_name,
       message_type,
       callback](const typename topic_tools::ShapeShifter::ConstPtr& msg) {
        if (!callback) {
          return;
        }

        try {
          // Get the message definition (type) from the ShapeShifter
          std::string actual_type = msg->getDataType();

          // Serialize the message to a byte array
          uint32_t serial_size = ros::serialization::serializationLength(*msg);
          std::vector<uint8_t> data(serial_size);

          ros::serialization::OStream stream(data.data(), serial_size);
          ros::serialization::serialize(stream, *msg);

          // Get current time as timestamp (nanoseconds)
          uint64_t timestamp = static_cast<uint64_t>(ros::Time::now().toNSec());

          // Invoke callback with serialized data
          callback(topic_name, actual_type, data, timestamp);

        } catch (const std::exception& e) {
          ROS_ERROR("Failed to handle message on topic %s: %s", topic_name.c_str(), e.what());
        }
      }
    );

    if (!subscriber) {
      ROS_ERROR("Failed to create subscription for: %s", topic_name.c_str());
      return false;
    }

    // Store subscription info with callback
    SubscriptionInfo info;
    info.subscriber = subscriber;
    info.callback = callback;
    info.message_type = message_type;
    subscriptions_[topic_name] = std::move(info);

    ROS_INFO("Subscribed to topic: %s (%s)", topic_name.c_str(), message_type.c_str());

    return true;

  } catch (const std::exception& e) {
    ROS_ERROR("Exception subscribing to %s: %s", topic_name.c_str(), e.what());
    return false;
  }
}

bool SubscriptionManager::unsubscribe(const std::string& topic_name) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = subscriptions_.find(topic_name);
  if (it == subscriptions_.end()) {
    ROS_WARN("Not subscribed to topic: %s", topic_name.c_str());
    return false;
  }

  subscriptions_.erase(it);
  ROS_INFO("Unsubscribed from topic: %s", topic_name.c_str());
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

}  // namespace ros1_plugin
