// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "ros2_subscription_wrapper.hpp"

#include <rcutils/logging_macros.h>

#include "depth_compression_filter.hpp"

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
  const std::string& topic_name, const std::string& message_type, const SubscribeOptions& options,
  MessageCallback callback
) {
  std::lock_guard<std::mutex> lock(mutex_);

  // Check if already subscribed
  if (subscriptions_.find(topic_name) != subscriptions_.end()) {
    RCUTILS_LOG_WARN("Already subscribed to topic: %s", topic_name.c_str());
    return true;
  }

  try {
    // 创建压缩过滤器（如果配置了）
    std::unique_ptr<DepthCompressionFilter> compression_filter;
    if (options.depth_compression.has_value()) {
      compression_filter = std::make_unique<DepthCompressionFilter>(*options.depth_compression);
      RCUTILS_LOG_INFO(
        "Depth compression enabled for topic %s: enabled=%s, level=%s",
        topic_name.c_str(),
        options.depth_compression->enabled ? "true" : "false",
        options.depth_compression->level.c_str()
      );
    }

    // Create generic subscription using ROS2's built-in API
    // Capture topic_name by value and lookup filter in subscriptions_ map when message arrives
    auto subscription = node_->create_generic_subscription(
      topic_name,
      message_type,
      options.qos,
      [this, topic_name, message_type, callback](
        std::shared_ptr<rclcpp::SerializedMessage> msg
      ) {
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

          // 查找压缩过滤器（如果存在）
          auto it = subscriptions_.find(topic_name);
          bool has_filter = (it != subscriptions_.end() && it->second.compression_filter != nullptr);

          if (has_filter) {
            it->second.compression_filter->filter_and_process(
              topic_name,
              message_type,
              data,
              timestamp.nanoseconds(),
              [&](
                const std::string& filtered_topic,
                const std::string& filtered_type,
                const std::vector<uint8_t>& filtered_data,
                uint64_t filtered_timestamp_ns
              ) {
                RCUTILS_LOG_DEBUG(
                  "Processed message on topic %s with type %s, size %zu bytes",
                  filtered_topic.c_str(), filtered_type.c_str(), filtered_data.size()
                );
                // 将纳秒时间戳转换回 rclcpp::Time
                rclcpp::Time filtered_timestamp(filtered_timestamp_ns);
                callback(filtered_topic, filtered_type, filtered_data, filtered_timestamp);
              }
            );
          } else {
            // 直接调用回调
            callback(topic_name, message_type, data, timestamp);
          }

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

    // Store subscription info with callback and filter
    SubscriptionInfo info;
    info.subscription = subscription;
    info.callback = callback;
    info.compression_filter = std::move(compression_filter);
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
