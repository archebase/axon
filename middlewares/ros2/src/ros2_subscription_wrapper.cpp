// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "ros2_subscription_wrapper.hpp"

#include <rcutils/logging_macros.h>

#ifdef AXON_ENABLE_DEPTH_COMPRESSION
#include "depth_compression_filter.hpp"
#endif

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
#ifdef AXON_ENABLE_DEPTH_COMPRESSION
    // Create compression filter (if configured)
    std::shared_ptr<DepthCompressionFilter> compression_filter;
    if (options.depth_compression.has_value() && options.depth_compression->enabled) {
      compression_filter = std::make_shared<DepthCompressionFilter>(*options.depth_compression);
      RCUTILS_LOG_INFO(
        "Depth compression enabled for topic %s: level=%s",
        topic_name.c_str(),
        options.depth_compression->level.c_str()
      );
    }
#else
    // Depth compression not available at compile time
    if (options.depth_compression.has_value() && options.depth_compression->enabled) {
      RCUTILS_LOG_WARN(
        "Depth compression requested for topic %s but not enabled at build time. "
        "Rebuild with -DAXON_ENABLE_DEPTH_COMPRESSION=ON to enable.",
        topic_name.c_str()
      );
    }
#endif

    // Create generic subscription using ROS2's built-in API
    // Capture compression_filter by value to avoid race condition with subscriptions_ map
#ifdef AXON_ENABLE_DEPTH_COMPRESSION
    auto* captured_filter = compression_filter.get();
#else
    auto* captured_filter = nullptr;
#endif

    auto subscription = node_->create_generic_subscription(
      topic_name,
      message_type,
      options.qos,
      [this, topic_name, message_type, callback, captured_filter](
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

#ifdef AXON_ENABLE_DEPTH_COMPRESSION
          // Use captured filter pointer instead of accessing subscriptions_ map
          if (captured_filter) {
            captured_filter->filter_and_process(
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
                // Convert nanosecond timestamp back to rclcpp::Time
                rclcpp::Time filtered_timestamp(filtered_timestamp_ns);
                callback(filtered_topic, filtered_type, filtered_data, filtered_timestamp);
              }
            );
          } else {
            // Direct callback
            callback(topic_name, message_type, data, timestamp);
          }
#else
          // Direct callback (no depth compression support)
          callback(topic_name, message_type, data, timestamp);
#endif

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
#ifdef AXON_ENABLE_DEPTH_COMPRESSION
    info.compression_filter = compression_filter;
#endif
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
