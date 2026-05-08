// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "ros2_subscription_wrapper.hpp"

#ifdef AXON_ENABLE_DEPTH_COMPRESSION
#include "depth_compression_filter.hpp"
#endif

// Define component name for logging
#define AXON_LOG_COMPONENT "ros2_subscription"
#include <axon_log_macros.hpp>

using axon::logging::kv;

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
    AXON_LOG_WARN("Already subscribed to topic: " << kv("topic", topic_name));
    return true;
  }

  try {
#ifdef AXON_ENABLE_DEPTH_COMPRESSION
    // Create compression filter (if configured)
    std::shared_ptr<DepthCompressionFilter> compression_filter;
    if (options.depth_compression.has_value() && options.depth_compression->enabled) {
      compression_filter = std::make_shared<DepthCompressionFilter>(*options.depth_compression);
      AXON_LOG_INFO(
        "Depth compression enabled for topic "
        << kv("topic", topic_name) << ": level=" << kv("level", options.depth_compression->level)
      );
    }
#else
    // Depth compression not available at compile time
    if (options.depth_compression.has_value() && options.depth_compression->enabled) {
      AXON_LOG_WARN(
        "Depth compression requested for topic "
        << kv("topic", topic_name)
        << " but not enabled at build time. "
           "Rebuild with -DAXON_ENABLE_DEPTH_COMPRESSION=ON to enable."
      );
    }
#endif

    // Create generic subscription using ROS2's built-in API
    // Capture compression_filter by value to avoid race condition with subscriptions_ map
#ifdef AXON_ENABLE_DEPTH_COMPRESSION
    auto* captured_filter = compression_filter.get();
#else
    void* captured_filter = nullptr;  // No compression filter available
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
            static_cast<DepthCompressionFilter*>(captured_filter)
              ->filter_and_process(
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
          AXON_LOG_ERROR(
            "Failed to handle message on topic " << kv("topic", topic_name) << ": "
                                                 << kv("error", e.what())
          );
        }
      }
    );

    if (!subscription) {
      AXON_LOG_ERROR("Failed to create subscription for: " << kv("topic", topic_name));
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

    AXON_LOG_INFO(
      "Subscribed to topic " << kv("topic", topic_name) << " (" << kv("type", message_type) << ")"
    );

    return true;

  } catch (const std::exception& e) {
    AXON_LOG_ERROR(
      "Exception subscribing to " << kv("topic", topic_name) << ": " << kv("error", e.what())
    );
    return false;
  }
}

bool SubscriptionManager::unsubscribe(const std::string& topic_name) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = subscriptions_.find(topic_name);
  if (it == subscriptions_.end()) {
    AXON_LOG_WARN("Not subscribed to topic: " << kv("topic", topic_name));
    return false;
  }

  subscriptions_.erase(it);
  AXON_LOG_INFO("Unsubscribed from topic: " << kv("topic", topic_name));
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
