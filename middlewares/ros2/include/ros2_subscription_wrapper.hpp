// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef ROS2_PLUGIN_SUBSCRIPTION_WRAPPER_HPP
#define ROS2_PLUGIN_SUBSCRIPTION_WRAPPER_HPP

#include <rclcpp/rclcpp.hpp>

#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#ifdef AXON_ENABLE_DEPTH_COMPRESSION
// Include depth compression filter for DepthCompressionConfig definition
// This is safe here because dlz.hpp is only included in .cpp files
#include "depth_compression_filter.hpp"
#else
// When depth compression is disabled, define a minimal config struct
namespace ros2_plugin {
struct DepthCompressionConfig {
  bool enabled = false;
  std::string level = "medium";
};
}  // namespace ros2_plugin
#endif

namespace ros2_plugin {

// =============================================================================
// Depth Compression Support (conditional compilation)
// =============================================================================
#ifdef AXON_ENABLE_DEPTH_COMPRESSION
class DepthCompressionFilter;
#endif

// Message callback type - passes raw serialized message data
using MessageCallback = std::function<void(
  const std::string& topic_name, const std::string& message_type,
  const std::vector<uint8_t>& message_data, rclcpp::Time timestamp
)>;

/**
 * @brief Subscription options
 */
struct SubscribeOptions {
  rclcpp::QoS qos;
  std::optional<DepthCompressionConfig> depth_compression;  // Depth compression config (optional)

  // Default constructor
  SubscribeOptions()
      : qos(10) {}
};

class SubscriptionManager {
public:
  explicit SubscriptionManager(rclcpp::Node::SharedPtr node);
  ~SubscriptionManager();

  // Subscribe to any topic with any message type
  bool subscribe(
    const std::string& topic_name, const std::string& message_type, const SubscribeOptions& options,
    MessageCallback callback
  );

  // Unsubscribe from a topic
  bool unsubscribe(const std::string& topic_name);

  // Get all active subscriptions
  std::vector<std::string> get_subscribed_topics() const;

private:
  rclcpp::Node::SharedPtr node_;
  struct SubscriptionInfo {
    rclcpp::GenericSubscription::SharedPtr subscription;
    MessageCallback callback;
#ifdef AXON_ENABLE_DEPTH_COMPRESSION
    std::shared_ptr<DepthCompressionFilter> compression_filter;
#endif
  };
  std::unordered_map<std::string, SubscriptionInfo> subscriptions_;
  mutable std::mutex mutex_;
};

}  // namespace ros2_plugin

#endif  // ROS2_PLUGIN_SUBSCRIPTION_WRAPPER_HPP
