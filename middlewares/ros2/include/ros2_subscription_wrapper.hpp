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

namespace ros2_plugin {

// =============================================================================
// Depth Compression Support (conditional compilation)
// =============================================================================
#ifdef AXON_ENABLE_DEPTH_COMPRESSION

#include <depth_compression_filter.hpp>

#else

// Forward declarations when depth compression is disabled
struct DepthCompressionConfig {
  bool enabled = false;
  std::string level = "medium";
};

class DepthCompressionFilter;

#endif  // AXON_ENABLE_DEPTH_COMPRESSION

// Message callback type - passes raw serialized message data
using MessageCallback = std::function<void(
  const std::string& topic_name, const std::string& message_type,
  const std::vector<uint8_t>& message_data, rclcpp::Time timestamp
)>;

/**
 * 订阅选项
 */
struct SubscribeOptions {
  rclcpp::QoS qos;
  std::optional<DepthCompressionConfig> depth_compression;  // 深度压缩配置（可选）

  // 默认构造函数
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
