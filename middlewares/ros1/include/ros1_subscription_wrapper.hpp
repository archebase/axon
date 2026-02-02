// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef ROS1_PLUGIN_SUBSCRIPTION_WRAPPER_HPP
#define ROS1_PLUGIN_SUBSCRIPTION_WRAPPER_HPP

#include <ros/ros.h>

#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace ros1_plugin {

// =============================================================================
// Depth Compression Support (conditional compilation)
// =============================================================================
#ifdef AXON_ENABLE_DEPTH_COMPRESSION

// Forward declarations
struct DepthCompressionConfig;
class DepthCompressionFilter;

#endif  // AXON_ENABLE_DEPTH_COMPRESSION

// Message callback type - passes raw serialized message data
using MessageCallback = std::function<void(
  const std::string& topic_name, const std::string& message_type,
  const std::vector<uint8_t>& message_data, uint64_t timestamp
)>;

class SubscriptionManager {
public:
  explicit SubscriptionManager(ros::NodeHandlePtr node_handle);
  ~SubscriptionManager();

  // Subscribe to any topic with any message type
  bool subscribe(
    const std::string& topic_name, const std::string& message_type, uint32_t queue_size,
    MessageCallback callback
  );

#ifdef AXON_ENABLE_DEPTH_COMPRESSION

  // Subscribe with optional depth compression (only available when enabled)
  bool subscribe(
    const std::string& topic_name, const std::string& message_type, uint32_t queue_size,
    MessageCallback callback, const std::optional<DepthCompressionConfig>& depth_compression
  );

#endif  // AXON_ENABLE_DEPTH_COMPRESSION

  // Unsubscribe from a topic
  bool unsubscribe(const std::string& topic_name);

  // Get all active subscriptions
  std::vector<std::string> get_subscribed_topics() const;

private:
  ros::NodeHandlePtr node_handle_;

#ifdef AXON_ENABLE_DEPTH_COMPRESSION
  // Forward declarations for conditional members
  struct DepthCompressionConfig;
  class DepthCompressionFilter;
#endif

  struct SubscriptionInfo {
    ros::Subscriber subscriber;
    MessageCallback callback;
    std::string message_type;
#ifdef AXON_ENABLE_DEPTH_COMPRESSION
    std::shared_ptr<DepthCompressionFilter> depth_filter;
#endif
  };

  std::unordered_map<std::string, SubscriptionInfo> subscriptions_;
  mutable std::mutex mutex_;
};

}  // namespace ros1_plugin

#endif  // ROS1_PLUGIN_SUBSCRIPTION_WRAPPER_HPP
