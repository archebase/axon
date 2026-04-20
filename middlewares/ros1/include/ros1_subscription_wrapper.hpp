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

#ifdef AXON_ENABLE_DEPTH_COMPRESSION
// Include depth compression filter for DepthCompressionConfig definition
// This is safe here because dlz.hpp is only included in .cpp files
#include "depth_compression_filter.hpp"
#else
// When depth compression is disabled, define a minimal config struct
namespace ros1_plugin {
struct DepthCompressionConfig {
  bool enabled = false;
  std::string level = "medium";
};
}  // namespace ros1_plugin
#endif

namespace ros1_plugin {

// =============================================================================
// Depth Compression Support (conditional compilation)
// =============================================================================
#ifdef AXON_ENABLE_DEPTH_COMPRESSION
class DepthCompressionFilter;
#endif

// Message callback type - passes raw serialized message data (v1.x ABI).
// The buffer referenced by `message_data` is owned by the plugin and must
// be copied out synchronously from within the callback.
using MessageCallback = std::function<void(
  const std::string& topic_name, const std::string& message_type,
  const std::vector<uint8_t>& message_data, uint64_t timestamp
)>;

// ABI v1.2 zero-copy callback. Unlike MessageCallback, this one transfers
// ownership of the serialized buffer to the recorder: the callback hands
// back a raw pointer plus a release function + opaque, and the recorder
// calls `release_fn(release_opaque)` exactly once when done (either after
// MCAP write or on a queue-full drop). This lets us skip the extra memcpy
// the recorder would otherwise do in its on_message() path.
//
// For ROS1 specifically we still pay one `ros::serialization::serialize`
// cost — ShapeShifter does not expose the raw wire bytes — but the buffer
// produced by that serialize is what we adopt here, so the recorder never
// re-copies it.
using MessageCallbackV2 = std::function<void(
  const std::string& topic_name, const std::string& message_type,
  const uint8_t* message_data, size_t message_size, uint64_t timestamp,
  void (*release_fn)(void*), void* release_opaque
)>;

class SubscriptionManager {
public:
  explicit SubscriptionManager(ros::NodeHandlePtr node_handle);
  ~SubscriptionManager();

  // Subscribe to any topic with any message type (v1.x copy-out ABI).
  bool subscribe(
    const std::string& topic_name, const std::string& message_type, uint32_t queue_size,
    MessageCallback callback
  );

#ifdef AXON_ENABLE_DEPTH_COMPRESSION

  // Subscribe with optional depth compression (only available when enabled).
  bool subscribe(
    const std::string& topic_name, const std::string& message_type, uint32_t queue_size,
    MessageCallback callback, const std::optional<DepthCompressionConfig>& depth_compression
  );

#endif  // AXON_ENABLE_DEPTH_COMPRESSION

  // ABI v1.2 zero-copy subscribe. See MessageCallbackV2 for the ownership
  // contract. When depth_compression is enabled we internally fall back to
  // a copy path (filter reshapes the wire bytes anyway), but still present
  // an ownership-transfer buffer to the recorder so the C ABI contract is
  // uniform.
  bool subscribe_v2(
    const std::string& topic_name, const std::string& message_type, uint32_t queue_size,
    MessageCallbackV2 callback,
    const std::optional<DepthCompressionConfig>& depth_compression = std::nullopt
  );

  // Unsubscribe from a topic
  bool unsubscribe(const std::string& topic_name);

  // Get all active subscriptions
  std::vector<std::string> get_subscribed_topics() const;

private:
  ros::NodeHandlePtr node_handle_;

  struct SubscriptionInfo {
    ros::Subscriber subscriber;
    // Either v1 or v2 callback is populated (or neither, during teardown).
    // Keep both slots so SubscriptionInfo has a single shape regardless of
    // which subscribe overload created it.
    MessageCallback callback;
    MessageCallbackV2 callback_v2;
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
