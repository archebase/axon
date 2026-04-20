// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef ROS1_PLUGIN_HPP
#define ROS1_PLUGIN_HPP

#include <ros/ros.h>

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "ros1_subscription_wrapper.hpp"

namespace ros1_plugin {

class Ros1Plugin {
public:
  Ros1Plugin();
  ~Ros1Plugin();

  // Initialize the ROS1 plugin with JSON config
  // Config format: {"node_name": "optional_name", "namespace": "optional_ns"}
  bool init(const char* config_json);

  // Start spinning (create async spinner)
  bool start();

  // Stop the plugin
  bool stop();

  // Check if initialized
  bool is_initialized() const {
    return initialized_;
  }

  // Check if spinning
  bool is_running() const {
    return spinning_;
  }

  // Options forwarded from the host recorder's subscribe call.
  // Fields populated from `options_json` in the C ABI entry point
  // (see ros1_plugin_export.cpp). All fields are optional; unset fields
  // fall back to ROS1 defaults (queue size 10, no depth compression).
  //
  // depth_compression is a plain struct whose own `enabled` bool is the
  // single source of truth for whether compression is requested. We
  // intentionally do not wrap it in std::optional because that would pull
  // a C++17 dependency into this standalone plugin's CMake configuration
  // when AXON_ENABLE_DEPTH_COMPRESSION is OFF (where std::optional is not
  // otherwise needed for this struct definition).
  struct SubscribeOptions {
    uint32_t queue_size = 10;
    DepthCompressionConfig depth_compression;  // disabled by default
  };

  // Subscribe to a topic with a callback (default options)
  bool subscribe(
    const std::string& topic_name, const std::string& message_type, MessageCallback callback
  );

  // Subscribe to a topic with a callback and options parsed from the host.
  // This is the entry point used by the C ABI (axon_subscribe) so that
  // depth_compression and other options_json fields actually take effect.
  bool subscribe(
    const std::string& topic_name, const std::string& message_type,
    const SubscribeOptions& options, MessageCallback callback
  );

  // ABI v1.2 zero-copy subscribe. The callback receives a raw pointer +
  // release_fn/release_opaque so the recorder can adopt the buffer without
  // an extra copy. See SubscriptionManager::subscribe_v2 for the ownership
  // contract; the recorder must call release_fn(release_opaque) exactly
  // once per message.
  bool subscribe_v2(
    const std::string& topic_name, const std::string& message_type,
    const SubscribeOptions& options, MessageCallbackV2 callback
  );

  // Unsubscribe from a topic
  bool unsubscribe(const std::string& topic_name);

  // Get list of subscribed topics
  std::vector<std::string> get_subscribed_topics() const;

  // Get the ROS1 node handle (for advanced usage)
  ros::NodeHandlePtr get_node_handle() {
    return node_handle_;
  }

private:
  ros::NodeHandlePtr node_handle_;
  std::unique_ptr<SubscriptionManager> subscription_manager_;
  std::unique_ptr<ros::AsyncSpinner> async_spinner_;

  std::string node_name_;
  std::string namespace_;

  std::atomic<bool> initialized_;
  std::atomic<bool> spinning_;
};

}  // namespace ros1_plugin

#endif  // ROS1_PLUGIN_HPP
