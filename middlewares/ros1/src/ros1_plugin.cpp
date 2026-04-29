// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "ros1_plugin.hpp"

#include <nlohmann/json.hpp>

#include <optional>

// Define component name for logging
#define AXON_LOG_COMPONENT "ros1_plugin"
#include <axon_log_macros.hpp>

using axon::logging::kv;

namespace ros1_plugin {

Ros1Plugin::Ros1Plugin()
    : initialized_(false)
    , spinning_(false) {}

Ros1Plugin::~Ros1Plugin() {
  stop();
}

void Ros1Plugin::ensure_subscription_manager() {
  if (!subscription_manager_ && node_handle_) {
    subscription_manager_ = std::make_unique<SubscriptionManager>(node_handle_);
  }
}

bool Ros1Plugin::init(const char* config_json) {
  if (initialized_.load()) {
    AXON_LOG_ERROR("ROS1 plugin already initialized");
    return false;
  }

  try {
    // Parse configuration
    node_name_ = "axon_ros1_plugin";
    namespace_ = "";

    if (config_json && std::strlen(config_json) > 0) {
      auto config = nlohmann::json::parse(config_json);

      if (config.contains("node_name")) {
        node_name_ = config["node_name"];
      }

      if (config.contains("namespace")) {
        namespace_ = config["namespace"];
      }
    }

    // Initialize ROS1 if not already initialized
    if (!ros::isInitialized()) {
      int argc = 0;
      ros::init(argc, nullptr, node_name_, ros::init_options::AnonymousName);
    }

    // Create node handle with namespace if provided
    if (namespace_.empty()) {
      node_handle_ = boost::make_shared<ros::NodeHandle>();
    } else {
      node_handle_ = boost::make_shared<ros::NodeHandle>(namespace_);
    }

    // Create subscription manager
    ensure_subscription_manager();

    initialized_.store(true);
    AXON_LOG_INFO("ROS1 plugin initialized: " << kv("node", node_name_));

    return true;

  } catch (const std::exception& e) {
    AXON_LOG_ERROR("Failed to initialize ROS1 plugin: " << kv("error", e.what()));
    return false;
  }
}

bool Ros1Plugin::start() {
  if (!initialized_.load()) {
    AXON_LOG_ERROR("ROS1 plugin not initialized");
    return false;
  }

  if (spinning_.load()) {
    AXON_LOG_ERROR("ROS1 plugin already spinning");
    return false;
  }

  try {
    ensure_subscription_manager();

    // Create async spinner with 1 thread
    // ROS1 uses AsyncSpinner instead of SingleThreadedExecutor
    async_spinner_ = std::make_unique<ros::AsyncSpinner>(1);

    // Start spinning
    spinning_.store(true);
    async_spinner_->start();

    AXON_LOG_INFO("ROS1 plugin spinning");
    return true;

  } catch (const std::exception& e) {
    AXON_LOG_ERROR("Failed to start ROS1 plugin spin: " << kv("error", e.what()));
    spinning_.store(false);
    return false;
  }
}

bool Ros1Plugin::stop() {
  const bool should_shutdown_ros = initialized_.load();
  if (!stop_session()) {
    return false;
  }

  if (should_shutdown_ros) {
    AXON_LOG_INFO("Shutting down ROS1 process state");
    node_handle_.reset();
    initialized_.store(false);
    shutdown_ros();
    AXON_LOG_INFO("ROS1 plugin shut down");
  }

  return true;
}

bool Ros1Plugin::stop_session() {
  if (!initialized_.load()) {
    return true;
  }

  AXON_LOG_INFO("Stopping ROS1 plugin session");

  // Stop spinning
  if (spinning_.load()) {
    spinning_.store(false);

    if (async_spinner_) {
      async_spinner_->stop();
    }

    async_spinner_.reset();
  }

  // Clear per-session subscriptions while preserving the NodeHandle and ROS
  // process state for subsequent recordings in this process.
  subscription_manager_.reset();

  AXON_LOG_INFO("ROS1 plugin session stopped");

  return true;
}

void Ros1Plugin::shutdown_ros() {
  if (ros::isStarted() || ros::isInitialized()) {
    ros::shutdown();
  }
}

bool Ros1Plugin::subscribe(
  const std::string& topic_name, const std::string& message_type, MessageCallback callback
) {
  return subscribe(topic_name, message_type, SubscribeOptions{}, callback);
}

bool Ros1Plugin::subscribe(
  const std::string& topic_name, const std::string& message_type,
  const SubscribeOptions& options, MessageCallback callback
) {
  if (!initialized_.load()) {
    AXON_LOG_ERROR("Cannot subscribe: plugin not initialized");
    return false;
  }
  ensure_subscription_manager();

#ifdef AXON_ENABLE_DEPTH_COMPRESSION
  // When depth compression is compiled in, route through the overload
  // that accepts a DepthCompressionConfig — but only as a non-empty
  // std::optional when the caller actually asked for it. This preserves
  // the wrapper's existing "filter disabled" fast path for the common
  // case where no depth_compression block was present in options_json.
  std::optional<DepthCompressionConfig> dc;
  if (options.depth_compression.enabled) {
    dc = options.depth_compression;
  }
  return subscription_manager_->subscribe(
    topic_name, message_type, options.queue_size, callback, dc
  );
#else
  // Without depth compression support, the options.depth_compression field
  // is ignored (a warning is logged at the ABI layer). The only option
  // currently honored in this build is queue_size.
  return subscription_manager_->subscribe(
    topic_name, message_type, options.queue_size, callback
  );
#endif
}

bool Ros1Plugin::subscribe_v2(
  const std::string& topic_name, const std::string& message_type,
  const SubscribeOptions& options, MessageCallbackV2 callback
) {
  if (!initialized_.load()) {
    AXON_LOG_ERROR("Cannot subscribe_v2: plugin not initialized");
    return false;
  }
  ensure_subscription_manager();

  // The SubscriptionManager::subscribe_v2 always takes an optional<DC>, so
  // we only wrap the DC when the caller actually enabled it. This matches
  // the v1 subscribe() behavior above and keeps the filter opt-in.
  std::optional<DepthCompressionConfig> dc;
  if (options.depth_compression.enabled) {
    dc = options.depth_compression;
  }
  return subscription_manager_->subscribe_v2(
    topic_name, message_type, options.queue_size, callback, dc
  );
}

bool Ros1Plugin::unsubscribe(const std::string& topic_name) {
  if (!initialized_.load()) {
    AXON_LOG_ERROR("Cannot unsubscribe: plugin not initialized");
    return false;
  }
  ensure_subscription_manager();

  return subscription_manager_->unsubscribe(topic_name);
}

std::vector<std::string> Ros1Plugin::get_subscribed_topics() const {
  if (!initialized_.load()) {
    return {};
  }

  if (!subscription_manager_) {
    return {};
  }

  return subscription_manager_->get_subscribed_topics();
}

}  // namespace ros1_plugin
