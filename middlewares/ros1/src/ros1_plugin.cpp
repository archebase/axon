// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "ros1_plugin.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <optional>
#include <thread>

// Define component name for logging
#define AXON_LOG_COMPONENT "ros1_plugin"
#include <axon_log_macros.hpp>

using axon::logging::kv;

namespace ros1_plugin {

namespace {

size_t read_thread_count(const nlohmann::json& value) {
  if (value.is_number_unsigned()) {
    return value.get<size_t>();
  }
  if (value.is_number_integer()) {
    const auto signed_value = value.get<int64_t>();
    return signed_value > 0 ? static_cast<size_t>(signed_value) : 0;
  }
  return 0;
}

void apply_config_object(
  const nlohmann::json& config, std::string& node_name, std::string& namespace_str,
  size_t& spinner_threads
) {
  if (!config.is_object()) {
    return;
  }

  if (config.contains("node_name") && config["node_name"].is_string()) {
    node_name = config["node_name"].get<std::string>();
  }

  if (config.contains("namespace") && config["namespace"].is_string()) {
    namespace_str = config["namespace"].get<std::string>();
  }

  for (const char* key : {"spinner_threads", "spinner_thread_count", "num_threads"}) {
    if (config.contains(key)) {
      spinner_threads = read_thread_count(config[key]);
    }
  }

  if (config.contains("spinner") && config["spinner"].is_object()) {
    const auto& spinner = config["spinner"];
    for (const char* key : {"threads", "thread_count", "num_threads"}) {
      if (spinner.contains(key)) {
        spinner_threads = read_thread_count(spinner[key]);
      }
    }
  }
}

void apply_embedded_plugin_config(
  const nlohmann::json& root, std::string& node_name, std::string& namespace_str,
  size_t& spinner_threads
) {
  if (!root.contains("plugin") || !root["plugin"].is_object()) {
    return;
  }

  const auto& plugin = root["plugin"];
  if (!plugin.contains("config")) {
    return;
  }

  if (plugin["config"].is_object()) {
    apply_config_object(plugin["config"], node_name, namespace_str, spinner_threads);
    return;
  }

  if (!plugin["config"].is_string()) {
    return;
  }

  const auto config_text = plugin["config"].get<std::string>();
  if (config_text.empty()) {
    return;
  }

  apply_config_object(
    nlohmann::json::parse(config_text), node_name, namespace_str, spinner_threads
  );
}

}  // namespace

Ros1Plugin::Ros1Plugin()
    : initialized_(false)
    , spinning_(false)
    , configured_spinner_threads_(0)
    , spinner_thread_count_(0) {}

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
    configured_spinner_threads_ = 0;

    if (config_json && std::strlen(config_json) > 0) {
      auto config = nlohmann::json::parse(config_json);
      apply_config_object(config, node_name_, namespace_, configured_spinner_threads_);
      if (config.contains("ros1")) {
        apply_config_object(config["ros1"], node_name_, namespace_, configured_spinner_threads_);
      }
      apply_embedded_plugin_config(config, node_name_, namespace_, configured_spinner_threads_);
    }

    // Initialize ROS1 if not already initialized. The recorder process owns
    // SIGINT/SIGTERM handling; letting roscpp install its SIGINT handler would
    // swallow Ctrl+C and leave the recorder main loop waiting forever.
    if (!ros::isInitialized()) {
      int argc = 0;
      ros::init(
        argc,
        nullptr,
        node_name_,
        ros::init_options::AnonymousName | ros::init_options::NoSigintHandler
      );
      AXON_LOG_INFO("ROS1 initialized without installing SIGINT handler");
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

size_t Ros1Plugin::resolve_spinner_thread_count() const {
  if (configured_spinner_threads_ > 0) {
    return configured_spinner_threads_;
  }

  const size_t hardware_threads =
    std::max<size_t>(2, static_cast<size_t>(std::thread::hardware_concurrency()));
  size_t desired_threads = 2;
  if (subscription_manager_) {
    desired_threads =
      std::max<size_t>(desired_threads, subscription_manager_->get_subscribed_topics().size());
  }
  return std::min(desired_threads, hardware_threads);
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

    spinner_thread_count_ = resolve_spinner_thread_count();
    async_spinner_ =
      std::make_unique<ros::AsyncSpinner>(static_cast<uint32_t>(spinner_thread_count_));

    // Start spinning
    spinning_.store(true);
    async_spinner_->start();

    AXON_LOG_INFO("ROS1 plugin spinning: " << kv("spinner_threads", spinner_thread_count_));
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
      AXON_LOG_INFO("Stopping ROS1 async spinner");
      async_spinner_->stop();
      AXON_LOG_INFO("ROS1 async spinner stopped");
    }

    async_spinner_.reset();
    spinner_thread_count_ = 0;
  }

  // Clear per-session subscriptions while preserving the NodeHandle and ROS
  // process state for subsequent recordings in this process.
  AXON_LOG_INFO("Clearing ROS1 subscriptions");
  subscription_manager_.reset();
  AXON_LOG_INFO("ROS1 subscriptions cleared");

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
  const std::string& topic_name, const std::string& message_type, const SubscribeOptions& options,
  MessageCallback callback
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
  return subscription_manager_->subscribe(topic_name, message_type, options.queue_size, callback);
#endif
}

bool Ros1Plugin::subscribe_v2(
  const std::string& topic_name, const std::string& message_type, const SubscribeOptions& options,
  MessageCallbackV2 callback
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
