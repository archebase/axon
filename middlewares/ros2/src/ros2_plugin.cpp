// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "ros2_plugin.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>

// Define component name for logging
#define AXON_LOG_COMPONENT "ros2_plugin"
#include <axon_log_macros.hpp>

using axon::logging::kv;

namespace ros2_plugin {

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
  size_t& executor_threads
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

  for (const char* key : {"executor_threads", "executor_thread_count", "num_threads"}) {
    if (config.contains(key)) {
      executor_threads = read_thread_count(config[key]);
    }
  }

  if (config.contains("executor") && config["executor"].is_object()) {
    const auto& executor = config["executor"];
    for (const char* key : {"threads", "thread_count", "num_threads"}) {
      if (executor.contains(key)) {
        executor_threads = read_thread_count(executor[key]);
      }
    }
  }
}

void apply_embedded_plugin_config(
  const nlohmann::json& root, std::string& node_name, std::string& namespace_str,
  size_t& executor_threads
) {
  if (!root.contains("plugin") || !root["plugin"].is_object()) {
    return;
  }

  const auto& plugin = root["plugin"];
  if (!plugin.contains("config")) {
    return;
  }

  if (plugin["config"].is_object()) {
    apply_config_object(plugin["config"], node_name, namespace_str, executor_threads);
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
    nlohmann::json::parse(config_text), node_name, namespace_str, executor_threads
  );
}

}  // namespace

Ros2Plugin::Ros2Plugin()
    : initialized_(false)
    , spinning_(false)
    , configured_executor_threads_(0)
    , executor_thread_count_(0) {}

Ros2Plugin::~Ros2Plugin() {
  stop();
}

bool Ros2Plugin::ensure_subscription_manager() {
  if (!node_) {
    return false;
  }

  if (!subscription_manager_) {
    subscription_manager_ = std::make_unique<SubscriptionManager>(node_);
  }

  return true;
}

bool Ros2Plugin::init(const char* config_json) {
  if (initialized_.load()) {
    AXON_LOG_ERROR("ROS2 plugin already initialized");
    return false;
  }

  try {
    // Parse configuration
    std::string node_name = "axon_ros2_plugin";
    std::string namespace_str = "";
    configured_executor_threads_ = 0;

    if (config_json && std::strlen(config_json) > 0) {
      auto config = nlohmann::json::parse(config_json);
      apply_config_object(config, node_name, namespace_str, configured_executor_threads_);
      if (config.contains("ros2")) {
        apply_config_object(config["ros2"], node_name, namespace_str, configured_executor_threads_);
      }
      apply_embedded_plugin_config(config, node_name, namespace_str, configured_executor_threads_);
    }

    // Initialize ROS2
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    // Create node options
    rclcpp::NodeOptions options;
    options.start_parameter_event_publisher(false);
    options.start_parameter_services(false);

    // Create node
    if (namespace_str.empty()) {
      node_ = rclcpp::Node::make_shared(node_name, options);
    } else {
      node_ = rclcpp::Node::make_shared(node_name, namespace_str, options);
    }

    // Create subscription manager
    if (!ensure_subscription_manager()) {
      AXON_LOG_ERROR("Failed to create ROS2 subscription manager");
      node_.reset();
      return false;
    }

    initialized_.store(true);
    AXON_LOG_INFO("ROS2 plugin initialized: " << kv("node", node_->get_fully_qualified_name()));

    return true;

  } catch (const std::exception& e) {
    AXON_LOG_ERROR("Failed to initialize ROS2 plugin: " << kv("error", e.what()));
    return false;
  }
}

size_t Ros2Plugin::resolve_executor_thread_count() const {
  if (configured_executor_threads_ > 0) {
    return configured_executor_threads_;
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

bool Ros2Plugin::start() {
  if (!initialized_.load()) {
    AXON_LOG_ERROR("ROS2 plugin not initialized");
    return false;
  }

  if (spinning_.load()) {
    AXON_LOG_ERROR("ROS2 plugin already spinning");
    return false;
  }

  try {
    executor_thread_count_ = resolve_executor_thread_count();
    rclcpp::ExecutorOptions executor_options;
    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(
      executor_options, executor_thread_count_
    );
    executor_->add_node(node_);

    spinning_.store(true);
    executor_thread_ = std::thread([this]() {
      try {
        executor_->spin();
      } catch (const std::exception& e) {
        AXON_LOG_ERROR("ROS2 executor spin failed: " << kv("error", e.what()));
      }
      spinning_.store(false);
    });

    const auto spin_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (!executor_->is_spinning() && std::chrono::steady_clock::now() < spin_deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (!executor_->is_spinning()) {
      AXON_LOG_ERROR("ROS2 executor did not enter spinning state");
      spinning_.store(false);
      executor_->cancel();
      if (executor_thread_.joinable()) {
        executor_thread_.join();
      }
      executor_.reset();
      executor_thread_count_ = 0;
      return false;
    }

    AXON_LOG_INFO("ROS2 plugin spinning: " << kv("executor_threads", executor_thread_count_));
    return true;

  } catch (const std::exception& e) {
    AXON_LOG_ERROR("Failed to start ROS2 plugin spin: " << kv("error", e.what()));
    spinning_.store(false);
    return false;
  }
}

bool Ros2Plugin::stop() {
  const bool should_release_node = initialized_.load();
  if (!stop_session()) {
    return false;
  }

  if (should_release_node) {
    AXON_LOG_INFO("Shutting down ROS2 plugin");

    // Shutdown node
    node_.reset();

    initialized_.store(false);
    AXON_LOG_INFO("ROS2 plugin shut down");
  }

  return true;
}

bool Ros2Plugin::stop_session() {
  if (!initialized_.load()) {
    return true;
  }

  AXON_LOG_INFO("Stopping ROS2 plugin session");

  // Stop spinning
  if (spinning_.exchange(false)) {
    if (executor_) {
      executor_->cancel();
    }
  } else if (executor_) {
    executor_->cancel();
  }

  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }

  executor_.reset();
  executor_thread_count_ = 0;

  // Clear per-session subscriptions while preserving the node for the next recording.
  subscription_manager_.reset();

  AXON_LOG_INFO("ROS2 plugin session stopped");

  return true;
}

bool Ros2Plugin::subscribe(
  const std::string& topic_name, const std::string& message_type, MessageCallback callback
) {
  if (!initialized_.load()) {
    RCUTILS_LOG_ERROR("Cannot subscribe: plugin not initialized");
    return false;
  }

  if (!ensure_subscription_manager()) {
    AXON_LOG_ERROR("Cannot subscribe: subscription manager unavailable");
    return false;
  }

  // Default QoS: keep last 10, reliable, volatile
  SubscribeOptions options;
  options.qos = rclcpp::QoS(10);
  options.qos.reliable();
  options.qos.durability_volatile();

  return subscription_manager_->subscribe(topic_name, message_type, options, callback);
}

bool Ros2Plugin::subscribe(
  const std::string& topic_name, const std::string& message_type, const SubscribeOptions& options,
  MessageCallback callback
) {
  if (!initialized_.load()) {
    RCUTILS_LOG_ERROR("Cannot subscribe: plugin not initialized");
    return false;
  }

  if (!ensure_subscription_manager()) {
    AXON_LOG_ERROR("Cannot subscribe: subscription manager unavailable");
    return false;
  }

  return subscription_manager_->subscribe(topic_name, message_type, options, callback);
}

bool Ros2Plugin::subscribe_v2(
  const std::string& topic_name, const std::string& message_type, const SubscribeOptions& options,
  MessageCallbackV2 callback
) {
  if (!initialized_.load()) {
    RCUTILS_LOG_ERROR("Cannot subscribe_v2: plugin not initialized");
    return false;
  }

  if (!ensure_subscription_manager()) {
    AXON_LOG_ERROR("Cannot subscribe_v2: subscription manager unavailable");
    return false;
  }

  return subscription_manager_->subscribe_v2(topic_name, message_type, options, callback);
}

bool Ros2Plugin::unsubscribe(const std::string& topic_name) {
  if (!initialized_.load()) {
    RCUTILS_LOG_ERROR("Cannot unsubscribe: plugin not initialized");
    return false;
  }

  if (!ensure_subscription_manager()) {
    AXON_LOG_ERROR("Cannot unsubscribe: subscription manager unavailable");
    return false;
  }

  return subscription_manager_->unsubscribe(topic_name);
}

std::vector<std::string> Ros2Plugin::get_subscribed_topics() const {
  if (!initialized_.load() || !subscription_manager_) {
    return {};
  }

  return subscription_manager_->get_subscribed_topics();
}

}  // namespace ros2_plugin
