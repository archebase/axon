// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "ros2_plugin.hpp"

#include <cstring>

#include <nlohmann/json.hpp>

// Define component name for logging
#define AXON_LOG_COMPONENT "ros2_plugin"
#include <axon_log_macros.hpp>

using axon::logging::kv;

namespace ros2_plugin {

Ros2Plugin::Ros2Plugin()
    : initialized_(false)
    , spinning_(false) {}

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

    if (config_json && std::strlen(config_json) > 0) {
      auto config = nlohmann::json::parse(config_json);

      if (config.contains("node_name")) {
        node_name = config["node_name"];
      }

      if (config.contains("namespace")) {
        namespace_str = config["namespace"];
      }
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
    // Create executor
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);

    // Start executor in separate thread using lambda
    spinning_.store(true);
    executor_thread_ = std::thread([this]() {
      while (spinning_.load() && rclcpp::ok()) {
        executor_->spin_once(std::chrono::milliseconds(100));
      }
    });

    AXON_LOG_INFO("ROS2 plugin spinning");
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
  const std::string& topic_name, const std::string& message_type,
  const SubscribeOptions& options, MessageCallbackV2 callback
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
