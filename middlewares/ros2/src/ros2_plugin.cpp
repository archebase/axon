// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "ros2_plugin.hpp"

#include <nlohmann/json.hpp>
#include <rcutils/logging_macros.h>

namespace ros2_plugin {

Ros2Plugin::Ros2Plugin()
    : initialized_(false)
    , spinning_(false) {}

Ros2Plugin::~Ros2Plugin() {
  stop();
}

bool Ros2Plugin::init(const char* config_json) {
  if (initialized_.load()) {
    RCUTILS_LOG_ERROR("ROS2 plugin already initialized");
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
    subscription_manager_ = std::make_unique<SubscriptionManager>(node_);

    initialized_.store(true);
    RCUTILS_LOG_INFO("ROS2 plugin initialized: %s", node_->get_fully_qualified_name());

    return true;

  } catch (const std::exception& e) {
    RCUTILS_LOG_ERROR("Failed to initialize ROS2 plugin: %s", e.what());
    return false;
  }
}

bool Ros2Plugin::start() {
  if (!initialized_.load()) {
    RCUTILS_LOG_ERROR("ROS2 plugin not initialized");
    return false;
  }

  if (spinning_.load()) {
    RCUTILS_LOG_ERROR("ROS2 plugin already spinning");
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

    RCUTILS_LOG_INFO("ROS2 plugin spinning");
    return true;

  } catch (const std::exception& e) {
    RCUTILS_LOG_ERROR("Failed to start ROS2 plugin spin: %s", e.what());
    spinning_.store(false);
    return false;
  }
}

bool Ros2Plugin::stop() {
  if (!initialized_.load()) {
    return true;
  }

  RCUTILS_LOG_INFO("Shutting down ROS2 plugin");

  // Stop spinning
  if (spinning_.load()) {
    spinning_.store(false);

    if (executor_) {
      executor_->cancel();
    }

    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }

    executor_.reset();
  }

  // Clear subscription manager
  subscription_manager_.reset();

  // Shutdown node
  node_.reset();

  initialized_.store(false);
  RCUTILS_LOG_INFO("ROS2 plugin shut down");

  return true;
}

bool Ros2Plugin::subscribe(
  const std::string& topic_name, const std::string& message_type, MessageCallback callback
) {
  if (!initialized_.load()) {
    RCUTILS_LOG_ERROR("Cannot subscribe: plugin not initialized");
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

  return subscription_manager_->subscribe(topic_name, message_type, options, callback);
}

bool Ros2Plugin::unsubscribe(const std::string& topic_name) {
  if (!initialized_.load()) {
    RCUTILS_LOG_ERROR("Cannot unsubscribe: plugin not initialized");
    return false;
  }

  return subscription_manager_->unsubscribe(topic_name);
}

std::vector<std::string> Ros2Plugin::get_subscribed_topics() const {
  if (!initialized_.load()) {
    return {};
  }

  return subscription_manager_->get_subscribed_topics();
}

}  // namespace ros2_plugin
