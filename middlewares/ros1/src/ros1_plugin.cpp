// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "ros1_plugin.hpp"

#include <nlohmann/json.hpp>
#include <ros/console.h>

namespace ros1_plugin {

Ros1Plugin::Ros1Plugin()
    : initialized_(false)
    , spinning_(false) {}

Ros1Plugin::~Ros1Plugin() {
  stop();
}

bool Ros1Plugin::init(const char* config_json) {
  if (initialized_.load()) {
    ROS_ERROR("ROS1 plugin already initialized");
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
    subscription_manager_ = std::make_unique<SubscriptionManager>(node_handle_);

    initialized_.store(true);
    ROS_INFO("ROS1 plugin initialized: %s", node_name_.c_str());

    return true;

  } catch (const std::exception& e) {
    ROS_ERROR("Failed to initialize ROS1 plugin: %s", e.what());
    return false;
  }
}

bool Ros1Plugin::start() {
  if (!initialized_.load()) {
    ROS_ERROR("ROS1 plugin not initialized");
    return false;
  }

  if (spinning_.load()) {
    ROS_ERROR("ROS1 plugin already spinning");
    return false;
  }

  try {
    // Create async spinner with 1 thread
    // ROS1 uses AsyncSpinner instead of SingleThreadedExecutor
    async_spinner_ = std::make_unique<ros::AsyncSpinner>(1);

    // Start spinning
    spinning_.store(true);
    async_spinner_->start();

    ROS_INFO("ROS1 plugin spinning");
    return true;

  } catch (const std::exception& e) {
    ROS_ERROR("Failed to start ROS1 plugin spin: %s", e.what());
    spinning_.store(false);
    return false;
  }
}

bool Ros1Plugin::stop() {
  if (!initialized_.load()) {
    return true;
  }

  ROS_INFO("Shutting down ROS1 plugin");

  // Stop spinning
  if (spinning_.load()) {
    spinning_.store(false);

    if (async_spinner_) {
      async_spinner_->stop();
    }

    async_spinner_.reset();
  }

  // Clear subscription manager
  subscription_manager_.reset();

  // Shutdown node handle
  node_handle_.reset();

  initialized_.store(false);
  ROS_INFO("ROS1 plugin shut down");

  return true;
}

bool Ros1Plugin::subscribe(
  const std::string& topic_name, const std::string& message_type, MessageCallback callback
) {
  if (!initialized_.load()) {
    ROS_ERROR("Cannot subscribe: plugin not initialized");
    return false;
  }

  // Default queue size: 10
  uint32_t queue_size = 10;

  return subscription_manager_->subscribe(topic_name, message_type, queue_size, callback);
}

bool Ros1Plugin::unsubscribe(const std::string& topic_name) {
  if (!initialized_.load()) {
    ROS_ERROR("Cannot unsubscribe: plugin not initialized");
    return false;
  }

  return subscription_manager_->unsubscribe(topic_name);
}

std::vector<std::string> Ros1Plugin::get_subscribed_topics() const {
  if (!initialized_.load()) {
    return {};
  }

  return subscription_manager_->get_subscribed_topics();
}

}  // namespace ros1_plugin
