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

  // Subscribe to a topic with a callback
  bool subscribe(
    const std::string& topic_name, const std::string& message_type, MessageCallback callback
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
