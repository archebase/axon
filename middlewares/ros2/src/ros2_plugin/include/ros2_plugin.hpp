#ifndef ROS2_PLUGIN_HPP
#define ROS2_PLUGIN_HPP

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "ros2_subscription_wrapper.hpp"

namespace ros2_plugin {

class Ros2Plugin {
public:
  Ros2Plugin();
  ~Ros2Plugin();

  // Initialize the ROS2 plugin with JSON config
  // Config format: {"node_name": "optional_name", "namespace": "optional_ns"}
  bool init(const char* config_json);

  // Start spinning (create executor and start spinning)
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

  // Get the ROS2 node (for advanced usage)
  rclcpp::Node::SharedPtr get_node() {
    return node_;
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<SubscriptionManager> subscription_manager_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;

  std::atomic<bool> initialized_;
  std::atomic<bool> spinning_;
};

}  // namespace ros2_plugin

#endif  // ROS2_PLUGIN_HPP
