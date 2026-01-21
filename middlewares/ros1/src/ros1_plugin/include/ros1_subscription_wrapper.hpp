// Copyright (c) 2026 ArcheBase
// Axon is licensed under Mulan PSL v2.
// You can use this software according to the terms and conditions of the Mulan PSL v2.
// You may obtain a copy of Mulan PSL v2 at:
//          http://license.coscl.org.cn/MulanPSL2
// THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
// EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
// MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
// See the Mulan PSL v2 for more details.

#ifndef ROS1_PLUGIN_SUBSCRIPTION_WRAPPER_HPP
#define ROS1_PLUGIN_SUBSCRIPTION_WRAPPER_HPP

#include <ros/ros.h>

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace ros1_plugin {

// Message callback type - passes raw serialized message data
using MessageCallback = std::function<void(
  const std::string& topic_name, const std::string& message_type,
  const std::vector<uint8_t>& message_data, uint64_t timestamp
)>;

class SubscriptionManager {
public:
  explicit SubscriptionManager(ros::NodeHandlePtr node_handle);
  ~SubscriptionManager();

  // Subscribe to any topic with any message type
  bool subscribe(
    const std::string& topic_name, const std::string& message_type, uint32_t queue_size,
    MessageCallback callback
  );

  // Unsubscribe from a topic
  bool unsubscribe(const std::string& topic_name);

  // Get all active subscriptions
  std::vector<std::string> get_subscribed_topics() const;

private:
  ros::NodeHandlePtr node_handle_;
  struct SubscriptionInfo {
    ros::Subscriber subscriber;
    MessageCallback callback;
    std::string message_type;
  };
  std::unordered_map<std::string, SubscriptionInfo> subscriptions_;
  mutable std::mutex mutex_;
};

}  // namespace ros1_plugin

#endif  // ROS1_PLUGIN_SUBSCRIPTION_WRAPPER_HPP
