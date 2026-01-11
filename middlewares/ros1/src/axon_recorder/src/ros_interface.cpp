#include "ros_interface.hpp"

#include <ros/ros.h>
#include <ros/serialization.h>
#include <topic_tools/shape_shifter.h>

#include <cstdlib>
#include <functional>
#include <map>
#include <memory>
#include <string>

#include "message_factory.hpp"
#include "register_common_messages.hpp"

namespace axon {
namespace recorder {

// ============================================================================
// ROS 1 Implementation
// ============================================================================

class RosInterfaceImpl : public RosInterface {
public:
  RosInterfaceImpl()
      : node_handle_(nullptr)
      , initialized_(false) {}

  ~RosInterfaceImpl() override {
    shutdown();
    // Cleanup subscriptions
    for (auto& pair : subscriptions_) {
      delete static_cast<ros::Subscriber*>(pair.first);
    }
    subscriptions_.clear();
  }

  bool init(int argc, char** argv, const std::string& node_name) override {
    if (initialized_) {
      return true;
    }

    // Only initialize ROS if not already initialized (allows test frameworks to pre-init)
    if (!ros::isInitialized()) {
      ros::init(argc, argv, node_name, ros::init_options::AnonymousName);
      owns_ros_context_ = true;
    }

    if (!ros::ok()) {
      return false;
    }

    node_handle_ = std::make_unique<ros::NodeHandle>();

    // Register common message types for full typed subscription support
    register_common_message_types();

    initialized_ = true;
    return true;
  }

  void shutdown() override {
    if (initialized_) {
      for (auto it = subscriptions_.begin(); it != subscriptions_.end();) {
        auto* sub = static_cast<ros::Subscriber*>(it->first);
        subscriptions_.erase(it++);
        delete sub;
      }

      // Only shutdown ROS if this instance initialized it
      if (owns_ros_context_) {
        ros::shutdown();
      }
      node_handle_.reset();
      initialized_ = false;
    }
  }

  bool ok() const override {
    return ros::ok() && initialized_;
  }

  void* get_node_handle() override {
    return node_handle_.get();
  }

  void* subscribe(
    const std::string& topic, const std::string& message_type,
    std::function<void(const void*)> callback
  ) override {
    if (!node_handle_) {
      return nullptr;
    }

    ros::Subscriber* sub = nullptr;

    try {
      // Use ShapeShifter for generic subscription (type-agnostic)
      auto shape_callback = [callback](const topic_tools::ShapeShifter::ConstPtr& msg) {
        if (msg) {
          callback(static_cast<const void*>(msg.get()));
        }
      };

      sub = new ros::Subscriber(
        node_handle_->subscribe<topic_tools::ShapeShifter>(topic, 10, shape_callback)
      );

    } catch (const std::exception& e) {
      ROS_ERROR(
        "Failed to subscribe to topic %s (type: %s): %s",
        topic.c_str(),
        message_type.c_str(),
        e.what()
      );
      if (sub) {
        delete sub;
        sub = nullptr;
      }
    }

    if (sub) {
      subscriptions_[sub] = std::make_pair(topic, message_type);
      ROS_INFO(
        "Successfully subscribed to topic %s (type: %s)", topic.c_str(), message_type.c_str()
      );
    }

    return sub;
  }

  void unsubscribe(void* subscription_handle) override {
    if (!subscription_handle) {
      return;
    }

    auto it = subscriptions_.find(subscription_handle);
    if (it != subscriptions_.end()) {
      auto* sub = static_cast<ros::Subscriber*>(subscription_handle);
      subscriptions_.erase(it);
      delete sub;
    }
  }

  void* subscribe_zero_copy(
    const std::string& topic, const std::string& message_type,
    std::function<void(SerializedMessageData&&)> callback, const SubscriptionConfig& /* config */
  ) override {
    if (!node_handle_) {
      return nullptr;
    }

    ros::Subscriber* sub = nullptr;

    try {
      // Use ShapeShifter for zero-copy access to serialized data
      auto shape_callback = [callback, this](const topic_tools::ShapeShifter::ConstPtr& msg) {
        if (!msg) {
          return;
        }

        // Get receive timestamp
        int64_t timestamp_ns = now_nsec();

        // Get serialized data size
        uint32_t serialized_size = msg->size();

        // Allocate buffer and serialize
        std::vector<uint8_t> buffer(serialized_size);
        ros::serialization::OStream stream(buffer.data(), serialized_size);
        msg->write(stream);

        // Create SerializedMessageData with ownership transfer
        SerializedMessageData data(std::move(buffer), timestamp_ns);
        callback(std::move(data));
      };

      sub = new ros::Subscriber(
        node_handle_->subscribe<topic_tools::ShapeShifter>(topic, 100, shape_callback)
      );

    } catch (const std::exception& e) {
      ROS_ERROR(
        "Failed to subscribe (zero-copy) to topic %s (type: %s): %s",
        topic.c_str(),
        message_type.c_str(),
        e.what()
      );
      if (sub) {
        delete sub;
        sub = nullptr;
      }
    }

    if (sub) {
      subscriptions_[sub] = std::make_pair(topic, message_type);
      ROS_INFO(
        "Successfully subscribed (zero-copy) to topic %s (type: %s)",
        topic.c_str(),
        message_type.c_str()
      );
    }

    return sub;
  }

  void* advertise_service(
    const std::string& service_name, const std::string& service_type,
    std::function<bool(const void*, void*)> /* callback */
  ) override {
    // Note: ROS 1 generic service advertisement requires compile-time type knowledge.
    // Services are registered directly in recorder_node using generated service types.
    // This generic interface is primarily for ROS 2 compatibility.
    ROS_WARN(
      "Generic service advertisement not supported in ROS 1. "
      "Use typed service registration instead for: %s (%s)",
      service_name.c_str(),
      service_type.c_str()
    );
    return nullptr;
  }

  void spin_once() override {
    ros::spinOnce();
  }

  void spin() override {
    ros::spin();
  }

  int64_t now_nsec() const override {
    auto now = ros::Time::now();
    return static_cast<int64_t>(now.sec) * 1000000000LL + static_cast<int64_t>(now.nsec);
  }

  void log_info(const std::string& message) const override {
    ROS_INFO("%s", message.c_str());
  }

  void log_warn(const std::string& message) const override {
    ROS_WARN("%s", message.c_str());
  }

  void log_error(const std::string& message) const override {
    ROS_ERROR("%s", message.c_str());
  }

  void log_debug(const std::string& message) const override {
    ROS_DEBUG("%s", message.c_str());
  }

  std::string get_message_definition(const std::string& message_type) const override {
    // For ROS 1, try to get message definition from MessageFactory
    if (MessageFactory::is_registered(message_type)) {
      MessageFactory::MessageInfo info;
      if (MessageFactory::get_message_info(message_type, info)) {
        return info.definition;
      }
    }
    // Return placeholder for unknown types
    return "# Message definition not available for: " + message_type;
  }

private:
  std::unique_ptr<ros::NodeHandle> node_handle_;
  bool initialized_;
  bool owns_ros_context_ = false;  // Track if this instance initialized ROS
  std::map<void*, std::pair<std::string, std::string>> subscriptions_;
};

// ============================================================================
// Factory Implementation
// ============================================================================

std::unique_ptr<RosInterface> RosInterfaceFactory::create() {
  return std::make_unique<RosInterfaceImpl>();
}

}  // namespace recorder
}  // namespace axon
