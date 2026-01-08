#include "ros2_plugin/subscriber.hpp"

#include <algorithm>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <sstream>

namespace ros2_plugin {

// Static ROS2 context instance
static rclcpp::Context::SharedPtr g_ros2_context = nullptr;
static bool g_context_initialized = false;

// Initialize ROS2 context
void GenericSubscriber::init_context() {
  if (!g_context_initialized) {
    g_ros2_context = std::make_shared<rclcpp::Context>();
    g_ros2_context->init(0, nullptr);
    g_context_initialized = true;
  }
}

// Shutdown ROS2 context
void GenericSubscriber::shutdown_context() {
  if (g_context_initialized && g_ros2_context) {
    g_ros2_context->shutdown("normal shutdown");
    g_ros2_context.reset();
    g_context_initialized = false;
  }
}

// Constructor - creates a generic subscriber for any message type
GenericSubscriber::GenericSubscriber(const std::string& topic_name, const std::string& message_type)
    : Node("generic_subscriber", "", rclcpp::NodeOptions().context(g_ros2_context))
    , topic_name_(topic_name)
    , message_type_str_(message_type) {
  setup_generic_subscription(topic_name, message_type);

  RCLCPP_INFO(
    this->get_logger(),
    "GenericSubscriber initialized for topic '%s' with type '%s'",
    topic_name.c_str(),
    message_type.c_str()
  );
}

GenericSubscriber::~GenericSubscriber() {
  RCLCPP_INFO(this->get_logger(), "Destroying subscriber for topic '%s'", topic_name_.c_str());
}

void GenericSubscriber::setup_generic_subscription(
  const std::string& topic_name, const std::string& message_type
) {
  // Use ROS2 native GenericSubscription with rclcpp::SerializedMessage
  // This allows subscribing to ANY message type without knowing it at compile time

  auto callback =
    [this, topic_name, message_type](const std::shared_ptr<rclcpp::SerializedMessage> msg) {
      RCLCPP_INFO(
        this->get_logger(),
        "Topic '%s' [%s]: Received serialized message (%zu bytes)",
        topic_name.c_str(),
        message_type.c_str(),
        msg->size()
      );

// Optionally: Log message hex dump for debugging
#ifdef RCLCPP_LOG_DEBUG_ENABLED
      std::stringstream ss;
      ss << "Message data (first 64 bytes): ";
      const auto* data = reinterpret_cast<const uint8_t*>(msg->get_rcl_serialized_message().buffer);
      size_t bytes_to_show = std::min(size_t(64), msg->size());
      for (size_t i = 0; i < bytes_to_show; ++i) {
        ss << std::hex << std::setw(2) << std::setfill('0') << (int)data[i] << " ";
      }
      RCLCPP_DEBUG(this->get_logger(), "%s", ss.str().c_str());
#endif

      // Invoke data callback if set (for MCAP recording)
      if (data_callback_) {
        // Access raw serialized message bytes
        const auto& rcl_msg = msg->get_rcl_serialized_message();
        const uint8_t* data_ptr = reinterpret_cast<const uint8_t*>(rcl_msg.buffer);
        size_t data_size = rcl_msg.buffer_length;

        // Get timestamp
        uint64_t timestamp = this->now().nanoseconds();

        // Create MessageData with raw bytes
        MessageData data(data_ptr, data_size, timestamp, topic_name, message_type);
        data_callback_(data);
      }
    };

  // Create GenericSubscription using ROS2's native create_generic_subscription API
  // This loads the type support library dynamically at runtime
  subscription_ = this->create_generic_subscription(
    topic_name, message_type, rclcpp::QoS(rclcpp::KeepLast(10)), callback
  );

  RCLCPP_INFO(
    this->get_logger(),
    "Created generic subscription for topic '%s' with type '%s'",
    topic_name.c_str(),
    message_type.c_str()
  );
}

void GenericSubscriber::spin() {
  rclcpp::spin(this->get_node_base_interface());
}

// Factory functions for dynamic loading
extern "C" {
void init_ros2_context() {
  GenericSubscriber::init_context();
}

void shutdown_ros2_context() {
  GenericSubscriber::shutdown_context();
}

GenericSubscriber* create_subscriber(const char* topic_name, const char* message_type) {
  if (topic_name == nullptr || strlen(topic_name) == 0) {
    return new ros2_plugin::GenericSubscriber("topic", std::string("std_msgs/msg/String"));
  }
  if (message_type == nullptr || strlen(message_type) == 0) {
    return new ros2_plugin::GenericSubscriber(
      std::string(topic_name), std::string("std_msgs/msg/String")
    );
  }
  return new ros2_plugin::GenericSubscriber(std::string(topic_name), std::string(message_type));
}

void destroy_subscriber(GenericSubscriber* subscriber) {
  delete subscriber;
}

void spin_subscriber(GenericSubscriber* subscriber) {
  if (subscriber) {
    subscriber->spin();
  }
}

const char* get_topic_name(GenericSubscriber* subscriber) {
  if (subscriber) {
    return subscriber->get_topic_name().c_str();
  }
  return "";
}

const char* get_message_type(GenericSubscriber* subscriber) {
  if (subscriber) {
    return subscriber->get_message_type().c_str();
  }
  return "";
}

void set_data_callback(GenericSubscriber* subscriber, RawDataCallback callback) {
  if (subscriber && callback) {
    // Wrap C callback in C++ lambda
    subscriber->set_data_callback([callback](const MessageData& data) {
      callback(
        data.data, data.size, data.timestamp, data.topic_name.c_str(), data.message_type.c_str()
      );
    });
  }
}
}

}  // namespace ros2_plugin
