#include "ros1_plugin/subscriber.hpp"

#include <ros/serialization.h>

#include <algorithm>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <sstream>

namespace ros1_plugin {

// Static ROS1 state
static bool g_ros_initialized = false;
static std::unique_ptr<ros::NodeHandle> g_node_handle nullptr;

// Initialize ROS1
void GenericSubscriber::init_ros(int argc, char** argv) {
  if (!g_ros_initialized) {
    ros::init(
      argc,
      argv,
      "generic_subscriber_node",
      ros::init_options::AnonymousName | ros::init_options::NoSigintHandler
    );
    g_node_handle = std::make_unique<ros::NodeHandle>();
    g_ros_initialized = true;
    ROS_INFO("ROS1 initialized for plugin");
  }
}

// Shutdown ROS1
void GenericSubscriber::shutdown_ros() {
  if (g_ros_initialized) {
    g_node_handle.reset();
    ros::shutdown();
    g_ros_initialized = false;
    ROS_INFO("ROS1 shutdown complete");
  }
}

// Constructor - creates a generic subscriber for any message type
GenericSubscriber::GenericSubscriber(const std::string& topic_name, const std::string& message_type)
    : topic_name_(topic_name)
    , message_type_str_(message_type) {
  // Create node handle for this subscriber
  node_handle_ = std::make_unique<ros::NodeHandle>();

  setup_generic_subscription(topic_name, message_type);

  ROS_INFO(
    "GenericSubscriber initialized for topic '%s' with type '%s'",
    topic_name.c_str(),
    message_type.c_str()
  );
}

GenericSubscriber::~GenericSubscriber() {
  ROS_INFO("Destroying subscriber for topic '%s'", topic_name_.c_str());
}

void GenericSubscriber::setup_generic_subscription(
  const std::string& topic_name, const std::string& message_type
) {
  // For ROS1, we need to use topic_tools::ShapeShifter for type-agnostic subscription
  // This allows subscribing to ANY message type without knowing it at compile time

  // Note: In ROS1, true generic subscription requires topic_tools::ShapeShifter
  // For simplicity, we'll subscribe to any topic and deserialize using shape shifter

  auto callback = [this,
                   topic_name,
                   message_type](const boost::shared_ptr<topic_tools::ShapeShifter const>& msg) {
    ROS_INFO(
      "Topic '%s' [%s]: Received serialized message (%zu bytes)",
      topic_name.c_str(),
      message_type.c_str(),
      msg->size()
    );

// Optionally: Log message hex dump for debugging
#ifdef ROSCONSOLE_BACKEND_LOG4CXX
    std::stringstream ss;
    ss << "Message data (first 64 bytes): ";
    const uint8_t* data = reinterpret_cast<const uint8_t*>(msg->msgBuf());
    size_t bytes_to_show = std::min(size_t(64), msg->size());
    for (size_t i = 0; i < bytes_to_show; ++i) {
      ss << std::hex << std::setw(2) << std::setfill('0') << (int)data[i] << " ";
    }
    ROS_DEBUG("%s", ss.str().c_str());
#endif

    // Invoke data callback if set (for MCAP recording)
    if (data_callback_) {
      // Access raw serialized message bytes from ShapeShifter
      const uint8_t* data_ptr = reinterpret_cast<const uint8_t*>(msg->msgBuf());
      size_t data_size = msg->size();

      // Get timestamp
      uint64_t timestamp = ros::Time::now().toNSec();

      // Create MessageData with raw bytes
      MessageData data(data_ptr, data_size, timestamp, topic_name, message_type);
      data_callback_(data);
    }
  };

  // Create generic subscriber using ShapeShifter
  auto* sub = new ros::Subscriber();
  *sub = g_node_handle->subscribe<topic_tools::ShapeShifter>(topic_name, 10, callback);

  // Store subscription
  subscriber_.reset(sub);

  ROS_INFO(
    "Created generic subscription for topic '%s' with type '%s'",
    topic_name.c_str(),
    message_type.c_str()
  );
}

void GenericSubscriber::spin_once() {
  if (ros::ok()) {
    ros::spinOnce();
  }
}

// Factory functions for dynamic loading
extern "C" {
void init_ros1() {
  GenericSubscriber::init_ros();
}

void shutdown_ros1() {
  GenericSubscriber::shutdown_ros();
}

GenericSubscriber* create_subscriber(const char* topic_name, const char* message_type) {
  if (topic_name == nullptr || strlen(topic_name) == 0) {
    return new ros1_plugin::GenericSubscriber("topic", std::string("std_msgs/String"));
  }
  if (message_type == nullptr || strlen(message_type) == 0) {
    return new ros1_plugin::GenericSubscriber(
      std::string(topic_name), std::string("std_msgs/String")
    );
  }
  return new ros1_plugin::GenericSubscriber(std::string(topic_name), std::string(message_type));
}

void destroy_subscriber(GenericSubscriber* subscriber) {
  delete subscriber;
}

void spin_subscriber_once(GenericSubscriber* subscriber) {
  if (subscriber) {
    subscriber->spin_once();
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

}  // namespace ros1_plugin
