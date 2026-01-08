#ifndef ROS1_PLUGIN_SUBSCRIBER_HPP_
#define ROS1_PLUGIN_SUBSCRIBER_HPP_

#include <ros/ros.h>

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace ros1_plugin {

// Callback data structure for raw message data
struct MessageData {
  const uint8_t* data;       // Pointer to serialized message data
  size_t size;               // Size of serialized data in bytes
  uint64_t timestamp;        // Message timestamp (nanoseconds)
  std::string topic_name;    // Topic name
  std::string message_type;  // Message type (e.g., "std_msgs/String")

  MessageData()
      : data(nullptr)
      , size(0)
      , timestamp(0) {}

  MessageData(const uint8_t* d, size_t s, uint64_t ts, const std::string& tn, const std::string& mt)
      : data(d)
      , size(s)
      , timestamp(ts)
      , topic_name(tn)
      , message_type(mt) {}
};

// Callback type for data recording (e.g., to MCAP)
using DataCallback = std::function<void(const MessageData&)>;

// Generic subscriber class that can subscribe to ANY topic type
class GenericSubscriber {
public:
  // Initialize ROS1 (must be called before creating subscribers)
  static void init_ros(int argc = 0, char** argv = nullptr);

  // Shutdown ROS1
  static void shutdown_ros();

  // Constructor - subscribes to any message type
  GenericSubscriber(const std::string& topic_name, const std::string& message_type);

  ~GenericSubscriber();

  void spin_once();
  std::string get_topic_name() const {
    return topic_name_;
  }
  std::string get_message_type() const {
    return message_type_str_;
  }

  // Set callback for receiving raw message data (for MCAP recording)
  void set_data_callback(DataCallback callback) {
    data_callback_ = callback;
  }

private:
  void setup_generic_subscription(const std::string& topic_name, const std::string& message_type);

  std::string topic_name_;
  std::string message_type_str_;

  // Node handle for ROS1
  std::unique_ptr<ros::NodeHandle> node_handle_;

  // Store subscription as generic pointer
  std::shared_ptr<void> subscriber_;

  // Callback for raw data access (for MCAP recording)
  DataCallback data_callback_;
};

// Factory functions for dynamic loading
extern "C" {
// Initialize ROS1 (must be called before create_subscriber)
void init_ros1();

// Shutdown ROS1
void shutdown_ros1();

// Create subscriber with message type string (supports any ROS1 message type)
GenericSubscriber* create_subscriber(const char* topic_name, const char* message_type);

// Destroy subscriber
void destroy_subscriber(GenericSubscriber* subscriber);

// Spin the subscriber once (non-blocking)
void spin_subscriber_once(GenericSubscriber* subscriber);

// Get topic name
const char* get_topic_name(GenericSubscriber* subscriber);

// Get message type string
const char* get_message_type(GenericSubscriber* subscriber);

// Set data callback for MCAP recording
// Callback signature: void (*)(const uint8_t* data, size_t size, uint64_t timestamp, const char*
// topic, const char* msg_type)
typedef void (*RawDataCallback)(const uint8_t*, size_t, uint64_t, const char*, const char*);
void set_data_callback(GenericSubscriber* subscriber, RawDataCallback callback);
}

// Helper function to set data callback from C++ code
inline void set_data_callback_cpp(GenericSubscriber* subscriber, DataCallback callback) {
  if (subscriber) {
    subscriber->set_data_callback(callback);
  }
}

}  // namespace ros1_plugin

#endif  // ROS1_PLUGIN_SUBSCRIBER_HPP_
