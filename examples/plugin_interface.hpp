#ifndef PLUGIN_INTERFACE_HPP_
#define PLUGIN_INTERFACE_HPP_

#include <cstdint>
#include <functional>

// Unified plugin interface for both ROS1 and ROS2
// This header has NO dependencies on ROS1 or ROS2 libraries

namespace plugin_interface {

// Raw data callback structure
struct MessageData {
  const uint8_t* data;       // Pointer to serialized message data
  size_t size;               // Size of serialized data in bytes
  uint64_t timestamp;        // Message timestamp (nanoseconds)
  const char* topic_name;    // Topic name
  const char* message_type;  // Message type
};

// C-style callback function pointer type
// Used for both ROS1 and ROS2 plugins
typedef void (*RawDataCallback
)(const uint8_t* data, size_t size, uint64_t timestamp, const char* topic, const char* msg_type);

// Factory function pointer types for dynamic loading
typedef void (*InitContextFunc)();
typedef void (*ShutdownContextFunc)();
typedef void* (*CreateSubscriberFunc)(const char* topic, const char* type);
typedef void (*DestroySubscriberFunc)(void* subscriber);
typedef void (*SpinSubscriberFunc)(void* subscriber);
typedef const char* (*GetTopicNameFunc)(void* subscriber);
typedef const char* (*GetMessageTypeFunc)(void* subscriber);
typedef void (*SetDataCallbackFunc)(void* subscriber, RawDataCallback callback);

// Plugin type enumeration
enum class PluginType { ROS1, ROS2 };

// Plugin information structure
struct PluginInfo {
  PluginType type;
  const char* name;
  const char* library_name;
  const char* init_func_name;
  const char* shutdown_func_name;
  const char* spin_func_name;
};

// Get plugin information for ROS1
inline PluginInfo get_ros1_plugin_info() {
  return {
    PluginType::ROS1,
    "ROS1",
    "libros1_plugin.so",
    "init_ros1",
    "shutdown_ros1",
    "spin_subscriber_once"};
}

// Get plugin information for ROS2
inline PluginInfo get_ros2_plugin_info() {
  return {
    PluginType::ROS2,
    "ROS2",
    "libros2_plugin.so",
    "init_ros2_context",
    "shutdown_ros2_context",
    "spin_subscriber"};
}

}  // namespace plugin_interface

#endif  // PLUGIN_INTERFACE_HPP_
