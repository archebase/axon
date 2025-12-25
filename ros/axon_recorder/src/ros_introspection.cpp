#include "ros_introspection.hpp"

#if defined(AXON_ROS1)
#include <ros/ros.h>
#elif defined(AXON_ROS2)
#include <rclcpp/rclcpp.hpp>
#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <rosidl_typesupport_cpp/message_type_support.hpp>
#endif

#include <cstdlib>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace axon {
namespace recorder {

// ============================================================================
// MessageIntrospectorFactory Implementation
// ============================================================================

static MessageIntrospectorFactory::FactoryFunction g_factory = nullptr;

MessageIntrospectorFactory::FactoryFunction& MessageIntrospectorFactory::get_factory() {
  return g_factory;
}

std::unique_ptr<MessageIntrospector> MessageIntrospectorFactory::create() {
  if (g_factory) {
    return g_factory();
  }
  return nullptr;
}

void MessageIntrospectorFactory::set_factory(FactoryFunction factory) {
  g_factory = std::move(factory);
}

bool MessageIntrospectorFactory::has_factory() {
  return g_factory != nullptr;
}

#if defined(AXON_ROS1)
// ============================================================================
// ROS 1 Message Introspection Implementation
// ============================================================================

class RosIntrospectorImpl : public MessageIntrospector {
public:
  bool get_message_descriptor(
    const std::string& message_type, MessageTypeDescriptor& descriptor
  ) override {
    descriptor.full_name = message_type;

    // Parse package/name - handle both ROS1 (pkg/Msg) and ROS2 (pkg/msg/Msg) styles
    size_t slash_pos = message_type.find('/');
    if (slash_pos != std::string::npos) {
      descriptor.package = message_type.substr(0, slash_pos);
      std::string remainder = message_type.substr(slash_pos + 1);
      // Check for ROS2 style (has another slash)
      size_t second_slash = remainder.find('/');
      if (second_slash != std::string::npos) {
        descriptor.name = remainder.substr(second_slash + 1);
      } else {
        descriptor.name = remainder;
      }
    } else {
      descriptor.name = message_type;
    }

    return true;
  }

  bool get_field_value(const void* message, const std::string& field_name, void* output) override {
    // ROS 1 doesn't have direct field access
    return false;
  }

  const void* get_field_pointer(
    const void* /* message */, const std::string& /* field_name */, size_t& size_bytes
  ) override {
    // ROS 1 doesn't have a common base class for messages with raw data access.
    // Use ShapeShifter for serialized data access instead.
    size_bytes = 0;
    return nullptr;
  }
};

#elif defined(AXON_ROS2)
// ============================================================================
// ROS 2 Message Introspection Implementation
// ============================================================================

class RosIntrospectorImpl : public MessageIntrospector {
public:
  bool get_message_descriptor(
    const std::string& message_type, MessageTypeDescriptor& descriptor
  ) override {
    descriptor.full_name = message_type;

    // Parse package/name - handle both ROS1 (pkg/Msg) and ROS2 (pkg/msg/Msg) styles
    size_t slash_pos = message_type.find('/');
    if (slash_pos != std::string::npos) {
      descriptor.package = message_type.substr(0, slash_pos);
      std::string remainder = message_type.substr(slash_pos + 1);
      // Check for ROS2 style (has another slash)
      size_t second_slash = remainder.find('/');
      if (second_slash != std::string::npos) {
        descriptor.name = remainder.substr(second_slash + 1);
      } else {
        descriptor.name = remainder;
      }
    } else {
      descriptor.name = message_type;
    }

    // For high-performance recording, we use a simplified schema:
    // Store the entire serialized message as binary data
    FieldDescriptor data_field;
    data_field.name = "message_data";
    data_field.type_name = "binary";
    data_field.is_array = false;
    data_field.is_builtin = false;
    descriptor.fields.push_back(data_field);

    return true;
  }

  bool get_field_value(const void* message, const std::string& field_name, void* output) override {
    // Not used - we store serialized data directly
    return false;
  }

  const void* get_field_pointer(
    const void* message, const std::string& field_name, size_t& size_bytes
  ) override {
    // Return the serialized message data
    if (!message) {
      return nullptr;
    }

    const rclcpp::SerializedMessage* msg = static_cast<const rclcpp::SerializedMessage*>(message);
    if (msg) {
      size_bytes = msg->size();
      return msg->get_rcl_serialized_message().buffer;
    }
    return nullptr;
  }
};

#else
#error "Either AXON_ROS1 or AXON_ROS2 must be defined"
#endif

// ============================================================================
// Factory Implementation
// ============================================================================

std::unique_ptr<MessageIntrospector> RosIntrospectorFactory::create() {
  return std::make_unique<RosIntrospectorImpl>();
}

}  // namespace recorder
}  // namespace axon
