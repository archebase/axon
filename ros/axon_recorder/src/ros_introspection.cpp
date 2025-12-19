#include "ros_introspection.hpp"

#include "message_introspection.hpp"

#if defined(AXON_ROS1)
#include <ros/message.h>
#include <ros/message_traits.h>
#include <ros/serialization.h>
#include <ros/types.h>
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

    // Parse package/name
    size_t slash_pos = message_type.find('/');
    if (slash_pos != std::string::npos) {
      descriptor.package = message_type.substr(0, slash_pos);
      descriptor.name = message_type.substr(slash_pos + 1);
    } else {
      descriptor.name = message_type;
    }

    // For ROS 1, we use message_traits and MD5Sum to get message metadata
    // Full field extraction would require:
    // 1. Loading message definition from ROS package
    // 2. Parsing .msg file format
    // 3. Extracting field names and types recursively
    // This is implemented via MessageFactory registration system instead

    // Try to get message definition
    std::string definition = ros::message_traits::Definition<ros::Message>::value();

    return true;
  }

  bool get_field_value(const void* message, const std::string& field_name, void* output) override {
    // ROS 1 doesn't have direct field access
    // Would need to use serialization or message introspection
    return false;
  }

  const void* get_field_pointer(
    const void* message, const std::string& field_name, size_t& size_bytes
  ) override {
    // For ROS 1, get serialized message data
    const ros::Message* msg = static_cast<const ros::Message*>(message);
    if (msg) {
      size_bytes = msg->size();
      return msg->raw();
    }
    return nullptr;
  }
};

#elif defined(AXON_ROS2)
// ============================================================================
// ROS 2 Message Introspection Implementation
// ============================================================================

// Include rosidl introspection headers for runtime type information
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

namespace {

// Helper to create a field descriptor
core::FieldDescriptor make_field(
  const std::string& name, const std::string& type, bool is_array = false, bool is_builtin = true
) {
  core::FieldDescriptor fd;
  fd.name = name;
  fd.type_name = type;
  fd.is_array = is_array;
  fd.is_fixed_size = false;
  fd.array_size = 0;
  fd.is_builtin = is_builtin;
  return fd;
}

// Convert rosidl type ID to our type name
std::string rosidl_type_to_string(uint8_t type_id) {
  using namespace rosidl_typesupport_introspection_cpp;
  switch (type_id) {
    case ROS_TYPE_BOOL:
      return "bool";
    case ROS_TYPE_BYTE:
      return "byte";
    case ROS_TYPE_CHAR:
      return "char";
    case ROS_TYPE_FLOAT:
      return "float32";
    case ROS_TYPE_DOUBLE:
      return "float64";
    case ROS_TYPE_INT8:
      return "int8";
    case ROS_TYPE_INT16:
      return "int16";
    case ROS_TYPE_INT32:
      return "int32";
    case ROS_TYPE_INT64:
      return "int64";
    case ROS_TYPE_UINT8:
      return "uint8";
    case ROS_TYPE_UINT16:
      return "uint16";
    case ROS_TYPE_UINT32:
      return "uint32";
    case ROS_TYPE_UINT64:
      return "uint64";
    case ROS_TYPE_STRING:
      return "string";
    case ROS_TYPE_WSTRING:
      return "wstring";
    case ROS_TYPE_MESSAGE:
      return "message";
    default:
      return "binary";
  }
}

}  // anonymous namespace

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
    // This minimizes conversion overhead and preserves all message data
    // Field extraction can be done on the read side when needed

    core::FieldDescriptor data_field;
    data_field.name = "message_data";
    data_field.type_name = "binary";
    data_field.is_array = false;
    data_field.is_builtin = false;  // Treated as binary blob
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
