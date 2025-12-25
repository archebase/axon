#ifndef AXON_ROS_INTROSPECTION_HPP
#define AXON_ROS_INTROSPECTION_HPP

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace axon {
namespace recorder {

/**
 * Field descriptor for ROS message introspection
 */
struct FieldDescriptor {
  std::string name;
  std::string type_name;
  bool is_array;
  bool is_fixed_size;
  size_t array_size;  // For fixed-size arrays
  bool is_builtin;

  FieldDescriptor()
      : is_array(false)
      , is_fixed_size(false)
      , array_size(0)
      , is_builtin(false) {}
};

/**
 * Message type descriptor
 */
struct MessageTypeDescriptor {
  std::string full_name;  // e.g., "sensor_msgs/Image"
  std::string package;    // e.g., "sensor_msgs"
  std::string name;       // e.g., "Image"
  std::vector<FieldDescriptor> fields;
  size_t size_bytes;  // Size of message in memory

  MessageTypeDescriptor()
      : size_bytes(0) {}
};

/**
 * Abstract message introspection interface
 * Platform-specific implementations for ROS1/ROS2
 */
class MessageIntrospector {
public:
  virtual ~MessageIntrospector() = default;

  /**
   * Get message type descriptor from message type name
   */
  virtual bool get_message_descriptor(
    const std::string& message_type, MessageTypeDescriptor& descriptor
  ) = 0;

  /**
   * Get field value from message by name
   */
  virtual bool get_field_value(
    const void* message, const std::string& field_name, void* output
  ) = 0;

  /**
   * Get field pointer (for zero-copy access)
   */
  virtual const void* get_field_pointer(
    const void* message, const std::string& field_name, size_t& size_bytes
  ) = 0;
};

/**
 * Factory for creating message introspectors
 */
class MessageIntrospectorFactory {
public:
  using FactoryFunction = std::function<std::unique_ptr<MessageIntrospector>()>;

  static std::unique_ptr<MessageIntrospector> create();
  static void set_factory(FactoryFunction factory);
  static bool has_factory();

private:
  static FactoryFunction& get_factory();
};

/**
 * Factory for creating ROS message introspectors
 * Returns the appropriate implementation based on compile-time ROS version
 */
class RosIntrospectorFactory {
public:
  static std::unique_ptr<MessageIntrospector> create();
};

}  // namespace recorder
}  // namespace axon

#endif  // AXON_ROS_INTROSPECTION_HPP
