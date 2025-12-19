#ifndef MESSAGE_INTROSPECTION_HPP
#define MESSAGE_INTROSPECTION_HPP

#include <arrow/api.h>

#include <functional>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

namespace axon {
namespace core {

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

  /**
   * Convert ROS type name to Arrow type
   */
  static std::shared_ptr<arrow::DataType> ros_type_to_arrow(
    const std::string& ros_type, bool is_array = false
  );

  /**
   * Create Arrow schema from message descriptor
   */
  static std::shared_ptr<arrow::Schema> create_schema(const MessageTypeDescriptor& descriptor);
};

/**
 * Factory for creating message introspectors
 * Allows platform-specific implementations (ROS1/ROS2) to register their factory
 */
class MessageIntrospectorFactory {
public:
  using FactoryFunction = std::function<std::unique_ptr<MessageIntrospector>()>;

  /**
   * Create a message introspector using the registered factory
   * Returns nullptr if no factory is registered
   */
  static std::unique_ptr<MessageIntrospector> create();

  /**
   * Register a factory function for creating introspectors
   * This should be called at startup by the platform-specific layer (e.g., ROS recorder)
   */
  static void set_factory(FactoryFunction factory);

  /**
   * Check if a factory has been registered
   */
  static bool has_factory();

private:
  static FactoryFunction& get_factory();
};

}  // namespace core
}  // namespace axon

#endif  // MESSAGE_INTROSPECTION_HPP
