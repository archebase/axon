#ifndef AXON_ROS_INTROSPECTION_HPP
#define AXON_ROS_INTROSPECTION_HPP

#include <memory>
#include <string>
#include <vector>

#include "message_introspection.hpp"

namespace axon {
namespace recorder {

// Re-export core introspection types
using MessageIntrospector = core::MessageIntrospector;
using MessageTypeDescriptor = core::MessageTypeDescriptor;
using FieldDescriptor = core::FieldDescriptor;

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
