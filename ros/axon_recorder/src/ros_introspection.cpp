#include "ros_introspection.hpp"
#include "message_introspection.hpp"

#if defined(AXON_ROS1)
#include <ros/message_traits.h>
#include <ros/serialization.h>
#include <ros/message.h>
#include <ros/types.h>
#elif defined(AXON_ROS2)
#include <rclcpp/rclcpp.hpp>
#include <rosidl_typesupport_cpp/message_type_support.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <rosidl_runtime_cpp/bounded_vector.hpp>
#endif

#include <string>
#include <memory>
#include <cstdlib>

namespace axon {
namespace recorder {

#if defined(AXON_ROS1)
// ============================================================================
// ROS 1 Message Introspection Implementation
// ============================================================================

class RosIntrospectorImpl : public MessageIntrospector {
public:
    bool get_message_descriptor(const std::string& message_type,
                                MessageTypeDescriptor& descriptor) override {
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
    
    bool get_field_value(const void* message,
                        const std::string& field_name,
                        void* output) override {
        // ROS 1 doesn't have direct field access
        // Would need to use serialization or message introspection
        return false;
    }
    
    const void* get_field_pointer(const void* message,
                                 const std::string& field_name,
                                 size_t& size_bytes) override {
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

class RosIntrospectorImpl : public MessageIntrospector {
public:
    bool get_message_descriptor(const std::string& message_type,
                                MessageTypeDescriptor& descriptor) override {
        descriptor.full_name = message_type;
        
        // Parse package/name
        size_t slash_pos = message_type.find('/');
        if (slash_pos != std::string::npos) {
            descriptor.package = message_type.substr(0, slash_pos);
            descriptor.name = message_type.substr(slash_pos + 1);
        } else {
            descriptor.name = message_type;
        }
        
        // For ROS 2, we can use rosidl_get_message_type_support
        // This requires the message type to be available at runtime
        // Full field extraction would require:
        // 1. Loading message type support dynamically
        // 2. Using rosidl_typesupport to get field information
        // 3. Parsing field names and types from type support
        
        return true;
    }
    
    bool get_field_value(const void* message,
                        const std::string& field_name,
                        void* output) override {
        // ROS 2 has better introspection support
        // Would use rosidl_runtime_cpp::MessageMembers
        return false;
    }
    
    const void* get_field_pointer(const void* message,
                                 const std::string& field_name,
                                 size_t& size_bytes) override {
        // For ROS 2, can use serialized message
        const rclcpp::SerializedMessage* msg = 
            static_cast<const rclcpp::SerializedMessage*>(message);
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

} // namespace recorder
} // namespace axon
