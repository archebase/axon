#include "../core/message_introspection.hpp"
#include <ros/message_traits.h>
#include <ros/serialization.h>
#include <ros/message.h>
#include <ros/types.h>
#include <string>
#include <sstream>

namespace lance_recorder {
namespace core {

/**
 * ROS 1 message introspection implementation
 */
class Ros1MessageIntrospector : public MessageIntrospector {
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
        
        // Parse definition to extract fields
        // This is complex and would require a full .msg parser
        // For now, return basic structure
        
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
        // Full field-level access requires message definition parsing
        // This provides message-level serialized data access
        const ros::Message* msg = static_cast<const ros::Message*>(message);
        if (msg) {
            size_bytes = msg->size();
            return msg->raw();
        }
        return nullptr;
    }
};

std::unique_ptr<MessageIntrospector> MessageIntrospectorFactory::create() {
    // Check ROS version
    const char* ros_version = std::getenv("ROS_VERSION");
    if (ros_version && std::string(ros_version) == "1") {
        return std::make_unique<Ros1MessageIntrospector>();
    }
    
    // ROS 2 implementation in ros2_introspection.cpp
    return nullptr;
}

} // namespace core
} // namespace lance_recorder

