#ifndef AXON_MESSAGE_FACTORY_HPP
#define AXON_MESSAGE_FACTORY_HPP

#if defined(AXON_ROS1)
#include <ros/message.h>
#include <ros/serialization.h>
#include <ros/message_traits.h>
#include <ros/service_traits.h>
#elif defined(AXON_ROS2)
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialized_message.hpp>
#endif

#include <string>
#include <memory>
#include <functional>
#include <unordered_map>

namespace axon {
namespace recorder {

#if defined(AXON_ROS1)

/**
 * Message factory for creating typed subscribers dynamically (ROS 1)
 */
class MessageFactory {
public:
    using MessageCreator = std::function<std::shared_ptr<ros::Message>()>;
    using MessageDeserializer = std::function<void(const uint8_t*, uint32_t, ros::Message&)>;
    
    struct MessageInfo {
        std::string md5sum;
        std::string datatype;
        std::string definition;
        MessageCreator creator;
        MessageDeserializer deserializer;
        size_t size;
    };
    
    /**
     * Register a message type
     */
    template<typename MessageType>
    static void register_message_type(const std::string& message_type);
    
    /**
     * Create a message instance from type name
     */
    static std::shared_ptr<ros::Message> create_message(const std::string& message_type);
    
    /**
     * Deserialize message from buffer
     */
    static bool deserialize_message(const std::string& message_type,
                                   const uint8_t* buffer,
                                   uint32_t size,
                                   ros::Message& msg);
    
    /**
     * Get message info
     */
    static bool get_message_info(const std::string& message_type, MessageInfo& info);
    
    /**
     * Check if message type is registered
     */
    static bool is_registered(const std::string& message_type);
    
private:
    static std::unordered_map<std::string, MessageInfo>& get_registry();
};

/**
 * Template specialization for message registration (ROS 1)
 */
template<typename MessageType>
void MessageFactory::register_message_type(const std::string& message_type) {
    auto& registry = get_registry();
    
    MessageInfo info;
    info.md5sum = ros::message_traits::MD5Sum<MessageType>::value();
    info.datatype = ros::message_traits::DataType<MessageType>::value();
    info.definition = ros::message_traits::Definition<MessageType>::value();
    info.size = ros::message_traits::Size<MessageType>::value();
    
    info.creator = []() -> std::shared_ptr<ros::Message> {
        return std::make_shared<MessageType>();
    };
    
    info.deserializer = [](const uint8_t* buffer, uint32_t size, ros::Message& msg) {
        MessageType& typed_msg = static_cast<MessageType&>(msg);
        ros::serialization::IStream stream(const_cast<uint8_t*>(buffer), size);
        ros::serialization::deserialize(stream, typed_msg);
    };
    
    registry[message_type] = info;
}

#elif defined(AXON_ROS2)

/**
 * Message factory for creating typed subscribers dynamically (ROS 2)
 */
class MessageFactory {
public:
    using MessageCreator = std::function<std::shared_ptr<void>()>;
    using MessageDeserializer = std::function<void(const rclcpp::SerializedMessage&, void*)>;
    
    struct MessageInfo {
        std::string type_name;
        MessageCreator creator;
        MessageDeserializer deserializer;
    };
    
    /**
     * Register a message type
     */
    template<typename MessageType>
    static void register_message_type(const std::string& message_type);
    
    /**
     * Create a message instance from type name
     */
    static std::shared_ptr<void> create_message(const std::string& message_type);
    
    /**
     * Deserialize message from serialized message
     */
    static bool deserialize_message(const std::string& message_type,
                                   const rclcpp::SerializedMessage& serialized,
                                   void* msg);
    
    /**
     * Get message info
     */
    static bool get_message_info(const std::string& message_type, MessageInfo& info);
    
    /**
     * Check if message type is registered
     */
    static bool is_registered(const std::string& message_type);
    
private:
    static std::unordered_map<std::string, MessageInfo>& get_registry();
};

/**
 * Template specialization for message registration (ROS 2)
 */
template<typename MessageType>
void MessageFactory::register_message_type(const std::string& message_type) {
    auto& registry = get_registry();
    
    MessageInfo info;
    info.type_name = message_type;
    
    info.creator = []() -> std::shared_ptr<void> {
        return std::make_shared<MessageType>();
    };
    
    info.deserializer = [](const rclcpp::SerializedMessage& serialized, void* msg) {
        // ROS 2 deserialization would use rmw_deserialize
        // This is a placeholder for the actual implementation
        (void)serialized;
        (void)msg;
    };
    
    registry[message_type] = info;
}

#else
#error "Either AXON_ROS1 or AXON_ROS2 must be defined"
#endif

} // namespace recorder
} // namespace axon

#endif // AXON_MESSAGE_FACTORY_HPP
