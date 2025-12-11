#ifndef ROS1_MESSAGE_FACTORY_HPP
#define ROS1_MESSAGE_FACTORY_HPP

#include <ros/message.h>
#include <ros/serialization.h>
#include <ros/message_traits.h>
#include <ros/service_traits.h>
#include <string>
#include <memory>
#include <functional>
#include <unordered_map>

namespace lance_recorder {
namespace ros1 {

/**
 * Message factory for creating typed subscribers dynamically
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
 * Template specialization for message registration
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

} // namespace ros1
} // namespace lance_recorder

#endif // ROS1_MESSAGE_FACTORY_HPP

