#include "message_factory.hpp"
#include <ros/message.h>
#include <ros/serialization.h>
#include <iostream>

namespace lance_recorder {
namespace ros1 {

static std::unordered_map<std::string, MessageFactory::MessageInfo>* g_registry = nullptr;

std::unordered_map<std::string, MessageFactory::MessageInfo>& MessageFactory::get_registry() {
    if (!g_registry) {
        g_registry = new std::unordered_map<std::string, MessageFactory::MessageInfo>();
    }
    return *g_registry;
}

std::shared_ptr<ros::Message> MessageFactory::create_message(const std::string& message_type) {
    auto& registry = get_registry();
    auto it = registry.find(message_type);
    if (it != registry.end() && it->second.creator) {
        return it->second.creator();
    }
    return nullptr;
}

bool MessageFactory::deserialize_message(const std::string& message_type,
                                        const uint8_t* buffer,
                                        uint32_t size,
                                        ros::Message& msg) {
    auto& registry = get_registry();
    auto it = registry.find(message_type);
    if (it != registry.end() && it->second.deserializer) {
        it->second.deserializer(buffer, size, msg);
        return true;
    }
    return false;
}

bool MessageFactory::get_message_info(const std::string& message_type, MessageInfo& info) {
    auto& registry = get_registry();
    auto it = registry.find(message_type);
    if (it != registry.end()) {
        info = it->second;
        return true;
    }
    return false;
}

bool MessageFactory::is_registered(const std::string& message_type) {
    auto& registry = get_registry();
    return registry.find(message_type) != registry.end();
}

} // namespace ros1
} // namespace lance_recorder

