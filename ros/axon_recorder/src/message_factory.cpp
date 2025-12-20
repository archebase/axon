#include "message_factory.hpp"

#if defined(AXON_ROS1)
#include <ros/message.h>
#include <ros/serialization.h>
#endif

#include <iostream>

namespace axon {
namespace recorder {

#if defined(AXON_ROS1)
// ============================================================================
// ROS 1 Message Factory Implementation
// ============================================================================

static std::unordered_map<std::string, MessageFactory::MessageInfo>* g_registry = nullptr;

std::unordered_map<std::string, MessageFactory::MessageInfo>& MessageFactory::get_registry() {
  if (!g_registry) {
    g_registry = new std::unordered_map<std::string, MessageFactory::MessageInfo>();
  }
  return *g_registry;
}

std::shared_ptr<void> MessageFactory::create_message(const std::string& message_type) {
  auto& registry = get_registry();
  auto it = registry.find(message_type);
  if (it != registry.end() && it->second.creator) {
    return it->second.creator();
  }
  return nullptr;
}

bool MessageFactory::deserialize_message(
  const std::string& message_type, const uint8_t* buffer, uint32_t size, void* msg
) {
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

#elif defined(AXON_ROS2)
// ============================================================================
// ROS 2 Message Factory Implementation
// ============================================================================

static std::unordered_map<std::string, MessageFactory::MessageInfo>* g_registry = nullptr;

std::unordered_map<std::string, MessageFactory::MessageInfo>& MessageFactory::get_registry() {
  if (!g_registry) {
    g_registry = new std::unordered_map<std::string, MessageFactory::MessageInfo>();
  }
  return *g_registry;
}

std::shared_ptr<void> MessageFactory::create_message(const std::string& message_type) {
  auto& registry = get_registry();
  auto it = registry.find(message_type);
  if (it != registry.end() && it->second.creator) {
    return it->second.creator();
  }
  return nullptr;
}

bool MessageFactory::deserialize_message(
  const std::string& message_type, const rclcpp::SerializedMessage& serialized, void* msg
) {
  auto& registry = get_registry();
  auto it = registry.find(message_type);
  if (it != registry.end() && it->second.deserializer) {
    it->second.deserializer(serialized, msg);
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

#endif

}  // namespace recorder
}  // namespace axon
