#include "message_registry.hpp"
#include <iostream>

namespace axon {
namespace recorder {

static std::unordered_map<std::string, MessageRegistry::ConverterFactory>* g_registry = nullptr;

std::unordered_map<std::string, MessageRegistry::ConverterFactory>& MessageRegistry::get_registry() {
    if (!g_registry) {
        g_registry = new std::unordered_map<std::string, MessageRegistry::ConverterFactory>();
    }
    return *g_registry;
}

void MessageRegistry::register_message_type(const std::string& message_type,
                                            ConverterFactory factory) {
    auto& registry = get_registry();
    registry[message_type] = factory;
    
    // Also register with core factory
    core::MessageConverterFactory::register_converter(message_type, factory);
}

std::unique_ptr<MessageConverter> MessageRegistry::get_converter(const std::string& message_type) {
    auto& registry = get_registry();
    auto it = registry.find(message_type);
    if (it != registry.end()) {
        return it->second();
    }
    
    // Try to get from core factory
    return core::MessageConverterFactory::create(message_type);
}

bool MessageRegistry::has_converter(const std::string& message_type) {
    auto& registry = get_registry();
    if (registry.find(message_type) != registry.end()) {
        return true;
    }
    return core::MessageConverterFactory::has_converter(message_type);
}

std::vector<std::string> MessageRegistry::get_registered_types() {
    auto& registry = get_registry();
    std::vector<std::string> types;
    types.reserve(registry.size());
    for (const auto& pair : registry) {
        types.push_back(pair.first);
    }
    return types;
}

void MessageRegistry::initialize_default_converters() {
    // This will be populated by ROS-specific implementations
    // that register converters for sensor_msgs, std_msgs, etc.
    // For now, converters must be registered explicitly by the application
    std::cout << "MessageRegistry: Default converters should be registered by ROS-specific code" << std::endl;
}

} // namespace recorder
} // namespace axon
