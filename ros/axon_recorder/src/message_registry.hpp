#ifndef AXON_MESSAGE_REGISTRY_HPP
#define AXON_MESSAGE_REGISTRY_HPP

#include <string>
#include <unordered_map>
#include <memory>
#include <functional>
#include <vector>
#include <arrow/api.h>
#include "../../cpp/src/core/message_converter.hpp"

namespace axon {
namespace recorder {

// Use core::MessageConverter
using MessageConverter = core::MessageConverter;

/**
 * Registry for ROS message type handlers
 * Maps message types to conversion functions
 */
class MessageRegistry {
public:
    using ConverterFactory = std::function<std::unique_ptr<MessageConverter>()>;
    
    /**
     * Register a message type converter
     */
    static void register_message_type(const std::string& message_type,
                                     ConverterFactory factory);
    
    /**
     * Get converter for a message type
     */
    static std::unique_ptr<MessageConverter> get_converter(const std::string& message_type);
    
    /**
     * Check if a message type is registered
     */
    static bool has_converter(const std::string& message_type);
    
    /**
     * Get all registered message types
     */
    static std::vector<std::string> get_registered_types();
    
    /**
     * Initialize default converters for common ROS message types
     */
    static void initialize_default_converters();
    
private:
    static std::unordered_map<std::string, ConverterFactory>& get_registry();
};

} // namespace recorder
} // namespace axon

#endif // AXON_MESSAGE_REGISTRY_HPP
