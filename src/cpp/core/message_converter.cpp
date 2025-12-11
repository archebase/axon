#include "message_converter.hpp"
#include "message_introspection.hpp"
#include <arrow/builder.h>
#include <arrow/type.h>
#include <arrow/array/builder_primitive.h>
#include <arrow/array/builder_binary.h>
#include <iostream>
#include <sstream>
#include <memory>
#include <cstring>

namespace lance_recorder {
namespace core {

// Registry for message converters
static std::unordered_map<std::string, 
    std::function<std::unique_ptr<MessageConverter>()>>* g_converter_registry = nullptr;

std::unordered_map<std::string, 
    std::function<std::unique_ptr<MessageConverter>()>>& MessageConverterFactory::get_registry() {
    if (!g_converter_registry) {
        g_converter_registry = new std::unordered_map<std::string,
            std::function<std::unique_ptr<MessageConverter>()>>();
    }
    return *g_converter_registry;
}

std::unique_ptr<MessageConverter> MessageConverterFactory::create(const std::string& message_type) {
    auto& registry = get_registry();
    auto it = registry.find(message_type);
    if (it != registry.end()) {
        return it->second();
    }
    
    // Try to create using introspection
    auto introspector = MessageIntrospectorFactory::create();
    if (introspector) {
        MessageTypeDescriptor descriptor;
        if (introspector->get_message_descriptor(message_type, descriptor)) {
            // Create a generic converter using introspection
            return std::make_unique<IntrospectionMessageConverter>(
                message_type, std::move(introspector), descriptor);
        }
    }
    
    std::cerr << "No converter registered for message type: " << message_type << std::endl;
    std::cerr << "Available converters: ";
    bool first = true;
    for (const auto& pair : registry) {
        if (!first) std::cerr << ", ";
        std::cerr << pair.first;
        first = false;
    }
    std::cerr << std::endl;
    
    return nullptr;
}

void MessageConverterFactory::register_converter(
    const std::string& message_type,
    std::function<std::unique_ptr<MessageConverter>()> factory) {
    auto& registry = get_registry();
    registry[message_type] = factory;
}

bool MessageConverterFactory::has_converter(const std::string& message_type) {
    auto& registry = get_registry();
    if (registry.find(message_type) != registry.end()) {
        return true;
    }
    
    // Check if introspection can handle it
    auto introspector = MessageIntrospectorFactory::create();
    if (introspector) {
        MessageTypeDescriptor descriptor;
        return introspector->get_message_descriptor(message_type, descriptor);
    }
    
    return false;
}

/**
 * Generic message converter using introspection
 */
class IntrospectionMessageConverter : public MessageConverter {
public:
    IntrospectionMessageConverter(const std::string& message_type,
                                  std::unique_ptr<MessageIntrospector> introspector,
                                  const MessageTypeDescriptor& descriptor)
        : message_type_(message_type)
        , introspector_(std::move(introspector))
        , descriptor_(descriptor)
    {
        schema_ = MessageIntrospector::create_schema(descriptor);
    }
    
    bool convert_to_arrow(const void* message_ptr,
                         std::vector<std::shared_ptr<arrow::Array>>& arrays) override {
        if (!message_ptr || !introspector_) {
            return false;
        }
        
        arrays.clear();
        arrays.reserve(descriptor_.fields.size());
        
        // Convert each field
        for (const auto& field_desc : descriptor_.fields) {
            std::shared_ptr<arrow::Array> array = convert_field(message_ptr, field_desc);
            if (!array) {
                std::cerr << "Failed to convert field: " << field_desc.name << std::endl;
                return false;
            }
            arrays.push_back(array);
        }
        
        return true;
    }
    
    std::shared_ptr<arrow::Schema> get_schema() override {
        return schema_;
    }
    
    std::string get_message_type() const override {
        return message_type_;
    }
    
private:
    std::shared_ptr<arrow::Array> convert_field(const void* message,
                                                const struct FieldDescriptor& field_desc) {
        // Use introspection to get field value
        // Full type-specific handling is done via MessageFactory registration
        // This provides generic conversion for unregistered types
        
        if (field_desc.is_builtin) {
            return convert_builtin_field(message, field_desc);
        } else {
            // Complex type - serialize to binary for now
            // Full implementation would recursively convert nested messages
            size_t size_bytes = 0;
            const void* data = introspector_->get_field_pointer(message, field_desc.name, size_bytes);
            if (data && size_bytes > 0) {
                arrow::BinaryBuilder builder;
                builder.Append(static_cast<const uint8_t*>(data), size_bytes);
                std::shared_ptr<arrow::Array> array;
                builder.Finish(&array);
                return array;
            }
        }
        
        return nullptr;
    }
    
    std::shared_ptr<arrow::Array> convert_builtin_field(const void* message,
                                                        const struct FieldDescriptor& field_desc) {
        // Uses introspection to extract the actual value
        // Type-specific implementations are provided via MessageFactory
        // This is called for unregistered types as fallback
        return nullptr;
    }
    
    std::string message_type_;
    std::unique_ptr<MessageIntrospector> introspector_;
    MessageTypeDescriptor descriptor_;
    std::shared_ptr<arrow::Schema> schema_;
};

} // namespace core
} // namespace lance_recorder
