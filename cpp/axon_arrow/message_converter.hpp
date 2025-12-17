#ifndef MESSAGE_CONVERTER_HPP
#define MESSAGE_CONVERTER_HPP

#include <arrow/api.h>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// Include for full type definition (not just forward declaration)
#include "message_introspection.hpp"

namespace axon {
namespace core {

/**
 * Abstract interface for converting ROS messages to Arrow arrays
 */
class MessageConverter {
public:
    virtual ~MessageConverter() = default;
    
    /**
     * Convert a ROS message to Arrow arrays
     * @param message_ptr Pointer to ROS message (type-erased)
     * @param arrays Output vector of Arrow arrays (one per field)
     * @return true on success
     */
    virtual bool convert_to_arrow(const void* message_ptr, 
                                   std::vector<std::shared_ptr<arrow::Array>>& arrays) = 0;
    
    /**
     * Get the Arrow schema for this message type
     */
    virtual std::shared_ptr<arrow::Schema> get_schema() = 0;
    
    /**
     * Get the ROS message type name
     */
    virtual std::string get_message_type() const = 0;
};

/**
 * Factory for creating message converters
 */
class MessageConverterFactory {
public:
    static std::unique_ptr<MessageConverter> create(const std::string& message_type);
    
    /**
     * Register a custom converter for a message type
     */
    static void register_converter(const std::string& message_type,
                                   std::function<std::unique_ptr<MessageConverter>()> factory);
    
    /**
     * Check if a converter exists for a message type
     */
    static bool has_converter(const std::string& message_type);
    
private:
    static std::unordered_map<std::string, 
        std::function<std::unique_ptr<MessageConverter>()>>& get_registry();
};

/**
 * Generic message converter using ROS message introspection
 * This dynamically converts any ROS message type using introspection
 */
class IntrospectionMessageConverter : public MessageConverter {
public:
    IntrospectionMessageConverter(const std::string& message_type,
                                  std::unique_ptr<MessageIntrospector> introspector,
                                  const MessageTypeDescriptor& descriptor);
    
    bool convert_to_arrow(const void* message_ptr,
                         std::vector<std::shared_ptr<arrow::Array>>& arrays) override;
    
    std::shared_ptr<arrow::Schema> get_schema() override;
    std::string get_message_type() const override;
    
private:
    std::shared_ptr<arrow::Array> convert_field(const void* message,
                                                const struct FieldDescriptor& field_desc);
    std::shared_ptr<arrow::Array> convert_builtin_field(const void* message,
                                                        const struct FieldDescriptor& field_desc);
    
    std::string message_type_;
    std::unique_ptr<MessageIntrospector> introspector_;
    MessageTypeDescriptor descriptor_;
    std::shared_ptr<arrow::Schema> schema_;
};

// TypedMessageConverter is only available for ROS1 due to ros::message_traits dependency
// For ROS2, use IntrospectionMessageConverter or implement platform-specific converters
#if defined(ROS_VERSION) && ROS_VERSION == 1
#include <ros/message_traits.h>

/**
 * Template-based converter for specific message types (for optimization)
 * This provides compile-time type safety and better performance
 * Note: Only available for ROS1
 */
template<typename MessageType>
class TypedMessageConverter : public MessageConverter {
public:
    TypedMessageConverter() {
        message_type_name_ = ros::message_traits::DataType<MessageType>::value();
        // Build schema from message type
        build_schema();
    }
    
    bool convert_to_arrow(const void* message_ptr,
                          std::vector<std::shared_ptr<arrow::Array>>& arrays) override {
        if (!message_ptr) {
            return false;
        }
        const MessageType* msg = static_cast<const MessageType*>(message_ptr);
        return convert_impl(*msg, arrays);
    }
    
    std::shared_ptr<arrow::Schema> get_schema() override {
        return schema_;
    }
    
    std::string get_message_type() const override {
        return message_type_name_;
    }
    
private:
    void build_schema() {
        // Build schema from message type using traits
        // This is message-type specific
    }
    
    bool convert_impl(const MessageType& msg,
                     std::vector<std::shared_ptr<arrow::Array>>& arrays) {
        // Type-specific conversion
        // This would be specialized for each message type
        return false;
    }
    
    std::shared_ptr<arrow::Schema> schema_;
    std::string message_type_name_;
};
#endif // ROS_VERSION == 1

} // namespace core
} // namespace axon

#endif // MESSAGE_CONVERTER_HPP

