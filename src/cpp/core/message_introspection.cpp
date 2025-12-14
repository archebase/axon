#include "message_introspection.hpp"
#include <arrow/type.h>
#include <arrow/builder.h>
#include <sstream>
#include <algorithm>

namespace axon {
namespace core {

std::shared_ptr<arrow::DataType> MessageIntrospector::ros_type_to_arrow(
    const std::string& ros_type, bool is_array) {
    
    // Map ROS primitive types to Arrow types
    std::string base_type = ros_type;
    if (base_type.empty()) {
        return arrow::null();
    }
    
    // Remove array brackets if present
    size_t bracket_pos = base_type.find('[');
    if (bracket_pos != std::string::npos) {
        base_type = base_type.substr(0, bracket_pos);
        is_array = true;
    }
    
    // Handle ROS built-in types
    if (base_type == "bool" || base_type == "boolean") {
        return is_array ? arrow::list(arrow::boolean()) : arrow::boolean();
    } else if (base_type == "int8") {
        return is_array ? arrow::list(arrow::int8()) : arrow::int8();
    } else if (base_type == "uint8" || base_type == "byte") {
        return is_array ? arrow::list(arrow::uint8()) : arrow::uint8();
    } else if (base_type == "int16") {
        return is_array ? arrow::list(arrow::int16()) : arrow::int16();
    } else if (base_type == "uint16") {
        return is_array ? arrow::list(arrow::uint8()) : arrow::uint16();
    } else if (base_type == "int32") {
        return is_array ? arrow::list(arrow::int32()) : arrow::int32();
    } else if (base_type == "uint32") {
        return is_array ? arrow::list(arrow::uint32()) : arrow::uint32();
    } else if (base_type == "int64") {
        return is_array ? arrow::list(arrow::int64()) : arrow::int64();
    } else if (base_type == "uint64") {
        return is_array ? arrow::list(arrow::uint64()) : arrow::uint64();
    } else if (base_type == "float32" || base_type == "float") {
        return is_array ? arrow::list(arrow::float32()) : arrow::float32();
    } else if (base_type == "float64" || base_type == "double") {
        return is_array ? arrow::list(arrow::float64()) : arrow::float64();
    } else if (base_type == "string") {
        return is_array ? arrow::list(arrow::utf8()) : arrow::utf8();
    } else if (base_type == "time" || base_type == "duration") {
        // ROS time/duration as struct with sec/nsec
        std::vector<std::shared_ptr<arrow::Field>> time_fields = {
            arrow::field("sec", arrow::int32()),
            arrow::field("nsec", arrow::uint32())
        };
        auto time_type = std::make_shared<arrow::StructType>(time_fields);
        return is_array ? arrow::list(time_type) : time_type;
    }
    
    // For complex types (nested messages), serialize to binary
    // Full recursive struct building would require parsing .msg definitions
    // This is a design decision: complex types are serialized for flexibility
    return arrow::binary();
}

std::shared_ptr<arrow::Schema> MessageIntrospector::create_schema(
    const MessageTypeDescriptor& descriptor) {
    
    std::vector<std::shared_ptr<arrow::Field>> fields;
    fields.reserve(descriptor.fields.size());
    
    for (const auto& field_desc : descriptor.fields) {
        auto arrow_type = ros_type_to_arrow(field_desc.type_name, field_desc.is_array);
        fields.push_back(arrow::field(field_desc.name, arrow_type));
    }
    
    return std::make_shared<arrow::Schema>(fields);
}

std::unique_ptr<MessageIntrospector> MessageIntrospectorFactory::create() {
    // Platform-specific implementations are in ros1/ros2 directories
    // This function is implemented in ros1_introspection.cpp and ros2_introspection.cpp
    return nullptr;
}

} // namespace core
} // namespace axon

