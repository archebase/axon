#include "message_converter.hpp"

#include <arrow/array/builder_binary.h>
#include <arrow/array/builder_nested.h>
#include <arrow/array/builder_primitive.h>
#include <arrow/builder.h>
#include <arrow/type.h>

#include <cstring>
#include <iostream>
#include <memory>
#include <sstream>

#include "message_introspection.hpp"

namespace axon {
namespace core {

// Registry for message converters
static std::unordered_map<std::string, std::function<std::unique_ptr<MessageConverter>()>>*
  g_converter_registry = nullptr;

std::unordered_map<std::string, std::function<std::unique_ptr<MessageConverter>()>>&
MessageConverterFactory::get_registry() {
  if (!g_converter_registry) {
    g_converter_registry =
      new std::unordered_map<std::string, std::function<std::unique_ptr<MessageConverter>()>>();
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
        message_type, std::move(introspector), descriptor
      );
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
  const std::string& message_type, std::function<std::unique_ptr<MessageConverter>()> factory
) {
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

// IntrospectionMessageConverter implementation

IntrospectionMessageConverter::IntrospectionMessageConverter(
  const std::string& message_type, std::unique_ptr<MessageIntrospector> introspector,
  const MessageTypeDescriptor& descriptor
)
    : message_type_(message_type)
    , introspector_(std::move(introspector))
    , descriptor_(descriptor) {
  schema_ = MessageIntrospector::create_schema(descriptor);
}

bool IntrospectionMessageConverter::convert_to_arrow(
  const void* message_ptr, std::vector<std::shared_ptr<arrow::Array>>& arrays
) {
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

std::shared_ptr<arrow::Schema> IntrospectionMessageConverter::get_schema() {
  return schema_;
}

std::string IntrospectionMessageConverter::get_message_type() const {
  return message_type_;
}

std::shared_ptr<arrow::Array> IntrospectionMessageConverter::convert_field(
  const void* message, const struct FieldDescriptor& field_desc
) {
  // Use introspection to get field value
  // Full type-specific handling is done via MessageFactory registration
  // This provides generic conversion for unregistered types

  // Handle array types
  if (field_desc.is_array) {
    // For array types, create an empty list array
    // Real implementation would extract actual array data
    auto value_type = MessageIntrospector::ros_type_to_arrow(field_desc.type_name, false);
    auto list_builder = std::make_shared<arrow::ListBuilder>(
      arrow::default_memory_pool(), arrow::MakeBuilder(value_type).ValueOrDie()
    );

    // Append a null list entry
    auto status = list_builder->AppendNull();
    if (!status.ok()) {
      return nullptr;
    }

    std::shared_ptr<arrow::Array> array;
    status = list_builder->Finish(&array);
    if (!status.ok()) {
      return nullptr;
    }
    return array;
  }

  if (field_desc.is_builtin) {
    return convert_builtin_field(message, field_desc);
  } else {
    // Complex type - serialize to binary for now
    // Full implementation would recursively convert nested messages
    size_t size_bytes = 0;
    const void* data = introspector_->get_field_pointer(message, field_desc.name, size_bytes);
    if (data && size_bytes > 0) {
      arrow::BinaryBuilder builder;
      auto status = builder.Append(static_cast<const uint8_t*>(data), size_bytes);
      if (!status.ok()) {
        return nullptr;
      }
      std::shared_ptr<arrow::Array> array;
      status = builder.Finish(&array);
      if (!status.ok()) {
        return nullptr;
      }
      return array;
    }

    // Fallback: create null binary
    arrow::BinaryBuilder builder;
    auto status = builder.AppendNull();
    if (!status.ok()) {
      return nullptr;
    }
    std::shared_ptr<arrow::Array> array;
    status = builder.Finish(&array);
    return array;
  }
}

std::shared_ptr<arrow::Array> IntrospectionMessageConverter::convert_builtin_field(
  const void* message, const struct FieldDescriptor& field_desc
) {
  // For introspection-based conversion, we create placeholder arrays
  // The actual values would need type-specific extraction which requires
  // full rosidl introspection support
  //
  // This fallback creates arrays with null/default values to allow schema
  // validation and dataset creation. Real implementations should register
  // typed converters for proper field extraction.

  std::shared_ptr<arrow::Array> array;
  arrow::Status status;

  const std::string& type = field_desc.type_name;

  if (type == "bool" || type == "boolean") {
    arrow::BooleanBuilder builder;
    status = builder.AppendNull();
    if (status.ok()) status = builder.Finish(&array);
  } else if (type == "int8") {
    arrow::Int8Builder builder;
    status = builder.AppendNull();
    if (status.ok()) status = builder.Finish(&array);
  } else if (type == "uint8" || type == "byte") {
    arrow::UInt8Builder builder;
    status = builder.AppendNull();
    if (status.ok()) status = builder.Finish(&array);
  } else if (type == "int16") {
    arrow::Int16Builder builder;
    status = builder.AppendNull();
    if (status.ok()) status = builder.Finish(&array);
  } else if (type == "uint16") {
    arrow::UInt16Builder builder;
    status = builder.AppendNull();
    if (status.ok()) status = builder.Finish(&array);
  } else if (type == "int32") {
    arrow::Int32Builder builder;
    status = builder.AppendNull();
    if (status.ok()) status = builder.Finish(&array);
  } else if (type == "uint32") {
    arrow::UInt32Builder builder;
    status = builder.AppendNull();
    if (status.ok()) status = builder.Finish(&array);
  } else if (type == "int64") {
    arrow::Int64Builder builder;
    status = builder.AppendNull();
    if (status.ok()) status = builder.Finish(&array);
  } else if (type == "uint64") {
    arrow::UInt64Builder builder;
    status = builder.AppendNull();
    if (status.ok()) status = builder.Finish(&array);
  } else if (type == "float32" || type == "float") {
    arrow::FloatBuilder builder;
    status = builder.AppendNull();
    if (status.ok()) status = builder.Finish(&array);
  } else if (type == "float64" || type == "double") {
    arrow::DoubleBuilder builder;
    status = builder.AppendNull();
    if (status.ok()) status = builder.Finish(&array);
  } else if (type == "string") {
    arrow::StringBuilder builder;
    status = builder.AppendNull();
    if (status.ok()) status = builder.Finish(&array);
  } else {
    // Default to binary for unknown types
    arrow::BinaryBuilder builder;
    status = builder.AppendNull();
    if (status.ok()) status = builder.Finish(&array);
  }

  if (!status.ok()) {
    std::cerr << "Failed to build array for field " << field_desc.name << ": " << status.ToString()
              << std::endl;
    return nullptr;
  }

  return array;
}

}  // namespace core
}  // namespace axon
