#include "arrow_builder.hpp"
#include <arrow/builder.h>
#include <arrow/type.h>

namespace axon {
namespace core {

std::unique_ptr<ArrowBuilder> ArrowBuilderFactory::create_builder(std::shared_ptr<arrow::DataType> type) {
    switch (type->id()) {
        case arrow::Type::INT64:
            return std::make_unique<TypedArrowBuilder<arrow::Int64Builder>>(type);
        case arrow::Type::INT32:
            return std::make_unique<TypedArrowBuilder<arrow::Int32Builder>>(type);
        case arrow::Type::DOUBLE:
            return std::make_unique<TypedArrowBuilder<arrow::DoubleBuilder>>(type);
        case arrow::Type::FLOAT:
            return std::make_unique<TypedArrowBuilder<arrow::FloatBuilder>>(type);
        case arrow::Type::STRING:
            return std::make_unique<TypedArrowBuilder<arrow::StringBuilder>>(type);
        case arrow::Type::BINARY:
            return std::make_unique<TypedArrowBuilder<arrow::BinaryBuilder>>(type);
        case arrow::Type::BOOL:
            return std::make_unique<TypedArrowBuilder<arrow::BooleanBuilder>>(type);
        default:
            // For complex types, return nullptr and let caller handle
            return nullptr;
    }
}

std::unique_ptr<TypedArrowBuilder<arrow::Int64Builder>> ArrowBuilderFactory::create_int64_builder() {
    return std::make_unique<TypedArrowBuilder<arrow::Int64Builder>>(arrow::int64());
}

std::unique_ptr<TypedArrowBuilder<arrow::DoubleBuilder>> ArrowBuilderFactory::create_double_builder() {
    return std::make_unique<TypedArrowBuilder<arrow::DoubleBuilder>>(arrow::float64());
}

std::unique_ptr<TypedArrowBuilder<arrow::StringBuilder>> ArrowBuilderFactory::create_string_builder() {
    return std::make_unique<TypedArrowBuilder<arrow::StringBuilder>>(arrow::utf8());
}

std::unique_ptr<TypedArrowBuilder<arrow::BinaryBuilder>> ArrowBuilderFactory::create_binary_builder() {
    return std::make_unique<TypedArrowBuilder<arrow::BinaryBuilder>>(arrow::binary());
}

std::unique_ptr<TypedArrowBuilder<arrow::BooleanBuilder>> ArrowBuilderFactory::create_bool_builder() {
    return std::make_unique<TypedArrowBuilder<arrow::BooleanBuilder>>(arrow::boolean());
}

} // namespace core
} // namespace axon

