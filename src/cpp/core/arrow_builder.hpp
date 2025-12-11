#ifndef ARROW_BUILDER_HPP
#define ARROW_BUILDER_HPP

#include <arrow/api.h>
#include <arrow/builder.h>
#include <arrow/type.h>
#include <memory>
#include <string>
#include <vector>

namespace lance_recorder {
namespace core {

/**
 * Generic Arrow builder utilities for zero-copy data construction
 */
class ArrowBuilder {
public:
    ArrowBuilder() = default;
    virtual ~ArrowBuilder() = default;
    
    /**
     * Finish building and return the array
     */
    virtual arrow::Status Finish(std::shared_ptr<arrow::Array>* out) = 0;
    
    /**
     * Reset the builder for reuse
     */
    virtual arrow::Status Reset() = 0;
    
    /**
     * Get the number of elements currently in the builder
     */
    virtual int64_t length() const = 0;
};

/**
 * Template-based builder factory
 */
template<typename BuilderType>
class TypedArrowBuilder : public ArrowBuilder {
public:
    explicit TypedArrowBuilder(std::shared_ptr<arrow::DataType> type) 
        : builder_(type, arrow::default_memory_pool()) {}
    
    arrow::Status Finish(std::shared_ptr<arrow::Array>* out) override {
        return builder_.Finish(out);
    }
    
    arrow::Status Reset() override {
        builder_.Reset();
        return arrow::Status::OK();
    }
    
    int64_t length() const override {
        return builder_.length();
    }
    
    BuilderType& get() { return builder_; }
    const BuilderType& get() const { return builder_; }
    
private:
    BuilderType builder_;
};

/**
 * Helper functions for common Arrow types
 */
class ArrowBuilderFactory {
public:
    static std::unique_ptr<ArrowBuilder> create_builder(std::shared_ptr<arrow::DataType> type);
    
    // Convenience methods for common types
    static std::unique_ptr<TypedArrowBuilder<arrow::Int64Builder>> create_int64_builder();
    static std::unique_ptr<TypedArrowBuilder<arrow::DoubleBuilder>> create_double_builder();
    static std::unique_ptr<TypedArrowBuilder<arrow::StringBuilder>> create_string_builder();
    static std::unique_ptr<TypedArrowBuilder<arrow::BinaryBuilder>> create_binary_builder();
    static std::unique_ptr<TypedArrowBuilder<arrow::BooleanBuilder>> create_bool_builder();
    
    // For arrays/lists
    template<typename ValueBuilder>
    static std::unique_ptr<TypedArrowBuilder<arrow::ListBuilder>> create_list_builder(
        std::shared_ptr<arrow::DataType> value_type);
};

/**
 * Memory pool manager for efficient memory allocation
 */
class ArrowMemoryPool {
public:
    static std::shared_ptr<arrow::MemoryPool> get_default_pool() {
        return arrow::default_memory_pool();
    }
    
    static std::shared_ptr<arrow::MemoryPool> get_system_pool() {
        return arrow::system_memory_pool();
    }
};

} // namespace core
} // namespace lance_recorder

#endif // ARROW_BUILDER_HPP

