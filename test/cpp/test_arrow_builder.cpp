#include <gtest/gtest.h>
#include "../src/cpp/core/arrow_builder.hpp"
#include <arrow/api.h>

using namespace lance_recorder::core;

TEST(ArrowBuilderTest, CreateInt64Builder) {
    auto builder = ArrowBuilderFactory::create_int64_builder();
    ASSERT_NE(builder, nullptr);
    
    auto& typed_builder = static_cast<TypedArrowBuilder<arrow::Int64Builder>&>(*builder);
    typed_builder.get().Append(42);
    typed_builder.get().Append(100);
    
    std::shared_ptr<arrow::Array> array;
    ASSERT_TRUE(typed_builder.Finish(&array).ok());
    ASSERT_EQ(array->length(), 2);
}

TEST(ArrowBuilderTest, CreateStringBuilder) {
    auto builder = ArrowBuilderFactory::create_string_builder();
    ASSERT_NE(builder, nullptr);
    
    auto& typed_builder = static_cast<TypedArrowBuilder<arrow::StringBuilder>&>(*builder);
    typed_builder.get().Append("test");
    typed_builder.get().Append("string");
    
    std::shared_ptr<arrow::Array> array;
    ASSERT_TRUE(typed_builder.Finish(&array).ok());
    ASSERT_EQ(array->length(), 2);
}

TEST(ArrowBuilderTest, CreateBinaryBuilder) {
    auto builder = ArrowBuilderFactory::create_binary_builder();
    ASSERT_NE(builder, nullptr);
    
    auto& typed_builder = static_cast<TypedArrowBuilder<arrow::BinaryBuilder>&>(*builder);
    std::vector<uint8_t> data = {0x01, 0x02, 0x03};
    typed_builder.get().Append(data.data(), data.size());
    
    std::shared_ptr<arrow::Array> array;
    ASSERT_TRUE(typed_builder.Finish(&array).ok());
    ASSERT_EQ(array->length(), 1);
}

TEST(ArrowBuilderTest, BuilderFactory) {
    auto int64_builder = ArrowBuilderFactory::create_builder(arrow::int64());
    ASSERT_NE(int64_builder, nullptr);
    
    auto string_builder = ArrowBuilderFactory::create_builder(arrow::utf8());
    ASSERT_NE(string_builder, nullptr);
    
    auto double_builder = ArrowBuilderFactory::create_builder(arrow::float64());
    ASSERT_NE(double_builder, nullptr);
}

TEST(ArrowBuilderTest, ResetBuilder) {
    auto builder = ArrowBuilderFactory::create_int64_builder();
    auto& typed_builder = static_cast<TypedArrowBuilder<arrow::Int64Builder>&>(*builder);
    
    typed_builder.get().Append(1);
    typed_builder.get().Append(2);
    
    std::shared_ptr<arrow::Array> array1;
    typed_builder.Finish(&array1);
    ASSERT_EQ(array1->length(), 2);
    
    // Reset and build again
    typed_builder.Reset();
    typed_builder.get().Append(3);
    
    std::shared_ptr<arrow::Array> array2;
    typed_builder.Finish(&array2);
    ASSERT_EQ(array2->length(), 1);
}

