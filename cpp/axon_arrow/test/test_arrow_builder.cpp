#include <gtest/gtest.h>
#include "arrow_builder.hpp"
#include <arrow/api.h>

using namespace axon::core;

// =============================================================================
// Basic Builder Creation Tests
// =============================================================================

TEST(ArrowBuilderTest, CreateInt64Builder) {
    auto builder = ArrowBuilderFactory::create_int64_builder();
    ASSERT_NE(builder, nullptr);
    
    auto& typed_builder = static_cast<TypedArrowBuilder<arrow::Int64Builder>&>(*builder);
    ASSERT_TRUE(typed_builder.get().Append(42).ok());
    ASSERT_TRUE(typed_builder.get().Append(100).ok());
    
    std::shared_ptr<arrow::Array> array;
    ASSERT_TRUE(typed_builder.Finish(&array).ok());
    ASSERT_EQ(array->length(), 2);
}

TEST(ArrowBuilderTest, CreateStringBuilder) {
    auto builder = ArrowBuilderFactory::create_string_builder();
    ASSERT_NE(builder, nullptr);
    
    auto& typed_builder = static_cast<TypedArrowBuilder<arrow::StringBuilder>&>(*builder);
    ASSERT_TRUE(typed_builder.get().Append("test").ok());
    ASSERT_TRUE(typed_builder.get().Append("string").ok());
    
    std::shared_ptr<arrow::Array> array;
    ASSERT_TRUE(typed_builder.Finish(&array).ok());
    ASSERT_EQ(array->length(), 2);
}

TEST(ArrowBuilderTest, CreateBinaryBuilder) {
    auto builder = ArrowBuilderFactory::create_binary_builder();
    ASSERT_NE(builder, nullptr);
    
    auto& typed_builder = static_cast<TypedArrowBuilder<arrow::BinaryBuilder>&>(*builder);
    std::vector<uint8_t> data = {0x01, 0x02, 0x03};
    ASSERT_TRUE(typed_builder.get().Append(data.data(), data.size()).ok());
    
    std::shared_ptr<arrow::Array> array;
    ASSERT_TRUE(typed_builder.Finish(&array).ok());
    ASSERT_EQ(array->length(), 1);
}

TEST(ArrowBuilderTest, CreateDoubleBuilder) {
    auto builder = ArrowBuilderFactory::create_double_builder();
    ASSERT_NE(builder, nullptr);
    
    auto& typed_builder = static_cast<TypedArrowBuilder<arrow::DoubleBuilder>&>(*builder);
    ASSERT_TRUE(typed_builder.get().Append(3.14159).ok());
    ASSERT_TRUE(typed_builder.get().Append(-2.71828).ok());
    ASSERT_TRUE(typed_builder.get().Append(0.0).ok());
    
    std::shared_ptr<arrow::Array> array;
    ASSERT_TRUE(typed_builder.Finish(&array).ok());
    ASSERT_EQ(array->length(), 3);
    
    // Verify values
    auto double_array = std::static_pointer_cast<arrow::DoubleArray>(array);
    EXPECT_DOUBLE_EQ(double_array->Value(0), 3.14159);
    EXPECT_DOUBLE_EQ(double_array->Value(1), -2.71828);
    EXPECT_DOUBLE_EQ(double_array->Value(2), 0.0);
}

TEST(ArrowBuilderTest, CreateBoolBuilder) {
    auto builder = ArrowBuilderFactory::create_bool_builder();
    ASSERT_NE(builder, nullptr);
    
    auto& typed_builder = static_cast<TypedArrowBuilder<arrow::BooleanBuilder>&>(*builder);
    ASSERT_TRUE(typed_builder.get().Append(true).ok());
    ASSERT_TRUE(typed_builder.get().Append(false).ok());
    ASSERT_TRUE(typed_builder.get().Append(true).ok());
    
    std::shared_ptr<arrow::Array> array;
    ASSERT_TRUE(typed_builder.Finish(&array).ok());
    ASSERT_EQ(array->length(), 3);
    
    // Verify values
    auto bool_array = std::static_pointer_cast<arrow::BooleanArray>(array);
    EXPECT_TRUE(bool_array->Value(0));
    EXPECT_FALSE(bool_array->Value(1));
    EXPECT_TRUE(bool_array->Value(2));
}

// =============================================================================
// Factory Tests
// =============================================================================

TEST(ArrowBuilderTest, BuilderFactory) {
    auto int64_builder = ArrowBuilderFactory::create_builder(arrow::int64());
    ASSERT_NE(int64_builder, nullptr);
    
    auto string_builder = ArrowBuilderFactory::create_builder(arrow::utf8());
    ASSERT_NE(string_builder, nullptr);
    
    auto double_builder = ArrowBuilderFactory::create_builder(arrow::float64());
    ASSERT_NE(double_builder, nullptr);
}

TEST(ArrowBuilderTest, BuilderFactoryAllTypes) {
    // Test all supported types through the factory
    auto int32_builder = ArrowBuilderFactory::create_builder(arrow::int32());
    ASSERT_NE(int32_builder, nullptr);
    
    auto int64_builder = ArrowBuilderFactory::create_builder(arrow::int64());
    ASSERT_NE(int64_builder, nullptr);
    
    auto float_builder = ArrowBuilderFactory::create_builder(arrow::float32());
    ASSERT_NE(float_builder, nullptr);
    
    auto double_builder = ArrowBuilderFactory::create_builder(arrow::float64());
    ASSERT_NE(double_builder, nullptr);
    
    auto string_builder = ArrowBuilderFactory::create_builder(arrow::utf8());
    ASSERT_NE(string_builder, nullptr);
    
    auto binary_builder = ArrowBuilderFactory::create_builder(arrow::binary());
    ASSERT_NE(binary_builder, nullptr);
    
    auto bool_builder = ArrowBuilderFactory::create_builder(arrow::boolean());
    ASSERT_NE(bool_builder, nullptr);
}

TEST(ArrowBuilderTest, BuilderFactoryUnsupportedType) {
    // Test unsupported type returns nullptr
    auto list_builder = ArrowBuilderFactory::create_builder(arrow::list(arrow::int64()));
    ASSERT_EQ(list_builder, nullptr);
    
    auto struct_type = arrow::struct_({arrow::field("x", arrow::int64())});
    auto struct_builder = ArrowBuilderFactory::create_builder(struct_type);
    ASSERT_EQ(struct_builder, nullptr);
}

// =============================================================================
// Builder Methods Tests
// =============================================================================

TEST(ArrowBuilderTest, ResetBuilder) {
    auto builder = ArrowBuilderFactory::create_int64_builder();
    auto& typed_builder = static_cast<TypedArrowBuilder<arrow::Int64Builder>&>(*builder);
    
    ASSERT_TRUE(typed_builder.get().Append(1).ok());
    ASSERT_TRUE(typed_builder.get().Append(2).ok());
    
    std::shared_ptr<arrow::Array> array1;
    ASSERT_TRUE(typed_builder.Finish(&array1).ok());
    ASSERT_EQ(array1->length(), 2);
    
    // Reset and build again
    ASSERT_TRUE(typed_builder.Reset().ok());
    ASSERT_TRUE(typed_builder.get().Append(3).ok());
    
    std::shared_ptr<arrow::Array> array2;
    ASSERT_TRUE(typed_builder.Finish(&array2).ok());
    ASSERT_EQ(array2->length(), 1);
}

TEST(ArrowBuilderTest, LengthMethod) {
    auto builder = ArrowBuilderFactory::create_int64_builder();
    auto& typed_builder = static_cast<TypedArrowBuilder<arrow::Int64Builder>&>(*builder);
    
    EXPECT_EQ(typed_builder.length(), 0);
    
    ASSERT_TRUE(typed_builder.get().Append(1).ok());
    EXPECT_EQ(typed_builder.length(), 1);
    
    ASSERT_TRUE(typed_builder.get().Append(2).ok());
    EXPECT_EQ(typed_builder.length(), 2);
    
    ASSERT_TRUE(typed_builder.get().Append(3).ok());
    EXPECT_EQ(typed_builder.length(), 3);
}

TEST(ArrowBuilderTest, EmptyBuilder) {
    auto builder = ArrowBuilderFactory::create_string_builder();
    auto& typed_builder = static_cast<TypedArrowBuilder<arrow::StringBuilder>&>(*builder);
    
    // Finish without adding any values
    std::shared_ptr<arrow::Array> array;
    ASSERT_TRUE(typed_builder.Finish(&array).ok());
    ASSERT_EQ(array->length(), 0);
}

// =============================================================================
// Memory Pool Tests
// =============================================================================

TEST(ArrowMemoryPoolTest, GetDefaultPool) {
    auto pool = ArrowMemoryPool::get_default_pool();
    ASSERT_NE(pool, nullptr);
    
    // Pool should be usable
    int64_t bytes_before = pool->bytes_allocated();
    EXPECT_GE(bytes_before, 0);
}

TEST(ArrowMemoryPoolTest, GetSystemPool) {
    auto pool = ArrowMemoryPool::get_system_pool();
    ASSERT_NE(pool, nullptr);
}

TEST(ArrowMemoryPoolTest, PoolsSingleton) {
    // Getting pool multiple times should return same instance
    auto pool1 = ArrowMemoryPool::get_default_pool();
    auto pool2 = ArrowMemoryPool::get_default_pool();
    EXPECT_EQ(pool1, pool2);
}

// =============================================================================
// Edge Cases and Data Validation
// =============================================================================

TEST(ArrowBuilderTest, Int64EdgeValues) {
    auto builder = ArrowBuilderFactory::create_int64_builder();
    auto& typed_builder = static_cast<TypedArrowBuilder<arrow::Int64Builder>&>(*builder);
    
    // Test edge values
    ASSERT_TRUE(typed_builder.get().Append(INT64_MAX).ok());
    ASSERT_TRUE(typed_builder.get().Append(INT64_MIN).ok());
    ASSERT_TRUE(typed_builder.get().Append(0).ok());
    
    std::shared_ptr<arrow::Array> array;
    ASSERT_TRUE(typed_builder.Finish(&array).ok());
    
    auto int_array = std::static_pointer_cast<arrow::Int64Array>(array);
    EXPECT_EQ(int_array->Value(0), INT64_MAX);
    EXPECT_EQ(int_array->Value(1), INT64_MIN);
    EXPECT_EQ(int_array->Value(2), 0);
}

TEST(ArrowBuilderTest, StringSpecialCharacters) {
    auto builder = ArrowBuilderFactory::create_string_builder();
    auto& typed_builder = static_cast<TypedArrowBuilder<arrow::StringBuilder>&>(*builder);
    
    // Test special characters
    ASSERT_TRUE(typed_builder.get().Append("").ok());  // Empty string
    ASSERT_TRUE(typed_builder.get().Append("hello\nworld").ok());  // Newline
    ASSERT_TRUE(typed_builder.get().Append("tab\there").ok());  // Tab
    ASSERT_TRUE(typed_builder.get().Append("unicode: 你好").ok());  // Unicode
    
    std::shared_ptr<arrow::Array> array;
    ASSERT_TRUE(typed_builder.Finish(&array).ok());
    ASSERT_EQ(array->length(), 4);
}

TEST(ArrowBuilderTest, BinaryLargeData) {
    auto builder = ArrowBuilderFactory::create_binary_builder();
    auto& typed_builder = static_cast<TypedArrowBuilder<arrow::BinaryBuilder>&>(*builder);
    
    // Test large binary data
    std::vector<uint8_t> large_data(1024 * 10);  // 10KB
    for (size_t i = 0; i < large_data.size(); ++i) {
        large_data[i] = static_cast<uint8_t>(i % 256);
    }
    
    ASSERT_TRUE(typed_builder.get().Append(large_data.data(), large_data.size()).ok());
    
    std::shared_ptr<arrow::Array> array;
    ASSERT_TRUE(typed_builder.Finish(&array).ok());
    ASSERT_EQ(array->length(), 1);
    
    // Verify data integrity
    auto binary_array = std::static_pointer_cast<arrow::BinaryArray>(array);
    auto value = binary_array->GetView(0);
    ASSERT_EQ(value.size(), large_data.size());
}
