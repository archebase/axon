#include <arrow/api.h>
#include <gtest/gtest.h>

#include <unordered_map>

#include "schema_merger.hpp"

using namespace axon::core;

// =============================================================================
// Basic Schema Merger Tests
// =============================================================================

TEST(SchemaMergerTest, MergeSingleSchema) {
  std::unordered_map<std::string, std::shared_ptr<arrow::Schema>> topic_schemas;

  auto schema1 = arrow::schema(
    {arrow::field("x", arrow::float64()),
     arrow::field("y", arrow::float64()),
     arrow::field("z", arrow::float64())}
  );

  topic_schemas["/robot/position"] = schema1;

  auto merged = SchemaMerger::merge_schemas(topic_schemas);

  ASSERT_NE(merged, nullptr);
  // Should have timestamp + topic + 3 fields
  EXPECT_EQ(merged->num_fields(), 5);

  // Check timestamp and topic fields exist
  EXPECT_EQ(merged->field(0)->name(), "timestamp");
  EXPECT_EQ(merged->field(1)->name(), "topic");
}

TEST(SchemaMergerTest, MergeMultipleSchemas) {
  std::unordered_map<std::string, std::shared_ptr<arrow::Schema>> topic_schemas;

  auto position_schema =
    arrow::schema({arrow::field("x", arrow::float64()), arrow::field("y", arrow::float64())});

  auto velocity_schema =
    arrow::schema({arrow::field("vx", arrow::float64()), arrow::field("vy", arrow::float64())});

  topic_schemas["/robot/position"] = position_schema;
  topic_schemas["/robot/velocity"] = velocity_schema;

  auto merged = SchemaMerger::merge_schemas(topic_schemas);

  ASSERT_NE(merged, nullptr);
  // timestamp + topic + 4 position/velocity fields
  EXPECT_EQ(merged->num_fields(), 6);
}

TEST(SchemaMergerTest, MergeWithPrefixEnabled) {
  std::unordered_map<std::string, std::shared_ptr<arrow::Schema>> topic_schemas;

  auto schema1 = arrow::schema({arrow::field("value", arrow::int64())});

  auto schema2 = arrow::schema({
    arrow::field("value", arrow::int64())  // Same field name
  });

  topic_schemas["/topic1"] = schema1;
  topic_schemas["/topic2"] = schema2;

  auto merged = SchemaMerger::merge_schemas(topic_schemas, true);

  ASSERT_NE(merged, nullptr);
  // Fields should be prefixed to avoid collision
  // timestamp + topic + prefixed fields
  EXPECT_GE(merged->num_fields(), 3);
}

TEST(SchemaMergerTest, MergeWithPrefixDisabled) {
  std::unordered_map<std::string, std::shared_ptr<arrow::Schema>> topic_schemas;

  auto schema1 = arrow::schema({arrow::field("unique_field1", arrow::int64())});

  auto schema2 = arrow::schema({arrow::field("unique_field2", arrow::int64())});

  topic_schemas["/topic1"] = schema1;
  topic_schemas["/topic2"] = schema2;

  auto merged = SchemaMerger::merge_schemas(topic_schemas, false);

  ASSERT_NE(merged, nullptr);
  // Fields should not be prefixed
  int idx1 = merged->GetFieldIndex("unique_field1");
  int idx2 = merged->GetFieldIndex("unique_field2");
  EXPECT_GE(idx1, 0);
  EXPECT_GE(idx2, 0);
}

// =============================================================================
// Empty and Null Schema Tests
// =============================================================================

TEST(SchemaMergerTest, MergeEmptySchemaMap) {
  std::unordered_map<std::string, std::shared_ptr<arrow::Schema>> topic_schemas;

  auto merged = SchemaMerger::merge_schemas(topic_schemas);

  ASSERT_NE(merged, nullptr);
  // Should have at least timestamp and topic fields
  EXPECT_EQ(merged->num_fields(), 2);
  EXPECT_EQ(merged->field(0)->name(), "timestamp");
  EXPECT_EQ(merged->field(1)->name(), "topic");
}

TEST(SchemaMergerTest, MergeWithNullSchema) {
  std::unordered_map<std::string, std::shared_ptr<arrow::Schema>> topic_schemas;

  topic_schemas["/valid/topic"] = arrow::schema({arrow::field("value", arrow::int64())});
  topic_schemas["/null/topic"] = nullptr;  // Null schema

  auto merged = SchemaMerger::merge_schemas(topic_schemas);

  ASSERT_NE(merged, nullptr);
  // Should skip null schema
  EXPECT_GE(merged->num_fields(), 3);  // timestamp + topic + value
}

// =============================================================================
// Field Name Conflict Tests
// =============================================================================

TEST(SchemaMergerTest, CompatibleFieldsAreMerged) {
  std::unordered_map<std::string, std::shared_ptr<arrow::Schema>> topic_schemas;

  // Two topics with same field name and type
  auto schema1 = arrow::schema({arrow::field("data", arrow::int64(), false)});

  auto schema2 = arrow::schema({arrow::field("data", arrow::int64(), false)});

  topic_schemas["/topic1"] = schema1;
  topic_schemas["/topic2"] = schema2;

  // With prefix disabled, compatible fields should be merged
  auto merged = SchemaMerger::merge_schemas(topic_schemas, false);

  ASSERT_NE(merged, nullptr);
  // Should have timestamp + topic + data (deduplicated)
  // Count "data" fields
  int data_count = 0;
  for (int i = 0; i < merged->num_fields(); ++i) {
    if (merged->field(i)->name() == "data") {
      data_count++;
    }
  }
  EXPECT_EQ(data_count, 1);  // Only one "data" field
}

TEST(SchemaMergerTest, IncompatibleFieldsCreateUniqueNames) {
  std::unordered_map<std::string, std::shared_ptr<arrow::Schema>> topic_schemas;

  // Two topics with same field name but different types
  auto schema1 = arrow::schema({arrow::field("value", arrow::int64(), false)});

  auto schema2 = arrow::schema({
    arrow::field("value", arrow::utf8(), false)  // Different type!
  });

  topic_schemas["/topic1"] = schema1;
  topic_schemas["/topic2"] = schema2;

  auto merged = SchemaMerger::merge_schemas(topic_schemas, false);

  ASSERT_NE(merged, nullptr);
  // Both fields should exist with unique names
  EXPECT_GT(merged->num_fields(), 3);  // timestamp + topic + 2 value fields
}

// =============================================================================
// Recording Schema Tests
// =============================================================================

TEST(SchemaMergerTest, CreateRecordingSchema) {
  std::unordered_map<std::string, std::shared_ptr<arrow::Schema>> topic_schemas;

  // Add some topics (they don't matter for recording schema)
  topic_schemas["/topic1"] = arrow::schema({arrow::field("x", arrow::float64())});

  auto schema = SchemaMerger::create_recording_schema(topic_schemas);

  ASSERT_NE(schema, nullptr);
  EXPECT_EQ(schema->num_fields(), 3);

  // Check specific fields
  EXPECT_EQ(schema->field(0)->name(), "timestamp");
  EXPECT_TRUE(schema->field(0)->type()->Equals(arrow::int64()));

  EXPECT_EQ(schema->field(1)->name(), "topic");
  EXPECT_TRUE(schema->field(1)->type()->Equals(arrow::utf8()));

  EXPECT_EQ(schema->field(2)->name(), "message_data");
  EXPECT_TRUE(schema->field(2)->type()->Equals(arrow::binary()));
}

TEST(SchemaMergerTest, CreateRecordingSchemaEmpty) {
  std::unordered_map<std::string, std::shared_ptr<arrow::Schema>> empty_schemas;

  auto schema = SchemaMerger::create_recording_schema(empty_schemas);

  ASSERT_NE(schema, nullptr);
  // Should still create recording schema with standard fields
  EXPECT_EQ(schema->num_fields(), 3);
}

// =============================================================================
// Field Mapping Tests
// =============================================================================

TEST(SchemaMergerTest, GetFieldMapping) {
  std::unordered_map<std::string, std::shared_ptr<arrow::Schema>> topic_schemas;

  auto schema1 =
    arrow::schema({arrow::field("x", arrow::float64()), arrow::field("y", arrow::float64())});

  topic_schemas["/position"] = schema1;

  auto merged = SchemaMerger::merge_schemas(topic_schemas);
  auto mapping = SchemaMerger::get_field_mapping(topic_schemas, merged);

  // Check mappings exist
  EXPECT_GT(mapping.size(), 0u);

  // Mapping keys should be "topic/field" format
  for (const auto& kv : mapping) {
    EXPECT_NE(kv.first.find("/"), std::string::npos);
    EXPECT_GE(kv.second, 0);
    EXPECT_LT(kv.second, merged->num_fields());
  }
}

TEST(SchemaMergerTest, GetFieldMappingMultipleTopics) {
  std::unordered_map<std::string, std::shared_ptr<arrow::Schema>> topic_schemas;

  topic_schemas["/topic1"] =
    arrow::schema({arrow::field("a", arrow::int64()), arrow::field("b", arrow::int64())});

  topic_schemas["/topic2"] =
    arrow::schema({arrow::field("c", arrow::float64()), arrow::field("d", arrow::float64())});

  auto merged = SchemaMerger::merge_schemas(topic_schemas);
  auto mapping = SchemaMerger::get_field_mapping(topic_schemas, merged);

  // Should have mapping for all 4 fields
  EXPECT_GE(mapping.size(), 4u);
}

// =============================================================================
// Topic Name Sanitization Tests
// =============================================================================

TEST(SchemaMergerTest, TopicNameWithSlashes) {
  std::unordered_map<std::string, std::shared_ptr<arrow::Schema>> topic_schemas;

  topic_schemas["/robot/arm/joint1"] = arrow::schema({arrow::field("position", arrow::float64())});

  auto merged = SchemaMerger::merge_schemas(topic_schemas, true);

  ASSERT_NE(merged, nullptr);
  // Field name should have sanitized topic prefix (slashes replaced)
  bool found_sanitized = false;
  for (int i = 0; i < merged->num_fields(); ++i) {
    std::string name = merged->field(i)->name();
    if (name.find("robot_arm_joint1") != std::string::npos) {
      found_sanitized = true;
      break;
    }
  }
  EXPECT_TRUE(found_sanitized);
}

TEST(SchemaMergerTest, TopicNameWithDashes) {
  std::unordered_map<std::string, std::shared_ptr<arrow::Schema>> topic_schemas;

  topic_schemas["/sensor-data"] = arrow::schema({arrow::field("value", arrow::int64())});

  auto merged = SchemaMerger::merge_schemas(topic_schemas, true);

  ASSERT_NE(merged, nullptr);
  // Dashes should be replaced with underscores
  bool found_sanitized = false;
  for (int i = 0; i < merged->num_fields(); ++i) {
    std::string name = merged->field(i)->name();
    if (name.find("sensor_data") != std::string::npos) {
      found_sanitized = true;
      break;
    }
  }
  EXPECT_TRUE(found_sanitized);
}

// =============================================================================
// Data Type Tests
// =============================================================================

TEST(SchemaMergerTest, MergeVariousDataTypes) {
  std::unordered_map<std::string, std::shared_ptr<arrow::Schema>> topic_schemas;

  auto schema = arrow::schema(
    {arrow::field("int_val", arrow::int64()),
     arrow::field("float_val", arrow::float64()),
     arrow::field("string_val", arrow::utf8()),
     arrow::field("bool_val", arrow::boolean()),
     arrow::field("binary_val", arrow::binary())}
  );

  topic_schemas["/mixed/types"] = schema;

  auto merged = SchemaMerger::merge_schemas(topic_schemas);

  ASSERT_NE(merged, nullptr);
  EXPECT_EQ(merged->num_fields(), 7);  // timestamp + topic + 5 fields
}

TEST(SchemaMergerTest, MergeNestedTypes) {
  std::unordered_map<std::string, std::shared_ptr<arrow::Schema>> topic_schemas;

  // Schema with list type
  auto list_type = arrow::list(arrow::float64());
  auto schema = arrow::schema({arrow::field("points", list_type)});

  topic_schemas["/list/topic"] = schema;

  auto merged = SchemaMerger::merge_schemas(topic_schemas);

  ASSERT_NE(merged, nullptr);
  // Should handle nested types
  EXPECT_GE(merged->num_fields(), 3);
}

// =============================================================================
// Nullable Field Tests
// =============================================================================

TEST(SchemaMergerTest, NullableFieldsPreserved) {
  std::unordered_map<std::string, std::shared_ptr<arrow::Schema>> topic_schemas;

  auto schema = arrow::schema(
    {arrow::field("required_field", arrow::int64(), false),
     arrow::field("optional_field", arrow::int64(), true)}
  );

  topic_schemas["/nullable/test"] = schema;

  auto merged = SchemaMerger::merge_schemas(topic_schemas, true);

  ASSERT_NE(merged, nullptr);

  // Find and check nullable flag is preserved
  for (int i = 0; i < merged->num_fields(); ++i) {
    const auto& field = merged->field(i);
    if (field->name().find("required_field") != std::string::npos) {
      EXPECT_FALSE(field->nullable());
    }
    if (field->name().find("optional_field") != std::string::npos) {
      EXPECT_TRUE(field->nullable());
    }
  }
}

// =============================================================================
// Performance/Scale Tests
// =============================================================================

TEST(SchemaMergerTest, MergeManyTopics) {
  std::unordered_map<std::string, std::shared_ptr<arrow::Schema>> topic_schemas;

  // Create 50 topics with 5 fields each
  for (int i = 0; i < 50; ++i) {
    std::vector<std::shared_ptr<arrow::Field>> fields;
    for (int j = 0; j < 5; ++j) {
      fields.push_back(arrow::field("field" + std::to_string(j), arrow::float64()));
    }
    topic_schemas["/topic" + std::to_string(i)] = arrow::schema(fields);
  }

  auto merged = SchemaMerger::merge_schemas(topic_schemas);

  ASSERT_NE(merged, nullptr);
  // timestamp + topic + 250 fields = 252
  EXPECT_EQ(merged->num_fields(), 252);
}

TEST(SchemaMergerTest, MergeLargeSchema) {
  std::unordered_map<std::string, std::shared_ptr<arrow::Schema>> topic_schemas;

  // Single topic with many fields
  std::vector<std::shared_ptr<arrow::Field>> fields;
  for (int i = 0; i < 100; ++i) {
    fields.push_back(arrow::field("field_" + std::to_string(i), arrow::float64()));
  }

  topic_schemas["/large/schema"] = arrow::schema(fields);

  auto merged = SchemaMerger::merge_schemas(topic_schemas);

  ASSERT_NE(merged, nullptr);
  EXPECT_EQ(merged->num_fields(), 102);  // timestamp + topic + 100 fields
}
