#ifndef SCHEMA_MERGER_HPP
#define SCHEMA_MERGER_HPP

#include <arrow/api.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace axon {
namespace core {

/**
 * Schema field with topic information
 */
struct TopicField {
  std::string topic_name;
  std::shared_ptr<arrow::Field> field;

  TopicField(const std::string& topic, std::shared_ptr<arrow::Field> f)
      : topic_name(topic)
      , field(f) {}
};

/**
 * Merges multiple Arrow schemas into a single unified schema
 * Handles field name conflicts by prefixing with topic name
 */
class SchemaMerger {
public:
  /**
   * Merge multiple schemas from different topics
   * @param topic_schemas Map of topic name to schema
   * @return Merged schema
   */
  static std::shared_ptr<arrow::Schema> merge_schemas(
    const std::unordered_map<std::string, std::shared_ptr<arrow::Schema>>& topic_schemas
  );

  /**
   * Merge schemas with custom field naming strategy
   * @param topic_schemas Map of topic name to schema
   * @param prefix_fields Whether to prefix fields with topic name
   * @return Merged schema
   */
  static std::shared_ptr<arrow::Schema> merge_schemas(
    const std::unordered_map<std::string, std::shared_ptr<arrow::Schema>>& topic_schemas,
    bool prefix_fields
  );

  /**
   * Get field mapping: original topic+field -> merged field index
   */
  static std::unordered_map<std::string, int> get_field_mapping(
    const std::unordered_map<std::string, std::shared_ptr<arrow::Schema>>& topic_schemas,
    const std::shared_ptr<arrow::Schema>& merged_schema
  );

  /**
   * Create a schema with timestamp and topic fields prepended
   */
  static std::shared_ptr<arrow::Schema> create_recording_schema(
    const std::unordered_map<std::string, std::shared_ptr<arrow::Schema>>& topic_schemas
  );

private:
  static std::string make_field_name(
    const std::string& topic, const std::string& field_name, bool prefix_fields
  );

  static bool fields_compatible(
    const std::shared_ptr<arrow::Field>& f1, const std::shared_ptr<arrow::Field>& f2
  );
};

}  // namespace core
}  // namespace axon

#endif  // SCHEMA_MERGER_HPP
