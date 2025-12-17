#include "schema_merger.hpp"
#include <sstream>
#include <algorithm>

namespace axon {
namespace core {

std::string SchemaMerger::make_field_name(const std::string& topic, 
                                         const std::string& field_name,
                                         bool prefix_fields) {
    if (!prefix_fields) {
        return field_name;
    }
    
    // Sanitize topic name for use in field name
    std::string sanitized_topic = topic;
    std::replace(sanitized_topic.begin(), sanitized_topic.end(), '/', '_');
    std::replace(sanitized_topic.begin(), sanitized_topic.end(), '-', '_');
    
    return sanitized_topic + "_" + field_name;
}

bool SchemaMerger::fields_compatible(const std::shared_ptr<arrow::Field>& f1,
                                     const std::shared_ptr<arrow::Field>& f2) {
    return f1->type()->Equals(f2->type()) && 
           f1->nullable() == f2->nullable();
}

std::shared_ptr<arrow::Schema> SchemaMerger::merge_schemas(
    const std::unordered_map<std::string, std::shared_ptr<arrow::Schema>>& topic_schemas) {
    return merge_schemas(topic_schemas, true);  // Default: prefix fields
}

std::shared_ptr<arrow::Schema> SchemaMerger::merge_schemas(
    const std::unordered_map<std::string, std::shared_ptr<arrow::Schema>>& topic_schemas,
    bool prefix_fields) {
    
    std::vector<std::shared_ptr<arrow::Field>> merged_fields;
    std::unordered_map<std::string, int> field_name_map;  // For deduplication
    
    // Always start with timestamp and topic fields
    merged_fields.push_back(arrow::field("timestamp", arrow::int64(), false));
    merged_fields.push_back(arrow::field("topic", arrow::utf8(), false));
    
    // Merge fields from all topics
    for (const auto& topic_pair : topic_schemas) {
        const std::string& topic_name = topic_pair.first;
        const auto& schema = topic_pair.second;
        
        if (!schema) {
            continue;
        }
        
        for (int i = 0; i < schema->num_fields(); ++i) {
            const auto& field = schema->field(i);
            std::string merged_name = make_field_name(topic_name, field->name(), prefix_fields);
            
            // Check for conflicts
            if (field_name_map.find(merged_name) != field_name_map.end()) {
                // Field name conflict - check if compatible
                int existing_idx = field_name_map[merged_name];
                if (!fields_compatible(merged_fields[existing_idx], field)) {
                    // Incompatible types - create unique name
                    std::ostringstream oss;
                    oss << merged_name << "_" << topic_name;
                    merged_name = oss.str();
                } else {
                    // Compatible - skip duplicate
                    continue;
                }
            }
            
            // Add field with merged name
            auto merged_field = arrow::field(merged_name, field->type(), field->nullable());
            merged_fields.push_back(merged_field);
            field_name_map[merged_name] = merged_fields.size() - 1;
        }
    }
    
    return std::make_shared<arrow::Schema>(merged_fields);
}

std::shared_ptr<arrow::Schema> SchemaMerger::create_recording_schema(
    const std::unordered_map<std::string, std::shared_ptr<arrow::Schema>>& topic_schemas) {
    
    // Create a schema optimized for recording:
    // - timestamp (int64)
    // - topic (string)
    // - message_data (binary) - serialized message for flexibility
    
    std::vector<std::shared_ptr<arrow::Field>> fields = {
        arrow::field("timestamp", arrow::int64(), false),
        arrow::field("topic", arrow::utf8(), false),
        arrow::field("message_data", arrow::binary(), false)
    };
    
    return std::make_shared<arrow::Schema>(fields);
}

std::unordered_map<std::string, int> SchemaMerger::get_field_mapping(
    const std::unordered_map<std::string, std::shared_ptr<arrow::Schema>>& topic_schemas,
    const std::shared_ptr<arrow::Schema>& merged_schema) {
    
    std::unordered_map<std::string, int> mapping;
    
    // Map: "topic/field" -> merged_schema field index
    for (const auto& topic_pair : topic_schemas) {
        const std::string& topic_name = topic_pair.first;
        const auto& schema = topic_pair.second;
        
        if (!schema) continue;
        
        for (int i = 0; i < schema->num_fields(); ++i) {
            const auto& field = schema->field(i);
            std::string key = topic_name + "/" + field->name();
            
            // Find in merged schema
            std::string merged_name = make_field_name(topic_name, field->name(), true);
            int merged_idx = merged_schema->GetFieldIndex(merged_name);
            
            if (merged_idx >= 0) {
                mapping[key] = merged_idx;
            }
        }
    }
    
    return mapping;
}

} // namespace core
} // namespace axon

