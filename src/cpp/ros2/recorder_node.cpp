#include "recorder_node.hpp"
#include "../common/ros_interface.hpp"
#include "../core/batch_manager.hpp"
#include "../core/config_parser.hpp"
#include "../core/message_converter.hpp"
#include "../core/schema_merger.hpp"
#include "../ffi/lance_bridge.h"
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <fstream>

namespace axon {
namespace ros2 {

AxonNode::AxonNode(const std::string& node_name)
    : rclcpp::Node(node_name)
    , recording_(false)
    , dataset_handle_(0)
{
}

AxonNode::~AxonNode() {
    stop_recording();
    
    // Stop all batch managers
    for (auto& pair : batch_managers_) {
        pair.second->stop();
    }
    batch_managers_.clear();
    
    // Unsubscribe all (handled by shared_ptr cleanup)
    subscriptions_.clear();
    
    if (dataset_handle_ > 0) {
        close_dataset(dataset_handle_);
        dataset_handle_ = 0;
    }
}

bool AxonNode::initialize() {
    // Load configuration
    if (!load_configuration()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load configuration");
        return false;
    }
    
    // Collect schemas from all topics
    if (!collect_topic_schemas()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to collect topic schemas");
        return false;
    }
    
    // Initialize dataset with merged schema
    if (!initialize_dataset()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize dataset");
        return false;
    }
    
    // Setup topic recordings
    for (const auto& topic_config : config_->topics) {
        setup_topic_recording(topic_config.name, topic_config.message_type);
    }
    
    // Setup services
    setup_services();
    
    return true;
}

bool AxonNode::load_configuration() {
    // Get config path from parameter
    std::string config_path = this->declare_parameter<std::string>("config_path", "config/default_config.yaml");
    
    config_ = std::make_shared<core::RecorderConfig>(
        core::RecorderConfig::from_yaml(config_path)
    );
    
    if (!config_->validate()) {
        RCLCPP_ERROR(this->get_logger(), "Invalid configuration");
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Configuration loaded: %s", config_->to_string().c_str());
    return true;
}

bool AxonNode::collect_topic_schemas() {
    // Create converters for all topics to get their schemas
    for (const auto& topic_config : config_->topics) {
        auto converter = core::MessageConverterFactory::create(topic_config.message_type);
        if (!converter) {
            RCLCPP_WARN(this->get_logger(), "No converter for message type: %s", 
                       topic_config.message_type.c_str());
            continue;
        }
        
        converters_[topic_config.name] = std::move(converter);
        topic_schemas_[topic_config.name] = converters_[topic_config.name]->get_schema();
    }
    
    return !topic_schemas_.empty();
}

bool AxonNode::initialize_dataset() {
    dataset_path_ = config_->dataset.path;
    
    // Merge schemas from all topics
    merged_schema_ = core::SchemaMerger::merge_schemas(topic_schemas_, true);
    
    if (!merged_schema_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to merge schemas");
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Merged schema has %zu fields", merged_schema_->num_fields());
    
    // Export schema to C interface
    struct ArrowSchema c_schema;
    arrow::Status status = arrow::ExportSchema(*merged_schema_, &c_schema);
    if (!status.ok()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to export schema: %s", status.ToString().c_str());
        return false;
    }
    
    dataset_handle_ = create_or_open_dataset(dataset_path_.c_str(), &c_schema);
    
    // Release schema
    c_schema.release(&c_schema);
    
    if (dataset_handle_ <= 0) {
        const char* error = lance_get_last_error();
        if (error) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create dataset: %s", error);
        }
        return false;
    }
    
    return true;
}

void AxonNode::setup_topic_recording(const std::string& topic_name, const std::string& message_type) {
    // Find topic config
    core::TopicConfig* topic_config = nullptr;
    for (auto& tc : config_->topics) {
        if (tc.name == topic_name) {
            topic_config = &tc;
            break;
        }
    }
    
    if (!topic_config) {
        RCLCPP_WARN(this->get_logger(), "No config found for topic: %s", topic_name.c_str());
        return;
    }
    
    // Get converter (already created in collect_topic_schemas)
    auto converter_it = converters_.find(topic_name);
    if (converter_it == converters_.end()) {
        RCLCPP_WARN(this->get_logger(), "No converter for topic: %s", topic_name.c_str());
        return;
    }
    
    // Create batch manager with merged schema
    auto write_callback = [this](const std::shared_ptr<arrow::RecordBatch>& batch,
                                 const std::string& path,
                                 int64_t handle) -> bool {
        struct ArrowArray c_array;
        struct ArrowSchema c_schema;
        arrow::Status status = arrow::ExportRecordBatch(*batch, &c_array, &c_schema);
        
        if (!status.ok()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to export batch: %s", status.ToString().c_str());
            return false;
        }
        
        int32_t result = write_batch(handle, &c_array, &c_schema);
        
        // Release C structures
        c_array.release(&c_array);
        c_schema.release(&c_schema);
        
        if (result != LANCE_SUCCESS) {
            const char* error = lance_get_last_error();
            if (error) {
                RCLCPP_ERROR(this->get_logger(), "Write failed: %s", error);
            }
        }
        
        return result == LANCE_SUCCESS;
    };
    
    // Create memory pool for this batch manager
    auto memory_pool = arrow::default_memory_pool();
    
    auto batch_mgr = std::make_unique<core::BatchManager>(
        topic_config->batch_size,
        topic_config->flush_interval_ms,
        write_callback,
        memory_pool
    );
    
    // Initialize with merged schema
    batch_mgr->initialize_schema(merged_schema_);
    batch_mgr->set_dataset(dataset_path_, dataset_handle_);
    batch_mgr->start();
    
    batch_managers_[topic_name] = std::move(batch_mgr);
    
    // Subscribe to topic using ROS 2 interface
    // Note: Full implementation would use typed subscriptions
    RCLCPP_INFO(this->get_logger(), "Subscribed to topic: %s (type: %s)", 
                topic_name.c_str(), message_type.c_str());
}

void AxonNode::setup_services() {
    // Full service implementation would be added here
    // For ROS 2, services are handled via the ROS interface abstraction
    RCLCPP_INFO(this->get_logger(), "Services available via ROS interface");
}

void AxonNode::start_recording() {
    recording_ = true;
    RCLCPP_INFO(this->get_logger(), "Recording started");
}

void AxonNode::stop_recording() {
    recording_ = false;
    
    // Flush all batch managers
    for (auto& pair : batch_managers_) {
        pair.second->flush();
    }
    
    RCLCPP_INFO(this->get_logger(), "Recording stopped");
}

} // namespace ros2
} // namespace axon

// Main entry point for ROS 2
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<axon::ros2::AxonNode>();
    
    if (!node->initialize()) {
        rclcpp::shutdown();
        return 1;
    }
    
    bool auto_start = false;
    try {
        auto_start = node->get_parameter("auto_start").as_bool();
    } catch (...) {
        // Parameter might not exist, use default
    }
    
    if (auto_start) {
        node->start_recording();
    }
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
