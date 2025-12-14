#include "../common/ros_interface.hpp"
#include "../core/batch_manager.hpp"
#include "../core/config_parser.hpp"
#include "../core/message_converter.hpp"
#include "../core/schema_merger.hpp"
#include "../ffi/lance_bridge.h"
#include <ros/ros.h>
#include <ros/param.h>
#include <memory>
#include <unordered_map>
#include <vector>
#include <thread>
#include <atomic>

namespace axon {
namespace ros1 {

class AxonNode {
public:
    AxonNode() : recording_(false), dataset_handle_(0) {}
    
    ~AxonNode() {
        shutdown();
    }
    
    bool initialize(int argc, char** argv) {
        // Initialize ROS interface
        ros_interface_ = common::RosInterfaceFactory::create(common::RosInterfaceFactory::ROS1);
        if (!ros_interface_) {
            std::cerr << "Failed to create ROS interface" << std::endl;
            return false;
        }
        
        if (!ros_interface_->init(argc, argv, "axon")) {
            std::cerr << "Failed to initialize ROS" << std::endl;
            return false;
        }
        
        // Load configuration
        std::string config_path = get_config_path();
        config_ = core::RecorderConfig::from_yaml(config_path);
        
        if (!config_.validate()) {
            ros_interface_->log_error("Invalid configuration");
            return false;
        }
        
        ros_interface_->log_info("Configuration loaded: " + config_.to_string());
        
        // Collect schemas from all topics
        if (!collect_topic_schemas()) {
            ros_interface_->log_error("Failed to collect topic schemas");
            return false;
        }
        
        // Merge schemas and initialize dataset
        if (!initialize_dataset()) {
            ros_interface_->log_error("Failed to initialize dataset");
            return false;
        }
        
        // Setup batch managers for each topic
        for (const auto& topic_config : config_.topics) {
            setup_topic_recording(topic_config);
        }
        
        // Setup services
        setup_services();
        
        return true;
    }
    
    void run() {
        if (config_.recording.auto_start) {
            start_recording();
        }
        
        ros_interface_->spin();
    }
    
    void shutdown() {
        stop_recording();
        
        // Stop all batch managers
        for (auto& pair : batch_managers_) {
            pair.second->stop();
        }
        batch_managers_.clear();
        
        // Unsubscribe all
        for (auto& pair : subscriptions_) {
            ros_interface_->unsubscribe(pair.second);
        }
        subscriptions_.clear();
        
        if (dataset_handle_ > 0) {
            close_dataset(dataset_handle_);
            dataset_handle_ = 0;
        }
        
        ros_interface_->shutdown();
    }
    
private:
    std::unique_ptr<common::RosInterface> ros_interface_;
    core::RecorderConfig config_;
    std::atomic<bool> recording_;
    int64_t dataset_handle_;
    std::string dataset_path_;
    std::shared_ptr<arrow::Schema> merged_schema_;
    std::unordered_map<std::string, std::shared_ptr<arrow::Schema>> topic_schemas_;
    
    std::unordered_map<std::string, std::unique_ptr<core::BatchManager>> batch_managers_;
    std::unordered_map<std::string, void*> subscriptions_;
    std::unordered_map<std::string, std::unique_ptr<core::MessageConverter>> converters_;
    
    std::string get_config_path() {
        // Get from ROS parameter server
        std::string config_path;
        ros::NodeHandle nh;
        
        if (nh.getParam("config_path", config_path)) {
            return config_path;
        }
        
        // Try to get from package path
        std::string package_path = ros::package::getPath("axon");
        if (!package_path.empty()) {
            return package_path + "/config/default_config.yaml";
        }
        
        // Fallback to default
        return "config/default_config.yaml";
    }
    
    bool collect_topic_schemas() {
        // Create converters for all topics to get their schemas
        for (const auto& topic_config : config_.topics) {
            auto converter = core::MessageConverterFactory::create(topic_config.message_type);
            if (!converter) {
                ros_interface_->log_warn("No converter for message type: " + topic_config.message_type);
                continue;
            }
            
            converters_[topic_config.name] = std::move(converter);
            topic_schemas_[topic_config.name] = converters_[topic_config.name]->get_schema();
        }
        
        return !topic_schemas_.empty();
    }
    
    bool initialize_dataset() {
        dataset_path_ = config_.dataset.path;
        
        // Merge schemas from all topics
        merged_schema_ = core::SchemaMerger::merge_schemas(topic_schemas_, true);
        
        if (!merged_schema_) {
            ros_interface_->log_error("Failed to merge schemas");
            return false;
        }
        
        ros_interface_->log_info("Merged schema has " + 
                                 std::to_string(merged_schema_->num_fields()) + " fields");
        
        // Export schema to C interface
        struct ArrowSchema c_schema;
        arrow::Status status = arrow::ExportSchema(*merged_schema_, &c_schema);
        if (!status.ok()) {
            ros_interface_->log_error("Failed to export schema: " + status.ToString());
            return false;
        }
        
        dataset_handle_ = create_or_open_dataset(dataset_path_.c_str(), &c_schema);
        
        // Release schema
        c_schema.release(&c_schema);
        
        if (dataset_handle_ <= 0) {
            const char* error = lance_get_last_error();
            if (error) {
                ros_interface_->log_error("Failed to create dataset: " + std::string(error));
            }
            return false;
        }
        
        return true;
    }
    
    void setup_topic_recording(const core::TopicConfig& topic_config) {
        // Get converter (already created in collect_topic_schemas)
        auto converter_it = converters_.find(topic_config.name);
        if (converter_it == converters_.end()) {
            ros_interface_->log_warn("No converter for topic: " + topic_config.name);
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
                ros_interface_->log_error("Failed to export batch: " + status.ToString());
                return false;
            }
            
            int32_t result = write_batch(handle, &c_array, &c_schema);
            
            // Release C structures
            c_array.release(&c_array);
            c_schema.release(&c_schema);
            
            if (result != LANCE_SUCCESS) {
                const char* error = lance_get_last_error();
                if (error) {
                    ros_interface_->log_error("Write failed: " + std::string(error));
                }
            }
            
            return result == LANCE_SUCCESS;
        };
        
        // Create memory pool for this batch manager (can be shared or per-topic)
        auto memory_pool = arrow::default_memory_pool();
        
        auto batch_mgr = std::make_unique<core::BatchManager>(
            topic_config.batch_size,
            topic_config.flush_interval_ms,
            write_callback,
            memory_pool
        );
        
        // Initialize with merged schema
        batch_mgr->initialize_schema(merged_schema_);
        batch_mgr->set_dataset(dataset_path_, dataset_handle_);
        batch_mgr->start();
        
        batch_managers_[topic_config.name] = std::move(batch_mgr);
        
        // Subscribe to topic
        auto callback = [this, topic_name = topic_config.name](const void* msg_ptr) {
            if (!recording_.load()) {
                return;
            }
            
            auto converter_it = converters_.find(topic_name);
            if (converter_it == converters_.end()) {
                return;
            }
            
            std::vector<std::shared_ptr<arrow::Array>> arrays;
            if (converter_it->second->convert_to_arrow(msg_ptr, arrays)) {
                auto batch_mgr_it = batch_managers_.find(topic_name);
                if (batch_mgr_it != batch_managers_.end()) {
                    // Add timestamp and topic fields
                    int64_t timestamp_ns = ros_interface_->now_nsec();
                    arrow::Int64Builder timestamp_builder;
                    timestamp_builder.Append(timestamp_ns);
                    std::shared_ptr<arrow::Array> timestamp_array;
                    timestamp_builder.Finish(&timestamp_array);
                    
                    arrow::StringBuilder topic_builder;
                    topic_builder.Append(topic_name);
                    std::shared_ptr<arrow::Array> topic_array;
                    topic_builder.Finish(&topic_array);
                    
                    // Prepend timestamp and topic to arrays
                    std::vector<std::shared_ptr<arrow::Array>> full_arrays;
                    full_arrays.reserve(arrays.size() + 2);
                    full_arrays.push_back(timestamp_array);
                    full_arrays.push_back(topic_array);
                    full_arrays.insert(full_arrays.end(), arrays.begin(), arrays.end());
                    
                    batch_mgr_it->second->add_row(full_arrays);
                }
            }
        };
        
        void* sub = ros_interface_->subscribe(
            topic_config.name,
            topic_config.message_type,
            callback
        );
        
        if (sub) {
            subscriptions_[topic_config.name] = sub;
            ros_interface_->log_info("Subscribed to topic: " + topic_config.name);
        } else {
            ros_interface_->log_error("Failed to subscribe to topic: " + topic_config.name);
        }
    }
    
    void setup_services() {
        // Full service implementation would go here
        // For now, services are handled via ROS interface abstraction
        ros_interface_->log_info("Services available via ROS interface");
    }
    
    void start_recording() {
        recording_ = true;
        ros_interface_->log_info("Recording started");
    }
    
    void stop_recording() {
        recording_ = false;
        
        // Flush all batch managers
        for (auto& pair : batch_managers_) {
            pair.second->flush();
        }
        
        ros_interface_->log_info("Recording stopped");
    }
};

} // namespace ros1
} // namespace axon

// Main entry point for ROS 1
int main(int argc, char** argv) {
    axon::ros1::AxonNode node;
    
    if (!node.initialize(argc, argv)) {
        return 1;
    }
    
    node.run();
    node.shutdown();
    
    return 0;
}
