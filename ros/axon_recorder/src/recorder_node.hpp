#ifndef AXON_RECORDER_NODE_HPP
#define AXON_RECORDER_NODE_HPP

#include "ros_interface.hpp"
#include "batch_manager.hpp"
#include "config_parser.hpp"
#include "message_converter.hpp"

#include <arrow/api.h>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <atomic>

namespace axon {
namespace recorder {

/**
 * Main recorder node for Axon
 * Works with both ROS 1 and ROS 2 via the RosInterface abstraction
 */
class RecorderNode {
public:
    RecorderNode();
    ~RecorderNode();
    
    /**
     * Initialize the recorder node
     */
    bool initialize(int argc, char** argv);
    
    /**
     * Run the recorder (blocking)
     */
    void run();
    
    /**
     * Shutdown the recorder
     */
    void shutdown();
    
    /**
     * Start recording
     */
    void start_recording();
    
    /**
     * Stop recording
     */
    void stop_recording();
    
    /**
     * Check if recording is active
     */
    bool is_recording() const { return recording_.load(); }
    
private:
    bool load_configuration();
    bool collect_topic_schemas();
    bool initialize_dataset();
    void setup_topic_recording(const core::TopicConfig& topic_config);
    void setup_services();
    
    std::string get_config_path();
    
    std::unique_ptr<RosInterface> ros_interface_;
    core::RecorderConfig config_;
    std::atomic<bool> recording_;
    int64_t dataset_handle_;
    std::string dataset_path_;
    
    std::shared_ptr<arrow::Schema> merged_schema_;
    std::unordered_map<std::string, std::shared_ptr<arrow::Schema>> topic_schemas_;
    std::unordered_map<std::string, std::unique_ptr<core::BatchManager>> batch_managers_;
    std::unordered_map<std::string, void*> subscriptions_;
    std::unordered_map<std::string, std::unique_ptr<core::MessageConverter>> converters_;
};

} // namespace recorder
} // namespace axon

#endif // AXON_RECORDER_NODE_HPP
