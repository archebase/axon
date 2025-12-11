#ifndef RECORDER_NODE_HPP
#define RECORDER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <arrow/api.h>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <atomic>

namespace lance_recorder {
namespace ros2 {

// Forward declarations
namespace core {
    class BatchManager;
    class MessageConverter;
    class RecorderConfig;
}

class LanceRecorderNode : public rclcpp::Node {
public:
    explicit LanceRecorderNode(const std::string& node_name = "lance_recorder");
    ~LanceRecorderNode();
    
    bool initialize();
    void start_recording();
    void stop_recording();
    
private:
    bool load_configuration();
    bool collect_topic_schemas();
    bool initialize_dataset();
    void setup_topic_recording(const std::string& topic_name, const std::string& message_type);
    void setup_services();
    
    std::shared_ptr<core::RecorderConfig> config_;
    std::atomic<bool> recording_;
    int64_t dataset_handle_;
    std::string dataset_path_;
    std::shared_ptr<arrow::Schema> merged_schema_;
    std::unordered_map<std::string, std::shared_ptr<arrow::Schema>> topic_schemas_;
    
    std::unordered_map<std::string, std::unique_ptr<core::BatchManager>> batch_managers_;
    std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
    std::unordered_map<std::string, std::unique_ptr<core::MessageConverter>> converters_;
};

} // namespace ros2
} // namespace lance_recorder

#endif // RECORDER_NODE_HPP

