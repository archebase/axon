#ifndef ROS2_PLUGIN_SERVICE_ADAPTER_HPP_
#define ROS2_PLUGIN_SERVICE_ADAPTER_HPP_

#include <axon_utils/recorder_service_interface.hpp>

#include <rclcpp/rclcpp.hpp>
#include <ros2_plugin/srv/cached_recording_config.hpp>
#include <ros2_plugin/srv/is_recording_ready.hpp>
#include <ros2_plugin/srv/recording_control.hpp>
#include <ros2_plugin/srv/recording_status.hpp>

#include <memory>
#include <string>

namespace ros2_plugin {

/**
 * ROS2 implementation of the recorder service adapter.
 * This plugin registers and handles ROS2 services for the recorder.
 */
class ServiceAdapter : public axon::utils::IRecorderServiceAdapter {
public:
  /**
   * Construct with a shared pointer to the recorder context.
   * @param context Shared pointer to the recorder context
   * @param ros_node Raw pointer to ROS2 node (owned by caller)
   */
  ServiceAdapter(std::shared_ptr<axon::utils::IRecorderContext> context, rclcpp::Node* ros_node);

  ~ServiceAdapter() override;

  // Non-copyable
  ServiceAdapter(const ServiceAdapter&) = delete;
  ServiceAdapter& operator=(const ServiceAdapter&) = delete;

  /**
   * Register all recording services with the ROS2 node.
   */
  bool register_services() override;

  /**
   * Shutdown services cleanly.
   */
  void shutdown() override;

  // ==========================================================================
  // Service Handler Implementations
  // ==========================================================================

  bool handle_cached_recording_config(
    // Request fields
    const std::string& task_id, const std::string& device_id, const std::string& data_collector_id,
    const std::string& order_id, const std::string& operator_name, const std::string& scene,
    const std::string& subscene, const std::vector<std::string>& skills, const std::string& factory,
    const std::vector<std::string>& topics, const std::string& start_callback_url,
    const std::string& finish_callback_url, const std::string& user_token,
    // Response fields
    bool& success, std::string& message
  ) override;

  bool handle_is_recording_ready(
    // Response fields
    bool& success, std::string& message, bool& is_configured, bool& is_recording,
    std::string& task_id, std::string& device_id, std::string& order_id, std::string& operator_name,
    std::string& scene, std::string& subscene, std::vector<std::string>& skills,
    std::string& factory, std::string& data_collector_id, std::vector<std::string>& topics
  ) override;

  bool handle_recording_control(
    // Request fields
    const std::string& command, const std::string& task_id_request,
    // Response fields
    bool& success, std::string& message, std::string& task_id_response
  ) override;

  bool handle_recording_status(
    // Request fields
    const std::string& task_id_request,
    // Response fields
    bool& success, std::string& message, std::string& status, std::string& task_id,
    std::string& device_id, std::string& data_collector_id, std::string& order_id,
    std::string& operator_name, std::string& scene, std::string& subscene,
    std::vector<std::string>& skills, std::string& factory, std::vector<std::string>& active_topics,
    std::string& output_path, double& disk_usage_gb, double& duration_sec, int64_t& message_count,
    double& throughput_mb_sec, std::string& last_error
  ) override;

private:
  /**
   * Internal service handlers - called by ROS2
   */
  void handle_cached_recording_config_internal(
    const std::shared_ptr<ros2_plugin::srv::CachedRecordingConfig::Request> req,
    std::shared_ptr<ros2_plugin::srv::CachedRecordingConfig::Response> res
  );

  void handle_is_recording_ready_internal(
    const std::shared_ptr<ros2_plugin::srv::IsRecordingReady::Request> req,
    std::shared_ptr<ros2_plugin::srv::IsRecordingReady::Response> res
  );

  void handle_recording_control_internal(
    const std::shared_ptr<ros2_plugin::srv::RecordingControl::Request> req,
    std::shared_ptr<ros2_plugin::srv::RecordingControl::Response> res
  );

  void handle_recording_status_internal(
    const std::shared_ptr<ros2_plugin::srv::RecordingStatus::Request> req,
    std::shared_ptr<ros2_plugin::srv::RecordingStatus::Response> res
  );

  std::shared_ptr<axon::utils::IRecorderContext> context_;
  rclcpp::Node* ros_node_;

  // ROS2 service servers
  rclcpp::Service<ros2_plugin::srv::CachedRecordingConfig>::SharedPtr cached_config_server_;
  rclcpp::Service<ros2_plugin::srv::IsRecordingReady>::SharedPtr is_ready_server_;
  rclcpp::Service<ros2_plugin::srv::RecordingControl>::SharedPtr control_server_;
  rclcpp::Service<ros2_plugin::srv::RecordingStatus>::SharedPtr status_server_;
};

// Factory function for creating the ROS2 service adapter
extern "C" {
/**
 * Create ROS2 service adapter instance.
 * This function is called by the recorder core to create the ROS2 plugin.
 *
 * @param context Shared pointer to the recorder context
 * @param ros_node Raw pointer to ROS2 node
 * @return Pointer to the created service adapter
 */
ServiceAdapter* create_recorder_service_adapter_ros2(
  std::shared_ptr<axon::utils::IRecorderContext> context, void* ros_node
);

/**
 * Destroy ROS2 service adapter instance.
 */
void destroy_recorder_service_adapter_ros2(ServiceAdapter* adapter);
}

}  // namespace ros2_plugin

#endif  // ROS2_PLUGIN_SERVICE_ADAPTER_HPP_
