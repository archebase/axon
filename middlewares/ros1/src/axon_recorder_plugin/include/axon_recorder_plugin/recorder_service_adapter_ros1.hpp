#ifndef AXON_RECORDER_PLUGIN_RECORDER_SERVICE_ADAPTER_ROS1_HPP_
#define AXON_RECORDER_PLUGIN_RECORDER_SERVICE_ADAPTER_ROS1_HPP_

#include <axon_utils/recorder_service_interface.hpp>

#include <ros/ros.h>
#include <ros/service_server.h>

#include <memory>
#include <string>

// Forward declarations for generated service messages
namespace axon_recorder {
template<class M>
class ServiceServer;

struct CachedRecordingConfigRequest;
struct CachedRecordingConfigResponse;
struct IsRecordingReadyRequest;
struct IsRecordingReadyResponse;
struct RecordingControlRequest;
struct RecordingControlResponse;
struct RecordingStatusRequest;
struct RecordingStatusResponse;

}  // namespace axon_recorder

namespace axon_recorder_plugin {

/**
 * ROS1 implementation of the recorder service adapter.
 * This plugin registers and handles ROS1 services for the recorder.
 */
class RecorderServiceAdapterROS1 : public axon::utils::IRecorderServiceAdapter {
public:
  /**
   * Construct with a shared pointer to the recorder context.
   * @param context Shared pointer to the recorder context
   * @param ros_node_handle Raw pointer to ROS1 NodeHandle (owned by caller)
   */
  RecorderServiceAdapterROS1(
    std::shared_ptr<axon::utils::IRecorderContext> context, ros::NodeHandle* ros_node_handle
  );

  ~RecorderServiceAdapterROS1() override;

  // Non-copyable
  RecorderServiceAdapterROS1(const RecorderServiceAdapterROS1&) = delete;
  RecorderServiceAdapterROS1& operator=(const RecorderServiceAdapterROS1&) = delete;

  /**
   * Register all recording services with the ROS1 node.
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
   * Internal service handlers - called by ROS1
   */
  bool handle_cached_recording_config_ros1(
    axon_recorder::CachedRecordingConfigRequest& req,
    axon_recorder::CachedRecordingConfigResponse& res
  );

  bool handle_is_recording_ready_ros1(
    axon_recorder::IsRecordingReadyRequest& req, axon_recorder::IsRecordingReadyResponse& res
  );

  bool handle_recording_control_ros1(
    axon_recorder::RecordingControlRequest& req, axon_recorder::RecordingControlResponse& res
  );

  bool handle_recording_status_ros1(
    axon_recorder::RecordingStatusRequest& req, axon_recorder::RecordingStatusResponse& res
  );

  std::shared_ptr<axon::utils::IRecorderContext> context_;
  ros::NodeHandle* ros_node_handle_;

  // ROS1 service servers
  ros::ServiceServer cached_config_server_;
  ros::ServiceServer is_ready_server_;
  ros::ServiceServer control_server_;
  ros::ServiceServer status_server_;
};

// Factory function for creating the ROS1 service adapter
extern "C" {
/**
 * Create ROS1 service adapter instance.
 * This function is called by the recorder core to create the ROS1 plugin.
 *
 * @param context Shared pointer to the recorder context
 * @param ros_node_handle Raw pointer to ROS1 NodeHandle
 * @return Pointer to the created service adapter
 */
RecorderServiceAdapterROS1* create_recorder_service_adapter_ros1(
  std::shared_ptr<axon::utils::IRecorderContext> context, void* ros_node_handle
);

/**
 * Destroy ROS1 service adapter instance.
 */
void destroy_recorder_service_adapter_ros1(RecorderServiceAdapterROS1* adapter);
}

}  // namespace axon_recorder_plugin

#endif  // AXON_RECORDER_PLUGIN_RECORDER_SERVICE_ADAPTER_ROS1_HPP_
