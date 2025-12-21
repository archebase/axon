#ifndef AXON_RECORDER_SERVICE_ADAPTER_HPP
#define AXON_RECORDER_SERVICE_ADAPTER_HPP

#include <memory>
#include <string>

#include "recorder_context.hpp"
#include "recording_service_impl.hpp"

// Forward declarations for ROS interface
namespace axon {
namespace recorder {
class RosInterface;
}  // namespace recorder
}  // namespace axon

// Include generated service headers based on ROS version
#if defined(AXON_ROS1)
#include <axon_recorder/CachedRecordingConfig.h>
#include <axon_recorder/IsRecordingReady.h>
#include <axon_recorder/RecordingControl.h>
#include <axon_recorder/RecordingStatus.h>
#include <ros/ros.h>
#elif defined(AXON_ROS2)
#include <axon_recorder/srv/cached_recording_config.hpp>
#include <axon_recorder/srv/is_recording_ready.hpp>
#include <axon_recorder/srv/recording_control.hpp>
#include <axon_recorder/srv/recording_status.hpp>
#include <rclcpp/rclcpp.hpp>
#endif

namespace axon {
namespace recorder {

/**
 * ServiceAdapter registers and handles the recording services.
 * It adapts between the ROS service API and the RecordingServiceImpl.
 *
 * This class handles the ROS version-specific service registration and
 * message type conversions.
 *
 * Uses IRecorderContext interface for dependency injection, enabling
 * easier testing and better decoupling from RecorderNode.
 */
class ServiceAdapter {
public:
  /**
   * Construct with a shared pointer to the recorder context.
   * @param context Shared pointer to the recorder context (must not be null)
   * @param ros_interface Raw pointer to ROS interface (owned by RecorderNode)
   */
  ServiceAdapter(std::shared_ptr<IRecorderContext> context, RosInterface* ros_interface);
  ~ServiceAdapter();

  /**
   * Register all recording services.
   * Call this after the node is initialized.
   */
  bool register_services();

  /**
   * Shutdown services cleanly.
   */
  void shutdown();

private:
  std::shared_ptr<IRecorderContext> context_;
  RosInterface* ros_interface_;
  std::unique_ptr<RecordingServiceImpl> service_impl_;

#if defined(AXON_ROS1)
  // ROS 1 service servers
  ros::ServiceServer cached_config_server_;
  ros::ServiceServer is_ready_server_;
  ros::ServiceServer control_server_;
  ros::ServiceServer status_server_;

  // ROS 1 service callbacks
  bool handle_cached_recording_config_ros1(
    axon_recorder::CachedRecordingConfig::Request& req,
    axon_recorder::CachedRecordingConfig::Response& res
  );

  bool handle_is_recording_ready_ros1(
    axon_recorder::IsRecordingReady::Request& req, axon_recorder::IsRecordingReady::Response& res
  );

  bool handle_recording_control_ros1(
    axon_recorder::RecordingControl::Request& req, axon_recorder::RecordingControl::Response& res
  );

  bool handle_recording_status_ros1(
    axon_recorder::RecordingStatus::Request& req, axon_recorder::RecordingStatus::Response& res
  );

#elif defined(AXON_ROS2)
  // ROS 2 service servers
  rclcpp::Service<axon_recorder::srv::CachedRecordingConfig>::SharedPtr cached_config_server_;
  rclcpp::Service<axon_recorder::srv::IsRecordingReady>::SharedPtr is_ready_server_;
  rclcpp::Service<axon_recorder::srv::RecordingControl>::SharedPtr control_server_;
  rclcpp::Service<axon_recorder::srv::RecordingStatus>::SharedPtr status_server_;

  // ROS 2 service callbacks
  void handle_cached_recording_config_ros2(
    const std::shared_ptr<axon_recorder::srv::CachedRecordingConfig::Request> req,
    std::shared_ptr<axon_recorder::srv::CachedRecordingConfig::Response> res
  );

  void handle_is_recording_ready_ros2(
    const std::shared_ptr<axon_recorder::srv::IsRecordingReady::Request> req,
    std::shared_ptr<axon_recorder::srv::IsRecordingReady::Response> res
  );

  void handle_recording_control_ros2(
    const std::shared_ptr<axon_recorder::srv::RecordingControl::Request> req,
    std::shared_ptr<axon_recorder::srv::RecordingControl::Response> res
  );

  void handle_recording_status_ros2(
    const std::shared_ptr<axon_recorder::srv::RecordingStatus::Request> req,
    std::shared_ptr<axon_recorder::srv::RecordingStatus::Response> res
  );
#endif
};

}  // namespace recorder
}  // namespace axon

#endif  // AXON_RECORDER_SERVICE_ADAPTER_HPP

