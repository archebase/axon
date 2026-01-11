#include "service_adapter.hpp"

#include <rclcpp/rclcpp.hpp>

#include <stdexcept>

#include "recorder_context.hpp"
#include "ros_interface.hpp"

namespace axon {
namespace recorder {

ServiceAdapter::ServiceAdapter(
  std::shared_ptr<IRecorderContext> context, RosInterface* ros_interface
)
    : context_(std::move(context))
    , ros_interface_(ros_interface)
    , service_impl_(std::make_unique<RecordingServiceImpl>(context_)) {
  if (!context_) {
    throw std::invalid_argument("ServiceAdapter: context cannot be null");
  }
  if (!ros_interface_) {
    throw std::invalid_argument("ServiceAdapter: ros_interface cannot be null");
  }
}

ServiceAdapter::~ServiceAdapter() {
  shutdown();
}

void ServiceAdapter::shutdown() {
  cached_config_server_.reset();
  is_ready_server_.reset();
  control_server_.reset();
  status_server_.reset();
}

bool ServiceAdapter::register_services() {
  // Get the ROS 2 node
  auto* node_ptr = static_cast<rclcpp::Node*>(ros_interface_->get_node_handle());
  if (!node_ptr) {
    ros_interface_->log_error("Failed to get ROS 2 node for service registration");
    return false;
  }

  auto node = node_ptr->shared_from_this();

  // Register services with namespaced names
  cached_config_server_ = node->create_service<axon_recorder::srv::CachedRecordingConfig>(
    "~/cached_recording_config",
    std::bind(
      &ServiceAdapter::handle_cached_recording_config_ros2,
      this,
      std::placeholders::_1,
      std::placeholders::_2
    )
  );

  is_ready_server_ = node->create_service<axon_recorder::srv::IsRecordingReady>(
    "~/is_recording_ready",
    std::bind(
      &ServiceAdapter::handle_is_recording_ready_ros2,
      this,
      std::placeholders::_1,
      std::placeholders::_2
    )
  );

  control_server_ = node->create_service<axon_recorder::srv::RecordingControl>(
    "~/recording_control",
    std::bind(
      &ServiceAdapter::handle_recording_control_ros2,
      this,
      std::placeholders::_1,
      std::placeholders::_2
    )
  );

  status_server_ = node->create_service<axon_recorder::srv::RecordingStatus>(
    "~/recording_status",
    std::bind(
      &ServiceAdapter::handle_recording_status_ros2,
      this,
      std::placeholders::_1,
      std::placeholders::_2
    )
  );

  ros_interface_->log_info("ROS 2 recording services registered");
  return true;
}

// ============================================================================
// ROS 2 Service Callbacks
// ============================================================================

void ServiceAdapter::handle_cached_recording_config_ros2(
  const std::shared_ptr<axon_recorder::srv::CachedRecordingConfig::Request> req,
  std::shared_ptr<axon_recorder::srv::CachedRecordingConfig::Response> res
) {
  service_impl_->handle_cached_recording_config(
    req->task_id,
    req->device_id,
    req->data_collector_id,
    req->order_id,
    req->operator_name,
    req->scene,
    req->subscene,
    std::vector<std::string>(req->skills.begin(), req->skills.end()),
    req->factory,
    std::vector<std::string>(req->topics.begin(), req->topics.end()),
    req->start_callback_url,
    req->finish_callback_url,
    req->user_token,
    res->success,
    res->message
  );
}

void ServiceAdapter::handle_is_recording_ready_ros2(
  const std::shared_ptr<axon_recorder::srv::IsRecordingReady::Request> /* req */,
  std::shared_ptr<axon_recorder::srv::IsRecordingReady::Response> res
) {
  std::vector<std::string> skills;
  std::vector<std::string> topics;

  service_impl_->handle_is_recording_ready(
    res->success,
    res->message,
    res->is_configured,
    res->is_recording,
    res->task_id,
    res->device_id,
    res->order_id,
    res->operator_name,
    res->scene,
    res->subscene,
    skills,
    res->factory,
    res->data_collector_id,
    topics
  );

  // Convert vectors to ROS 2 sequences
  res->skills = skills;
  res->topics = topics;
}

void ServiceAdapter::handle_recording_control_ros2(
  const std::shared_ptr<axon_recorder::srv::RecordingControl::Request> req,
  std::shared_ptr<axon_recorder::srv::RecordingControl::Response> res
) {
  service_impl_->handle_recording_control(
    req->command, req->task_id, res->success, res->message, res->task_id
  );
}

void ServiceAdapter::handle_recording_status_ros2(
  const std::shared_ptr<axon_recorder::srv::RecordingStatus::Request> req,
  std::shared_ptr<axon_recorder::srv::RecordingStatus::Response> res
) {
  std::vector<std::string> skills;
  std::vector<std::string> active_topics;

  service_impl_->handle_recording_status(
    req->task_id,
    res->success,
    res->message,
    res->status,
    res->task_id,
    res->device_id,
    res->data_collector_id,
    res->order_id,
    res->operator_name,
    res->scene,
    res->subscene,
    skills,
    res->factory,
    active_topics,
    res->output_path,
    res->disk_usage_gb,
    res->duration_sec,
    res->message_count,
    res->throughput_mb_sec,
    res->last_error
  );

  // Convert vectors to ROS 2 sequences
  res->skills = skills;
  res->active_topics = active_topics;
}

}  // namespace recorder
}  // namespace axon
