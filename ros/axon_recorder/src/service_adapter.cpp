#include "service_adapter.hpp"

#include <stdexcept>

#include "recorder_context.hpp"
#include "ros_interface.hpp"

namespace axon {
namespace recorder {

ServiceAdapter::ServiceAdapter(std::shared_ptr<IRecorderContext> context, RosInterface* ros_interface)
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
#if defined(AXON_ROS1)
  cached_config_server_ = ros::ServiceServer();
  is_ready_server_ = ros::ServiceServer();
  control_server_ = ros::ServiceServer();
  status_server_ = ros::ServiceServer();
#elif defined(AXON_ROS2)
  cached_config_server_.reset();
  is_ready_server_.reset();
  control_server_.reset();
  status_server_.reset();
#endif
}

bool ServiceAdapter::register_services() {
#if defined(AXON_ROS1)
  // Get the ROS 1 node handle
  ros::NodeHandle* nh = static_cast<ros::NodeHandle*>(ros_interface_->get_node_handle());
  if (!nh) {
    ros_interface_->log_error("Failed to get ROS 1 node handle for service registration");
    return false;
  }

  // Create a private NodeHandle for services under the node's namespace
  // In ROS 1, we can't use "~" prefix directly with advertiseService()
  ros::NodeHandle private_nh("~");

  // Register services using the private NodeHandle
  cached_config_server_ = private_nh.advertiseService(
    "cached_recording_config", &ServiceAdapter::handle_cached_recording_config_ros1, this
  );

  is_ready_server_ =
    private_nh.advertiseService("is_recording_ready", &ServiceAdapter::handle_is_recording_ready_ros1, this);

  control_server_ =
    private_nh.advertiseService("recording_control", &ServiceAdapter::handle_recording_control_ros1, this);

  status_server_ =
    private_nh.advertiseService("recording_status", &ServiceAdapter::handle_recording_status_ros1, this);

  ros_interface_->log_info("ROS 1 recording services registered");
  return true;

#elif defined(AXON_ROS2)
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
      &ServiceAdapter::handle_cached_recording_config_ros2, this, std::placeholders::_1,
      std::placeholders::_2
    )
  );

  is_ready_server_ = node->create_service<axon_recorder::srv::IsRecordingReady>(
    "~/is_recording_ready",
    std::bind(
      &ServiceAdapter::handle_is_recording_ready_ros2, this, std::placeholders::_1,
      std::placeholders::_2
    )
  );

  control_server_ = node->create_service<axon_recorder::srv::RecordingControl>(
    "~/recording_control",
    std::bind(
      &ServiceAdapter::handle_recording_control_ros2, this, std::placeholders::_1,
      std::placeholders::_2
    )
  );

  status_server_ = node->create_service<axon_recorder::srv::RecordingStatus>(
    "~/recording_status",
    std::bind(
      &ServiceAdapter::handle_recording_status_ros2, this, std::placeholders::_1, std::placeholders::_2
    )
  );

  ros_interface_->log_info("ROS 2 recording services registered");
  return true;

#else
  ros_interface_->log_error("No ROS version defined - cannot register services");
  return false;
#endif
}

// ============================================================================
// ROS 1 Service Callbacks
// ============================================================================

#if defined(AXON_ROS1)

bool ServiceAdapter::handle_cached_recording_config_ros1(
  axon_recorder::CachedRecordingConfig::Request& req,
  axon_recorder::CachedRecordingConfig::Response& res
) {
  // Use local variables because ROS 1 message fields may be bit-fields
  // that can't be passed as non-const lvalue references
  bool success = false;
  std::string message;
  bool result = service_impl_->handle_cached_recording_config(
    req.task_id, req.device_id, req.data_collector_id, req.order_id, req.operator_name,
    req.scene, req.subscene, req.skills, req.factory, req.topics,
    req.start_callback_url, req.finish_callback_url, req.user_token,
    success, message
  );
  res.success = success;
  res.message = message;
  return result;
}

bool ServiceAdapter::handle_is_recording_ready_ros1(
  axon_recorder::IsRecordingReady::Request& /* req */,
  axon_recorder::IsRecordingReady::Response& res
) {
  // Use local variables because ROS 1 message fields may be bit-fields
  bool success = false;
  std::string message;
  bool is_configured = false;
  bool is_recording = false;
  std::string task_id, device_id, order_id, operator_name, scene, subscene, factory, data_collector_id;
  std::vector<std::string> skills, topics;
  
  bool result = service_impl_->handle_is_recording_ready(
    success, message, is_configured, is_recording, task_id, device_id,
    order_id, operator_name, scene, subscene, skills, factory, data_collector_id, topics
  );
  
  res.success = success;
  res.message = message;
  res.is_configured = is_configured;
  res.is_recording = is_recording;
  res.task_id = task_id;
  res.device_id = device_id;
  res.order_id = order_id;
  res.operator_name = operator_name;
  res.scene = scene;
  res.subscene = subscene;
  res.skills = skills;
  res.factory = factory;
  res.data_collector_id = data_collector_id;
  res.topics = topics;
  return result;
}

bool ServiceAdapter::handle_recording_control_ros1(
  axon_recorder::RecordingControl::Request& req, axon_recorder::RecordingControl::Response& res
) {
  // Use local variables because ROS 1 message fields may be bit-fields
  bool success = false;
  std::string message;
  std::string task_id_response;
  
  bool result = service_impl_->handle_recording_control(
    req.command, req.task_id, success, message, task_id_response
  );
  
  res.success = success;
  res.message = message;
  res.task_id = task_id_response;
  return result;
}

bool ServiceAdapter::handle_recording_status_ros1(
  axon_recorder::RecordingStatus::Request& req, axon_recorder::RecordingStatus::Response& res
) {
  // Use local variables because ROS 1 message fields may be bit-fields
  bool success = false;
  std::string message, status, task_id, device_id, data_collector_id, order_id, operator_name;
  std::string scene, subscene, factory, output_path, last_error;
  std::vector<std::string> skills, active_topics;
  double disk_usage_gb = 0.0, duration_sec = 0.0, throughput_mb_sec = 0.0;
  int64_t message_count = 0;
  
  bool result = service_impl_->handle_recording_status(
    req.task_id, success, message, status, task_id, device_id,
    data_collector_id, order_id, operator_name, scene, subscene, skills, factory, active_topics,
    output_path, disk_usage_gb, duration_sec, message_count, throughput_mb_sec,
    last_error
  );
  
  res.success = success;
  res.message = message;
  res.status = status;
  res.task_id = task_id;
  res.device_id = device_id;
  res.data_collector_id = data_collector_id;
  res.order_id = order_id;
  res.operator_name = operator_name;
  res.scene = scene;
  res.subscene = subscene;
  res.skills = skills;
  res.factory = factory;
  res.active_topics = active_topics;
  res.output_path = output_path;
  res.disk_usage_gb = disk_usage_gb;
  res.duration_sec = duration_sec;
  res.message_count = message_count;
  res.throughput_mb_sec = throughput_mb_sec;
  res.last_error = last_error;
  return result;
}

#endif  // AXON_ROS1

// ============================================================================
// ROS 2 Service Callbacks
// ============================================================================

#if defined(AXON_ROS2)

void ServiceAdapter::handle_cached_recording_config_ros2(
  const std::shared_ptr<axon_recorder::srv::CachedRecordingConfig::Request> req,
  std::shared_ptr<axon_recorder::srv::CachedRecordingConfig::Response> res
) {
  service_impl_->handle_cached_recording_config(
    req->task_id, req->device_id, req->data_collector_id, req->order_id, req->operator_name,
    req->scene, req->subscene,
    std::vector<std::string>(req->skills.begin(), req->skills.end()), req->factory,
    std::vector<std::string>(req->topics.begin(), req->topics.end()), req->start_callback_url,
    req->finish_callback_url, req->user_token,
    res->success, res->message
  );
}

void ServiceAdapter::handle_is_recording_ready_ros2(
  const std::shared_ptr<axon_recorder::srv::IsRecordingReady::Request> /* req */,
  std::shared_ptr<axon_recorder::srv::IsRecordingReady::Response> res
) {
  std::vector<std::string> skills;
  std::vector<std::string> topics;

  service_impl_->handle_is_recording_ready(
    res->success, res->message, res->is_configured, res->is_recording, res->task_id, res->device_id,
    res->order_id, res->operator_name, res->scene, res->subscene, skills, res->factory, res->data_collector_id, topics
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
    req->task_id, res->success, res->message, res->status, res->task_id, res->device_id,
    res->data_collector_id, res->order_id, res->operator_name, res->scene, res->subscene, skills, res->factory, active_topics,
    res->output_path, res->disk_usage_gb, res->duration_sec, res->message_count, res->throughput_mb_sec,
    res->last_error
  );

  // Convert vectors to ROS 2 sequences
  res->skills = skills;
  res->active_topics = active_topics;
}

#endif  // AXON_ROS2

}  // namespace recorder
}  // namespace axon

