#include "axon_recorder_plugin/recorder_service_adapter_ros1.hpp"

#include <axon_recorder/CachedRecordingConfig.h>
#include <axon_recorder/IsRecordingReady.h>
#include <axon_recorder/RecordingControl.h>
#include <axon_recorder/RecordingStatus.h>

namespace axon_recorder_plugin {

RecorderServiceAdapterROS1::RecorderServiceAdapterROS1(
  std::shared_ptr<axon::utils::IRecorderContext> context, ros::NodeHandle* ros_node_handle
)
    : context_(context)
    , ros_node_handle_(ros_node_handle) {}

RecorderServiceAdapterROS1::~RecorderServiceAdapterROS1() {
  shutdown();
}

bool RecorderServiceAdapterROS1::register_services() {
  if (!ros_node_handle_) {
    return false;
  }

  // Create service servers
  cached_config_server_ = ros_node_handle_->advertiseService(
    "axon_recorder/cached_recording_config",
    &RecorderServiceAdapterROS1::handle_cached_recording_config_ros1,
    this
  );

  is_ready_server_ = ros_node_handle_->advertiseService(
    "axon_recorder/is_recording_ready",
    &RecorderServiceAdapterROS1::handle_is_recording_ready_ros1,
    this
  );

  control_server_ = ros_node_handle_->advertiseService(
    "axon_recorder/recording_control",
    &RecorderServiceAdapterROS1::handle_recording_control_ros1,
    this
  );

  status_server_ = ros_node_handle_->advertiseService(
    "axon_recorder/recording_status",
    &RecorderServiceAdapterROS1::handle_recording_status_ros1,
    this
  );

  return true;
}

void RecorderServiceAdapterROS1::shutdown() {
  // Services will be automatically shut down when NodeHandle is destroyed
  cached_config_server_.shutdown();
  is_ready_server_.shutdown();
  control_server_.shutdown();
  status_server_.shutdown();
}

// ============================================================================
// Service Handler Implementations
// ============================================================================

bool RecorderServiceAdapterROS1::handle_cached_recording_config(
  const std::string& task_id, const std::string& device_id, const std::string& data_collector_id,
  const std::string& order_id, const std::string& operator_name, const std::string& scene,
  const std::string& subscene, const std::vector<std::string>& skills, const std::string& factory,
  const std::vector<std::string>& topics, const std::string& start_callback_url,
  const std::string& finish_callback_url, const std::string& user_token, bool& success,
  std::string& message
) {
  // Create task config
  axon::utils::TaskConfig config;
  config.task_id = task_id;
  config.device_id = device_id;
  config.data_collector_id = data_collector_id;
  config.order_id = order_id;
  config.operator_name = operator_name;
  config.scene = scene;
  config.subscene = subscene;
  config.skills = skills;
  config.factory = factory;
  config.topics = topics;
  config.start_callback_url = start_callback_url;
  config.finish_callback_url = finish_callback_url;
  config.user_token = user_token;

  // Cache the configuration
  std::string error_msg;
  success = context_->cache_task_config(config, error_msg);

  if (success) {
    message = "Configuration cached successfully. Ready to start recording.";
  } else {
    message = "Failed to cache configuration: " + error_msg;
  }

  return success;
}

bool RecorderServiceAdapterROS1::handle_is_recording_ready(
  bool& success, std::string& message, bool& is_configured, bool& is_recording,
  std::string& task_id, std::string& device_id, std::string& order_id, std::string& operator_name,
  std::string& scene, std::string& subscene, std::vector<std::string>& skills, std::string& factory,
  std::string& data_collector_id, std::vector<std::string>& topics
) {
  success = true;
  message = "Query executed successfully.";

  // Check if recording is active
  is_recording = context_->is_recording_active();

  // Get cached config
  auto cached_config = context_->get_cached_config();
  is_configured = cached_config.has_value();

  if (is_configured) {
    // Populate response fields
    task_id = cached_config->task_id;
    device_id = cached_config->device_id;
    data_collector_id = cached_config->data_collector_id;
    order_id = cached_config->order_id;
    operator_name = cached_config->operator_name;
    scene = cached_config->scene;
    subscene = cached_config->subscene;
    skills = cached_config->skills;
    factory = cached_config->factory;
    topics = cached_config->topics;
  }

  return true;
}

bool RecorderServiceAdapterROS1::handle_recording_control(
  const std::string& command, const std::string& task_id_request, bool& success,
  std::string& message, std::string& task_id_response
) {
  std::string error_msg;

  if (command == "start") {
    success = context_->start_recording(error_msg);
  } else if (command == "pause") {
    success = context_->pause_recording(error_msg);
  } else if (command == "resume") {
    success = context_->resume_recording(error_msg);
  } else if (command == "cancel") {
    success = context_->cancel_recording(error_msg);
  } else if (command == "finish") {
    success = context_->finish_recording(error_msg);
  } else if (command == "clear") {
    success = context_->clear_config(error_msg);
  } else {
    success = false;
    error_msg = "Unknown command: " + command;
  }

  if (success) {
    message = "Command '" + command + "' executed successfully.";
  } else {
    message = "Command '" + command + "' failed: " + error_msg;
  }

  // Get task ID from cached config
  auto cached_config = context_->get_cached_config();
  task_id_response = cached_config ? cached_config->task_id : "";

  return success;
}

bool RecorderServiceAdapterROS1::handle_recording_status(
  const std::string& task_id_request, bool& success, std::string& message, std::string& status,
  std::string& task_id, std::string& device_id, std::string& data_collector_id,
  std::string& order_id, std::string& operator_name, std::string& scene, std::string& subscene,
  std::vector<std::string>& skills, std::string& factory, std::vector<std::string>& active_topics,
  std::string& output_path, double& disk_usage_gb, double& duration_sec, int64_t& message_count,
  double& throughput_mb_sec, std::string& last_error
) {
  success = true;
  message = "Status retrieved successfully.";

  // Get state
  status = context_->get_state_string();

  // Get cached config
  auto cached_config = context_->get_cached_config();
  if (cached_config) {
    task_id = cached_config->task_id;
    device_id = cached_config->device_id;
    data_collector_id = cached_config->data_collector_id;
    order_id = cached_config->order_id;
    operator_name = cached_config->operator_name;
    scene = cached_config->scene;
    subscene = cached_config->subscene;
    skills = cached_config->skills;
    factory = cached_config->factory;
    active_topics = cached_config->topics;
  }

  // Get recording metrics
  context_->get_recording_metrics(
    output_path, disk_usage_gb, duration_sec, message_count, throughput_mb_sec, last_error
  );

  return true;
}

// ============================================================================
// Internal Service Handlers (called by ROS1)
// ============================================================================

bool RecorderServiceAdapterROS1::handle_cached_recording_config_ros1(
  axon_recorder::CachedRecordingConfigRequest& req,
  axon_recorder::CachedRecordingConfigResponse& res
) {
  return handle_cached_recording_config(
    req.task_id,
    req.device_id,
    req.data_collector_id,
    req.order_id,
    req.operator_name,
    req.scene,
    req.subscene,
    req.skills,
    req.factory,
    req.topics,
    req.start_callback_url,
    req.finish_callback_url,
    req.user_token,
    res.success,
    res.message
  );
}

bool RecorderServiceAdapterROS1::handle_is_recording_ready_ros1(
  axon_recorder::IsRecordingReadyRequest& req, axon_recorder::IsRecordingReadyResponse& res
) {
  return handle_is_recording_ready(
    res.success,
    res.message,
    res.is_configured,
    res.is_recording,
    res.task_id,
    res.device_id,
    res.order_id,
    res.operator_name,
    res.scene,
    res.subscene,
    res.skills,
    res.factory,
    res.data_collector_id,
    res.topics
  );
}

bool RecorderServiceAdapterROS1::handle_recording_control_ros1(
  axon_recorder::RecordingControlRequest& req, axon_recorder::RecordingControlResponse& res
) {
  return handle_recording_control(req.command, req.task_id, res.success, res.message, res.task_id);
}

bool RecorderServiceAdapterROS1::handle_recording_status_ros1(
  axon_recorder::RecordingStatusRequest& req, axon_recorder::RecordingStatusResponse& res
) {
  return handle_recording_status(
    req.task_id,
    res.success,
    res.message,
    res.status,
    res.task_id,
    res.device_id,
    res.data_collector_id,
    res.order_id,
    res.operator_name,
    res.scene,
    res.subscene,
    res.skills,
    res.factory,
    res.active_topics,
    res.output_path,
    res.disk_usage_gb,
    res.duration_sec,
    res.message_count,
    res.throughput_mb_sec,
    res.last_error
  );
}

// ============================================================================
// Factory Functions
// ============================================================================

extern "C" {

RecorderServiceAdapterROS1* create_recorder_service_adapter_ros1(
  std::shared_ptr<axon::utils::IRecorderContext> context, void* ros_node_handle
) {
  auto* nh = static_cast<ros::NodeHandle*>(ros_node_handle);
  return new RecorderServiceAdapterROS1(context, nh);
}

void destroy_recorder_service_adapter_ros1(RecorderServiceAdapterROS1* adapter) {
  delete adapter;
}
}

}  // namespace axon_recorder_plugin
