#include "ros2_plugin/service_adapter.hpp"

// Include TaskConfig definition from apps/axon_recorder
// This is needed because we use TaskConfig in the implementation
#include "../../../../apps/axon_recorder/src/common_types.hpp"

namespace ros2_plugin {

ServiceAdapter::ServiceAdapter(
  std::shared_ptr<axon::utils::IRecorderContext> context, rclcpp::Node* ros_node
)
    : context_(context)
    , ros_node_(ros_node) {}

ServiceAdapter::~ServiceAdapter() {
  shutdown();
}

bool ServiceAdapter::register_services() {
  if (!ros_node_) {
    return false;
  }

  // Create service servers
  cached_config_server_ = ros_node_->create_service<ros2_plugin::srv::CachedRecordingConfig>(
    "axon_recorder/cached_recording_config",
    [this](
      const std::shared_ptr<ros2_plugin::srv::CachedRecordingConfig::Request> req,
      std::shared_ptr<ros2_plugin::srv::CachedRecordingConfig::Response>
        res
    ) {
      handle_cached_recording_config_internal(req, res);
    }
  );

  is_ready_server_ = ros_node_->create_service<ros2_plugin::srv::IsRecordingReady>(
    "axon_recorder/is_recording_ready",
    [this](
      const std::shared_ptr<ros2_plugin::srv::IsRecordingReady::Request> req,
      std::shared_ptr<ros2_plugin::srv::IsRecordingReady::Response>
        res
    ) {
      handle_is_recording_ready_internal(req, res);
    }
  );

  control_server_ = ros_node_->create_service<ros2_plugin::srv::RecordingControl>(
    "axon_recorder/recording_control",
    [this](
      const std::shared_ptr<ros2_plugin::srv::RecordingControl::Request> req,
      std::shared_ptr<ros2_plugin::srv::RecordingControl::Response>
        res
    ) {
      handle_recording_control_internal(req, res);
    }
  );

  status_server_ = ros_node_->create_service<ros2_plugin::srv::RecordingStatus>(
    "axon_recorder/recording_status",
    [this](
      const std::shared_ptr<ros2_plugin::srv::RecordingStatus::Request> req,
      std::shared_ptr<ros2_plugin::srv::RecordingStatus::Response>
        res
    ) {
      handle_recording_status_internal(req, res);
    }
  );

  return true;
}

void ServiceAdapter::shutdown() {
  // Services will be automatically destroyed when shared_ptr goes out of scope
  cached_config_server_.reset();
  is_ready_server_.reset();
  control_server_.reset();
  status_server_.reset();
}

// ============================================================================
// Service Handler Implementations
// ============================================================================

bool ServiceAdapter::handle_cached_recording_config(
  const std::string& task_id, const std::string& device_id, const std::string& data_collector_id,
  const std::string& order_id, const std::string& operator_name, const std::string& scene,
  const std::string& subscene, const std::vector<std::string>& skills, const std::string& factory,
  const std::vector<std::string>& topics, const std::string& start_callback_url,
  const std::string& finish_callback_url, const std::string& user_token, bool& success,
  std::string& message
) {
  // Create task config
  axon::recorder::TaskConfig config;
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

bool ServiceAdapter::handle_is_recording_ready(
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

bool ServiceAdapter::handle_recording_control(
  const std::string& command, const std::string& task_id_request, bool& success,
  std::string& message, std::string& task_id_response
) {
  (void)task_id_request;  // Unused parameter - task_id is obtained from cached config
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

bool ServiceAdapter::handle_recording_status(
  const std::string& task_id_request, bool& success, std::string& message, std::string& status,
  std::string& task_id, std::string& device_id, std::string& data_collector_id,
  std::string& order_id, std::string& operator_name, std::string& scene, std::string& subscene,
  std::vector<std::string>& skills, std::string& factory, std::vector<std::string>& active_topics,
  std::string& output_path, double& disk_usage_gb, double& duration_sec, int64_t& message_count,
  double& throughput_mb_sec, std::string& last_error
) {
  (void)task_id_request;  // Unused parameter - task_id is obtained from cached config
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
// Internal Service Handlers (called by ROS2)
// ============================================================================

void ServiceAdapter::handle_cached_recording_config_internal(
  const std::shared_ptr<ros2_plugin::srv::CachedRecordingConfig::Request> req,
  std::shared_ptr<ros2_plugin::srv::CachedRecordingConfig::Response> res
) {
  handle_cached_recording_config(
    req->task_id,
    req->device_id,
    req->data_collector_id,
    req->order_id,
    req->operator_name,
    req->scene,
    req->subscene,
    req->skills,
    req->factory,
    req->topics,
    req->start_callback_url,
    req->finish_callback_url,
    req->user_token,
    res->success,
    res->message
  );
}

void ServiceAdapter::handle_is_recording_ready_internal(
  const std::shared_ptr<ros2_plugin::srv::IsRecordingReady::Request> req,
  std::shared_ptr<ros2_plugin::srv::IsRecordingReady::Response> res
) {
  (void)req;  // Unused parameter - queries from context, not request
  handle_is_recording_ready(
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
    res->skills,
    res->factory,
    res->data_collector_id,
    res->topics
  );
}

void ServiceAdapter::handle_recording_control_internal(
  const std::shared_ptr<ros2_plugin::srv::RecordingControl::Request> req,
  std::shared_ptr<ros2_plugin::srv::RecordingControl::Response> res
) {
  handle_recording_control(req->command, req->task_id, res->success, res->message, res->task_id);
}

void ServiceAdapter::handle_recording_status_internal(
  const std::shared_ptr<ros2_plugin::srv::RecordingStatus::Request> req,
  std::shared_ptr<ros2_plugin::srv::RecordingStatus::Response> res
) {
  handle_recording_status(
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
    res->skills,
    res->factory,
    res->active_topics,
    res->output_path,
    res->disk_usage_gb,
    res->duration_sec,
    res->message_count,
    res->throughput_mb_sec,
    res->last_error
  );
}

// ============================================================================
// Factory Functions
// ============================================================================

extern "C" {

ServiceAdapter* create_recorder_service_adapter_ros2(
  std::shared_ptr<axon::utils::IRecorderContext> context, void* ros_node
) {
  auto* node = static_cast<rclcpp::Node*>(ros_node);
  return new ServiceAdapter(context, node);
}

void destroy_recorder_service_adapter_ros2(ServiceAdapter* adapter) {
  delete adapter;
}
}

}  // namespace ros2_plugin
