#include "recording_service_impl.hpp"

#include <chrono>
#include <stdexcept>

#include "recorder_adapter.hpp"

namespace axon {
namespace recorder {

RecordingServiceImpl::RecordingServiceImpl(std::shared_ptr<IRecorderContext> context)
    : context_(std::move(context)) {
  if (!context_) {
    throw std::invalid_argument("RecordingServiceImpl: context cannot be null");
  }
}

// ============================================================================
// CachedRecordingConfig Service
// ============================================================================

bool RecordingServiceImpl::handle_cached_recording_config(
  const std::string& task_id, const std::string& device_id, const std::string& data_collector_id,
  const std::string& order_id, const std::string& operator_name, const std::string& scene,
  const std::string& subscene, const std::vector<std::string>& skills, const std::string& factory,
  const std::vector<std::string>& topics, const std::string& start_callback_url,
  const std::string& finish_callback_url, const std::string& user_token, bool& success,
  std::string& message
) {
  success = false;

  // Validate required fields
  if (task_id.empty()) {
    message = "ERR_INVALID_COMMAND: task_id is required";
    return true;
  }

  // Check current state - must be IDLE to cache new config
  std::string current_state = context_->get_state_string();
  if (current_state != "IDLE") {
    message = "ERR_INVALID_STATE: Cannot cache config in state " + current_state;
    return true;
  }

  // Build TaskConfig
  TaskConfig config;
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

  // Cache the configuration using interface method
  std::string error_msg;
  if (!context_->cache_task_config(config, error_msg)) {
    message = "ERR_INVALID_STATE: " + error_msg;
    return true;
  }

  success = true;
  message = "Config cached for task: " + task_id;
  return true;
}

// ============================================================================
// IsRecordingReady Service
// ============================================================================

bool RecordingServiceImpl::handle_is_recording_ready(
  bool& success, std::string& message, bool& is_configured, bool& is_recording,
  std::string& task_id, std::string& device_id, std::string& order_id, std::string& operator_name,
  std::string& scene, std::string& subscene, std::vector<std::string>& skills, std::string& factory,
  std::string& data_collector_id, std::vector<std::string>& topics
) {
  success = true;
  message = "OK";

  // Check state using interface methods
  std::string current_state = context_->get_state_string();
  is_configured =
    (current_state == "READY" || current_state == "RECORDING" || current_state == "PAUSED");
  is_recording = context_->is_recording_active();

  // Populate config details if available
  auto config_opt = context_->get_cached_config();
  if (config_opt) {
    task_id = config_opt->task_id;
    device_id = config_opt->device_id;
    order_id = config_opt->order_id;
    operator_name = config_opt->operator_name;
    scene = config_opt->scene;
    subscene = config_opt->subscene;
    skills = config_opt->skills;
    factory = config_opt->factory;
    data_collector_id = config_opt->data_collector_id;
    topics = config_opt->topics;
  } else {
    task_id.clear();
    device_id.clear();
    order_id.clear();
    operator_name.clear();
    scene.clear();
    subscene.clear();
    skills.clear();
    factory.clear();
    data_collector_id.clear();
    topics.clear();
  }

  return true;
}

// ============================================================================
// RecordingControl Service
// ============================================================================

bool RecordingServiceImpl::handle_recording_control(
  const std::string& command, const std::string& task_id_request, bool& success,
  std::string& message, std::string& task_id_response
) {
  success = false;

  // Get current task_id for response
  auto config_opt = context_->get_cached_config();
  task_id_response = config_opt ? config_opt->task_id : "";

  // Dispatch based on command
  if (command == "start") {
    success = handle_start_command(message, task_id_response);
  } else if (command == "pause") {
    success = handle_pause_command(task_id_request, message);
  } else if (command == "resume") {
    success = handle_resume_command(task_id_request, message);
  } else if (command == "cancel") {
    success = handle_cancel_command(task_id_request, message);
  } else if (command == "finish") {
    success = handle_finish_command(task_id_request, message);
  } else if (command == "clear") {
    success = handle_clear_command(message);
  } else {
    message = "ERR_INVALID_COMMAND: Unknown command: " + command;
  }

  return true;
}

bool RecordingServiceImpl::handle_start_command(
  std::string& message, std::string& task_id_response
) {
  // Must be in READY state
  std::string current_state = context_->get_state_string();
  if (current_state != "READY") {
    message = "ERR_INVALID_STATE: Cannot start recording in state " + current_state;
    return false;
  }

  // Get cached config
  auto config_opt = context_->get_cached_config();
  if (!config_opt) {
    message = "ERR_CONFIG_NOT_FOUND: No configuration cached";
    return false;
  }

  task_id_response = config_opt->task_id;

  // Start recording using interface method
  std::string error_msg;
  if (!context_->start_recording(error_msg)) {
    message = "ERR_INVALID_STATE: " + error_msg;
    return false;
  }

  message = "Recording started for task: " + task_id_response;
  return true;
}

bool RecordingServiceImpl::handle_pause_command(const std::string& task_id, std::string& message) {
  // Validate task_id
  std::string error_msg;
  if (!validate_task_id(task_id, error_msg)) {
    message = error_msg;
    return false;
  }

  // Must be in RECORDING state
  std::string current_state = context_->get_state_string();
  if (current_state != "RECORDING") {
    message = "ERR_INVALID_STATE: Cannot pause in state " + current_state;
    return false;
  }

  // Pause recording using interface method
  std::string pause_error;
  if (!context_->pause_recording(pause_error)) {
    message = "ERR_INVALID_STATE: " + pause_error;
    return false;
  }

  message = "Recording paused";
  return true;
}

bool RecordingServiceImpl::handle_resume_command(const std::string& task_id, std::string& message) {
  // Validate task_id
  std::string error_msg;
  if (!validate_task_id(task_id, error_msg)) {
    message = error_msg;
    return false;
  }

  // Must be in PAUSED state
  std::string current_state = context_->get_state_string();
  if (current_state != "PAUSED") {
    message = "ERR_INVALID_STATE: Cannot resume in state " + current_state;
    return false;
  }

  // Resume recording using interface method
  std::string resume_error;
  if (!context_->resume_recording(resume_error)) {
    message = "ERR_INVALID_STATE: " + resume_error;
    return false;
  }

  message = "Recording resumed";
  return true;
}

bool RecordingServiceImpl::handle_cancel_command(const std::string& task_id, std::string& message) {
  // Validate task_id
  std::string error_msg;
  if (!validate_task_id(task_id, error_msg)) {
    message = error_msg;
    return false;
  }

  // Must be in RECORDING or PAUSED state
  if (!context_->is_recording_active()) {
    std::string current_state = context_->get_state_string();
    message = "ERR_INVALID_STATE: Cannot cancel in state " + current_state;
    return false;
  }

  // Cancel recording using interface method
  std::string cancel_error;
  if (!context_->cancel_recording(cancel_error)) {
    message = "ERR_INVALID_STATE: " + cancel_error;
    return false;
  }

  message = "Recording cancelled";
  return true;
}

bool RecordingServiceImpl::handle_finish_command(const std::string& task_id, std::string& message) {
  // Validate task_id
  std::string error_msg;
  if (!validate_task_id(task_id, error_msg)) {
    message = error_msg;
    return false;
  }

  // Must be in RECORDING or PAUSED state
  if (!context_->is_recording_active()) {
    std::string current_state = context_->get_state_string();
    message = "ERR_INVALID_STATE: Cannot finish in state " + current_state;
    return false;
  }

  // Finish recording using interface method
  std::string finish_error;
  if (!context_->finish_recording(finish_error)) {
    message = "ERR_INVALID_STATE: " + finish_error;
    return false;
  }

  message = "Recording finished";
  return true;
}

bool RecordingServiceImpl::handle_clear_command(std::string& message) {
  // Must be in READY state
  std::string current_state = context_->get_state_string();
  if (current_state != "READY") {
    message = "ERR_INVALID_STATE: Cannot clear in state " + current_state;
    return false;
  }

  // Clear the cached config using interface method
  std::string error_msg;
  if (!context_->clear_config(error_msg)) {
    message = "ERR_INVALID_STATE: " + error_msg;
    return false;
  }

  message = "Config cleared";
  return true;
}

// ============================================================================
// RecordingStatus Service
// ============================================================================

bool RecordingServiceImpl::handle_recording_status(
  const std::string& task_id_request, bool& success, std::string& message, std::string& status,
  std::string& task_id, std::string& device_id, std::string& data_collector_id,
  std::string& order_id, std::string& operator_name, std::string& scene, std::string& subscene,
  std::vector<std::string>& skills, std::string& factory, std::vector<std::string>& active_topics,
  std::string& output_path, double& disk_usage_gb, double& duration_sec, int64_t& message_count,
  double& throughput_mb_sec, std::string& last_error
) {
  success = true;
  message = "OK";

  // Get current state
  status = context_->get_state_string();

  // Get cached config if available
  auto config_opt = context_->get_cached_config();
  if (config_opt) {
    task_id = config_opt->task_id;
    device_id = config_opt->device_id;
    data_collector_id = config_opt->data_collector_id;
    order_id = config_opt->order_id;
    operator_name = config_opt->operator_name;
    scene = config_opt->scene;
    subscene = config_opt->subscene;
    skills = config_opt->skills;
    factory = config_opt->factory;
    active_topics = config_opt->topics;
  } else {
    task_id.clear();
    device_id.clear();
    data_collector_id.clear();
    order_id.clear();
    operator_name.clear();
    scene.clear();
    subscene.clear();
    skills.clear();
    factory.clear();
    active_topics.clear();
  }

  // Get runtime metrics using interface method
  if (!context_->get_recording_metrics(
        output_path, disk_usage_gb, duration_sec, message_count, throughput_mb_sec, last_error
      )) {
    // If metrics unavailable, set defaults
    output_path.clear();
    disk_usage_gb = 0.0;
    duration_sec = 0.0;
    message_count = 0;
    throughput_mb_sec = 0.0;
    last_error.clear();
  }

  return true;
}

// ============================================================================
// Helpers
// ============================================================================

bool RecordingServiceImpl::validate_task_id(const std::string& task_id, std::string& error_msg) {
  if (task_id.empty()) {
    error_msg = "ERR_INVALID_COMMAND: task_id is required";
    return false;
  }

  auto config_opt = context_->get_cached_config();
  if (!config_opt || config_opt->task_id != task_id) {
    error_msg = "ERR_RECORDING_NOT_FOUND: task_id does not match current task";
    return false;
  }

  return true;
}

}  // namespace recorder
}  // namespace axon
