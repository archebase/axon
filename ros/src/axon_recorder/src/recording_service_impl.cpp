#include "recording_service_impl.hpp"

#include <chrono>
#include <stdexcept>

#include "recorder_context.hpp"

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
  auto& state_manager = context_->get_state_manager();
  if (!state_manager.is_state(RecorderState::IDLE)) {
    message = "ERR_INVALID_STATE: Cannot cache config in state " + state_manager.get_state_string();
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

  // Cache the configuration
  context_->get_task_config_cache().cache(config);

  // Transition to READY state
  std::string error_msg;
  if (!state_manager.transition_to(RecorderState::READY, error_msg)) {
    // Rollback - clear the config
    context_->get_task_config_cache().clear();
    context_->log_warn("State transition to READY failed: " + error_msg);
    message = error_msg;
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

  auto& state_manager = context_->get_state_manager();
  auto& task_cache = context_->get_task_config_cache();

  // Check state
  RecorderState state = state_manager.get_state();
  is_configured =
    (state == RecorderState::READY || state == RecorderState::RECORDING ||
     state == RecorderState::PAUSED);
  is_recording = (state == RecorderState::RECORDING || state == RecorderState::PAUSED);

  // Populate config details if available
  auto config_opt = task_cache.get();
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
  auto config_opt = context_->get_task_config_cache().get();
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
  auto& state_manager = context_->get_state_manager();

  // Must be in READY state
  if (!state_manager.is_state(RecorderState::READY)) {
    message =
      "ERR_INVALID_STATE: Cannot start recording in state " + state_manager.get_state_string();
    return false;
  }

  // Get cached config
  auto config_opt = context_->get_task_config_cache().get();
  if (!config_opt) {
    message = "ERR_CONFIG_NOT_FOUND: No configuration cached";
    return false;
  }

  task_id_response = config_opt->task_id;

  // Configure from task config (update output path, etc.)
  if (!context_->configure_from_task_config(*config_opt)) {
    message = "ERR_INVALID_STATE: Failed to configure from task config";
    return false;
  }

  // Transition to RECORDING state
  std::string error_msg;
  if (!state_manager.transition_to(RecorderState::RECORDING, error_msg)) {
    context_->log_warn("State transition to RECORDING failed: " + error_msg);
    message = error_msg;
    return false;
  }

  // Start recording
  context_->start_recording();

  message = "Recording started for task: " + task_id_response;
  return true;
}

bool RecordingServiceImpl::handle_pause_command(const std::string& task_id, std::string& message) {
  auto& state_manager = context_->get_state_manager();

  // Validate task_id
  std::string error_msg;
  if (!validate_task_id(task_id, error_msg)) {
    message = error_msg;
    return false;
  }

  // Must be in RECORDING state
  if (!state_manager.is_state(RecorderState::RECORDING)) {
    message = "ERR_INVALID_STATE: Cannot pause in state " + state_manager.get_state_string();
    return false;
  }

  // Transition to PAUSED state
  if (!state_manager.transition_to(RecorderState::PAUSED, error_msg)) {
    context_->log_warn("State transition to PAUSED failed: " + error_msg);
    message = error_msg;
    return false;
  }

  // Pause recording
  context_->pause_recording();

  message = "Recording paused";
  return true;
}

bool RecordingServiceImpl::handle_resume_command(const std::string& task_id, std::string& message) {
  auto& state_manager = context_->get_state_manager();

  // Validate task_id
  std::string error_msg;
  if (!validate_task_id(task_id, error_msg)) {
    message = error_msg;
    return false;
  }

  // Must be in PAUSED state
  if (!state_manager.is_state(RecorderState::PAUSED)) {
    message = "ERR_INVALID_STATE: Cannot resume in state " + state_manager.get_state_string();
    return false;
  }

  // Transition to RECORDING state
  if (!state_manager.transition_to(RecorderState::RECORDING, error_msg)) {
    context_->log_warn("State transition to RECORDING (resume) failed: " + error_msg);
    message = error_msg;
    return false;
  }

  // Resume recording
  context_->resume_recording();

  message = "Recording resumed";
  return true;
}

bool RecordingServiceImpl::handle_cancel_command(const std::string& task_id, std::string& message) {
  auto& state_manager = context_->get_state_manager();

  // Validate task_id
  std::string error_msg;
  if (!validate_task_id(task_id, error_msg)) {
    message = error_msg;
    return false;
  }

  // Must be in RECORDING or PAUSED state
  if (!state_manager.is_recording_active()) {
    message = "ERR_INVALID_STATE: Cannot cancel in state " + state_manager.get_state_string();
    return false;
  }

  // Cancel recording (sends callback and clears config)
  context_->cancel_recording();

  // Transition to IDLE state
  if (!state_manager.transition_to(RecorderState::IDLE, error_msg)) {
    context_->log_warn("State transition to IDLE (cancel) failed: " + error_msg);
    message = error_msg;
    return false;
  }

  message = "Recording cancelled";
  return true;
}

bool RecordingServiceImpl::handle_finish_command(const std::string& task_id, std::string& message) {
  auto& state_manager = context_->get_state_manager();

  // Validate task_id
  std::string error_msg;
  if (!validate_task_id(task_id, error_msg)) {
    message = error_msg;
    return false;
  }

  // Must be in RECORDING or PAUSED state
  if (!state_manager.is_recording_active()) {
    message = "ERR_INVALID_STATE: Cannot finish in state " + state_manager.get_state_string();
    return false;
  }

  // Stop recording (sends callback and clears config)
  context_->stop_recording();

  // Transition to IDLE state
  if (!state_manager.transition_to(RecorderState::IDLE, error_msg)) {
    context_->log_warn("State transition to IDLE (finish) failed: " + error_msg);
    message = error_msg;
    return false;
  }

  message = "Recording finished";
  return true;
}

bool RecordingServiceImpl::handle_clear_command(std::string& message) {
  auto& state_manager = context_->get_state_manager();

  // Must be in READY state
  if (!state_manager.is_state(RecorderState::READY)) {
    message = "ERR_INVALID_STATE: Cannot clear in state " + state_manager.get_state_string();
    return false;
  }

  // Clear the cached config
  context_->get_task_config_cache().clear();

  // Transition to IDLE state
  std::string error_msg;
  if (!state_manager.transition_to(RecorderState::IDLE, error_msg)) {
    context_->log_warn("State transition to IDLE (clear) failed: " + error_msg);
    message = error_msg;
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

  auto& state_manager = context_->get_state_manager();
  status = state_manager.get_state_string();

  // Get cached config if available
  auto config_opt = context_->get_task_config_cache().get();
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

  // Get runtime metrics from RecorderNode
  auto stats = context_->get_stats();
  output_path = context_->get_output_path();
  message_count = static_cast<int64_t>(stats.messages_written);
  last_error = "";  // Would need error tracking in RecorderNode

  // Calculate duration if recording
  duration_sec = context_->get_recording_duration_sec();

  // Disk usage and throughput would need additional tracking
  disk_usage_gb = 0.0;
  throughput_mb_sec = 0.0;

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

  auto& task_cache = context_->get_task_config_cache();
  if (!task_cache.matches_task_id(task_id)) {
    error_msg = "ERR_RECORDING_NOT_FOUND: task_id does not match current task";
    return false;
  }

  return true;
}

}  // namespace recorder
}  // namespace axon
