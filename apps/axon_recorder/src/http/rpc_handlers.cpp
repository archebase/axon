// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "rpc_handlers.hpp"

#include <atomic>
#include <vector>

#include "../config/task_config.hpp"
#include "version.hpp"

// Logging infrastructure
#define AXON_LOG_COMPONENT "rpc_handlers"
#include <axon_log_macros.hpp>

namespace axon {
namespace recorder {

namespace {

std::string get_current_state(const RpcCallbacks& callbacks) {
  return callbacks.get_state ? callbacks.get_state() : "unknown";
}

std::string get_last_error(const RpcCallbacks& callbacks) {
  return callbacks.get_last_error ? callbacks.get_last_error() : "";
}

nlohmann::json validation_errors_to_json(const std::vector<ValidationError>& errors) {
  nlohmann::json out = nlohmann::json::array();
  for (const auto& error : errors) {
    out.push_back(error.to_json());
  }
  return out;
}

void require_string_field(
  const nlohmann::json& object, const std::string& field, std::vector<ValidationError>& errors,
  bool required
) {
  if (!object.contains(field)) {
    if (required) {
      errors.push_back({field, "missing_required", field + " is required"});
    }
    return;
  }
  if (!object[field].is_string()) {
    errors.push_back({field, "invalid_type", field + " must be a string"});
    return;
  }
  if (required && object[field].get<std::string>().empty()) {
    errors.push_back({field, "empty", field + " must not be empty"});
  }
}

void validate_string_array_field(
  const nlohmann::json& object, const std::string& field, std::vector<ValidationError>& errors
) {
  if (!object.contains(field)) {
    return;
  }
  if (!object[field].is_array()) {
    errors.push_back({field, "invalid_type", field + " must be an array of strings"});
    return;
  }
  for (size_t i = 0; i < object[field].size(); ++i) {
    if (!object[field][i].is_string()) {
      errors.push_back(
        {field + "[" + std::to_string(i) + "]", "invalid_type", field + " entries must be strings"}
      );
    }
  }
}

std::vector<ValidationError> validate_task_config_json(const nlohmann::json& config_json) {
  std::vector<ValidationError> errors;
  if (!config_json.is_object()) {
    errors.push_back({"task_config", "invalid_type", "task_config must be an object"});
    return errors;
  }

  require_string_field(config_json, "task_id", errors, true);
  require_string_field(config_json, "device_id", errors, true);

  const std::vector<std::string> optional_string_fields = {
    "data_collector_id",
    "order_id",
    "operator_name",
    "scene",
    "subscene",
    "factory",
    "start_callback_url",
    "finish_callback_url",
    "user_token",
  };
  for (const auto& field : optional_string_fields) {
    require_string_field(config_json, field, errors, false);
  }
  validate_string_array_field(config_json, "skills", errors);
  validate_string_array_field(config_json, "topics", errors);

  return errors;
}

nlohmann::json task_config_to_public_json(const TaskConfig& task_config) {
  nlohmann::json config_json;
  config_json["task_id"] = task_config.task_id;
  config_json["device_id"] = task_config.device_id;
  config_json["data_collector_id"] = task_config.data_collector_id;
  config_json["order_id"] = task_config.order_id;
  config_json["scene"] = task_config.scene;
  config_json["subscene"] = task_config.subscene;
  config_json["skills"] = task_config.skills;
  config_json["factory"] = task_config.factory;
  config_json["operator_name"] = task_config.operator_name;
  config_json["topics"] = task_config.topics;
  return config_json;
}

nlohmann::json build_current_task_snapshot(const RpcCallbacks& callbacks) {
  nlohmann::json task = nlohmann::json::object();
  const std::string state = get_current_state(callbacks);
  if (state != "ready" && state != "recording" && state != "paused") {
    return task;
  }
  const TaskConfig* task_config = callbacks.get_task_config ? callbacks.get_task_config() : nullptr;
  if (!task_config || task_config->task_id.empty()) {
    return task;
  }

  task["task_id"] = task_config->task_id;
  task["state"] = state;
  task["active"] = (state == "ready" || state == "recording" || state == "paused");
  task["task_config"] = task_config_to_public_json(*task_config);

  if (callbacks.get_stats) {
    try {
      nlohmann::json stats = callbacks.get_stats();
      task["duration_sec"] = stats.value("duration_sec", 0.0);
      task["message_count"] = stats.value("message_count", stats.value("messages_written", 0));
      task["bytes_written"] = stats.value("bytes_written", 0);
    } catch (...) {
      task["duration_sec"] = 0.0;
      task["message_count"] = 0;
      task["bytes_written"] = 0;
    }
  }

  return task;
}

bool parse_task_ids(
  const RpcCallbacks& callbacks, const nlohmann::json& params, RpcResponse& response,
  std::vector<std::string>& task_ids
) {
  if (!params.contains("task_ids")) {
    response.success = false;
    response.message = "Missing required parameter: task_ids";
    response.data["state"] = get_current_state(callbacks);
    return false;
  }
  if (!params["task_ids"].is_array()) {
    response.success = false;
    response.message = "task_ids must be an array of strings";
    response.data["state"] = get_current_state(callbacks);
    return false;
  }
  if (params["task_ids"].empty()) {
    response.success = false;
    response.message = "task_ids must not be empty";
    response.data["state"] = get_current_state(callbacks);
    return false;
  }
  for (size_t i = 0; i < params["task_ids"].size(); ++i) {
    if (!params["task_ids"][i].is_string()) {
      response.success = false;
      response.message = "task_ids entries must be strings";
      response.data["state"] = get_current_state(callbacks);
      response.data["index"] = i;
      return false;
    }
    task_ids.push_back(params["task_ids"][i].get<std::string>());
  }
  return true;
}

RpcResponse handle_batch_task_action(
  const RpcCallbacks& callbacks, const nlohmann::json& params, const std::string& action
) {
  RpcResponse response;
  std::vector<std::string> task_ids;
  if (!parse_task_ids(callbacks, params, response, task_ids)) {
    return response;
  }

  nlohmann::json results = nlohmann::json::array();
  size_t succeeded = 0;
  for (const auto& task_id : task_ids) {
    RpcResponse item_response;
    if (action == "begin") {
      item_response = handle_rpc_begin(callbacks, {{"task_id", task_id}});
    } else if (action == "finish") {
      item_response = handle_rpc_finish(callbacks, {{"task_id", task_id}});
    } else if (action == "cancel") {
      const TaskConfig* task_config =
        callbacks.get_task_config ? callbacks.get_task_config() : nullptr;
      if (task_config && task_config->task_id == task_id) {
        item_response = handle_rpc_cancel(callbacks, nlohmann::json::object());
      } else {
        item_response.success = false;
        item_response.message = task_config ? "task_id mismatch: expected '" +
                                                task_config->task_id + "' but got '" + task_id + "'"
                                            : "No task configuration cached";
        item_response.data["state"] = get_current_state(callbacks);
      }
    }

    nlohmann::json item;
    item["task_id"] = task_id;
    item["success"] = item_response.success;
    item["message"] = item_response.message;
    item["data"] = item_response.data.is_null() ? nlohmann::json::object() : item_response.data;
    results.push_back(item);
    if (item_response.success) {
      ++succeeded;
    }
  }

  response.success = succeeded == task_ids.size();
  response.message = response.success ? "Batch " + action + " completed successfully"
                                      : "Batch " + action + " completed with errors";
  response.data["results"] = results;
  response.data["succeeded"] = succeeded;
  response.data["failed"] = task_ids.size() - succeeded;
  response.data["state"] = get_current_state(callbacks);
  return response;
}

}  // namespace

RpcResponse handle_rpc_begin(const RpcCallbacks& callbacks, const nlohmann::json& params) {
  RpcResponse response;

  // Check if task_id is provided and is a string
  if (!params.contains("task_id")) {
    response.success = false;
    response.message = "Missing required parameter: task_id";
    response.data["state"] = get_current_state(callbacks);
    return response;
  }

  if (!params["task_id"].is_string()) {
    response.success = false;
    response.message = "task_id must be a string";
    response.data["state"] = get_current_state(callbacks);
    return response;
  }

  std::string provided_task_id = params["task_id"].get<std::string>();

  // Check callback exists
  if (!callbacks.begin_recording) {
    response.success = false;
    response.message = "Begin recording callback not registered";
    return response;
  }

  // Call the callback
  bool success = callbacks.begin_recording(provided_task_id);

  if (success) {
    response.success = true;
    response.message = "Recording started successfully";
    response.data["state"] = get_current_state(callbacks);
    response.data["task_id"] = provided_task_id;
  } else {
    response.success = false;
    const std::string detail = get_last_error(callbacks);
    response.message = detail.empty() ? "Failed to start recording" : detail;
    response.data["state"] = get_current_state(callbacks);
  }

  return response;
}

RpcResponse handle_rpc_pause(const RpcCallbacks& callbacks, const nlohmann::json& params) {
  (void)params;  // Unused
  RpcResponse response;

  if (!callbacks.pause_recording) {
    response.success = false;
    response.message = "Pause recording callback not registered";
    return response;
  }

  bool success = callbacks.pause_recording();

  if (success) {
    response.success = true;
    response.message = "Recording paused successfully";
    response.data["state"] = get_current_state(callbacks);
  } else {
    response.success = false;
    const std::string detail = get_last_error(callbacks);
    response.message = detail.empty() ? "Failed to pause recording" : detail;
    response.data["state"] = get_current_state(callbacks);
  }

  return response;
}

RpcResponse handle_rpc_resume(const RpcCallbacks& callbacks, const nlohmann::json& params) {
  (void)params;  // Unused
  RpcResponse response;

  if (!callbacks.resume_recording) {
    response.success = false;
    response.message = "Resume recording callback not registered";
    return response;
  }

  bool success = callbacks.resume_recording();

  if (success) {
    response.success = true;
    response.message = "Recording resumed successfully";
    response.data["state"] = get_current_state(callbacks);
  } else {
    response.success = false;
    const std::string detail = get_last_error(callbacks);
    response.message = detail.empty() ? "Failed to resume recording" : detail;
    response.data["state"] = get_current_state(callbacks);
  }

  return response;
}

RpcResponse handle_rpc_finish(const RpcCallbacks& callbacks, const nlohmann::json& params) {
  RpcResponse response;

  // Check if task_id is provided and is a string
  if (!params.contains("task_id")) {
    response.success = false;
    response.message = "Missing required parameter: task_id";
    response.data["state"] = get_current_state(callbacks);
    return response;
  }

  if (!params["task_id"].is_string()) {
    response.success = false;
    response.message = "task_id must be a string";
    response.data["state"] = get_current_state(callbacks);
    return response;
  }

  std::string task_id = params["task_id"].get<std::string>();

  if (!callbacks.finish_recording) {
    response.success = false;
    response.message = "Finish recording callback not registered";
    return response;
  }

  bool success = callbacks.finish_recording(task_id);

  if (success) {
    response.success = true;
    response.message = "Recording finished successfully";
    response.data["state"] = get_current_state(callbacks);
    response.data["task_id"] = task_id;
  } else {
    response.success = false;
    const std::string detail = get_last_error(callbacks);
    response.message = detail.empty() ? "Failed to finish recording" : detail;
    response.data["state"] = get_current_state(callbacks);
  }

  return response;
}

RpcResponse handle_rpc_quit(const RpcCallbacks& callbacks, const nlohmann::json& params) {
  (void)params;  // Unused - quit doesn't require task_id

  RpcResponse response;

  // Call the quit callback to signal the main program to exit
  if (callbacks.quit) {
    callbacks.quit();
  }

  response.success = true;
  response.message = "Program quitting. Recording stopped and data saved.";
  response.data["state"] = get_current_state(callbacks);

  return response;
}

RpcResponse handle_rpc_cancel(const RpcCallbacks& callbacks, const nlohmann::json& params) {
  (void)params;  // Unused
  RpcResponse response;

  if (!callbacks.cancel_recording) {
    response.success = false;
    response.message = "Cancel recording callback not registered";
    return response;
  }

  bool success = callbacks.cancel_recording();

  if (success) {
    response.success = true;
    response.message = "Recording cancelled successfully";
    response.data["state"] = get_current_state(callbacks);
  } else {
    response.success = false;
    const std::string detail = get_last_error(callbacks);
    response.message = detail.empty() ? "Failed to cancel recording" : detail;
    response.data["state"] = get_current_state(callbacks);
  }

  return response;
}

RpcResponse handle_rpc_clear(const RpcCallbacks& callbacks, const nlohmann::json& params) {
  (void)params;  // Unused
  RpcResponse response;

  if (!callbacks.clear_config) {
    response.success = false;
    response.message = "Clear config callback not registered";
    return response;
  }

  bool success = callbacks.clear_config();

  if (success) {
    response.success = true;
    response.message = "Configuration cleared successfully";
    response.data["state"] = get_current_state(callbacks);
  } else {
    response.success = false;
    const std::string detail = get_last_error(callbacks);
    response.message = detail.empty() ? "Failed to clear configuration" : detail;
    response.data["state"] = get_current_state(callbacks);
  }

  return response;
}

RpcResponse handle_rpc_get_state(const RpcCallbacks& callbacks, const nlohmann::json& params) {
  (void)params;  // Unused
  RpcResponse response;

  response.success = true;
  response.message = "State retrieved successfully";

  // Add version information
  response.data["version"] = get_version();

  // Get state from callback
  const std::string state = get_current_state(callbacks);
  response.data["state"] = state;

  // Get task config if available (READY, RECORDING, PAUSED states)
  if ((state == "ready" || state == "recording" || state == "paused") &&
      callbacks.get_task_config) {
    const TaskConfig* task_config = callbacks.get_task_config();
    if (task_config) {
      // Note: Don't include sensitive fields like callback URLs and tokens
      response.data["task_config"] = task_config_to_public_json(*task_config);
    }
  }

  // Get stats from callback to check if running and surface the most useful
  // summary numbers for clients that only poll /rpc/state during a recording:
  // elapsed duration, total message count, and bytes written.
  if (callbacks.get_stats) {
    try {
      nlohmann::json stats = callbacks.get_stats();
      response.data["running"] =
        stats.value("messages_written", 0) > 0 || stats.value("messages_received", 0) > 0;
      response.data["duration_sec"] = stats.value("duration_sec", 0.0);
      response.data["message_count"] =
        stats.value("message_count", stats.value("messages_written", 0));
      response.data["bytes_written"] = stats.value("bytes_written", 0);
    } catch (...) {
      response.data["running"] = false;
      response.data["duration_sec"] = 0.0;
      response.data["message_count"] = 0;
      response.data["bytes_written"] = 0;
    }
  } else {
    response.data["running"] = false;
    response.data["duration_sec"] = 0.0;
    response.data["message_count"] = 0;
    response.data["bytes_written"] = 0;
  }

  return response;
}

RpcResponse handle_rpc_get_stats(const RpcCallbacks& callbacks, const nlohmann::json& params) {
  (void)params;  // Unused
  RpcResponse response;

  if (!callbacks.get_stats) {
    response.success = false;
    response.message = "Get stats callback not registered";
    return response;
  }

  try {
    nlohmann::json stats = callbacks.get_stats();
    response.success = true;
    response.message = "Statistics retrieved successfully";
    response.data = stats;
  } catch (const std::exception& e) {
    response.success = false;
    response.message = std::string("Failed to get statistics: ") + e.what();
  }

  return response;
}

RpcResponse handle_rpc_get_status(const RpcCallbacks& callbacks, const nlohmann::json& params) {
  RpcResponse response = handle_rpc_get_stats(callbacks, params);
  if (response.success) {
    response.message = "Status retrieved successfully";
  }
  return response;
}

RpcResponse handle_rpc_get_drop_stats(const RpcCallbacks& callbacks, const nlohmann::json& params) {
  (void)params;  // Unused
  RpcResponse response;

  if (!callbacks.get_drop_stats) {
    response.success = false;
    response.message = "Get drop stats callback not registered";
    return response;
  }

  try {
    nlohmann::json drop_stats = callbacks.get_drop_stats();
    response.success = true;
    response.message = "Drop statistics retrieved successfully";
    response.data = drop_stats;
  } catch (const std::exception& e) {
    response.success = false;
    response.message = std::string("Failed to get drop statistics: ") + e.what();
  }

  return response;
}

RpcResponse handle_rpc_get_latency_stats(
  const RpcCallbacks& callbacks, const nlohmann::json& params
) {
  (void)params;
  RpcResponse response;

  if (!callbacks.get_latency_stats) {
    response.success = false;
    response.message = "Get latency stats callback not registered";
    return response;
  }

  try {
    nlohmann::json latency_stats = callbacks.get_latency_stats();
    response.success = true;
    response.message = "Latency statistics retrieved successfully";
    response.data = latency_stats;
  } catch (const std::exception& e) {
    response.success = false;
    response.message = std::string("Failed to get latency statistics: ") + e.what();
  }

  return response;
}

RpcResponse handle_rpc_config(const RpcCallbacks& callbacks, const nlohmann::json& params) {
  RpcResponse response;

  // Check if task_config is present in params
  if (!params.contains("task_config")) {
    response.success = false;
    response.message = "Missing 'task_config' in request parameters";
    return response;
  }

  if (!callbacks.set_config) {
    response.success = false;
    response.message = "Set config callback not registered";
    return response;
  }

  try {
    const auto& config_json = params["task_config"];
    const auto validation_errors = validate_task_config_json(config_json);
    if (!validation_errors.empty()) {
      response.success = false;
      response.message = "Task configuration validation failed";
      response.data["state"] = get_current_state(callbacks);
      response.data["validation_errors"] = validation_errors_to_json(validation_errors);
      return response;
    }

    // Extract task_id for response
    std::string task_id = config_json["task_id"].get<std::string>();

    // Call the callback with the config
    bool success = callbacks.set_config(task_id, config_json);

    if (success) {
      response.success = true;
      response.message = "Task configuration set successfully";
      response.data["state"] = get_current_state(callbacks);
      if (!task_id.empty()) {
        response.data["task_id"] = task_id;
      }
    } else {
      response.success = false;
      const std::string detail = get_last_error(callbacks);
      response.message = detail.empty() ? "Failed to set task configuration" : detail;
      response.data["state"] = get_current_state(callbacks);
    }

  } catch (const std::exception& e) {
    response.success = false;
    response.message = std::string("Failed to parse task config: ") + e.what();
  }

  return response;
}

RpcResponse handle_rpc_test(const RpcCallbacks& callbacks, const nlohmann::json& params) {
  (void)callbacks;  // Unused - test command doesn't need callbacks
  RpcResponse response;

  response.success = true;
  response.message = "Test command received successfully";

  // Echo back the "echo" parameter if provided
  if (params.contains("echo")) {
    response.data["echo"] = params["echo"];
  } else {
    response.data["echo"] = "pong";
  }

  // Add state info
  response.data["state"] = get_current_state(callbacks);

  return response;
}

RpcResponse handle_rpc_list_tasks(const RpcCallbacks& callbacks, const nlohmann::json& params) {
  (void)params;
  RpcResponse response;
  nlohmann::json tasks = nlohmann::json::array();
  nlohmann::json current_task = build_current_task_snapshot(callbacks);
  if (!current_task.empty()) {
    tasks.push_back(current_task);
  }
  response.success = true;
  response.message = "Tasks retrieved successfully";
  response.data["tasks"] = tasks;
  response.data["count"] = tasks.size();
  response.data["state"] = get_current_state(callbacks);
  return response;
}

RpcResponse handle_rpc_get_task_status(
  const RpcCallbacks& callbacks, const nlohmann::json& params
) {
  RpcResponse response;
  if (!params.contains("task_id")) {
    response.success = false;
    response.message = "Missing required parameter: task_id";
    response.data["state"] = get_current_state(callbacks);
    return response;
  }
  if (!params["task_id"].is_string()) {
    response.success = false;
    response.message = "task_id must be a string";
    response.data["state"] = get_current_state(callbacks);
    return response;
  }

  const std::string task_id = params["task_id"].get<std::string>();
  nlohmann::json current_task = build_current_task_snapshot(callbacks);
  response.success = true;
  response.message = "Task status retrieved successfully";
  response.data["task_id"] = task_id;
  response.data["state"] = get_current_state(callbacks);
  if (!current_task.empty() && current_task.value("task_id", "") == task_id) {
    response.data["found"] = true;
    response.data["task"] = current_task;
  } else {
    response.data["found"] = false;
    response.data["status"] = "unknown";
  }
  return response;
}

RpcResponse handle_rpc_batch_begin(const RpcCallbacks& callbacks, const nlohmann::json& params) {
  return handle_batch_task_action(callbacks, params, "begin");
}

RpcResponse handle_rpc_batch_finish(const RpcCallbacks& callbacks, const nlohmann::json& params) {
  return handle_batch_task_action(callbacks, params, "finish");
}

RpcResponse handle_rpc_batch_cancel(const RpcCallbacks& callbacks, const nlohmann::json& params) {
  return handle_batch_task_action(callbacks, params, "cancel");
}

RpcResponse handle_rpc_get_log_levels(const RpcCallbacks& callbacks, const nlohmann::json& params) {
  (void)params;
  RpcResponse response;
  if (!callbacks.get_log_levels) {
    response.success = false;
    response.message = "Get log levels callback not registered";
    return response;
  }

  response.success = true;
  response.message = "Log levels retrieved successfully";
  response.data = callbacks.get_log_levels();
  return response;
}

RpcResponse handle_rpc_set_log_levels(const RpcCallbacks& callbacks, const nlohmann::json& params) {
  RpcResponse response;
  if (!callbacks.set_log_levels) {
    response.success = false;
    response.message = "Set log levels callback not registered";
    return response;
  }

  nlohmann::json result = nlohmann::json::object();
  if (callbacks.set_log_levels(params, result)) {
    response.success = true;
    response.message = "Log levels updated successfully";
    response.data = result;
  } else {
    response.success = false;
    response.message = result.value("message", "Failed to set log levels");
    response.data = result;
  }
  return response;
}

}  // namespace recorder
}  // namespace axon
