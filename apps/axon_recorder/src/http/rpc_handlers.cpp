// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "rpc_handlers.hpp"

#include <atomic>

#include "../config/task_config.hpp"
#include "version.hpp"

// Logging infrastructure
#define AXON_LOG_COMPONENT "rpc_handlers"
#include <axon_log_macros.hpp>

namespace axon {
namespace recorder {

RpcResponse handle_rpc_begin(const RpcCallbacks& callbacks, const nlohmann::json& params) {
  RpcResponse response;

  // Check if task_id is provided and is a string
  if (!params.contains("task_id")) {
    response.success = false;
    response.message = "Missing required parameter: task_id";
    response.data["state"] = callbacks.get_state ? callbacks.get_state() : "unknown";
    return response;
  }

  if (!params["task_id"].is_string()) {
    response.success = false;
    response.message = "task_id must be a string";
    response.data["state"] = callbacks.get_state ? callbacks.get_state() : "unknown";
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
    response.data["state"] = callbacks.get_state ? callbacks.get_state() : "unknown";
    response.data["task_id"] = provided_task_id;
  } else {
    response.success = false;
    response.message = "Failed to start recording";
    response.data["state"] = callbacks.get_state ? callbacks.get_state() : "unknown";
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
    response.data["state"] = callbacks.get_state ? callbacks.get_state() : "unknown";
  } else {
    response.success = false;
    response.message = "Failed to pause recording";
    response.data["state"] = callbacks.get_state ? callbacks.get_state() : "unknown";
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
    response.data["state"] = callbacks.get_state ? callbacks.get_state() : "unknown";
  } else {
    response.success = false;
    response.message = "Failed to resume recording";
    response.data["state"] = callbacks.get_state ? callbacks.get_state() : "unknown";
  }

  return response;
}

RpcResponse handle_rpc_finish(const RpcCallbacks& callbacks, const nlohmann::json& params) {
  RpcResponse response;

  // Check if task_id is provided and is a string
  if (!params.contains("task_id")) {
    response.success = false;
    response.message = "Missing required parameter: task_id";
    response.data["state"] = callbacks.get_state ? callbacks.get_state() : "unknown";
    return response;
  }

  if (!params["task_id"].is_string()) {
    response.success = false;
    response.message = "task_id must be a string";
    response.data["state"] = callbacks.get_state ? callbacks.get_state() : "unknown";
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
    response.data["state"] = callbacks.get_state ? callbacks.get_state() : "unknown";
    response.data["task_id"] = task_id;
  } else {
    response.success = false;
    response.message = "Failed to finish recording";
    response.data["state"] = callbacks.get_state ? callbacks.get_state() : "unknown";
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
  response.data["state"] = callbacks.get_state ? callbacks.get_state() : "unknown";

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
    response.data["state"] = callbacks.get_state ? callbacks.get_state() : "unknown";
  } else {
    response.success = false;
    response.message = "Failed to cancel recording";
    response.data["state"] = callbacks.get_state ? callbacks.get_state() : "unknown";
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
    response.data["state"] = callbacks.get_state ? callbacks.get_state() : "unknown";
  } else {
    response.success = false;
    response.message = "Failed to clear configuration";
    response.data["state"] = callbacks.get_state ? callbacks.get_state() : "unknown";
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
  if (callbacks.get_state) {
    response.data["state"] = callbacks.get_state();
  } else {
    response.data["state"] = "unknown";
  }

  // Get task config if available (READY, RECORDING, PAUSED states)
  if (callbacks.get_task_config) {
    const TaskConfig* task_config = callbacks.get_task_config();
    if (task_config) {
      nlohmann::json config_json;
      config_json["task_id"] = task_config->task_id;
      config_json["device_id"] = task_config->device_id;
      config_json["data_collector_id"] = task_config->data_collector_id;
      config_json["scene"] = task_config->scene;
      config_json["subscene"] = task_config->subscene;
      config_json["skills"] = task_config->skills;
      config_json["factory"] = task_config->factory;
      config_json["operator_name"] = task_config->operator_name;
      config_json["topics"] = task_config->topics;
      // Note: Don't include sensitive fields like callback URLs and tokens
      response.data["task_config"] = config_json;
    }
  }

  // Get stats from callback to check if running
  if (callbacks.get_stats) {
    try {
      nlohmann::json stats = callbacks.get_stats();
      response.data["running"] =
        stats.value("messages_written", 0) > 0 || stats.value("messages_received", 0) > 0;
    } catch (...) {
      response.data["running"] = false;
    }
  } else {
    response.data["running"] = false;
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

    // Extract task_id for response
    std::string task_id;
    if (config_json.contains("task_id")) {
      if (!config_json["task_id"].is_string()) {
        response.success = false;
        response.message = "task_id must be a string";
        response.data["state"] = callbacks.get_state ? callbacks.get_state() : "unknown";
        return response;
      }
      task_id = config_json["task_id"].get<std::string>();
    }

    // Call the callback with the config
    bool success = callbacks.set_config(task_id, config_json);

    if (success) {
      response.success = true;
      response.message = "Task configuration set successfully";
      response.data["state"] = callbacks.get_state ? callbacks.get_state() : "unknown";
      if (!task_id.empty()) {
        response.data["task_id"] = task_id;
      }
    } else {
      response.success = false;
      response.message = "Failed to set task configuration";
      response.data["state"] = callbacks.get_state ? callbacks.get_state() : "unknown";
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
  response.data["state"] = callbacks.get_state ? callbacks.get_state() : "unknown";

  return response;
}

}  // namespace recorder
}  // namespace axon
