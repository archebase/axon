// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_RECORDER_RPC_HANDLERS_HPP
#define AXON_RECORDER_RPC_HANDLERS_HPP

#include <nlohmann/json.hpp>

#include <functional>
#include <string>

namespace axon {
namespace recorder {

/**
 * Forward declaration
 */
struct TaskConfig;

/**
 * RPC response structure (matches HTTP RPC format)
 */
struct RpcResponse {
  bool success = true;
  std::string message;
  nlohmann::json data;

  nlohmann::json to_json() const {
    nlohmann::json j;
    j["success"] = success;
    j["message"] = message;
    if (!data.is_null()) {
      j["data"] = data;
    }
    return j;
  }
};

/**
 * Callback function types for RPC operations
 * These match the HttpServer::Callbacks but are defined here for shared use
 */
using GetStateCallback = std::function<std::string()>;
using GetStatsCallback = std::function<nlohmann::json()>;
using GetDropStatsCallback = std::function<nlohmann::json()>;
using GetLatencyStatsCallback = std::function<nlohmann::json()>;
using GetTaskConfigCallback = std::function<const TaskConfig*()>;
using GetLastErrorCallback = std::function<std::string()>;
using SetConfigCallback = std::function<bool(const std::string&, const nlohmann::json&)>;
using BeginRecordingCallback = std::function<bool(const std::string&)>;
using FinishRecordingCallback = std::function<bool(const std::string&)>;
using CancelRecordingCallback = std::function<bool()>;
using PauseRecordingCallback = std::function<bool()>;
using ResumeRecordingCallback = std::function<bool()>;
using ClearConfigCallback = std::function<bool()>;
using GetLogLevelsCallback = std::function<nlohmann::json()>;
using SetLogLevelsCallback = std::function<bool(const nlohmann::json&, nlohmann::json&)>;
using QuitCallback = std::function<void()>;

/**
 * Field-level validation error returned by control-plane APIs.
 */
struct ValidationError {
  std::string field;
  std::string code;
  std::string message;

  nlohmann::json to_json() const {
    return {
      {"field", field},
      {"code", code},
      {"message", message},
    };
  }
};

/**
 * Callback registration structure for RPC handlers
 */
struct RpcCallbacks {
  GetStateCallback get_state;
  GetStatsCallback get_stats;
  GetDropStatsCallback get_drop_stats;
  GetLatencyStatsCallback get_latency_stats;
  GetTaskConfigCallback get_task_config;
  GetLastErrorCallback get_last_error;
  SetConfigCallback set_config;
  BeginRecordingCallback begin_recording;
  FinishRecordingCallback finish_recording;
  CancelRecordingCallback cancel_recording;
  PauseRecordingCallback pause_recording;
  ResumeRecordingCallback resume_recording;
  ClearConfigCallback clear_config;
  GetLogLevelsCallback get_log_levels;
  SetLogLevelsCallback set_log_levels;
  QuitCallback quit;
};

/**
 * Shared RPC handler functions
 * These take callbacks reference and params, return RpcResponse
 * Can be called by both HTTP and WebSocket handlers
 */

/**
 * Handle RPC begin command
 * @param callbacks RPC callbacks to invoke
 * @param params Request parameters (must contain task_id)
 * @return RPC response
 */
RpcResponse handle_rpc_begin(const RpcCallbacks& callbacks, const nlohmann::json& params);

/**
 * Handle RPC finish command
 * @param callbacks RPC callbacks to invoke
 * @param params Request parameters (must contain task_id)
 * @return RPC response
 */
RpcResponse handle_rpc_finish(const RpcCallbacks& callbacks, const nlohmann::json& params);

/**
 * Handle RPC pause command
 * @param callbacks RPC callbacks to invoke
 * @param params Request parameters
 * @return RPC response
 */
RpcResponse handle_rpc_pause(const RpcCallbacks& callbacks, const nlohmann::json& params);

/**
 * Handle RPC resume command
 * @param callbacks RPC callbacks to invoke
 * @param params Request parameters
 * @return RPC response
 */
RpcResponse handle_rpc_resume(const RpcCallbacks& callbacks, const nlohmann::json& params);

/**
 * Handle RPC cancel command
 * @param callbacks RPC callbacks to invoke
 * @param params Request parameters
 * @return RPC response
 */
RpcResponse handle_rpc_cancel(const RpcCallbacks& callbacks, const nlohmann::json& params);

/**
 * Handle RPC clear command
 * @param callbacks RPC callbacks to invoke
 * @param params Request parameters
 * @return RPC response
 */
RpcResponse handle_rpc_clear(const RpcCallbacks& callbacks, const nlohmann::json& params);

/**
 * Handle RPC config command
 * @param callbacks RPC callbacks to invoke
 * @param params Request parameters (must contain task_config)
 * @return RPC response
 */
RpcResponse handle_rpc_config(const RpcCallbacks& callbacks, const nlohmann::json& params);

/**
 * Handle RPC quit command
 * @param callbacks RPC callbacks to invoke
 * @param params Request parameters
 * @return RPC response
 */
RpcResponse handle_rpc_quit(const RpcCallbacks& callbacks, const nlohmann::json& params);

/**
 * Handle RPC get_state command
 * @param callbacks RPC callbacks to invoke
 * @param params Request parameters
 * @return RPC response
 */
RpcResponse handle_rpc_get_state(const RpcCallbacks& callbacks, const nlohmann::json& params);

/**
 * Handle RPC get_stats command
 * @param callbacks RPC callbacks to invoke
 * @param params Request parameters
 * @return RPC response
 */
RpcResponse handle_rpc_get_stats(const RpcCallbacks& callbacks, const nlohmann::json& params);

/**
 * Handle RPC get_status command.
 * Returns the same operational snapshot as get_stats, including disk usage.
 * @param callbacks RPC callbacks to invoke
 * @param params Request parameters
 * @return RPC response
 */
RpcResponse handle_rpc_get_status(const RpcCallbacks& callbacks, const nlohmann::json& params);

/**
 * Handle RPC get_drop_stats command
 * Returns per-topic drop statistics and current recording state.
 * @param callbacks RPC callbacks to invoke
 * @param params Request parameters (unused)
 * @return RPC response with drop statistics
 */
RpcResponse handle_rpc_get_drop_stats(const RpcCallbacks& callbacks, const nlohmann::json& params);

/**
 * Handle RPC get_latency_stats command
 * Returns per-topic latency statistics including percentiles and anomaly counts.
 * @param callbacks RPC callbacks to invoke
 * @param params Request parameters (unused)
 * @return RPC response with latency statistics
 */
RpcResponse handle_rpc_get_latency_stats(
  const RpcCallbacks& callbacks, const nlohmann::json& params
);

/**
 * Handle RPC list_tasks command.
 * Returns the recorder's current cached/running task snapshot.
 * @param callbacks RPC callbacks to invoke
 * @param params Request parameters (unused)
 * @return RPC response with tasks array
 */
RpcResponse handle_rpc_list_tasks(const RpcCallbacks& callbacks, const nlohmann::json& params);

/**
 * Handle RPC get_task_status command.
 * @param callbacks RPC callbacks to invoke
 * @param params Request parameters (must contain task_id)
 * @return RPC response with task status
 */
RpcResponse handle_rpc_get_task_status(const RpcCallbacks& callbacks, const nlohmann::json& params);

/**
 * Handle RPC batch begin command.
 * @param callbacks RPC callbacks to invoke
 * @param params Request parameters (must contain task_ids array)
 * @return RPC response with per-task results
 */
RpcResponse handle_rpc_batch_begin(const RpcCallbacks& callbacks, const nlohmann::json& params);

/**
 * Handle RPC batch finish command.
 * @param callbacks RPC callbacks to invoke
 * @param params Request parameters (must contain task_ids array)
 * @return RPC response with per-task results
 */
RpcResponse handle_rpc_batch_finish(const RpcCallbacks& callbacks, const nlohmann::json& params);

/**
 * Handle RPC batch cancel command.
 * @param callbacks RPC callbacks to invoke
 * @param params Request parameters (must contain task_ids array)
 * @return RPC response with per-task results
 */
RpcResponse handle_rpc_batch_cancel(const RpcCallbacks& callbacks, const nlohmann::json& params);

/**
 * Handle RPC get_log_levels command.
 * @param callbacks RPC callbacks to invoke
 * @param params Request parameters (unused)
 * @return RPC response with console/file log levels
 */
RpcResponse handle_rpc_get_log_levels(const RpcCallbacks& callbacks, const nlohmann::json& params);

/**
 * Handle RPC set_log_levels command.
 * @param callbacks RPC callbacks to invoke
 * @param params Request parameters with console_level/file_level
 * @return RPC response with updated levels or validation errors
 */
RpcResponse handle_rpc_set_log_levels(const RpcCallbacks& callbacks, const nlohmann::json& params);

/**
 * Handle RPC test command (connectivity check)
 * @param callbacks RPC callbacks to invoke
 * @param params Request parameters (optional "echo" parameter)
 * @return RPC response with echoed data
 */
RpcResponse handle_rpc_test(const RpcCallbacks& callbacks, const nlohmann::json& params);

}  // namespace recorder
}  // namespace axon

#endif  // AXON_RECORDER_RPC_HANDLERS_HPP
