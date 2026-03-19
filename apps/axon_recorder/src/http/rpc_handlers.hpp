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
using GetTaskConfigCallback = std::function<const TaskConfig*()>;
using SetConfigCallback = std::function<bool(const std::string&, const nlohmann::json&)>;
using BeginRecordingCallback = std::function<bool(const std::string&)>;
using FinishRecordingCallback = std::function<bool(const std::string&)>;
using CancelRecordingCallback = std::function<bool()>;
using PauseRecordingCallback = std::function<bool()>;
using ResumeRecordingCallback = std::function<bool()>;
using ClearConfigCallback = std::function<bool()>;
using QuitCallback = std::function<void()>;

/**
 * Callback registration structure for RPC handlers
 */
struct RpcCallbacks {
  GetStateCallback get_state;
  GetStatsCallback get_stats;
  GetTaskConfigCallback get_task_config;
  SetConfigCallback set_config;
  BeginRecordingCallback begin_recording;
  FinishRecordingCallback finish_recording;
  CancelRecordingCallback cancel_recording;
  PauseRecordingCallback pause_recording;
  ResumeRecordingCallback resume_recording;
  ClearConfigCallback clear_config;
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
 * Handle RPC test command (connectivity check)
 * @param callbacks RPC callbacks to invoke
 * @param params Request parameters (optional "echo" parameter)
 * @return RPC response with echoed data
 */
RpcResponse handle_rpc_test(const RpcCallbacks& callbacks, const nlohmann::json& params);

}  // namespace recorder
}  // namespace axon

#endif  // AXON_RECORDER_RPC_HANDLERS_HPP
