// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_RECORDER_HTTP_SERVER_HPP
#define AXON_RECORDER_HTTP_SERVER_HPP

#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <nlohmann/json.hpp>

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "../config/task_config.hpp"
#include "event_broadcaster.hpp"
#include "rpc_handlers.hpp"
#include "websocket_server.hpp"

namespace axon {
namespace recorder {

/**
 * Forward declarations
 */
struct TaskConfig;
class WebSocketRpcHandler;

/**
 * HTTP RPC Server for AxonRecorder
 *
 * Provides RESTful JSON API for controlling the recorder remotely.
 * Uses callback interface to avoid direct dependency on AxonRecorder.
 * Uses shared RPC handlers that can also be used by WebSocket.
 */
class HttpServer {
public:
  /**
   * Callback registration structure
   * Alias to RpcCallbacks for backward compatibility
   */
  using Callbacks = RpcCallbacks;

  /**
   * Use shared RpcResponse from rpc_handlers.hpp
   */
  using RpcResponse = ::axon::recorder::RpcResponse;

  /**
   * Constructor
   * @param host Host address to bind to (default: "0.0.0.0")
   * @param port Port number to listen on (default: 8080)
   */
  HttpServer(const std::string& host = "0.0.0.0", uint16_t port = 8080);

  /**
   * Destructor - stops the server
   */
  ~HttpServer();

  // Non-copyable, non-movable
  HttpServer(const HttpServer&) = delete;
  HttpServer& operator=(const HttpServer&) = delete;
  HttpServer(HttpServer&&) = delete;
  HttpServer& operator=(HttpServer&&) = delete;

  /**
   * Register callbacks for RPC operations
   * Must be called before start()
   */
  void register_callbacks(const Callbacks& callbacks);

  /**
   * Start the HTTP server in a background thread
   * @return true on success, false on failure
   */
  bool start();

  /**
   * Stop the HTTP server
   */
  void stop();

  /**
   * Check if server is running
   */
  bool is_running() const;

  /**
   * Get the server URL
   */
  std::string get_url() const;

  /**
   * Get last error message
   */
  std::string get_last_error() const;

  /**
   * Get the WebSocket server instance
   */
  WebSocketServer* get_websocket_server() {
    return ws_server_.get();
  }

  /**
   * Get the EventBroadcaster instance
   */
  EventBroadcaster* get_event_broadcaster() {
    return event_broadcaster_.get();
  }

  /**
   * Broadcast state change via WebSocket
   * @param from Previous state
   * @param to New state
   * @param task_id Current task ID
   */
  void broadcast_state_change(RecorderState from, RecorderState to, const std::string& task_id);

  /**
   * Broadcast config change via WebSocket
   * @param config Task configuration (nullptr to indicate clear)
   */
  void broadcast_config_change(const TaskConfig* config);

  /**
   * Broadcast log event via WebSocket
   * @param level Log level (debug/info/warning/error)
   * @param message Log message
   */
  void broadcast_log(const std::string& level, const std::string& message);

private:
  /**
   * Server thread main function
   */
  void server_thread_func();

  /**
   * Handle HTTP request
   */
  void handle_request(
    const boost::beast::string_view& method, const boost::beast::string_view& target,
    const std::string& body, std::string& response_body, std::string& content_type, int& status_code
  );

  /**
   * RPC handlers
   */
  RpcResponse handle_rpc_begin(const nlohmann::json& params);
  RpcResponse handle_rpc_finish(const nlohmann::json& params);
  RpcResponse handle_rpc_quit(const nlohmann::json& params);
  RpcResponse handle_rpc_pause(const nlohmann::json& params);
  RpcResponse handle_rpc_resume(const nlohmann::json& params);
  RpcResponse handle_rpc_cancel(const nlohmann::json& params);
  RpcResponse handle_rpc_clear(const nlohmann::json& params);
  RpcResponse handle_rpc_get_state(const nlohmann::json& params);
  RpcResponse handle_rpc_get_stats(const nlohmann::json& params);
  RpcResponse handle_rpc_get_drop_stats(const nlohmann::json& params);
  RpcResponse handle_rpc_get_latency_stats(const nlohmann::json& params);
  RpcResponse handle_rpc_set_config(const nlohmann::json& params);

  /**
   * Helper to format HTTP response
   */
  std::string format_response(
    int status_code, const std::string& content_type, const std::string& body
  );

  /**
   * Helper to set error message (thread-safe)
   */
  void set_error_helper(const std::string& error);

  std::string host_;
  uint16_t port_;
  Callbacks callbacks_;

  std::unique_ptr<std::thread> server_thread_;
  std::atomic<bool> running_;
  std::atomic<bool> stop_requested_;

  // Error handling
  mutable std::mutex error_mutex_;
  std::string last_error_;

  // WebSocket server
  std::shared_ptr<WebSocketServer> ws_server_;
  boost::asio::io_context ws_io_context_;
  std::thread ws_server_thread_;

  // Event broadcaster for WebSocket push notifications
  std::unique_ptr<EventBroadcaster> event_broadcaster_;

  // WebSocket RPC handler for bidirectional RPC commands
  std::unique_ptr<WebSocketRpcHandler> ws_rpc_handler_;
};

}  // namespace recorder
}  // namespace axon

#endif  // AXON_RECORDER_HTTP_SERVER_HPP
