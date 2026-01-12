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

namespace axon {
namespace recorder {

// Forward declarations
class AxonRecorder;

/**
 * HTTP RPC Server for AxonRecorder
 *
 * Provides RESTful JSON API for controlling the recorder remotely.
 *
 * Endpoints:
 * - POST /rpc/start: Start recording
 * - POST /rpc/stop: Stop recording
 * - GET  /rpc/state: Get current state
 * - GET  /rpc/stats: Get recording statistics
 * - POST /rpc/config: Set task configuration
 */
class HttpServer {
public:
  /**
   * RPC response structure
   */
  struct RpcResponse {
    bool success;
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
   * Constructor
   * @param recorder Reference to the AxonRecorder instance
   * @param host Host address to bind to (default: "0.0.0.0")
   * @param port Port number to listen on (default: 8080)
   */
  HttpServer(AxonRecorder& recorder, const std::string& host = "0.0.0.0", uint16_t port = 8080);

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

  AxonRecorder& recorder_;
  std::string host_;
  uint16_t port_;

  std::unique_ptr<std::thread> server_thread_;
  std::atomic<bool> running_;
  std::atomic<bool> stop_requested_;

  // Error handling
  mutable std::mutex error_mutex_;
  std::string last_error_;
};

}  // namespace recorder
}  // namespace axon

#endif  // AXON_RECORDER_HTTP_SERVER_HPP
