// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "http_server.hpp"

#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>

#include <chrono>
#include <cstring>
#include <iostream>
#include <sstream>
#include <unordered_map>

#include "task_config.hpp"
#include "version.hpp"

// Logging infrastructure
#define AXON_LOG_COMPONENT "http_server"
#include <axon_log_macros.hpp>

namespace beast = boost::beast;
namespace http = beast::http;
namespace asio = boost::asio;
using tcp = asio::ip::tcp;

namespace axon {
namespace recorder {

HttpServer::HttpServer(const std::string& host, uint16_t port)
    : host_(host)
    , port_(port)
    , running_(false)
    , stop_requested_(false) {}

HttpServer::~HttpServer() {
  stop();
}

bool HttpServer::start() {
  if (running_.load()) {
    set_error_helper("Server already running");
    return false;
  }

  // Verify callbacks are registered
  if (!callbacks_.get_state || !callbacks_.quit) {
    set_error_helper("Callbacks not registered. Call register_callbacks() first.");
    return false;
  }

  try {
    stop_requested_.store(false);
    server_thread_ = std::make_unique<std::thread>(&HttpServer::server_thread_func, this);
    running_.store(true);
    return true;
  } catch (const std::exception& e) {
    set_error_helper(std::string("Failed to start server: ") + e.what());
    return false;
  }
}

void HttpServer::register_callbacks(const Callbacks& callbacks) {
  callbacks_ = callbacks;
}

void HttpServer::stop() {
  if (!running_.load()) {
    return;
  }

  stop_requested_.store(true);

  if (server_thread_ && server_thread_->joinable()) {
    server_thread_->join();
    server_thread_.reset();
  }

  running_.store(false);
}

bool HttpServer::is_running() const {
  return running_.load();
}

std::string HttpServer::get_url() const {
  return "http://" + host_ + ":" + std::to_string(port_);
}

std::string HttpServer::get_last_error() const {
  std::lock_guard<std::mutex> lock(error_mutex_);
  return last_error_;
}

void HttpServer::server_thread_func() {
  try {
    asio::io_context io_context(1);

    tcp::acceptor acceptor(io_context);
    tcp::endpoint endpoint{asio::ip::make_address(host_), port_};

    acceptor.open(endpoint.protocol());
    acceptor.set_option(asio::socket_base::reuse_address(true));
    acceptor.bind(endpoint);
    acceptor.listen(asio::socket_base::max_listen_connections);

    AXON_LOG_INFO("HTTP RPC server listening on " << get_url());

    while (!stop_requested_.load()) {
      tcp::socket socket(io_context);

      // Set a timeout for accept to allow checking stop_requested_
      acceptor.set_option(tcp::acceptor::reuse_address(true));

      // Wait for connection (with timeout check)
      bool timeout = false;
      std::chrono::seconds timeout_sec(1);

      // Use async accept with timeout
      asio::steady_timer timer(io_context);
      timer.expires_after(timeout_sec);

      bool accept_complete = false;
      boost::system::error_code accept_ec;

      acceptor.async_accept(socket, [&](const boost::system::error_code& ec) {
        accept_ec = ec;
        if (!ec) {
          accept_complete = true;
          timer.cancel();  // Cancel timer when accept completes
        }
      });

      timer.async_wait([&](const boost::system::error_code& ec) {
        if (ec != asio::error::operation_aborted && !accept_complete) {
          // Timer fired naturally, cancel the pending accept
          acceptor.cancel();
        }
      });

      io_context.restart();
      io_context.run();

      if (stop_requested_.load()) {
        break;
      }

      if (!accept_complete && accept_ec != asio::error::operation_aborted) {
        // Actual error (not timeout)
        AXON_LOG_ERROR("Accept failed: " << accept_ec.message());
        continue;
      }

      if (!accept_complete) {
        // Timeout, continue loop
        continue;
      }

      // Process the connection
      try {
        beast::flat_buffer buffer;

        http::request<http::string_body> request;
        http::read(socket, buffer, request);

        std::string response_body;
        std::string content_type = "application/json";
        int status_code = 200;

        handle_request(
          request.method_string(),
          request.target(),
          request.body(),
          response_body,
          content_type,
          status_code
        );

        http::response<http::string_body> response{
          static_cast<http::status>(status_code), request.version()
        };
        response.set(http::field::server, "AxonRecorder/0.1.0");
        response.set(http::field::content_type, content_type);
        response.keep_alive(request.keep_alive());
        response.body() = response_body;
        response.prepare_payload();

        http::write(socket, response);

        // Gracefully close the connection
        beast::error_code ec;
        socket.shutdown(tcp::socket::shutdown_send, ec);

      } catch (const beast::system_error& se) {
        // Don't report on normal shutdown
        if (se.code() != beast::errc::connection_reset &&
            se.code() != beast::errc::operation_canceled && se.code() != asio::error::eof) {
          AXON_LOG_ERROR("Connection error: " << se.what());
        }
        // Ensure socket is closed on error
        beast::error_code ec;
        socket.shutdown(tcp::socket::shutdown_both, ec);
        socket.close(ec);
      } catch (const std::exception& e) {
        AXON_LOG_ERROR("Request processing error: " << e.what());
        // Ensure socket is closed on error
        beast::error_code ec;
        socket.shutdown(tcp::socket::shutdown_both, ec);
        socket.close(ec);
      }
    }

    AXON_LOG_INFO("HTTP RPC server stopped");

  } catch (const std::exception& e) {
    set_error_helper(std::string("Server error: ") + e.what());
    AXON_LOG_ERROR("HTTP server error: " << e.what());
  }
}

void HttpServer::handle_request(
  const boost::beast::string_view& method, const boost::beast::string_view& target,
  const std::string& body, std::string& response_body, std::string& content_type, int& status_code
) {
  AXON_LOG_DEBUG("Received request: " << method << " " << target);

  // Convert string_view to string for easier handling
  std::string method_str(method);
  std::string target_str(target);

  AXON_LOG_DEBUG("Method: [" << method_str << "], Target: [" << target_str << "]");

  // Check if it's an RPC call
  if (target_str.find("/rpc/") == 0) {
    std::string rpc_method = target_str.substr(5);  // Remove "/rpc/"
    AXON_LOG_DEBUG("RPC method: [" << rpc_method << "]");

    try {
      // Parse request body if present
      nlohmann::json params;
      if (!body.empty()) {
        params = nlohmann::json::parse(body);
      }

      RpcResponse rpc_response;

      AXON_LOG_DEBUG(
        "Routing: rpc_method=[" << rpc_method << "], method_str=[" << method_str << "]"
      );
      AXON_LOG_DEBUG(
        "Comparison: begin=" << (rpc_method == "begin") << ", finish=" << (rpc_method == "finish")
                             << ", quit=" << (rpc_method == "quit")
                             << ", POST=" << (method_str == "POST")
      );

      // Route to appropriate RPC handler
      if (rpc_method == "begin" && method_str == "POST") {
        AXON_LOG_DEBUG("Routing to handle_rpc_begin");
        rpc_response = handle_rpc_begin(params);
      } else if (rpc_method == "finish" && method_str == "POST") {
        AXON_LOG_DEBUG("Routing to handle_rpc_finish");
        rpc_response = handle_rpc_finish(params);
      } else if (rpc_method == "quit" && method_str == "POST") {
        AXON_LOG_DEBUG("Routing to handle_rpc_quit");
        rpc_response = handle_rpc_quit(params);
      } else if (rpc_method == "pause" && method_str == "POST") {
        AXON_LOG_DEBUG("Routing to handle_rpc_pause");
        rpc_response = handle_rpc_pause(params);
      } else if (rpc_method == "resume" && method_str == "POST") {
        AXON_LOG_DEBUG("Routing to handle_rpc_resume");
        rpc_response = handle_rpc_resume(params);
      } else if (rpc_method == "cancel" && method_str == "POST") {
        AXON_LOG_DEBUG("Routing to handle_rpc_cancel");
        rpc_response = handle_rpc_cancel(params);
      } else if (rpc_method == "clear" && method_str == "POST") {
        AXON_LOG_DEBUG("Routing to handle_rpc_clear");
        rpc_response = handle_rpc_clear(params);
      } else if (rpc_method == "state" && method_str == "GET") {
        AXON_LOG_DEBUG("Routing to handle_rpc_get_state");
        rpc_response = handle_rpc_get_state(params);
      } else if (rpc_method == "stats" && method_str == "GET") {
        AXON_LOG_DEBUG("Routing to handle_rpc_get_stats");
        rpc_response = handle_rpc_get_stats(params);
      } else if (rpc_method == "config" && method_str == "POST") {
        AXON_LOG_DEBUG("Routing to handle_rpc_set_config");
        rpc_response = handle_rpc_set_config(params);
      } else {
        AXON_LOG_DEBUG("No match found, returning 404");
        rpc_response.success = false;
        rpc_response.message = "Unknown RPC method or invalid HTTP method: " + rpc_method;
        status_code = 404;
      }

      response_body = rpc_response.to_json().dump();
      content_type = "application/json";

    } catch (const nlohmann::json::exception& e) {
      nlohmann::json error_json;
      error_json["success"] = false;
      error_json["message"] = std::string("JSON parse error: ") + e.what();
      response_body = error_json.dump();
      content_type = "application/json";
      status_code = 400;
    }

  } else if (target_str == "/" || target_str == "/health") {
    // Health check endpoint
    nlohmann::json health;
    health["status"] = "ok";
    health["service"] = "AxonRecorder";
    health["version"] = "0.1.0";
    health["running"] = running_.load();
    health["state"] = callbacks_.get_state ? callbacks_.get_state() : "unknown";
    response_body = health.dump();
    content_type = "application/json";
  } else {
    // Unknown endpoint
    nlohmann::json error;
    error["success"] = false;
    error["message"] = "Not found";
    response_body = error.dump();
    content_type = "application/json";
    status_code = 404;
  }
}

HttpServer::RpcResponse HttpServer::handle_rpc_begin(const nlohmann::json& params) {
  RpcResponse response;

  // Check if task_id is provided and is a string
  if (!params.contains("task_id")) {
    response.success = false;
    response.message = "Missing required parameter: task_id";
    response.data["state"] = callbacks_.get_state ? callbacks_.get_state() : "unknown";
    return response;
  }

  if (!params["task_id"].is_string()) {
    response.success = false;
    response.message = "task_id must be a string";
    response.data["state"] = callbacks_.get_state ? callbacks_.get_state() : "unknown";
    return response;
  }

  std::string provided_task_id = params["task_id"].get<std::string>();

  // Check callback exists
  if (!callbacks_.begin_recording) {
    response.success = false;
    response.message = "Begin recording callback not registered";
    return response;
  }

  // Call the callback
  bool success = callbacks_.begin_recording(provided_task_id);

  if (success) {
    response.success = true;
    response.message = "Recording started successfully";
    response.data["state"] = callbacks_.get_state ? callbacks_.get_state() : "unknown";
    response.data["task_id"] = provided_task_id;
  } else {
    response.success = false;
    response.message = "Failed to start recording";
    response.data["state"] = callbacks_.get_state ? callbacks_.get_state() : "unknown";
  }

  return response;
}

HttpServer::RpcResponse HttpServer::handle_rpc_pause(const nlohmann::json& params) {
  RpcResponse response;

  if (!callbacks_.pause_recording) {
    response.success = false;
    response.message = "Pause recording callback not registered";
    return response;
  }

  bool success = callbacks_.pause_recording();

  if (success) {
    response.success = true;
    response.message = "Recording paused successfully";
    response.data["state"] = callbacks_.get_state ? callbacks_.get_state() : "unknown";
  } else {
    response.success = false;
    response.message = "Failed to pause recording";
    response.data["state"] = callbacks_.get_state ? callbacks_.get_state() : "unknown";
  }

  return response;
}

HttpServer::RpcResponse HttpServer::handle_rpc_resume(const nlohmann::json& params) {
  RpcResponse response;

  if (!callbacks_.resume_recording) {
    response.success = false;
    response.message = "Resume recording callback not registered";
    return response;
  }

  bool success = callbacks_.resume_recording();

  if (success) {
    response.success = true;
    response.message = "Recording resumed successfully";
    response.data["state"] = callbacks_.get_state ? callbacks_.get_state() : "unknown";
  } else {
    response.success = false;
    response.message = "Failed to resume recording";
    response.data["state"] = callbacks_.get_state ? callbacks_.get_state() : "unknown";
  }

  return response;
}

HttpServer::RpcResponse HttpServer::handle_rpc_finish(const nlohmann::json& params) {
  RpcResponse response;

  // Check if task_id is provided and is a string
  if (!params.contains("task_id")) {
    response.success = false;
    response.message = "Missing required parameter: task_id";
    response.data["state"] = callbacks_.get_state ? callbacks_.get_state() : "unknown";
    return response;
  }

  if (!params["task_id"].is_string()) {
    response.success = false;
    response.message = "task_id must be a string";
    response.data["state"] = callbacks_.get_state ? callbacks_.get_state() : "unknown";
    return response;
  }

  std::string task_id = params["task_id"].get<std::string>();

  if (!callbacks_.finish_recording) {
    response.success = false;
    response.message = "Finish recording callback not registered";
    return response;
  }

  bool success = callbacks_.finish_recording(task_id);

  if (success) {
    response.success = true;
    response.message = "Recording finished successfully";
    response.data["state"] = callbacks_.get_state ? callbacks_.get_state() : "unknown";
    response.data["task_id"] = task_id;
  } else {
    response.success = false;
    response.message = "Failed to finish recording";
    response.data["state"] = callbacks_.get_state ? callbacks_.get_state() : "unknown";
  }

  return response;
}

HttpServer::RpcResponse HttpServer::handle_rpc_quit(const nlohmann::json& params) {
  (void)params;  // Unused - quit doesn't require task_id

  RpcResponse response;

  // Call the quit callback to signal the main program to exit
  if (callbacks_.quit) {
    callbacks_.quit();
  }

  // Stop the HTTP server
  AXON_LOG_INFO("Stopping HTTP server...");
  stop_requested_.store(true);

  response.success = true;
  response.message = "Program quitting. Recording stopped and data saved.";
  response.data["state"] = callbacks_.get_state ? callbacks_.get_state() : "unknown";

  return response;
}

HttpServer::RpcResponse HttpServer::handle_rpc_cancel(const nlohmann::json& params) {
  RpcResponse response;

  if (!callbacks_.cancel_recording) {
    response.success = false;
    response.message = "Cancel recording callback not registered";
    return response;
  }

  bool success = callbacks_.cancel_recording();

  if (success) {
    response.success = true;
    response.message = "Recording cancelled successfully";
    response.data["state"] = callbacks_.get_state ? callbacks_.get_state() : "unknown";
  } else {
    response.success = false;
    response.message = "Failed to cancel recording";
    response.data["state"] = callbacks_.get_state ? callbacks_.get_state() : "unknown";
  }

  return response;
}

HttpServer::RpcResponse HttpServer::handle_rpc_clear(const nlohmann::json& params) {
  RpcResponse response;

  if (!callbacks_.clear_config) {
    response.success = false;
    response.message = "Clear config callback not registered";
    return response;
  }

  bool success = callbacks_.clear_config();

  if (success) {
    response.success = true;
    response.message = "Configuration cleared successfully";
    response.data["state"] = callbacks_.get_state ? callbacks_.get_state() : "unknown";
  } else {
    response.success = false;
    response.message = "Failed to clear configuration";
    response.data["state"] = callbacks_.get_state ? callbacks_.get_state() : "unknown";
  }

  return response;
}

HttpServer::RpcResponse HttpServer::handle_rpc_get_state(const nlohmann::json& params) {
  RpcResponse response;

  response.success = true;
  response.message = "State retrieved successfully";

  // Add version information
  response.data["version"] = get_version();

  // Get state from callback
  if (callbacks_.get_state) {
    response.data["state"] = callbacks_.get_state();
  } else {
    response.data["state"] = "unknown";
  }

  // Get task config if available (READY, RECORDING, PAUSED states)
  if (callbacks_.get_task_config) {
    const TaskConfig* task_config = callbacks_.get_task_config();
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
  if (callbacks_.get_stats) {
    try {
      nlohmann::json stats = callbacks_.get_stats();
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

HttpServer::RpcResponse HttpServer::handle_rpc_get_stats(const nlohmann::json& params) {
  RpcResponse response;

  if (!callbacks_.get_stats) {
    response.success = false;
    response.message = "Get stats callback not registered";
    return response;
  }

  try {
    nlohmann::json stats = callbacks_.get_stats();
    response.success = true;
    response.message = "Statistics retrieved successfully";
    response.data = stats;
  } catch (const std::exception& e) {
    response.success = false;
    response.message = std::string("Failed to get statistics: ") + e.what();
  }

  return response;
}

HttpServer::RpcResponse HttpServer::handle_rpc_set_config(const nlohmann::json& params) {
  RpcResponse response;

  // Check if task_config is present in params
  if (!params.contains("task_config")) {
    response.success = false;
    response.message = "Missing 'task_config' in request parameters";
    return response;
  }

  if (!callbacks_.set_config) {
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
        response.data["state"] = callbacks_.get_state ? callbacks_.get_state() : "unknown";
        return response;
      }
      task_id = config_json["task_id"].get<std::string>();
    }

    // Call the callback with the config
    bool success = callbacks_.set_config(task_id, config_json);

    if (success) {
      response.success = true;
      response.message = "Task configuration set successfully";
      response.data["state"] = callbacks_.get_state ? callbacks_.get_state() : "unknown";
      if (!task_id.empty()) {
        response.data["task_id"] = task_id;
      }
    } else {
      response.success = false;
      response.message = "Failed to set task configuration";
      response.data["state"] = callbacks_.get_state ? callbacks_.get_state() : "unknown";
    }

  } catch (const std::exception& e) {
    response.success = false;
    response.message = std::string("Failed to parse task config: ") + e.what();
  }

  return response;
}

std::string HttpServer::format_response(
  int status_code, const std::string& content_type, const std::string& body
) {
  std::ostringstream oss;
  oss << "HTTP/1.1 " << status_code << " ";

  // Add status reason
  switch (status_code) {
    case 200:
      oss << "OK";
      break;
    case 400:
      oss << "Bad Request";
      break;
    case 404:
      oss << "Not Found";
      break;
    case 500:
      oss << "Internal Server Error";
      break;
    default:
      oss << "Unknown";
      break;
  }

  oss << "\r\n"
      << "Content-Type: " << content_type << "\r\n"
      << "Content-Length: " << body.length() << "\r\n"
      << "Connection: close\r\n"
      << "\r\n"
      << body;
  return oss.str();
}

void HttpServer::set_error_helper(const std::string& error) {
  std::lock_guard<std::mutex> lock(error_mutex_);
  last_error_ = error;
}

}  // namespace recorder
}  // namespace axon
