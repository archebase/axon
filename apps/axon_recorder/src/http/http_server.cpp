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

#include "../config/task_config.hpp"
#include "event_broadcaster.hpp"
#include "version.hpp"
#include "websocket_rpc_handler.hpp"

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

    // Create and start WebSocket server on port (HTTP port + 1)
    WebSocketServer::Config ws_config;
    ws_config.host = host_;
    ws_config.port = static_cast<uint16_t>(port_ + 1);  // WebSocket port = HTTP port + 1
    ws_server_ = std::make_shared<WebSocketServer>(ws_io_context_, ws_config);
    ws_server_->start();
    ws_server_thread_ = std::thread([this]() {
      ws_io_context_.run();
    });

    AXON_LOG_INFO(
      std::string("WebSocket server started on ws://0.0.0.0:") + std::to_string(ws_config.port) +
      "/ws"
    );

    // Create and start EventBroadcaster
    event_broadcaster_ = std::make_unique<EventBroadcaster>(*ws_server_);

    // Set up stats callback for periodic broadcasting
    event_broadcaster_->set_stats_callback([this]() -> nlohmann::json {
      if (callbacks_.get_stats) {
        return callbacks_.get_stats();
      }
      return nlohmann::json::object();
    });

    // Set up state callback for broadcasting state changes
    event_broadcaster_->set_state_callback([this]() -> std::pair<RecorderState, std::string> {
      if (callbacks_.get_state) {
        std::string state_str = callbacks_.get_state();
        RecorderState state = RecorderState::IDLE;
        if (state_str == "ready")
          state = RecorderState::READY;
        else if (state_str == "recording")
          state = RecorderState::RECORDING;
        else if (state_str == "paused")
          state = RecorderState::PAUSED;
        return {state, state_str};
      }
      return {RecorderState::IDLE, "idle"};
    });

    // Set up config callback for broadcasting config changes
    event_broadcaster_->set_config_callback([this]() -> const TaskConfig* {
      if (callbacks_.get_task_config) {
        return callbacks_.get_task_config();
      }
      return nullptr;
    });

    // Start periodic stats broadcast (every 1 second)
    event_broadcaster_->start_stats_broadcast(std::chrono::milliseconds(1000));

    AXON_LOG_INFO("EventBroadcaster started with 1s stats interval");

    // Set up WebSocket RPC handler
    // Convert callbacks to RpcCallbacks for the RPC handler
    RpcCallbacks rpc_callbacks;
    rpc_callbacks.get_state = callbacks_.get_state;
    rpc_callbacks.get_stats = callbacks_.get_stats;
    rpc_callbacks.get_task_config = callbacks_.get_task_config;
    rpc_callbacks.set_config = callbacks_.set_config;
    rpc_callbacks.begin_recording = callbacks_.begin_recording;
    rpc_callbacks.finish_recording = callbacks_.finish_recording;
    rpc_callbacks.cancel_recording = callbacks_.cancel_recording;
    rpc_callbacks.pause_recording = callbacks_.pause_recording;
    rpc_callbacks.resume_recording = callbacks_.resume_recording;
    rpc_callbacks.clear_config = callbacks_.clear_config;
    rpc_callbacks.quit = callbacks_.quit;

    ws_rpc_handler_ = std::make_unique<WebSocketRpcHandler>(
      rpc_callbacks, [this](const std::string& client_id, const nlohmann::json& response) {
        // Send RPC response to specific client via WebSocket
        if (ws_server_) {
          ws_server_->send_to_client(client_id, response.dump());
        }
      }
    );

    // Set WebSocket message handler to route RPC commands
    ws_server_->set_message_handler(
      [this](const std::string& client_id, const std::string& message) {
        ws_rpc_handler_->handle_message(client_id, message);
      }
    );

    AXON_LOG_INFO("WebSocket RPC handler enabled");

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

  // Stop EventBroadcaster first
  if (event_broadcaster_) {
    event_broadcaster_->stop_stats_broadcast();
    event_broadcaster_.reset();
  }

  // Stop WebSocket server
  if (ws_server_) {
    ws_server_->stop();
  }
  // Stop io_context to unblock the WebSocket thread
  ws_io_context_.stop();
  if (ws_server_thread_.joinable()) {
    ws_server_thread_.join();
  }

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

        // Handle CORS preflight
        if (request.method() == http::verb::options) {
          status_code = 204;
        } else {
          handle_request(
            request.method_string(),
            request.target(),
            request.body(),
            response_body,
            content_type,
            status_code
          );
        }

        // Reflect the request Origin back rather than using a wildcard.
        // This allows only the actual requesting origin (e.g. the panel on port+2)
        // while still supporting any hostname without hardcoding.
        std::string origin = std::string(request[http::field::origin]);
        if (origin.empty()) {
          origin = "*";
        }

        http::response<http::string_body> response{
          static_cast<http::status>(status_code), request.version()
        };
        response.set(http::field::server, "AxonRecorder/0.1.0");
        response.set(http::field::content_type, content_type);
        response.set("Access-Control-Allow-Origin", origin);
        response.set("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
        response.set("Access-Control-Allow-Headers", "Content-Type, Authorization");
        response.set("Vary", "Origin");
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
    health["version"] = "0.0.0";
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
  return axon::recorder::handle_rpc_begin(callbacks_, params);
}

HttpServer::RpcResponse HttpServer::handle_rpc_pause(const nlohmann::json& params) {
  return axon::recorder::handle_rpc_pause(callbacks_, params);
}

HttpServer::RpcResponse HttpServer::handle_rpc_resume(const nlohmann::json& params) {
  return axon::recorder::handle_rpc_resume(callbacks_, params);
}

HttpServer::RpcResponse HttpServer::handle_rpc_finish(const nlohmann::json& params) {
  return axon::recorder::handle_rpc_finish(callbacks_, params);
}

HttpServer::RpcResponse HttpServer::handle_rpc_quit(const nlohmann::json& params) {
  auto response = axon::recorder::handle_rpc_quit(callbacks_, params);
  // Stop the HTTP server after quit callback
  stop_requested_.store(true);
  return response;
}

HttpServer::RpcResponse HttpServer::handle_rpc_cancel(const nlohmann::json& params) {
  return axon::recorder::handle_rpc_cancel(callbacks_, params);
}

HttpServer::RpcResponse HttpServer::handle_rpc_clear(const nlohmann::json& params) {
  return axon::recorder::handle_rpc_clear(callbacks_, params);
}

HttpServer::RpcResponse HttpServer::handle_rpc_get_state(const nlohmann::json& params) {
  return axon::recorder::handle_rpc_get_state(callbacks_, params);
}

HttpServer::RpcResponse HttpServer::handle_rpc_get_stats(const nlohmann::json& params) {
  return axon::recorder::handle_rpc_get_stats(callbacks_, params);
}

HttpServer::RpcResponse HttpServer::handle_rpc_set_config(const nlohmann::json& params) {
  return axon::recorder::handle_rpc_config(callbacks_, params);
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

void HttpServer::broadcast_state_change(
  RecorderState from, RecorderState to, const std::string& task_id
) {
  if (event_broadcaster_) {
    event_broadcaster_->broadcast_state_change(from, to, task_id);
  }
}

void HttpServer::broadcast_config_change(const TaskConfig* config) {
  if (event_broadcaster_) {
    event_broadcaster_->broadcast_config_change(config);
  }
}

void HttpServer::broadcast_log(const std::string& level, const std::string& message) {
  if (event_broadcaster_) {
    event_broadcaster_->broadcast_log(level, message);
  }
}

}  // namespace recorder
}  // namespace axon
