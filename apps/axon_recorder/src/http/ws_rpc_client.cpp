// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "ws_rpc_client.hpp"

#include <boost/beast/websocket.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>

// Logging infrastructure
#define AXON_LOG_COMPONENT "ws_rpc_client"
#include <axon_log_init.hpp>
#include <axon_log_macros.hpp>

using axon::logging::kv;

namespace axon {
namespace recorder {

WsRpcClient::WsRpcClient(net::io_context& ioc, const WsClientConfig& config)
    : config_(config)
    , resolver_(net::make_strand(ioc))
    , ws_(net::make_strand(ioc))
    , strand_(net::make_strand(ioc))
    , reconnect_timer_(ioc)
    , ping_timer_(ioc) {
  // Set up control callback for pong handling
  ws_.control_callback([](beast::websocket::frame_type type, beast::string_view) {
    if (type == beast::websocket::frame_type::pong) {
      // Pong received, connection is alive
    }
  });
}

WsRpcClient::~WsRpcClient() {
  stop();
}

void WsRpcClient::start() {
  stopped_ = false;
  connected_ = false;
  reconnect_attempt_ = 0;
  do_resolve();
}

void WsRpcClient::stop() {
  stopped_ = true;
  reconnect_timer_.cancel();
  ping_timer_.cancel();
  beast::error_code ec;
  ws_.close(beast::websocket::close_code::normal, ec);
}

bool WsRpcClient::is_connected() const {
  return connected_.load();
}

void WsRpcClient::register_callbacks(const RpcCallbacks& callbacks) {
  callbacks_ = callbacks;
}

void WsRpcClient::send_state_update(
  RecorderState from, RecorderState to, const std::string& task_id
) {
  nlohmann::json msg;
  msg["type"] = "state_update";
  msg["timestamp"] = std::chrono::duration_cast<std::chrono::milliseconds>(
                       std::chrono::system_clock::now().time_since_epoch()
  )
                       .count();

  nlohmann::json data;
  data["previous"] = state_to_string(from);
  data["current"] = state_to_string(to);
  if (!task_id.empty()) {
    data["task_id"] = task_id;
  }
  msg["data"] = data;

  send_message(msg);
}

void WsRpcClient::do_resolve() {
  if (stopped_) {
    return;
  }

  std::string host, port, path;
  if (!parse_url(config_.url, host, port, path)) {
    AXON_LOG_ERROR("Invalid WebSocket URL" << kv("url", config_.url));
    schedule_reconnect();
    return;
  }

  AXON_LOG_INFO("Resolving WebSocket server" << kv("host", host) << kv("port", port));

  resolver_.async_resolve(host, port, beast::bind_front_handler(&WsRpcClient::on_resolve, this));
}

void WsRpcClient::on_resolve(beast::error_code ec, tcp::resolver::results_type results) {
  if (ec) {
    AXON_LOG_ERROR("WebSocket resolve error" << kv("error", ec.message()));
    schedule_reconnect();
    return;
  }

  do_connect(results);
}

void WsRpcClient::do_connect(tcp::resolver::results_type results) {
  if (stopped_) {
    return;
  }

  AXON_LOG_INFO("Connecting to WebSocket server");

  beast::get_lowest_layer(ws_).async_connect(
    results, beast::bind_front_handler(&WsRpcClient::on_connect, this)
  );
}

void WsRpcClient::on_connect(beast::error_code ec, tcp::endpoint endpoint) {
  if (ec) {
    AXON_LOG_ERROR("WebSocket connect error" << kv("error", ec.message()));
    schedule_reconnect();
    return;
  }

  (void)endpoint;  // Unused

  do_handshake();
}

void WsRpcClient::do_handshake() {
  if (stopped_) {
    return;
  }

  std::string host, port, path;
  parse_url(config_.url, host, port, path);

  AXON_LOG_INFO("Performing WebSocket handshake" << kv("host", host) << kv("path", path));

  // Set authorization header if token is provided
  if (!config_.auth_token.empty()) {
    ws_.set_option(
      beast::websocket::stream_base::decorator([this](beast::websocket::request_type& req) {
        req.set(beast::http::field::authorization, "Bearer " + config_.auth_token);
      })
    );
  }

  ws_.async_handshake(host, path, beast::bind_front_handler(&WsRpcClient::on_handshake, this));
}

void WsRpcClient::on_handshake(beast::error_code ec) {
  if (ec) {
    AXON_LOG_ERROR("WebSocket handshake error" << kv("error", ec.message()));
    schedule_reconnect();
    return;
  }

  AXON_LOG_INFO("WebSocket connected successfully");
  connected_ = true;
  reconnect_attempt_ = 0;

  // Start ping timer for keepalive
  start_ping_timer();

  // Start reading messages
  do_read();
}

void WsRpcClient::do_read() {
  if (stopped_ || !connected_) {
    return;
  }

  ws_.async_read(read_buffer_, beast::bind_front_handler(&WsRpcClient::on_read, this));
}

void WsRpcClient::on_read(beast::error_code ec, std::size_t bytes) {
  if (ec) {
    AXON_LOG_ERROR("WebSocket read error" << kv("error", ec.message()));
    on_disconnect(ec);
    return;
  }

  std::string data = beast::buffers_to_string(read_buffer_.data());
  read_buffer_.consume(read_buffer_.size());

  AXON_LOG_DEBUG("Received WebSocket message" << kv("bytes", bytes));

  try {
    auto json = nlohmann::json::parse(data);
    handle_server_message(json);
  } catch (const std::exception& e) {
    AXON_LOG_ERROR("Failed to parse WebSocket message" << kv("error", e.what()));
  }

  // Continue reading
  do_read();
}

void WsRpcClient::do_write() {
  if (stopped_ || !connected_) {
    return;
  }

  if (writing_) {
    return;
  }

  std::lock_guard<std::mutex> lock(write_mutex_);
  if (write_queue_.empty()) {
    return;
  }

  writing_ = true;
  auto msg = std::move(write_queue_.front());
  write_queue_.pop();

  ws_.async_write(net::buffer(msg), beast::bind_front_handler(&WsRpcClient::on_write, this));
}

void WsRpcClient::on_write(beast::error_code ec, std::size_t bytes) {
  writing_ = false;

  if (ec) {
    AXON_LOG_ERROR("WebSocket write error" << kv("error", ec.message()));
    on_disconnect(ec);
    return;
  }

  (void)bytes;  // Unused

  std::lock_guard<std::mutex> lock(write_mutex_);
  if (!write_queue_.empty()) {
    net::post(strand_, [this]() {
      this->do_write();
    });
  }
}

void WsRpcClient::handle_server_message(const nlohmann::json& msg) {
  // Check if this is an RPC request
  if (!msg.contains("action") || !msg.contains("request_id")) {
    AXON_LOG_WARN("Invalid RPC message format" << kv("msg", msg.dump()));
    return;
  }

  std::string action = msg["action"].get<std::string>();
  std::string request_id = msg["request_id"].get<std::string>();
  nlohmann::json params = msg.value("params", nlohmann::json::object());

  AXON_LOG_INFO("Received RPC command" << kv("action", action) << kv("request_id", request_id));

  RpcResponse response;

  // Handle each action using the shared RPC handlers
  if (action == "config") {
    response = handle_rpc_config(callbacks_, params);
  } else if (action == "begin") {
    response = handle_rpc_begin(callbacks_, params);
  } else if (action == "pause") {
    response = handle_rpc_pause(callbacks_, params);
  } else if (action == "resume") {
    response = handle_rpc_resume(callbacks_, params);
  } else if (action == "finish") {
    response = handle_rpc_finish(callbacks_, params);
  } else if (action == "cancel") {
    response = handle_rpc_cancel(callbacks_, params);
  } else if (action == "clear") {
    response = handle_rpc_clear(callbacks_, params);
  } else if (action == "quit") {
    response = handle_rpc_quit(callbacks_, params);
  } else if (action == "get_state") {
    response = handle_rpc_get_state(callbacks_, params);
  } else if (action == "get_stats") {
    response = handle_rpc_get_stats(callbacks_, params);
  } else if (action == "test") {
    response = handle_rpc_test(callbacks_, params);
  } else {
    response.success = false;
    response.message = "Unknown action: " + action;
  }

  // Send response
  send_rpc_response(request_id, response);
}

void WsRpcClient::send_rpc_response(const std::string& request_id, const RpcResponse& response) {
  nlohmann::json msg;
  msg["type"] = "rpc_response";
  msg["request_id"] = request_id;
  msg["success"] = response.success;
  msg["message"] = response.message;
  if (!response.data.is_null()) {
    msg["data"] = response.data;
  }

  send_message(msg);
}

void WsRpcClient::send_message(const nlohmann::json& msg) {
  {
    std::lock_guard<std::mutex> lock(write_mutex_);
    write_queue_.push(msg.dump());
  }
  net::post(strand_, [this]() {
    this->do_write();
  });
}

void WsRpcClient::on_disconnect(beast::error_code ec) {
  if (!connected_.exchange(false)) {
    return;  // Already disconnected
  }

  ping_timer_.cancel();

  AXON_LOG_WARN("WebSocket disconnected" << kv("error", ec.message()));

  schedule_reconnect();
}

void WsRpcClient::schedule_reconnect() {
  if (stopped_) {
    return;
  }

  // Calculate delay with exponential backoff
  long long delay = static_cast<long long>(config_.reconnect_initial_delay_ms);
  delay = static_cast<long long>(
    delay * std::pow(config_.reconnect_backoff_multiplier, reconnect_attempt_)
  );
  delay = std::min(delay, static_cast<long long>(config_.reconnect_max_delay_ms));

  // Add jitter
  if (config_.reconnect_jitter_factor > 0) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(
      -config_.reconnect_jitter_factor, config_.reconnect_jitter_factor
    );
    auto jitter = static_cast<long long>(delay * dis(gen));
    delay += jitter;
  }

  delay = std::max(100LL, delay);  // Minimum 100ms

  ++reconnect_attempt_;

  AXON_LOG_INFO(
    "Scheduling WebSocket reconnect" << kv("delay_ms", delay) << kv("attempt", reconnect_attempt_)
  );

  reconnect_timer_.expires_after(std::chrono::milliseconds(delay));
  reconnect_timer_.async_wait([this](beast::error_code ec) {
    if (ec || stopped_) {
      return;
    }
    do_resolve();
  });
}

void WsRpcClient::start_ping_timer() {
  if (config_.ping_interval_ms <= 0) {
    return;  // Ping disabled
  }

  ping_timer_.expires_after(std::chrono::milliseconds(config_.ping_interval_ms));
  ping_timer_.async_wait([this](beast::error_code ec) {
    if (ec || stopped_ || !connected_) {
      return;
    }

    // Send ping frame
    ws_.async_ping(beast::websocket::ping_data{}, [this](beast::error_code ping_ec) {
      if (ping_ec) {
        AXON_LOG_WARN("WebSocket ping failed" << kv("error", ping_ec.message()));
        on_disconnect(ping_ec);
        return;
      }
      // Schedule next ping
      start_ping_timer();
    });
  });
}

bool WsRpcClient::parse_url(
  const std::string& url, std::string& host, std::string& port, std::string& path
) {
  // Parse URL: ws://host:port/path or wss://host:port/path
  std::string::size_type proto_end = url.find("://");
  if (proto_end == std::string::npos) {
    return false;
  }

  std::string proto = url.substr(0, proto_end);
  if (proto != "ws" && proto != "wss") {
    return false;
  }

  std::string::size_type host_start = proto_end + 3;
  std::string::size_type path_start = url.find('/', host_start);

  std::string host_port;
  if (path_start != std::string::npos) {
    host_port = url.substr(host_start, path_start - host_start);
    path = url.substr(path_start);
  } else {
    host_port = url.substr(host_start);
    path = "/";
  }

  // Parse host:port
  std::string::size_type colon_pos = host_port.find(':');
  if (colon_pos != std::string::npos) {
    host = host_port.substr(0, colon_pos);
    port = host_port.substr(colon_pos + 1);
  } else {
    host = host_port;
    port = (proto == "wss") ? "443" : "80";
  }

  return true;
}

}  // namespace recorder
}  // namespace axon
