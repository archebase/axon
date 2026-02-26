// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "websocket_server.hpp"

#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>

#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>

#define AXON_LOG_COMPONENT "websocket_server"
#include <axon_log_macros.hpp>

#include "websocket_session.hpp"

namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace net = boost::asio;

namespace {

std::string iso8601_now() {
  auto now = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now);
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

  std::tm tm_buf;
  localtime_r(&time_t_now, &tm_buf);

  std::ostringstream oss;
  oss << std::put_time(&tm_buf, "%Y-%m-%dT%H:%M:%S");
  oss << "." << std::setfill('0') << std::setw(3) << ms.count() << "Z";
  return oss.str();
}

}  // namespace

using tcp = boost::asio::ip::tcp;

namespace axon {
namespace recorder {

WebSocketServer::WebSocketServer(net::io_context& ioc, const Config& config)
    : ioc_(ioc)
    , config_(config)
    , acceptor_(ioc)
    , running_(false) {}

WebSocketServer::~WebSocketServer() {
  stop();
}

bool WebSocketServer::start() {
  if (running_.load()) {
    return true;
  }

  try {
    // Open acceptor
    tcp::endpoint endpoint(net::ip::address::from_string(config_.host), config_.port);
    acceptor_.open(endpoint.protocol());
    acceptor_.set_option(net::socket_base::reuse_address(true));
    acceptor_.bind(endpoint);
    acceptor_.listen(net::socket_base::max_listen_connections);

    std::cout << "[WebSocketServer] Listening on " << config_.host << ":" << config_.port
              << std::endl;

    AXON_LOG_INFO(
      "WebSocket server started" << axon::logging::kv("host", config_.host)
                                 << axon::logging::kv("port", config_.port)
    );

    running_.store(true);
    do_accept();

    return true;
  } catch (const std::exception& e) {
    AXON_LOG_ERROR("Failed to start WebSocket server" << axon::logging::kv("error", e.what()));
    return false;
  }
}

void WebSocketServer::stop() {
  if (!running_.load()) {
    return;
  }

  running_.store(false);

  // Close acceptor
  try {
    acceptor_.close();
  } catch (...) {
    // Ignore
  }

  // Disconnect all sessions
  {
    std::unique_lock lock(sessions_mutex_);
    for (auto& [id, weak_session] : sessions_) {
      if (auto session = weak_session.lock()) {
        session->close(1000, "Server shutdown");
      }
    }
    sessions_.clear();
  }
  AXON_LOG_INFO("WebSocket server stopped");
}

bool WebSocketServer::is_running() const {
  return running_.load();
}

void WebSocketServer::do_accept() {
  if (!running_.load()) {
    return;
  }

  acceptor_.async_accept([self = shared_from_this()](beast::error_code ec, tcp::socket socket) {
    self->on_accept(ec, std::move(socket));
  });
}

void WebSocketServer::on_accept(beast::error_code ec, tcp::socket&& socket) {
  if (!running_.load()) {
    return;
  }

  if (ec) {
    AXON_LOG_ERROR("WebSocket accept error" << axon::logging::kv("error", ec.message()));
    do_accept();
    return;
  }

  // Check max connections
  size_t conn_count = connection_count();
  if (conn_count >= config_.max_connections) {
    AXON_LOG_WARN(
      "Max connections reached, rejecting new connection"
      << axon::logging::kv("current", conn_count)
      << axon::logging::kv("max", config_.max_connections)
    );
    // Close the client socket, not the acceptor
    boost::system::error_code ec;
    socket.close(ec);
    do_accept();
    return;
  }

  // Generate client ID
  auto client_id = "conn_" + std::to_string(connection_counter_.fetch_add(1));

  // Get remote address before moving socket
  std::string remote_address;
  try {
    remote_address = socket.remote_endpoint().address().to_string();
  } catch (...) {
    remote_address = "unknown";
  }

  // Create and start session
  auto session = std::make_shared<WebSocketSession>(
    std::move(socket), client_id, config_.ping_interval_ms, config_.ping_timeout_ms
  );

  // Set message handler
  session->set_message_handler([self = shared_from_this(), client_id](const std::string& message) {
    if (self->message_handler_) {
      self->message_handler_(client_id, message);
    }
  });

  // Set close handler for disconnection notification
  session->set_close_handler([self = shared_from_this()](const std::string& client_id) {
    self->remove_session(client_id);
  });

  add_session(session);
  AXON_LOG_INFO(
    "Client connected" << axon::logging::kv("client_id", client_id)
                       << axon::logging::kv("remote_address", remote_address)
  );
  session->run();

  // Accept next connection
  do_accept();
}

void WebSocketServer::add_session(std::shared_ptr<WebSocketSession> session) {
  {
    std::unique_lock lock(sessions_mutex_);
    sessions_[session->client_id()] = session;
  }

  // Initialize rate limit entry
  {
    std::lock_guard lock(rate_mutex_);
    rate_limits_[session->client_id()] = ClientRateInfo();
  }
}

void WebSocketServer::remove_session(const std::string& client_id) {
  {
    std::unique_lock lock(sessions_mutex_);
    sessions_.erase(client_id);
  }

  {
    std::lock_guard lock(rate_mutex_);
    rate_limits_.erase(client_id);
  }

  AXON_LOG_INFO("Client disconnected" << axon::logging::kv("client_id", client_id));
}

bool WebSocketServer::check_rate_limit(const std::string& client_id) {
  std::lock_guard lock(rate_mutex_);

  auto it = rate_limits_.find(client_id);
  if (it == rate_limits_.end()) {
    return true;
  }

  auto now = std::chrono::steady_clock::now();
  auto& info = it->second;

  // Reset counter every second
  if (now - info.last_reset > std::chrono::seconds(1)) {
    info.message_count = 0;
    info.last_reset = now;
  }

  if (info.message_count >= config_.max_messages_per_second) {
    return false;
  }

  info.message_count++;
  return true;
}

void WebSocketServer::broadcast(const std::string& type, const nlohmann::json& data) {
  auto message =
    nlohmann::json{{"type", type}, {"timestamp", iso8601_now()}, {"data", data}}.dump();

  std::shared_lock lock(sessions_mutex_);
  for (auto& [id, weak_session] : sessions_) {
    if (auto session = weak_session.lock()) {
      if (session->is_open()) {
        session->send(message);
      }
    }
  }
}

void WebSocketServer::broadcast_to_subscribers(
  const std::string& event_type, const std::string& type, const nlohmann::json& data
) {
  auto message =
    nlohmann::json{{"type", type}, {"timestamp", iso8601_now()}, {"data", data}}.dump();

  std::shared_lock lock(sessions_mutex_);
  for (auto& [id, weak_session] : sessions_) {
    if (auto session = weak_session.lock()) {
      if (session->is_open() && session->is_subscribed(event_type)) {
        session->send(message);
      }
    }
  }
}

size_t WebSocketServer::connection_count() const {
  std::shared_lock lock(sessions_mutex_);
  size_t count = 0;
  for (auto& [id, weak_session] : sessions_) {
    if (weak_session.lock()) {
      count++;
    }
  }
  return count;
}

void WebSocketServer::set_message_handler(MessageHandler handler) {
  message_handler_ = std::move(handler);
}

std::string WebSocketServer::get_url() const {
  return "ws://" + config_.host + ":" + std::to_string(config_.port) + "/ws";
}

}  // namespace recorder
}  // namespace axon
