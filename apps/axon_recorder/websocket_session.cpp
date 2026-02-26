// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "websocket_session.hpp"

#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/websocket.hpp>

#include <iomanip>
#include <iostream>
#include <sstream>

#define AXON_LOG_COMPONENT "websocket_session"
#include <axon_log_macros.hpp>

#include "version.hpp"

namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace net = boost::asio;

namespace axon {
namespace recorder {

WebSocketSession::WebSocketSession(
  tcp::socket&& socket, const std::string& client_id, uint32_t ping_interval_ms,
  uint32_t ping_timeout_ms
)
    : ws_(std::move(socket))
    , client_id_(client_id)
    , ping_interval_ms_(ping_interval_ms)
    , ping_timeout_ms_(ping_timeout_ms)
    , ping_timer_(ws_.get_executor())
    , writing_(false)
    , closed_(false) {}

void WebSocketSession::run() {
  // Set WebSocket options
  ws_.set_option(websocket::stream_base::timeout::suggested(beast::role_type::server));

  // Set per-message deflate
  ws_.set_option(websocket::stream_base::decorator([](websocket::response_type& res) {
    res.set(http::field::server, std::string(BOOST_BEAST_VERSION_STRING) + " axon-websocket");
  }));

  // Accept the WebSocket handshake
  ws_.async_accept([self = shared_from_this()](beast::error_code ec) {
    self->on_accept(ec);
  });
}

void WebSocketSession::on_accept(beast::error_code ec) {
  if (ec) {
    AXON_LOG_ERROR(
      "WebSocket handshake failed" << axon::logging::kv("client_id", client_id_)
                                   << axon::logging::kv("error", ec.message())
    );
    return;
  }

  AXON_LOG_INFO("Client connected" << axon::logging::kv("client_id", client_id_));

  // Send connected message
  auto connected_msg =
    build_message("connected", {{"client_id", client_id_}, {"version", get_version()}});
  send(connected_msg);

  // Start ping timer
  start_ping_timer();

  // Start reading
  do_read();
}

void WebSocketSession::do_read() {
  if (closed_.load()) {
    return;
  }

  ws_.async_read(
    read_buffer_, [self = shared_from_this()](beast::error_code ec, std::size_t bytes) {
      self->on_read(ec, bytes);
    }
  );
}

void WebSocketSession::on_read(beast::error_code ec, std::size_t bytes_transferred) {
  if (ec == websocket::error::closed) {
    AXON_LOG_INFO(
      "Client disconnected" << axon::logging::kv("client_id", client_id_)
                            << axon::logging::kv("reason", "client_close")
    );
    // Notify server about disconnection
    if (close_handler_) {
      close_handler_(client_id_);
    }
    return;
  }

  if (ec) {
    AXON_LOG_ERROR(
      "Read error from client" << axon::logging::kv("client_id", client_id_)
                               << axon::logging::kv("error", ec.message())
    );
    return;
  }

  // Get the message
  auto data = beast::buffers_to_string(read_buffer_.data());
  read_buffer_.consume(read_buffer_.size());

  // Handle the message
  handle_text_message(data);

  // Continue reading
  do_read();
}

void WebSocketSession::handle_text_message(const std::string& message) {
  try {
    auto j = nlohmann::json::parse(message);

    // Handle action-based messages
    if (j.contains("action")) {
      std::string action = j["action"];

      if (action == "subscribe") {
        if (j.contains("events") && j["events"].is_array()) {
          for (const auto& event : j["events"]) {
            subscribe(event.get<std::string>());
          }
        }
        std::cout << "[WebSocketSession] Client subscribed: " << client_id_ << std::endl;
      } else if (action == "unsubscribe") {
        if (j.contains("events") && j["events"].is_array()) {
          for (const auto& event : j["events"]) {
            unsubscribe(event.get<std::string>());
          }
        }
      } else if (action == "get_state") {
        // Request current state - this would need a callback
        AXON_LOG_DEBUG("get_state requested" << axon::logging::kv("client_id", client_id_));
      } else if (action == "ping") {
        // Handle client ping
        handle_ping(message);
      }
    }

    // Call message handler if set
    if (message_handler_) {
      message_handler_(message);
    }

  } catch (const std::exception& e) {
    AXON_LOG_ERROR(
      "Message parse error" << axon::logging::kv("client_id", client_id_)
                            << axon::logging::kv("error", e.what())
    );
  }
}

void WebSocketSession::handle_ping(const std::string& /*data*/) {
  // Respond with pong
  try {
    ws_.async_pong(websocket::ping_data{}, [](beast::error_code /*ec*/) {
      // Pong sent
    });
  } catch (...) {
    // Ignore
  }
}

void WebSocketSession::send(const std::string& message) {
  if (closed_.load()) {
    return;
  }

  {
    std::lock_guard lock(write_mutex_);
    write_queue_.push(message);
  }

  // Start writing if not already
  if (!writing_.load()) {
    do_write();
  }
}

void WebSocketSession::send_message(const std::string& type, const nlohmann::json& data) {
  auto message = build_message(type, data);
  send(message);
}

void WebSocketSession::do_write() {
  if (closed_.load()) {
    return;
  }

  std::string message;
  {
    std::lock_guard lock(write_mutex_);
    if (write_queue_.empty()) {
      writing_.store(false);
      return;
    }
    message = write_queue_.front();
    write_queue_.pop();
  }

  writing_.store(true);

  ws_.async_write(
    net::buffer(message), [self = shared_from_this()](beast::error_code ec, std::size_t bytes) {
      self->on_write(ec, bytes);
    }
  );
}

void WebSocketSession::on_write(beast::error_code ec, std::size_t /*bytes_transferred*/) {
  if (ec == websocket::error::closed) {
    writing_.store(false);
    return;
  }

  if (ec) {
    AXON_LOG_ERROR(
      "Write error" << axon::logging::kv("client_id", client_id_)
                    << axon::logging::kv("error", ec.message())
    );
    writing_.store(false);
    return;
  }

  // Write next message
  do_write();
}

void WebSocketSession::start_ping_timer() {
  ping_timer_.expires_after(std::chrono::milliseconds(ping_interval_ms_));
  ping_timer_.async_wait([self = shared_from_this()](beast::error_code ec) {
    if (!ec) {
      self->on_ping_timeout(ec);
    }
  });
}

void WebSocketSession::on_ping_timeout(beast::error_code ec) {
  if (ec == net::error::operation_aborted) {
    return;
  }

  if (closed_.load()) {
    return;
  }

  if (ping_pending_.load()) {
    // Previous ping not answered, close connection
    AXON_LOG_WARN("Ping timeout, closing connection" << axon::logging::kv("client_id", client_id_));
    close(1006, "Ping timeout");
    return;
  }

  // Send ping
  ping_pending_.store(true);
  ws_.async_ping(websocket::ping_data{}, [self = shared_from_this()](beast::error_code ec) {
    self->ping_pending_.store(false);
    if (!ec) {
      self->on_pong(ec);
    }
  });

  // Schedule next ping
  start_ping_timer();
}

void WebSocketSession::on_pong(beast::error_code ec) {
  if (ec) {
    AXON_LOG_ERROR(
      "Pong error" << axon::logging::kv("client_id", client_id_)
                   << axon::logging::kv("error", ec.message())
    );
  }
  // Pong received, reset timer will handle next ping
}

void WebSocketSession::close(uint16_t code, const std::string& reason) {
  bool expected = false;
  if (!closed_.compare_exchange_strong(expected, true)) {
    return;  // Already closed
  }

  try {
    ping_timer_.cancel();
    ws_.close(websocket::close_reason{static_cast<websocket::close_code>(code), reason});
  } catch (...) {
    // Ignore
  }
}

std::string WebSocketSession::build_message(const std::string& type, const nlohmann::json& data) {
  auto now = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now);
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

  std::ostringstream oss;
  oss << std::put_time(std::localtime(&time_t_now), "%Y-%m-%dT%H:%M:%S");
  oss << "." << std::setfill('0') << std::setw(3) << ms.count() << "Z";

  nlohmann::json msg;
  msg["type"] = type;
  msg["timestamp"] = oss.str();
  msg["data"] = data;

  return msg.dump();
}

std::set<std::string> WebSocketSession::subscribed_events() const {
  std::shared_lock lock(subs_mutex_);
  return subscriptions_;
}

void WebSocketSession::subscribe(const std::string& event) {
  std::unique_lock lock(subs_mutex_);
  subscriptions_.insert(event);
}

void WebSocketSession::unsubscribe(const std::string& event) {
  std::unique_lock lock(subs_mutex_);
  subscriptions_.erase(event);
}

bool WebSocketSession::is_subscribed(const std::string& event) const {
  std::shared_lock lock(subs_mutex_);
  return subscriptions_.count(event) > 0;
}

const std::string& WebSocketSession::client_id() const {
  return client_id_;
}

bool WebSocketSession::is_open() const {
  return !closed_.load() && ws_.is_open();
}

void WebSocketSession::set_message_handler(MessageHandler handler) {
  message_handler_ = std::move(handler);
}

void WebSocketSession::set_close_handler(CloseHandler handler) {
  close_handler_ = std::move(handler);
}

}  // namespace recorder
}  // namespace axon
