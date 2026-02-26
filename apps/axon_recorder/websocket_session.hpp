// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_RECORDER_WEBSOCKET_SESSION_HPP
#define AXON_RECORDER_WEBSOCKET_SESSION_HPP

#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <nlohmann/json.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <queue>
#include <set>
#include <shared_mutex>
#include <string>

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;

using tcp = boost::asio::ip::tcp;

namespace axon {
namespace recorder {

/**
 * WebSocket Session for a single client connection
 *
 * Handles WebSocket handshake, message reading/writing,
 * ping/pong for keepalive, and subscription management.
 */
class WebSocketSession : public std::enable_shared_from_this<WebSocketSession> {
public:
  /**
   * Constructor
   * @param socket TCP socket for the connection
   * @param client_id Unique identifier for this client
   * @param ping_interval_ms Interval for server-initiated ping
   * @param ping_timeout_ms Timeout for client pong response
   */
  WebSocketSession(
    tcp::socket&& socket, const std::string& client_id, uint32_t ping_interval_ms = 30000,
    uint32_t ping_timeout_ms = 10000
  );

  // Non-copyable
  WebSocketSession(const WebSocketSession&) = delete;
  WebSocketSession& operator=(const WebSocketSession&) = delete;

  /**
   * Start the WebSocket session
   */
  void run();

  /**
   * Send a message to this client
   * @param message JSON message to send
   */
  void send(const std::string& message);

  /**
   * Send a structured message
   * @param type Event type
   * @param data JSON data
   */
  void send_message(const std::string& type, const nlohmann::json& data);

  /**
   * Close the connection
   * @param code Close code (default: 1000 normal close)
   * @param reason Close reason message
   */
  void close(uint16_t code = 1000, const std::string& reason = "Normal close");

  /**
   * Get subscribed events
   */
  std::set<std::string> subscribed_events() const;

  /**
   * Subscribe to an event type
   */
  void subscribe(const std::string& event);

  /**
   * Unsubscribe from an event type
   */
  void unsubscribe(const std::string& event);

  /**
   * Check if subscribed to event
   */
  bool is_subscribed(const std::string& event) const;

  /**
   * Get client ID
   */
  const std::string& client_id() const;

  /**
   * Check if connection is still open
   */
  bool is_open() const;

  /**
   * Callback for incoming messages
   */
  using MessageHandler = std::function<void(const std::string& message)>;
  void set_message_handler(MessageHandler handler);

  /**
   * Callback for session close event
   */
  using CloseHandler = std::function<void(const std::string& client_id)>;
  void set_close_handler(CloseHandler handler);

private:
  /**
   * Perform WebSocket handshake
   */
  void on_accept(beast::error_code ec);

  /**
   * Start reading messages from client
   */
  void do_read();

  /**
   * Handle received message
   */
  void on_read(beast::error_code ec, std::size_t bytes_transferred);

  /**
   * Write queued messages
   */
  void do_write();

  /**
   * Handle write completion
   */
  void on_write(beast::error_code ec, std::size_t bytes_transferred);

  /**
   * Start ping timer
   */
  void start_ping_timer();

  /**
   * Handle ping timeout
   */
  void on_ping_timeout(beast::error_code ec);

  /**
   * Handle incoming pong
   */
  void on_pong(beast::error_code ec);

  /**
   * Handle client-initiated ping
   */
  void handle_ping(const std::string& data);

  /**
   * Handle incoming text message
   */
  void handle_text_message(const std::string& message);

  /**
   * Build JSON message with type and timestamp
   */
  std::string build_message(const std::string& type, const nlohmann::json& data);

  websocket::stream<beast::tcp_stream> ws_;
  beast::flat_buffer read_buffer_;

  std::string client_id_;
  uint32_t ping_interval_ms_;
  uint32_t ping_timeout_ms_;

  net::steady_timer ping_timer_;
  std::atomic<bool> ping_pending_{false};

  mutable std::shared_mutex subs_mutex_;
  std::set<std::string> subscriptions_;

  // Write queue
  std::queue<std::string> write_queue_;
  std::atomic<bool> writing_{false};
  std::mutex write_mutex_;

  MessageHandler message_handler_;

  CloseHandler close_handler_;

  std::atomic<bool> closed_{false};
};

}  // namespace recorder
}  // namespace axon

#endif  // AXON_RECORDER_WEBSOCKET_SESSION_HPP
