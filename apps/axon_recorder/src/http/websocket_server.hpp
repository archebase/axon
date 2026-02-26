// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_RECORDER_WEBSOCKET_SERVER_HPP
#define AXON_RECORDER_WEBSOCKET_SERVER_HPP

#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <nlohmann/json.hpp>

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <set>
#include <shared_mutex>
#include <string>
#include <thread>
#include <unordered_map>

namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace net = boost::asio;

using tcp = boost::asio::ip::tcp;

namespace axon {
namespace recorder {

/**
 * Forward declarations
 */
class WebSocketSession;

/**
 * WebSocket message handler callback
 */
using MessageHandler =
  std::function<void(const std::string& client_id, const std::string& message)>;

/**
 * WebSocket Server for AxonRecorder
 *
 * Provides WebSocket-based real-time push notifications.
 * Supports multiple concurrent connections with event broadcasting.
 */
class WebSocketServer : public std::enable_shared_from_this<WebSocketServer> {
public:
  /**
   * Configuration for WebSocket server
   */
  struct Config {
    std::string host;
    uint16_t port;
    size_t max_connections;
    uint32_t ping_interval_ms;
    uint32_t ping_timeout_ms;
    uint32_t stats_interval_ms;
    size_t max_messages_per_second;

    Config()
        : host("0.0.0.0")
        , port(8080)
        , max_connections(100)
        , ping_interval_ms(30000)
        , ping_timeout_ms(10000)
        , stats_interval_ms(1000)
        , max_messages_per_second(100) {}
  };

  /**
   * Constructor
   * @param ioc Shared io_context for async operations
   * @param config Server configuration
   */
  explicit WebSocketServer(net::io_context& ioc, const Config& config = Config());

  // Non-copyable, non-movable
  WebSocketServer(const WebSocketServer&) = delete;
  WebSocketServer& operator=(const WebSocketServer&) = delete;
  WebSocketServer(WebSocketServer&&) = delete;
  WebSocketServer& operator=(WebSocketServer&&) = delete;

  /**
   * Destructor - stops the server
   */
  ~WebSocketServer();

  /**
   * Start accepting WebSocket connections
   * @return true on success, false on failure
   */
  bool start();

  /**
   * Stop the server and disconnect all clients
   */
  void stop();

  /**
   * Check if server is running
   */
  bool is_running() const;

  /**
   * Broadcast message to all connected clients
   * @param type Event type
   * @param data JSON data to broadcast
   */
  void broadcast(const std::string& type, const nlohmann::json& data);

  /**
   * Broadcast to clients subscribed to specific event type
   * @param event_type Event type to filter subscribers
   * @param type Message type
   * @param data JSON data
   */
  void broadcast_to_subscribers(
    const std::string& event_type, const std::string& type, const nlohmann::json& data
  );

  /**
   * Get number of connected clients
   */
  size_t connection_count() const;

  /**
   * Register message handler for client-to-server messages
   */
  void set_message_handler(MessageHandler handler);

  /**
   * Get the server URL
   */
  std::string get_url() const;

private:
  /**
   * Accept loop
   */
  void do_accept();

  /**
   * Handle incoming connection
   */
  void on_accept(beast::error_code ec, tcp::socket&& socket);

  /**
   * Add session to tracking
   */
  void add_session(std::shared_ptr<WebSocketSession> session);

  /**
   * Remove session from tracking
   */
  void remove_session(const std::string& client_id);

  /**
   * Check if rate limit is exceeded for client
   */
  bool check_rate_limit(const std::string& client_id);

  net::io_context& ioc_;
  Config config_;

  tcp::acceptor acceptor_;
  std::atomic<bool> running_{false};

  mutable std::shared_mutex sessions_mutex_;
  std::unordered_map<std::string, std::weak_ptr<WebSocketSession>> sessions_;

  std::atomic<size_t> connection_counter_{0};

  // Rate limiting
  struct ClientRateInfo {
    size_t message_count;
    std::chrono::steady_clock::time_point last_reset;

    ClientRateInfo()
        : message_count(0)
        , last_reset(std::chrono::steady_clock::now()) {}
  };
  std::unordered_map<std::string, ClientRateInfo> rate_limits_;
  mutable std::mutex rate_mutex_;

  MessageHandler message_handler_;
};

}  // namespace recorder
}  // namespace axon

#endif  // AXON_RECORDER_WEBSOCKET_SERVER_HPP
