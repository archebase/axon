// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_RECORDER_WS_RPC_CLIENT_HPP
#define AXON_RECORDER_WS_RPC_CLIENT_HPP

#include <boost/asio.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <nlohmann/json.hpp>

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <string>

#include "../core/recorder.hpp"
#include "rpc_handlers.hpp"

namespace beast = boost::beast;
namespace net = boost::asio;
using tcp = boost::asio::ip::tcp;

namespace axon {
namespace recorder {

/**
 * WebSocket RPC client for connecting to keystone server
 *
 * This client connects to a keystone server via WebSocket and receives
 * RPC commands (config, begin, pause, resume, finish, cancel, etc.).
 * It reuses the same RpcCallbacks and handle_rpc_* functions as the HTTP server.
 *
 * Architecture:
 * 1. Connect to keystone server via WebSocket
 * 2. Receive RPC commands as JSON messages
 * 3. Execute commands via RpcCallbacks (same as HTTP server)
 * 4. Send responses and state updates back to keystone
 * 5. Automatic reconnection with exponential backoff
 */
class WsRpcClient {
public:
  using StateUpdateCallback = std::function<void(RecorderState, RecorderState, const std::string&)>;

  WsRpcClient(net::io_context& ioc, const WsClientConfig& config);
  ~WsRpcClient();

  // Non-copyable, non-movable
  WsRpcClient(const WsRpcClient&) = delete;
  WsRpcClient& operator=(const WsRpcClient&) = delete;

  /**
   * Start the WebSocket client
   * Begins connection process (resolve -> connect -> handshake)
   */
  void start();

  /**
   * Stop the WebSocket client
   * Closes connection and stops reconnection attempts
   */
  void stop();

  /**
   * Register RPC callbacks (same as HTTP server)
   */
  void register_callbacks(const RpcCallbacks& callbacks);

  /**
   * Send state update to keystone server
   * Called automatically on state transitions
   */
  void send_state_update(RecorderState from, RecorderState to, const std::string& task_id);

  /**
   * Check if connected to keystone
   */
  bool is_connected() const;

private:
  // Connection management
  void do_resolve();
  void on_resolve(beast::error_code ec, tcp::resolver::results_type results);
  void do_connect(tcp::resolver::results_type results);
  void on_connect(beast::error_code ec, tcp::endpoint endpoint);
  void do_handshake();
  void on_handshake(beast::error_code ec);

  // Read/write
  void do_read();
  void on_read(beast::error_code ec, std::size_t bytes);
  void do_write();
  void on_write(beast::error_code ec, std::size_t bytes);

  // Message handling
  void handle_server_message(const nlohmann::json& msg);
  void send_rpc_response(const std::string& request_id, const RpcResponse& response);
  void send_message(const nlohmann::json& msg);

  // Connection lifecycle
  void on_disconnect(beast::error_code ec);
  void schedule_reconnect();

  // Keepalive
  void start_ping_timer();
  void on_ping_timer(beast::error_code ec);

  // URL parsing
  static bool parse_url(
    const std::string& url, std::string& host, std::string& port, std::string& path
  );

  // Configuration
  WsClientConfig config_;

  // ASIO components
  tcp::resolver resolver_;
  beast::websocket::stream<beast::tcp_stream> ws_;
  net::strand<net::io_context::executor_type> strand_;
  beast::flat_buffer read_buffer_;

  // Write queue
  std::queue<std::string> write_queue_;
  std::mutex write_mutex_;
  std::atomic<bool> writing_{false};

  // Timers
  net::steady_timer reconnect_timer_;
  net::steady_timer ping_timer_;

  // State
  std::atomic<bool> connected_{false};
  std::atomic<bool> stopped_{false};
  uint32_t reconnect_attempt_{0};

  // Callbacks
  RpcCallbacks callbacks_;
};

}  // namespace recorder
}  // namespace axon

#endif  // AXON_RECORDER_WS_RPC_CLIENT_HPP
