// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_TRANSFER_WS_CLIENT_HPP
#define AXON_TRANSFER_WS_CLIENT_HPP

#include <boost/asio.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <nlohmann/json.hpp>

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <string>

#include "transfer_config.hpp"

namespace beast = boost::beast;
namespace net = boost::asio;
using tcp = boost::asio::ip::tcp;

namespace axon {
namespace transfer {

class WsClient {
public:
  using MessageHandler = std::function<void(const nlohmann::json&)>;
  using ConnectHandler = std::function<void()>;
  using DisconnectHandler = std::function<void()>;

  WsClient(net::io_context& ioc, const WsConfig& config);

  void start();
  void stop();

  void send(const nlohmann::json& msg);

  void set_message_handler(MessageHandler handler);
  void set_connect_handler(ConnectHandler handler);
  void set_disconnect_handler(DisconnectHandler handler);

  bool is_connected() const;

private:
  void do_resolve();
  void on_resolve(beast::error_code ec, tcp::resolver::results_type results);
  void do_connect(tcp::resolver::results_type results);
  void on_connect(beast::error_code ec, tcp::endpoint endpoint);
  void do_handshake();
  void on_handshake(beast::error_code ec);

  void do_read();
  void on_read(beast::error_code ec, std::size_t bytes);

  void do_write();
  void on_write(beast::error_code ec, std::size_t bytes);

  void on_disconnect(beast::error_code ec);
  void schedule_reconnect();

  void start_ping_timer();
  void on_ping_timer(beast::error_code ec);

  WsConfig config_;
  net::strand<net::io_context::executor_type> strand_;
  tcp::resolver resolver_;
  beast::websocket::stream<beast::tcp_stream> ws_;
  beast::flat_buffer read_buffer_;

  std::queue<std::string> write_queue_;
  std::mutex write_mutex_;
  std::atomic<bool> writing_{false};

  net::steady_timer reconnect_timer_;
  net::steady_timer ping_timer_;
  std::atomic<bool> ping_pending_{false};

  uint32_t reconnect_attempt_{0};
  std::atomic<bool> connected_{false};
  std::atomic<bool> stopped_{false};

  MessageHandler message_handler_;
  ConnectHandler connect_handler_;
  DisconnectHandler disconnect_handler_;
};

}  // namespace transfer
}  // namespace axon

#endif  // AXON_TRANSFER_WS_CLIENT_HPP
