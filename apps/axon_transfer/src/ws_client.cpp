// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "ws_client.hpp"

#include <boost/beast/websocket.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <random>

namespace axon {
namespace transfer {

namespace {

std::string random_jitter(double factor) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(-factor, factor);
  return std::to_string(dis(gen));
}

}  // namespace

WsClient::WsClient(net::io_context& ioc, const WsConfig& config)
    : config_(config)
    , strand_(net::make_strand(ioc))
    , resolver_(strand_)
    , ws_(strand_)
    , reconnect_timer_(strand_)
    , ping_timer_(strand_) {
  ws_.control_callback([](beast::websocket::frame_type type, beast::string_view) {
    if (type == beast::websocket::frame_type::pong) {
    }
  });
}

void WsClient::start() {
  net::post(strand_, [this]() {
    stopped_ = false;
    connected_ = false;
    reconnect_attempt_ = 0;
    do_resolve();
  });
}

void WsClient::stop() {
  net::dispatch(strand_, [this]() {
    stopped_ = true;
    connected_ = false;
    reconnect_timer_.cancel();
    ping_timer_.cancel();

    beast::error_code ec;
    if (ws_.is_open()) {
      ws_.close(beast::websocket::close_code::normal, ec);
    }
    beast::get_lowest_layer(ws_).socket().shutdown(tcp::socket::shutdown_both, ec);
    beast::get_lowest_layer(ws_).socket().close(ec);
  });
}

bool WsClient::is_connected() const {
  return connected_.load();
}

void WsClient::set_message_handler(MessageHandler handler) {
  message_handler_ = std::move(handler);
}

void WsClient::set_connect_handler(ConnectHandler handler) {
  connect_handler_ = std::move(handler);
}

void WsClient::set_disconnect_handler(DisconnectHandler handler) {
  disconnect_handler_ = std::move(handler);
}

void WsClient::send(const nlohmann::json& msg) {
  {
    std::lock_guard<std::mutex> lock(write_mutex_);
    write_queue_.push(msg.dump());
  }
  net::post(strand_, [self = this]() {
    self->do_write();
  });
}

void WsClient::do_resolve() {
  if (stopped_) {
    return;
  }

  auto host_port = config_.url.substr(config_.url.find("://") + 3);

  auto colon_pos = host_port.find(':');
  auto path_pos = host_port.find('/');

  std::string host = host_port.substr(0, colon_pos != std::string::npos ? colon_pos : path_pos);
  std::string port = colon_pos != std::string::npos
                       ? host_port.substr(colon_pos + 1, path_pos - colon_pos - 1)
                       : "80";

  resolver_.async_resolve(host, port, beast::bind_front_handler(&WsClient::on_resolve, this));
}

void WsClient::on_resolve(beast::error_code ec, tcp::resolver::results_type results) {
  if (stopped_) {
    return;
  }

  if (ec) {
    std::cerr << "WsClient resolve error: " << ec.message() << "\n";
    schedule_reconnect();
    return;
  }

  do_connect(results);
}

void WsClient::do_connect(tcp::resolver::results_type results) {
  if (stopped_) {
    return;
  }

  beast::get_lowest_layer(ws_).async_connect(
    results, beast::bind_front_handler(&WsClient::on_connect, this)
  );
}

void WsClient::on_connect(beast::error_code ec, tcp::endpoint endpoint) {
  if (stopped_) {
    return;
  }

  if (ec) {
    std::cerr << "WsClient connect error: " << ec.message() << "\n";
    schedule_reconnect();
    return;
  }

  (void)endpoint;
  do_handshake();
}

void WsClient::do_handshake() {
  if (stopped_) {
    return;
  }

  auto host_port = config_.url.substr(config_.url.find("://") + 3);
  auto path_pos = host_port.find('/');
  std::string host = host_port.substr(0, path_pos);
  std::string path = path_pos != std::string::npos ? host_port.substr(path_pos) : "/";

  ws_.async_handshake(host, path, beast::bind_front_handler(&WsClient::on_handshake, this));
}

void WsClient::on_handshake(beast::error_code ec) {
  if (ec) {
    std::cerr << "WsClient handshake error: " << ec.message() << "\n";
    schedule_reconnect();
    return;
  }

  connected_ = true;
  reconnect_attempt_ = 0;

  if (connect_handler_) {
    connect_handler_();
  }

  start_ping_timer();
  do_read();
}

void WsClient::do_read() {
  if (stopped_ || !connected_) {
    return;
  }

  ws_.async_read(read_buffer_, beast::bind_front_handler(&WsClient::on_read, this));
}

void WsClient::on_read(beast::error_code ec, std::size_t bytes) {
  if (ec) {
    std::cerr << "WsClient read error: " << ec.message() << "\n";
    on_disconnect(ec);
    return;
  }

  std::string data = beast::buffers_to_string(read_buffer_.data());
  read_buffer_.consume(read_buffer_.size());

  try {
    auto json = nlohmann::json::parse(data);
    if (message_handler_) {
      message_handler_(json);
    }
  } catch (const std::exception& e) {
    std::cerr << "WsClient JSON parse error: " << e.what() << "\n";
  }

  do_read();
}

void WsClient::do_write() {
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

  ws_.async_write(net::buffer(msg), beast::bind_front_handler(&WsClient::on_write, this));
}

void WsClient::on_write(beast::error_code ec, std::size_t bytes) {
  writing_ = false;

  if (ec) {
    std::cerr << "WsClient write error: " << ec.message() << "\n";
    on_disconnect(ec);
    return;
  }

  (void)bytes;

  std::lock_guard<std::mutex> lock(write_mutex_);
  if (!write_queue_.empty()) {
    net::post(strand_, [self = this]() {
      self->do_write();
    });
  }
}

void WsClient::on_disconnect(beast::error_code ec) {
  net::dispatch(strand_, [this, ec]() {
    if (stopped_) {
      return;
    }

    if (!connected_.exchange(false)) {
      return;
    }

    ping_timer_.cancel();
    ping_pending_ = false;

    beast::error_code close_ec;
    if (ws_.is_open()) {
      ws_.close(beast::websocket::close_code::normal, close_ec);
    }
    beast::get_lowest_layer(ws_).socket().shutdown(tcp::socket::shutdown_both, close_ec);
    beast::get_lowest_layer(ws_).socket().close(close_ec);

    if (disconnect_handler_) {
      disconnect_handler_();
    }

    schedule_reconnect();
  });
}

void WsClient::schedule_reconnect() {
  if (stopped_) {
    return;
  }

  long long delay = static_cast<long long>(config_.reconnect.initial_delay_ms.count());
  delay = static_cast<long long>(
    delay * std::pow(config_.reconnect.backoff_multiplier, reconnect_attempt_)
  );
  delay = std::min(delay, static_cast<long long>(config_.reconnect.max_delay_ms.count()));

  auto jitter = static_cast<long long>(
    delay * config_.reconnect.jitter_factor * (std::rand() % 100 - 50) / 50.0
  );
  delay += jitter;
  delay = std::max(100LL, delay);

  ++reconnect_attempt_;

  std::cout << "WsClient: scheduling reconnect in " << delay << "ms (attempt " << reconnect_attempt_
            << ")\n";

  reconnect_timer_.expires_after(std::chrono::milliseconds(delay));
  reconnect_timer_.async_wait([this](beast::error_code ec) {
    if (ec || stopped_) {
      return;
    }
    do_resolve();
  });
}

void WsClient::start_ping_timer() {
  if (config_.ping_interval_ms.count() <= 0) {
    return;
  }

  ping_timer_.expires_after(config_.ping_interval_ms);
  ping_timer_.async_wait([this](beast::error_code ec) {
    if (ec || stopped_ || !connected_) {
      return;
    }

    ws_.async_ping(beast::websocket::ping_data{}, [this](beast::error_code ping_ec) {
      if (ping_ec) {
        std::cerr << "WsClient: ping error: " << ping_ec.message() << "\n";
        on_disconnect(ping_ec);
        return;
      }
      start_ping_timer();
    });
  });
}

void WsClient::on_ping_timer(beast::error_code ec) {
  if (ec || stopped_ || !connected_) {
    return;
  }

  ws_.async_ping(beast::websocket::ping_data{}, [this](beast::error_code ping_ec) {
    if (ping_ec) {
      on_disconnect(ping_ec);
      return;
    }
    start_ping_timer();
  });
}

}  // namespace transfer
}  // namespace axon
