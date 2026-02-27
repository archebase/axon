// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "axon_udp/udp_server.hpp"

#include <cstring>

#define AXON_LOG_COMPONENT "udp_server"
#include <axon_log_macros.hpp>

using axon::logging::kv;

namespace axon {
namespace udp {

UdpServer::UdpServer(boost::asio::io_context& io_context)
    : io_context_(io_context) {}

UdpServer::~UdpServer() {
  stop();
}

bool UdpServer::start(const std::vector<UdpStreamConfig>& streams) {
  if (running_.load()) {
    AXON_LOG_WARN("UDP server already running");
    return true;
  }

  std::lock_guard<std::mutex> lock(streams_mutex_);

  // Set running_ to true BEFORE starting async operations
  // so that async_receive() doesn't exit early
  running_.store(true);

  boost::asio::ip::udp::endpoint listen_endpoint;

  for (const auto& config : streams) {
    if (!config.enabled) {
      AXON_LOG_INFO("Stream " << kv("name", config.name) << " disabled, skipping");
      continue;
    }

    try {
      auto endpoint = std::make_unique<StreamEndpoint>();
      endpoint->config = config;
      endpoint->receive_buffer.resize(65536);  // Max UDP payload size

      // Create and bind socket
      endpoint->socket = std::make_unique<boost::asio::ip::udp::socket>(
        io_context_,
        boost::asio::ip::udp::endpoint(boost::asio::ip::make_address("0.0.0.0"), config.port)
      );

      // Set socket buffer size
      boost::asio::socket_base::receive_buffer_size option(65536);
      endpoint->socket->set_option(option);

      uint16_t port = config.port;
      streams_[port] = std::move(endpoint);

      AXON_LOG_INFO(
        "UDP stream started: " << kv("name", config.name) << kv("port", config.port)
                               << kv("topic", config.topic)
      );

      // Start async receive
      async_receive(*streams_[port]);

    } catch (const std::exception& e) {
      AXON_LOG_ERROR(
        "Failed to start UDP stream " << kv("name", config.name) << kv("port", config.port)
                                      << kv("error", e.what())
      );
      running_.store(false);  // Reset on error
      return false;
    }
  }

  return true;
}

void UdpServer::stop() {
  if (!running_.load()) {
    return;
  }

  running_.store(false);

  std::lock_guard<std::mutex> lock(streams_mutex_);

  // Close all sockets
  for (auto& [port, endpoint] : streams_) {
    if (endpoint && endpoint->socket) {
      boost::system::error_code ec;
      endpoint->socket->close(ec);
      if (ec) {
        AXON_LOG_WARN("Error closing socket: " << kv("port", port) << kv("error", ec.message()));
      }
    }
  }

  streams_.clear();
  AXON_LOG_INFO("UDP server stopped");
}

void UdpServer::set_message_callback(MessageCallback callback) {
  message_callback_ = std::move(callback);
}

UdpStreamStats UdpServer::get_stats(uint16_t port) const {
  std::lock_guard<std::mutex> lock(streams_mutex_);

  auto it = streams_.find(port);
  if (it == streams_.end()) {
    return UdpStreamStats{};
  }

  std::lock_guard<std::mutex> stats_lock(it->second->stats_mutex);
  return it->second->stats;
}

std::unordered_map<uint16_t, UdpStreamStats> UdpServer::get_all_stats() const {
  std::lock_guard<std::mutex> lock(streams_mutex_);

  std::unordered_map<uint16_t, UdpStreamStats> result;
  for (const auto& [port, endpoint] : streams_) {
    std::lock_guard<std::mutex> stats_lock(endpoint->stats_mutex);
    result[port] = endpoint->stats;
  }
  return result;
}

void UdpServer::async_receive(StreamEndpoint& stream) {
  if (!running_.load() || !stream.socket) {
    return;
  }

  stream.socket->async_receive_from(
    boost::asio::buffer(stream.receive_buffer),
    stream.sender_endpoint,
    [this, &stream](const boost::system::error_code& ec, std::size_t bytes_transferred) {
      if (ec) {
        if (ec != boost::asio::error::operation_aborted) {
          AXON_LOG_WARN(
            "UDP receive error: " << kv("port", stream.config.port) << kv("error", ec.message())
          );
          {
            std::lock_guard<std::mutex> lock(stream.stats_mutex);
            stream.stats.parse_errors++;
          }
        }
        return;
      }

      handle_receive(stream, bytes_transferred);

      // Continue receiving
      async_receive(stream);
    }
  );
}

void UdpServer::handle_receive(StreamEndpoint& stream, std::size_t bytes_transferred) {
  // Update statistics
  {
    std::lock_guard<std::mutex> lock(stream.stats_mutex);
    stream.stats.packets_received++;
    stream.stats.bytes_received += bytes_transferred;
  }

  // Check buffer overrun
  if (bytes_transferred > stream.receive_buffer.size()) {
    std::lock_guard<std::mutex> lock(stream.stats_mutex);
    stream.stats.buffer_overruns++;
    AXON_LOG_WARN(
      "UDP buffer overrun: " << kv("port", stream.config.port) << kv("size", bytes_transferred)
    );
    return;
  }

  // Call message callback if set
  if (message_callback_) {
    // Get current time as default timestamp
    auto now = std::chrono::system_clock::now();
    uint64_t arrival_time =
      std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();

    message_callback_(
      stream.config.topic, stream.receive_buffer.data(), bytes_transferred, arrival_time
    );
  }
}

}  // namespace udp
}  // namespace axon
