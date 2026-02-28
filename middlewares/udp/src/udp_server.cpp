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

bool UdpServer::start(
  const std::string& bind_address, const std::vector<UdpStreamConfig>& streams
) {
  if (running_.load()) {
    AXON_LOG_WARN("UDP server already running");
    return true;
  }

  std::lock_guard<std::mutex> lock(streams_mutex_);

  // Set running_ to true BEFORE starting async operations
  // so that async_receive() doesn't exit early
  running_.store(true);

  for (const auto& config : streams) {
    if (!config.enabled) {
      AXON_LOG_INFO("Stream " << kv("name", config.name) << " disabled, skipping");
      continue;
    }

    try {
      auto endpoint = std::make_unique<StreamEndpoint>();
      endpoint->config = config;
      endpoint->receive_buffer.resize(config.buffer_size);  // Use configured buffer size

      // Create and bind socket with configured bind address
      endpoint->socket = std::make_unique<boost::asio::ip::udp::socket>(
        io_context_,
        boost::asio::ip::udp::endpoint(boost::asio::ip::make_address(bind_address), config.port)
      );

      // Set socket buffer size
      boost::asio::socket_base::receive_buffer_size option(config.buffer_size);
      endpoint->socket->set_option(option);

      uint16_t port = config.port;
      streams_[port] = std::move(endpoint);

      AXON_LOG_INFO(
        "UDP stream started: " << kv("name", config.name) << kv("port", config.port)
                               << kv("topic", config.topic) << kv("bind_address", bind_address)
                               << kv("buffer_size", config.buffer_size)
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

void UdpServer::close_all_streams() {
  // Close all sockets and clear streams_ map.
  // Called both from stop() and from the catch block in start() to avoid
  // leaking sockets that were successfully opened before a partial failure.
  // Caller must hold streams_mutex_.
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
}

void UdpServer::stop() {
  if (!running_.load()) {
    // Even if not fully started, clean up any sockets that may have been
    // opened during a partial startup (e.g. after an exception in start()).
    std::lock_guard<std::mutex> lock(streams_mutex_);
    if (!streams_.empty()) {
      close_all_streams();
    }
    return;
  }

  running_.store(false);

  std::lock_guard<std::mutex> lock(streams_mutex_);
  close_all_streams();
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

  // Capture pointer instead of reference for async safety
  // The stream is owned by streams_ map and will remain valid until stop() completes
  StreamEndpoint* stream_ptr = &stream;

  stream.socket->async_receive_from(
    boost::asio::buffer(stream.receive_buffer),
    stream.sender_endpoint,
    [this, stream_ptr](const boost::system::error_code& ec, std::size_t bytes_transferred) {
      // Check if we're still running before processing
      if (!running_.load()) {
        return;
      }

      if (ec) {
        if (ec != boost::asio::error::operation_aborted) {
          AXON_LOG_WARN(
            "UDP receive error: " << kv("port", stream_ptr->config.port)
                                  << kv("error", ec.message())
          );
          {
            std::lock_guard<std::mutex> lock(stream_ptr->stats_mutex);
            stream_ptr->stats.parse_errors++;
          }
        }
        return;
      }

      handle_receive(*stream_ptr, bytes_transferred);

      // Continue receiving
      async_receive(*stream_ptr);
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
