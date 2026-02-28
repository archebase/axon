// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_UDP_UDP_SERVER_HPP
#define AXON_UDP_UDP_SERVER_HPP

#include <boost/asio.hpp>

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace axon {
namespace udp {

/// UDP stream configuration
struct UdpStreamConfig {
  std::string name;            // Stream identifier
  uint16_t port;               // UDP port to listen on
  std::string topic;           // MCAP topic name
  std::string schema_name;     // Schema identifier (or "raw_json" for passthrough)
  bool enabled = true;         // Stream active flag
  size_t buffer_size = 65536;  // Receive buffer size
};

/// Per-stream statistics
struct UdpStreamStats {
  uint64_t packets_received = 0;
  uint64_t bytes_received = 0;
  uint64_t parse_errors = 0;
  uint64_t buffer_overruns = 0;
};

/// UDP server for receiving JSON messages
class UdpServer {
public:
  using MessageCallback = std::function<
    void(const std::string& topic, const uint8_t* data, size_t size, uint64_t timestamp)>;

  explicit UdpServer(boost::asio::io_context& io_context);
  ~UdpServer();

  // Non-copyable
  UdpServer(const UdpServer&) = delete;
  UdpServer& operator=(const UdpServer&) = delete;

  /// Start listening on configured streams
  /// @param bind_address IP address to bind to (e.g., "0.0.0.0" for all interfaces)
  /// @param streams Stream configurations
  /// @return true if all streams started successfully
  bool start(const std::string& bind_address, const std::vector<UdpStreamConfig>& streams);

  /// Stop all streams and close sockets
  void stop();

  /// Set callback for received messages
  void set_message_callback(MessageCallback callback);

  /// Get statistics for a specific stream
  /// @param port UDP port of the stream
  /// @return Statistics structure
  UdpStreamStats get_stats(uint16_t port) const;

  /// Get all stream statistics
  std::unordered_map<uint16_t, UdpStreamStats> get_all_stats() const;

  /// Check if server is running
  bool is_running() const {
    return running_.load();
  }

private:
  /// Per-port socket and receive buffer
  struct StreamEndpoint {
    std::unique_ptr<boost::asio::ip::udp::socket> socket;
    std::vector<uint8_t> receive_buffer;
    boost::asio::ip::udp::endpoint sender_endpoint;  // For async_receive_from
    UdpStreamConfig config;
    UdpStreamStats stats;
    std::mutex stats_mutex;
  };

  /// Start async receive for a stream
  void async_receive(StreamEndpoint& stream);

  /// Handle received data
  void handle_receive(StreamEndpoint& stream, std::size_t bytes_transferred);

  /// Close all sockets and clear streams_ map (caller must hold streams_mutex_)
  void close_all_streams();

  boost::asio::io_context& io_context_;
  std::unordered_map<uint16_t, std::unique_ptr<StreamEndpoint>> streams_;
  mutable std::mutex streams_mutex_;  // mutable for const methods
  MessageCallback message_callback_;
  std::atomic<bool> running_{false};
};

}  // namespace udp
}  // namespace axon

#endif  // AXON_UDP_UDP_SERVER_HPP
