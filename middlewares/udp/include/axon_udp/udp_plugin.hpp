// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_UDP_UDP_PLUGIN_HPP
#define AXON_UDP_UDP_PLUGIN_HPP

#include <boost/asio.hpp>
#include <nlohmann/json.hpp>

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "udp_server.hpp"

namespace axon {
namespace udp {

/// Timestamp extraction source
enum class TimestampSource { Field, Arrival };

/// Timestamp extraction configuration
struct TimestampExtractionConfig {
  TimestampSource source = TimestampSource::Field;
  std::string field = "timestamp";  // JSON field path, supports dot notation
};

/// UDP plugin configuration (parsed from JSON)
struct UdpPluginConfig {
  bool enabled = false;
  std::string bind_address = "0.0.0.0";
  uint16_t default_port = 4242;
  size_t buffer_size = 65536;
  TimestampExtractionConfig timestamp_extraction;
  std::vector<UdpStreamConfig> streams;
};

/// UDP JSON recording plugin
///
/// Receives JSON messages via UDP and forwards them to the recorder
/// for MCAP storage. Supports multiple simultaneous UDP streams.
class UdpPlugin {
public:
  using MessageCallback = std::function<void(
    const std::string& topic, const std::vector<uint8_t>& data, const std::string& message_type,
    uint64_t timestamp
  )>;

  UdpPlugin();
  ~UdpPlugin();

  // Non-copyable
  UdpPlugin(const UdpPlugin&) = delete;
  UdpPlugin& operator=(const UdpPlugin&) = delete;

  /// Initialize plugin with JSON configuration
  /// @param config_json JSON string containing UDP configuration
  /// @return true if initialization succeeded
  bool init(const std::string& config_json);

  /// Start the UDP server and begin receiving messages
  /// @return true if started successfully
  bool start();

  /// Stop the UDP server and cleanup resources
  /// @return true if stopped successfully
  bool stop();

  /// Set callback for received messages
  /// @param callback Function to call when a message is received
  void set_message_callback(MessageCallback callback);

  /// Get statistics for all streams
  std::unordered_map<uint16_t, UdpStreamStats> get_stats() const;

  /// Check if plugin is initialized
  bool is_initialized() const {
    return initialized_.load();
  }

  /// Check if plugin is running
  bool is_running() const {
    return running_.load();
  }

private:
  /// Parse JSON configuration
  bool parse_config(const std::string& config_json);

  /// Handle incoming UDP message
  void on_udp_message(
    const std::string& topic, const uint8_t* data, size_t size, uint64_t timestamp
  );

  /// Extract timestamp from JSON message
  /// @param json Parsed JSON object
  /// @param arrival_time Packet arrival time in nanoseconds
  /// @return Extracted timestamp in nanoseconds
  uint64_t extract_timestamp(const nlohmann::json& json, uint64_t arrival_time);

  /// Extract value from JSON using dot-notation path
  /// @param json JSON object
  /// @param path Dot-notation path (e.g., "header.stamp")
  /// @return JSON value at path, or null if not found
  nlohmann::json get_json_value(const nlohmann::json& json, const std::string& path) const;

  UdpPluginConfig config_;
  std::unique_ptr<boost::asio::io_context> io_context_;
  std::unique_ptr<UdpServer> server_;
  std::unique_ptr<std::thread> io_thread_;
  using WorkGuard = boost::asio::executor_work_guard<boost::asio::io_context::executor_type>;
  std::unique_ptr<WorkGuard> work_guard_;
  MessageCallback message_callback_;

  std::atomic<bool> initialized_{false};
  std::atomic<bool> running_{false};
  mutable std::mutex mutex_;
};

}  // namespace udp
}  // namespace axon

#endif  // AXON_UDP_UDP_PLUGIN_HPP
