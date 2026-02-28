// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "axon_udp/udp_plugin.hpp"

#include <chrono>

#define AXON_LOG_COMPONENT "udp_plugin"
#include <axon_log_macros.hpp>

using axon::logging::kv;

namespace axon {
namespace udp {

UdpPlugin::UdpPlugin() = default;

UdpPlugin::~UdpPlugin() {
  stop();
}

bool UdpPlugin::init(const std::string& config_json) {
  if (initialized_.load()) {
    AXON_LOG_WARN("UDP plugin already initialized");
    return true;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  if (!parse_config(config_json)) {
    return false;
  }

  if (!config_.enabled) {
    AXON_LOG_INFO("UDP plugin disabled in configuration");
    initialized_.store(true);
    return true;
  }

  // Create io_context
  io_context_ = std::make_unique<boost::asio::io_context>();

  // Create UDP server
  server_ = std::make_unique<UdpServer>(*io_context_);

  // Set message callback
  server_->set_message_callback(
    [this](const std::string& topic, const uint8_t* data, size_t size, uint64_t timestamp) {
      on_udp_message(topic, data, size, timestamp);
    }
  );

  initialized_.store(true);
  AXON_LOG_INFO(
    "UDP plugin initialized: " << kv("streams", config_.streams.size())
                               << kv(
                                    "timestamp_source",
                                    config_.timestamp_extraction.source == TimestampSource::Field
                                      ? "field"
                                      : "arrival"
                                  )
  );

  return true;
}

bool UdpPlugin::start() {
  if (!initialized_.load()) {
    AXON_LOG_ERROR("UDP plugin not initialized");
    return false;
  }

  if (running_.load()) {
    AXON_LOG_WARN("UDP plugin already running");
    return true;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  if (!config_.enabled) {
    AXON_LOG_INFO("UDP plugin disabled, not starting");
    running_.store(true);
    return true;
  }

  // Start UDP server with configured bind address
  if (!server_->start(config_.bind_address, config_.streams)) {
    AXON_LOG_ERROR("Failed to start UDP server");
    return false;
  }

  // Start io_context thread with work guard
  work_guard_ = std::make_unique<WorkGuard>(io_context_->get_executor());
  io_thread_ = std::make_unique<std::thread>([this]() {
    io_context_->run();
  });

  running_.store(true);
  AXON_LOG_INFO("UDP plugin started");

  return true;
}

bool UdpPlugin::stop() {
  if (!running_.load()) {
    return true;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  // Stop UDP server first
  if (server_) {
    server_->stop();
  }

  // Reset work guard to allow io_context to stop
  work_guard_.reset();

  // Stop io_context
  if (io_context_) {
    io_context_->stop();
  }

  // Wait for io_thread to finish
  if (io_thread_ && io_thread_->joinable()) {
    io_thread_->join();
  }

  running_.store(false);
  AXON_LOG_INFO("UDP plugin stopped");

  return true;
}

void UdpPlugin::set_message_callback(MessageCallback callback) {
  std::lock_guard<std::mutex> lock(mutex_);
  message_callback_ = std::move(callback);
}

std::unordered_map<uint16_t, UdpStreamStats> UdpPlugin::get_stats() const {
  if (server_) {
    return server_->get_all_stats();
  }
  return {};
}

bool UdpPlugin::parse_config(const std::string& config_json) {
  try {
    nlohmann::json config = nlohmann::json::parse(config_json);

    // Parse UDP configuration
    if (config.contains("udp")) {
      auto udp = config["udp"];

      config_.enabled = udp.value("enabled", false);
      config_.bind_address = udp.value("bind_address", "0.0.0.0");
      config_.default_port = udp.value("default_port", 4242);
      config_.buffer_size = udp.value("buffer_size", 65536);

      // Parse timestamp extraction
      if (udp.contains("timestamp_extraction")) {
        auto ts = udp["timestamp_extraction"];
        std::string source = ts.value("source", "field");

        if (source == "arrival") {
          config_.timestamp_extraction.source = TimestampSource::Arrival;
        } else {
          config_.timestamp_extraction.source = TimestampSource::Field;
          config_.timestamp_extraction.field = ts.value("field", "timestamp");
        }
      }

      // Parse streams
      if (udp.contains("streams")) {
        for (const auto& stream : udp["streams"]) {
          UdpStreamConfig stream_config;
          stream_config.name = stream.value("name", "");
          stream_config.port = stream.value("port", config_.default_port);
          stream_config.topic = stream.value("topic", "");
          stream_config.schema_name = stream.value("schema", "raw_json");
          stream_config.enabled = stream.value("enabled", true);
          stream_config.buffer_size = stream.value("buffer_size", config_.buffer_size);

          if (stream_config.name.empty() || stream_config.topic.empty()) {
            AXON_LOG_WARN("Skipping stream with missing name or topic");
            continue;
          }

          config_.streams.push_back(stream_config);
        }
      }
    }

    return true;

  } catch (const std::exception& e) {
    AXON_LOG_ERROR("Failed to parse UDP config: " << kv("error", e.what()));
    return false;
  }
}

void UdpPlugin::on_udp_message(
  const std::string& topic, const uint8_t* data, size_t size, uint64_t timestamp
) {
  if (!message_callback_) {
    return;
  }

  // Parse JSON to extract timestamp
  std::vector<uint8_t> json_data(data, data + size);

  try {
    nlohmann::json json = nlohmann::json::parse(data, data + size);
    uint64_t final_timestamp = extract_timestamp(json, timestamp);

    // Forward to recorder callback
    message_callback_(topic, json_data, "axon_udp/json", final_timestamp);

  } catch (const nlohmann::json::parse_error& e) {
    AXON_LOG_WARN("Failed to parse JSON on topic " << kv("topic", topic) << kv("error", e.what()));
    // Still forward with arrival timestamp
    message_callback_(topic, json_data, "axon_udp/json", timestamp);
  }
}

uint64_t UdpPlugin::extract_timestamp(const nlohmann::json& json, uint64_t arrival_time) {
  if (config_.timestamp_extraction.source == TimestampSource::Arrival) {
    return arrival_time;
  }

  // Extract from JSON field
  nlohmann::json value = get_json_value(json, config_.timestamp_extraction.field);

  if (value.is_null()) {
    AXON_LOG_DEBUG(
      "Timestamp field not found: " << kv("field", config_.timestamp_extraction.field)
    );
    return arrival_time;
  }

  // Value is expected to be in nanoseconds
  if (value.is_number_integer()) {
    return value.get<uint64_t>();
  }

  AXON_LOG_WARN("Timestamp field is not an integer, using arrival time");
  return arrival_time;
}

nlohmann::json UdpPlugin::get_json_value(
  const nlohmann::json& json, const std::string& path
) const {
  nlohmann::json current = json;

  // Split path by dots
  size_t start = 0;
  size_t end = path.find('.');

  while (end != std::string::npos) {
    std::string key = path.substr(start, end - start);
    if (!current.contains(key)) {
      return nullptr;
    }
    current = current[key];
    start = end + 1;
    end = path.find('.', start);
  }

  // Get final key
  std::string final_key = path.substr(start);
  if (!current.contains(final_key)) {
    return nullptr;
  }

  return current[final_key];
}

}  // namespace udp
}  // namespace axon
