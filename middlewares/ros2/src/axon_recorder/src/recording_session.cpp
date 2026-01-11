#include "recording_session.hpp"

#include <filesystem>

// Logging infrastructure
#define AXON_LOG_COMPONENT "recording_session"
#include <axon_log_macros.hpp>

namespace axon {
namespace recorder {

RecordingSession::RecordingSession()
    : writer_(std::make_unique<mcap_wrapper::McapWriterWrapper>()) {}

RecordingSession::~RecordingSession() {
  if (is_open()) {
    close();
  }
}

bool RecordingSession::open(
  const std::string& path, const mcap_wrapper::McapWriterOptions& options
) {
  if (is_open()) {
    AXON_LOG_WARN("Already open, close first");
    return false;
  }

  if (!writer_->open(path, options)) {
    AXON_LOG_ERROR("Failed to open" << axon::logging::kv("error", writer_->get_last_error()));
    return false;
  }

  output_path_ = path;
  start_time_ = std::chrono::system_clock::now();
  messages_written_.store(0, std::memory_order_relaxed);

  // Set recording start time for metadata
  metadata_injector_.set_recording_start_time(start_time_);

  // Clear registries
  {
    std::lock_guard<std::mutex> lock(registry_mutex_);
    topic_channel_ids_.clear();
    message_type_schema_ids_.clear();
  }

  // Clear topic stats
  {
    std::lock_guard<std::mutex> lock(topic_stats_mutex_);
    topic_message_counts_.clear();
    topic_message_types_.clear();
  }

  return true;
}

void RecordingSession::close() {
  if (!is_open()) {
    return;
  }

  writer_->flush();

  // 1. Inject metadata BEFORE close (metadata must be in file before footer)
  if (has_task_config_) {
    // Sync topic stats to metadata injector
    {
      std::lock_guard<std::mutex> lock(topic_stats_mutex_);
      for (const auto& [topic, count] : topic_message_counts_) {
        std::string msg_type;
        auto it = topic_message_types_.find(topic);
        if (it != topic_message_types_.end()) {
          msg_type = it->second;
        }
        metadata_injector_.update_topic_stats(topic, msg_type, count);
      }
    }

    auto stats = get_stats();
    metadata_injector_.inject_metadata(*writer_, stats.messages_written, stats.bytes_written);
  }

  // 2. Close MCAP file (writes footer, finalizes)
  writer_->close();

  // 3. Generate sidecar JSON AFTER close (always enabled, needs actual file size)
  if (has_task_config_ && !output_path_.empty()) {
    std::error_code ec;
    auto actual_size = std::filesystem::file_size(output_path_, ec);
    if (ec) {
      // LCOV_EXCL_BR_START - logging macro branch coverage
      AXON_LOG_WARN(
        "Failed to get file size for sidecar" << axon::logging::kv("path", output_path_)
                                              << axon::logging::kv("error", ec.message())
      );
      // LCOV_EXCL_BR_STOP
    } else {
      // LCOV_EXCL_BR_START - logging macro branch coverage
      if (!metadata_injector_.generate_sidecar_json(output_path_, actual_size)) {
        AXON_LOG_WARN("Failed to generate sidecar JSON" << axon::logging::kv("path", output_path_));
      }
      // LCOV_EXCL_BR_STOP
    }
  }

  output_path_.clear();
}

bool RecordingSession::is_open() const {
  return writer_ && writer_->is_open();
}

void RecordingSession::flush() {
  if (is_open()) {
    writer_->flush();
  }
}

uint16_t RecordingSession::register_schema(
  const std::string& name, const std::string& encoding, const std::string& definition
) {
  if (!is_open()) {
    AXON_LOG_WARN("Cannot register schema, session not open");
    return 0;
  }

  // Check if already registered
  {
    std::lock_guard<std::mutex> lock(registry_mutex_);
    auto it = message_type_schema_ids_.find(name);
    if (it != message_type_schema_ids_.end()) {
      return it->second;
    }
  }

  uint16_t schema_id = writer_->register_schema(name, encoding, definition);
  if (schema_id == 0) {
    AXON_LOG_ERROR("Failed to register schema" << axon::logging::kv("name", name));
    return 0;
  }

  {
    std::lock_guard<std::mutex> lock(registry_mutex_);
    message_type_schema_ids_[name] = schema_id;
  }

  return schema_id;
}

uint16_t RecordingSession::register_channel(
  const std::string& topic, const std::string& message_encoding, uint16_t schema_id,
  const std::unordered_map<std::string, std::string>& metadata
) {
  if (!is_open()) {
    AXON_LOG_WARN("Cannot register channel, session not open");
    return 0;
  }

  // Check if already registered
  {
    std::lock_guard<std::mutex> lock(registry_mutex_);
    auto it = topic_channel_ids_.find(topic);
    if (it != topic_channel_ids_.end()) {
      return it->second;
    }
  }

  uint16_t channel_id = writer_->register_channel(topic, message_encoding, schema_id, metadata);
  if (channel_id == 0) {
    AXON_LOG_ERROR("Failed to register channel" << axon::logging::kv("topic", topic));
    return 0;
  }

  {
    std::lock_guard<std::mutex> lock(registry_mutex_);
    topic_channel_ids_[topic] = channel_id;
  }

  return channel_id;
}

bool RecordingSession::write(
  uint16_t channel_id, uint32_t sequence, uint64_t log_time_ns, uint64_t publish_time_ns,
  const uint8_t* data, size_t data_size
) {
  if (!is_open()) {
    return false;
  }

  bool success =
    writer_->write(channel_id, sequence, log_time_ns, publish_time_ns, data, data_size);
  if (success) {
    messages_written_.fetch_add(1, std::memory_order_relaxed);
  }

  return success;
}

uint16_t RecordingSession::get_channel_id(const std::string& topic) const {
  std::lock_guard<std::mutex> lock(registry_mutex_);
  auto it = topic_channel_ids_.find(topic);
  return (it != topic_channel_ids_.end()) ? it->second : 0;
}

uint16_t RecordingSession::get_schema_id(const std::string& message_type) const {
  std::lock_guard<std::mutex> lock(registry_mutex_);
  auto it = message_type_schema_ids_.find(message_type);
  return (it != message_type_schema_ids_.end()) ? it->second : 0;
}

RecordingSession::Stats RecordingSession::get_stats() const {
  Stats stats;
  stats.messages_written = messages_written_.load(std::memory_order_relaxed);
  stats.start_time = start_time_;

  if (writer_ && writer_->is_open()) {
    auto writer_stats = writer_->get_statistics();
    stats.bytes_written = writer_stats.bytes_written;
    stats.schemas_registered = writer_stats.schemas_registered;
    stats.channels_registered = writer_stats.channels_registered;
  }

  return stats;
}

double RecordingSession::get_duration_sec() const {
  if (!is_open()) {
    return 0.0;
  }

  auto now = std::chrono::system_clock::now();
  return std::chrono::duration<double>(now - start_time_).count();
}

std::string RecordingSession::get_last_error() const {
  return writer_ ? writer_->get_last_error() : "Writer not initialized";
}

std::string RecordingSession::get_path() const {
  return output_path_;
}

void RecordingSession::set_task_config(const TaskConfig& config) {
  metadata_injector_.set_task_config(config);
  has_task_config_ = true;
}

std::string RecordingSession::get_sidecar_path() const {
  return metadata_injector_.get_sidecar_path();
}

std::string RecordingSession::get_checksum() const {
  return metadata_injector_.get_checksum();
}

void RecordingSession::update_topic_stats(
  const std::string& topic, const std::string& message_type
) {
  std::lock_guard<std::mutex> lock(topic_stats_mutex_);

  // Increment message count for this topic
  topic_message_counts_[topic]++;

  // Store message type if not already set
  if (topic_message_types_.find(topic) == topic_message_types_.end()) {
    topic_message_types_[topic] = message_type;
  }
}

}  // namespace recorder
}  // namespace axon
