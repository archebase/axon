#include "recording_session.hpp"

#include <iostream>

namespace axon {
namespace recorder {

RecordingSession::RecordingSession()
    : writer_(std::make_unique<mcap_wrapper::McapWriterWrapper>()) {}

RecordingSession::~RecordingSession() {
  if (is_open()) {
    close();
  }
}

bool RecordingSession::open(const std::string& path,
                            const mcap_wrapper::McapWriterOptions& options) {
  if (is_open()) {
    std::cerr << "[RecordingSession] Already open, close first" << std::endl;
    return false;
  }

  if (!writer_->open(path, options)) {
    std::cerr << "[RecordingSession] Failed to open: " << writer_->get_last_error() << std::endl;
    return false;
  }

  output_path_ = path;
  start_time_ = std::chrono::system_clock::now();
  messages_written_.store(0, std::memory_order_relaxed);

  // Clear registries
  {
    std::lock_guard<std::mutex> lock(registry_mutex_);
    topic_channel_ids_.clear();
    message_type_schema_ids_.clear();
  }

  return true;
}

void RecordingSession::close() {
  if (!is_open()) {
    return;
  }

  writer_->flush();
  writer_->close();
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

uint16_t RecordingSession::register_schema(const std::string& name, const std::string& encoding,
                                           const std::string& definition) {
  if (!is_open()) {
    std::cerr << "[RecordingSession] Cannot register schema, session not open" << std::endl;
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
    std::cerr << "[RecordingSession] Failed to register schema: " << name << std::endl;
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
    std::cerr << "[RecordingSession] Cannot register channel, session not open" << std::endl;
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
    std::cerr << "[RecordingSession] Failed to register channel: " << topic << std::endl;
    return 0;
  }

  {
    std::lock_guard<std::mutex> lock(registry_mutex_);
    topic_channel_ids_[topic] = channel_id;
  }

  return channel_id;
}

bool RecordingSession::write(uint16_t channel_id, uint32_t sequence, uint64_t log_time_ns,
                             uint64_t publish_time_ns, const uint8_t* data, size_t data_size) {
  if (!is_open()) {
    return false;
  }

  bool success = writer_->write(channel_id, sequence, log_time_ns, publish_time_ns, data, data_size);
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

}  // namespace recorder
}  // namespace axon

