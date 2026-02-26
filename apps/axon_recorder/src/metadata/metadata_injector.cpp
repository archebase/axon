// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "metadata_injector.hpp"

#include <nlohmann/json.hpp>
#include <openssl/evp.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <unistd.h>

#include "mcap_writer_wrapper.hpp"
#include "version.hpp"

namespace axon {
namespace recorder {

// ============================================================================
// TopicStats implementation
// ============================================================================

double TopicStats::compute_frequency_hz() const {
  if (message_count < 2) {
    return 0.0;
  }

  // Use microseconds for better precision with high-frequency topics
  auto duration_us =
    std::chrono::duration_cast<std::chrono::microseconds>(last_message_time - first_message_time);

  if (duration_us.count() <= 0) {
    return 0.0;
  }

  // Frequency = (N-1) intervals / duration
  // Using microseconds: freq_hz = (message_count - 1) * 1,000,000 / duration_us
  return static_cast<double>(message_count - 1) * 1000000.0 /
         static_cast<double>(duration_us.count());
}

// ============================================================================
// MetadataInjector implementation
// ============================================================================

MetadataInjector::MetadataInjector() = default;
MetadataInjector::~MetadataInjector() = default;

void MetadataInjector::set_task_config(const TaskConfig& config) {
  task_config_ = config;
  has_task_config_ = true;
}

void MetadataInjector::set_recording_start_time(std::chrono::system_clock::time_point time) {
  start_time_ = time;
}

void MetadataInjector::set_ros_param_loader(ConfigLoaderCallback callback) {
  ros_param_loader_ = std::move(callback);
}

void MetadataInjector::update_topic_stats(
  const std::string& topic, const std::string& message_type, uint64_t count
) {
  auto now = std::chrono::system_clock::now();

  auto it = topic_stats_.find(topic);
  if (it == topic_stats_.end()) {
    // First message for this topic
    TopicStats stats;
    stats.topic = topic;
    stats.message_type = message_type;
    stats.message_count = count;
    stats.first_message_time = now;
    stats.last_message_time = now;
    topic_stats_[topic] = stats;
  } else {
    it->second.message_count = count;
    it->second.last_message_time = now;
    // Update message type if it was empty
    if (it->second.message_type.empty()) {
      it->second.message_type = message_type;
    }
  }
}

bool MetadataInjector::inject_metadata(
  mcap_wrapper::McapWriterWrapper& writer, uint64_t message_count, uint64_t file_size
) {
  if (!has_task_config_) {
    return false;
  }

  // Record finish time
  finish_time_ = std::chrono::system_clock::now();

  // 1. Write task context metadata (axon.task)
  auto task_meta = build_task_metadata();
  if (!writer.write_metadata("axon.task", task_meta)) {
    return false;
  }

  // 2. Write device metadata (axon.device)
  auto device_meta = build_device_metadata();
  if (!writer.write_metadata("axon.device", device_meta)) {
    return false;
  }

  // 3. Write recording metadata (axon.recording)
  auto recording_meta = build_recording_metadata(message_count, file_size);
  if (!writer.write_metadata("axon.recording", recording_meta)) {
    return false;
  }

  return true;
}

bool MetadataInjector::generate_sidecar_json(
  const std::string& mcap_path, uint64_t actual_file_size
) {
  if (!has_task_config_) {
    return false;
  }

  // Compute SHA-256 checksum (always enabled)
  checksum_ = compute_sha256(mcap_path);

  // Build JSON sidecar
  nlohmann::json sidecar;
  sidecar["version"] = "1.0";
  sidecar["mcap_file"] = std::filesystem::path(mcap_path).filename().string();

  // Task metadata
  sidecar["task"] = nlohmann::json::object();
  sidecar["task"]["task_id"] = task_config_.task_id;
  if (!task_config_.order_id.empty()) {
    sidecar["task"]["order_id"] = task_config_.order_id;
  }
  sidecar["task"]["scene"] = task_config_.scene;
  if (!task_config_.subscene.empty()) {
    sidecar["task"]["subscene"] = task_config_.subscene;
  }
  if (!task_config_.skills.empty()) {
    sidecar["task"]["skills"] = task_config_.skills;
  }
  sidecar["task"]["factory"] = task_config_.factory;
  if (!task_config_.data_collector_id.empty()) {
    sidecar["task"]["data_collector_id"] = task_config_.data_collector_id;
  }
  if (!task_config_.operator_name.empty()) {
    sidecar["task"]["operator_name"] = task_config_.operator_name;
  }

  // Device metadata
  sidecar["device"] = nlohmann::json::object();
  sidecar["device"]["device_id"] = task_config_.device_id;
  std::string device_model = get_device_model();
  if (!device_model.empty()) {
    sidecar["device"]["device_model"] = device_model;
  }
  std::string device_serial = get_device_serial();
  if (!device_serial.empty()) {
    sidecar["device"]["device_serial"] = device_serial;
  }
  sidecar["device"]["hostname"] = get_hostname();
  sidecar["device"]["ros_distro"] = get_ros_distro();

  // Recording metadata
  auto duration_sec = std::chrono::duration<double>(finish_time_ - start_time_).count();

  // Count total messages from topic stats
  uint64_t total_messages = 0;
  for (const auto& [topic, stats] : topic_stats_) {
    total_messages += stats.message_count;
  }

  sidecar["recording"] = nlohmann::json::object();
  sidecar["recording"]["recorder_version"] = AXON_RECORDER_VERSION;
  sidecar["recording"]["recording_started_at"] = format_iso8601(start_time_);
  sidecar["recording"]["recording_finished_at"] = format_iso8601(finish_time_);
  sidecar["recording"]["duration_sec"] = duration_sec;
  sidecar["recording"]["message_count"] = total_messages;
  sidecar["recording"]["file_size_bytes"] = actual_file_size;
  sidecar["recording"]["checksum_sha256"] = checksum_;

  // Build topics list
  std::vector<std::string> topics_recorded;
  for (const auto& [topic, stats] : topic_stats_) {
    topics_recorded.push_back(topic);
  }
  std::sort(topics_recorded.begin(), topics_recorded.end());
  sidecar["recording"]["topics_recorded"] = topics_recorded;

  // Topics summary
  sidecar["topics_summary"] = nlohmann::json::array();
  for (const auto& [topic, stats] : topic_stats_) {
    nlohmann::json topic_info;
    topic_info["topic"] = stats.topic;
    topic_info["message_type"] = stats.message_type;
    topic_info["message_count"] = stats.message_count;
    topic_info["frequency_hz"] = stats.compute_frequency_hz();
    sidecar["topics_summary"].push_back(topic_info);
  }

  // Sort topics_summary by topic name
  std::sort(
    sidecar["topics_summary"].begin(),
    sidecar["topics_summary"].end(),
    [](const nlohmann::json& a, const nlohmann::json& b) {
      return a["topic"].get<std::string>() < b["topic"].get<std::string>();
    }
  );

  // Atomic write: write to temp file, then rename
  std::filesystem::path mcap_fs_path(mcap_path);
  std::filesystem::path json_path = mcap_fs_path;
  json_path.replace_extension(".json");

  std::filesystem::path tmp_path = json_path;
  tmp_path += ".tmp";

  try {
    std::ofstream ofs(tmp_path);
    if (!ofs) {
      return false;
    }
    ofs << sidecar.dump(2);
    ofs.close();

    // Rename to final path
    std::filesystem::rename(tmp_path, json_path);
    sidecar_path_ = json_path.string();

    return true;
  } catch (const std::exception& e) {
    // Clean up temp file if it exists
    std::error_code ec;
    std::filesystem::remove(tmp_path, ec);
    return false;
  }
}

std::string MetadataInjector::get_sidecar_path() const {
  return sidecar_path_;
}

std::string MetadataInjector::get_checksum() const {
  return checksum_;
}

// ============================================================================
// Private methods
// ============================================================================

std::unordered_map<std::string, std::string> MetadataInjector::build_task_metadata() const {
  std::unordered_map<std::string, std::string> meta;

  meta["task_id"] = sanitize_field(task_config_.task_id, kMaxTaskIdLength);
  meta["scene"] = sanitize_field(task_config_.scene, kMaxSceneLength);
  meta["factory"] = sanitize_field(task_config_.factory, kMaxFactoryLength);

  if (!task_config_.order_id.empty()) {
    meta["order_id"] = sanitize_field(task_config_.order_id, kMaxIdLength);
  }
  if (!task_config_.subscene.empty()) {
    meta["subscene"] = sanitize_field(task_config_.subscene, kMaxSceneLength);
  }
  if (!task_config_.skills.empty()) {
    // Sanitize each skill
    std::vector<std::string> sanitized_skills;
    for (const auto& skill : task_config_.skills) {
      sanitized_skills.push_back(sanitize_field(skill, kMaxSkillLength));
    }
    meta["skills"] = join(sanitized_skills, ",");
  }
  if (!task_config_.data_collector_id.empty()) {
    meta["data_collector_id"] = sanitize_field(task_config_.data_collector_id, kMaxIdLength);
  }
  if (!task_config_.operator_name.empty()) {
    meta["operator_name"] = sanitize_field(task_config_.operator_name, kMaxOperatorNameLength);
  }

  return meta;
}

std::unordered_map<std::string, std::string> MetadataInjector::build_device_metadata() const {
  std::unordered_map<std::string, std::string> meta;

  meta["device_id"] = sanitize_field(task_config_.device_id, kMaxIdLength);
  meta["hostname"] = get_hostname();
  meta["ros_distro"] = get_ros_distro();

  std::string device_model = get_device_model();
  if (!device_model.empty()) {
    meta["device_model"] = device_model;
  }

  std::string device_serial = get_device_serial();
  if (!device_serial.empty()) {
    meta["device_serial"] = device_serial;
  }

  return meta;
}

std::unordered_map<std::string, std::string> MetadataInjector::build_recording_metadata(
  uint64_t message_count, uint64_t file_size
) const {
  std::unordered_map<std::string, std::string> meta;

  meta["recorder_version"] = AXON_RECORDER_VERSION;
  meta["recording_started_at"] = format_iso8601(start_time_);
  meta["recording_finished_at"] = format_iso8601(finish_time_);

  auto duration_sec = std::chrono::duration<double>(finish_time_ - start_time_).count();
  std::ostringstream duration_ss;
  duration_ss << std::fixed << std::setprecision(3) << duration_sec;
  meta["duration_sec"] = duration_ss.str();

  meta["message_count"] = std::to_string(message_count);
  meta["file_size_bytes"] = std::to_string(file_size);

  // Build topics list
  std::vector<std::string> topics;
  for (const auto& [topic, stats] : topic_stats_) {
    topics.push_back(topic);
  }
  std::sort(topics.begin(), topics.end());
  meta["topics_recorded"] = join(topics, ",");

  return meta;
}

std::string MetadataInjector::get_hostname() const {
  char hostname[256];
  if (gethostname(hostname, sizeof(hostname)) == 0) {
    hostname[sizeof(hostname) - 1] = '\0';
    return std::string(hostname);
  }
  return "";
}

std::string MetadataInjector::get_ros_distro() const {
  const char* distro = std::getenv("ROS_DISTRO");
  return distro ? std::string(distro) : "";
}

std::string MetadataInjector::get_device_model() const {
  return resolve_config_value("AXON_DEVICE_MODEL", "", "device_model");
}

std::string MetadataInjector::get_device_serial() const {
  return resolve_config_value("AXON_DEVICE_SERIAL", "", "device_serial");
}

std::string MetadataInjector::resolve_config_value(
  const char* env_var, const std::string& config_value, const std::string& ros_param_name
) const {
  // 1. Try environment variable (highest priority)
  const char* env_value = std::getenv(env_var);
  if (env_value && env_value[0] != '\0') {
    return std::string(env_value);
  }

  // 2. Try config file value
  if (!config_value.empty()) {
    return config_value;
  }

  // 3. Try ROS parameter (lowest priority)
  if (ros_param_loader_) {
    std::string ros_value = ros_param_loader_(ros_param_name);
    if (!ros_value.empty()) {
      return ros_value;
    }
  }

  return "";
}

std::string MetadataInjector::sanitize_field(const std::string& value, size_t max_length) const {
  std::string result;
  result.reserve(std::min(value.size(), max_length));

  for (char c : value) {
    // Skip control characters (except newline, tab)
    if (c < 32 && c != '\n' && c != '\t') {
      continue;
    }
    result += c;
    if (result.size() >= max_length) {
      break;
    }
  }

  return result;
}

std::string MetadataInjector::compute_sha256(const std::string& file_path) const {
  std::ifstream file(file_path, std::ios::binary);
  if (!file) {
    return "";
  }

  // Use OpenSSL 3.0 EVP API (replaces deprecated SHA256_* functions)
  EVP_MD_CTX* ctx = EVP_MD_CTX_new();
  if (!ctx) {
    return "";
  }

  if (EVP_DigestInit_ex(ctx, EVP_sha256(), nullptr) != 1) {
    EVP_MD_CTX_free(ctx);
    return "";
  }

  constexpr size_t buffer_size = 64 * 1024;  // 64KB buffer
  std::vector<char> buffer(buffer_size);

  while (file.read(buffer.data(), buffer_size) || file.gcount() > 0) {
    if (EVP_DigestUpdate(ctx, buffer.data(), file.gcount()) != 1) {
      EVP_MD_CTX_free(ctx);
      return "";
    }
  }

  unsigned char hash[EVP_MAX_MD_SIZE];
  unsigned int hash_len = 0;
  if (EVP_DigestFinal_ex(ctx, hash, &hash_len) != 1) {
    EVP_MD_CTX_free(ctx);
    return "";
  }

  EVP_MD_CTX_free(ctx);

  // Convert to hex string
  std::ostringstream ss;
  ss << std::hex << std::setfill('0');
  for (unsigned int i = 0; i < hash_len; ++i) {
    ss << std::setw(2) << static_cast<unsigned>(hash[i]);
  }

  return ss.str();
}

std::string MetadataInjector::format_iso8601(std::chrono::system_clock::time_point time) const {
  auto time_t = std::chrono::system_clock::to_time_t(time);
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(time.time_since_epoch()) % 1000;

  std::tm tm_utc;
  gmtime_r(&time_t, &tm_utc);

  std::ostringstream ss;
  ss << std::put_time(&tm_utc, "%Y-%m-%dT%H:%M:%S") << '.' << std::setfill('0') << std::setw(3)
     << ms.count() << 'Z';

  return ss.str();
}

std::string MetadataInjector::join(
  const std::vector<std::string>& items, const std::string& delimiter
) const {
  if (items.empty()) {
    return "";
  }

  std::ostringstream ss;
  ss << items[0];
  for (size_t i = 1; i < items.size(); ++i) {
    ss << delimiter << items[i];
  }
  return ss.str();
}

}  // namespace recorder
}  // namespace axon
