// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "metadata_injector.hpp"

#include <nlohmann/json.hpp>
#include <openssl/evp.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
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

namespace {

constexpr const char* kSidecarVersion = "1.0";
constexpr const char* kSidecarMetadataRecord = "axon.sidecar";
constexpr const char* kTaskMetadataRecord = "axon.task";
constexpr const char* kDeviceMetadataRecord = "axon.device";
constexpr const char* kRecordingMetadataRecord = "axon.recording";
constexpr const char* kTopicsMetadataRecord = "axon.topics";
constexpr const char* kFileSizeMetadataKey = "file_size_bytes";
constexpr size_t kFileSizeMetadataWidth = 20;
constexpr size_t kPatchBufferSize = 64 * 1024;

std::string to_json_array_string(const std::vector<std::string>& values) {
  nlohmann::json array = nlohmann::json::array();
  for (const auto& value : values) {
    array.push_back(value);
  }
  return array.dump();
}

void append_uint32_le(std::string& output, uint32_t value) {
  for (int i = 0; i < 4; ++i) {
    output.push_back(static_cast<char>((value >> (i * 8)) & 0xFF));
  }
}

std::string build_file_size_patch_pattern(const std::string& value) {
  std::string pattern;
  append_uint32_le(pattern, static_cast<uint32_t>(std::strlen(kFileSizeMetadataKey)));
  pattern += kFileSizeMetadataKey;
  append_uint32_le(pattern, static_cast<uint32_t>(value.size()));
  pattern += value;
  return pattern;
}

bool has_key(const std::unordered_map<std::string, std::string>& metadata, const char* key) {
  return metadata.find(key) != metadata.end();
}

const std::string& get_value(
  const std::unordered_map<std::string, std::string>& metadata, const char* key
) {
  static const std::string empty;
  auto it = metadata.find(key);
  return it == metadata.end() ? empty : it->second;
}

void copy_string_if_present(
  nlohmann::json& object, const std::unordered_map<std::string, std::string>& metadata,
  const char* key
) {
  if (has_key(metadata, key)) {
    object[key] = get_value(metadata, key);
  }
}

uint64_t parse_uint64_or_zero(const std::string& value) {
  try {
    size_t parsed = 0;
    uint64_t result = std::stoull(value, &parsed);
    return parsed == value.size() ? result : 0;
  } catch (const std::exception&) {
    return 0;
  }
}

double parse_double_or_zero(const std::string& value) {
  try {
    size_t parsed = 0;
    double result = std::stod(value, &parsed);
    return parsed == value.size() ? result : 0.0;
  } catch (const std::exception&) {
    return 0.0;
  }
}

nlohmann::json parse_json_or(const std::string& value, const nlohmann::json& fallback) {
  try {
    return nlohmann::json::parse(value);
  } catch (const nlohmann::json::exception&) {
    return fallback;
  }
}

nlohmann::json build_sidecar_from_metadata(
  const std::unordered_map<std::string, std::string>& sidecar_meta,
  const std::unordered_map<std::string, std::string>& task_meta,
  const std::unordered_map<std::string, std::string>& device_meta,
  const std::unordered_map<std::string, std::string>& recording_meta,
  const std::unordered_map<std::string, std::string>& topics_meta
) {
  nlohmann::json sidecar;
  sidecar["version"] = get_value(sidecar_meta, "version");
  sidecar["mcap_file"] = get_value(sidecar_meta, "mcap_file");

  auto& task = sidecar["task"] = nlohmann::json::object();
  copy_string_if_present(task, task_meta, "task_id");
  copy_string_if_present(task, task_meta, "order_id");
  copy_string_if_present(task, task_meta, "scene");
  copy_string_if_present(task, task_meta, "subscene");
  if (has_key(task_meta, "skills")) {
    task["skills"] = parse_json_or(get_value(task_meta, "skills"), nlohmann::json::array());
  }
  copy_string_if_present(task, task_meta, "factory");
  copy_string_if_present(task, task_meta, "data_collector_id");
  copy_string_if_present(task, task_meta, "operator_name");

  auto& device = sidecar["device"] = nlohmann::json::object();
  copy_string_if_present(device, device_meta, "device_id");
  copy_string_if_present(device, device_meta, "device_model");
  copy_string_if_present(device, device_meta, "device_serial");
  copy_string_if_present(device, device_meta, "hostname");
  copy_string_if_present(device, device_meta, "ros_distro");

  auto& recording = sidecar["recording"] = nlohmann::json::object();
  copy_string_if_present(recording, recording_meta, "recorder_version");
  copy_string_if_present(recording, recording_meta, "recording_started_at");
  copy_string_if_present(recording, recording_meta, "recording_finished_at");
  recording["duration_sec"] = parse_double_or_zero(get_value(recording_meta, "duration_sec"));
  recording["message_count"] = parse_uint64_or_zero(get_value(recording_meta, "message_count"));
  recording["file_size_bytes"] =
    parse_uint64_or_zero(get_value(recording_meta, kFileSizeMetadataKey));
  recording["topics_recorded"] =
    parse_json_or(get_value(recording_meta, "topics_recorded"), nlohmann::json::array());

  sidecar["topics_summary"] =
    parse_json_or(get_value(topics_meta, "topics_summary"), nlohmann::json::array());

  return sidecar;
}

}  // namespace

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
  (void)file_size;

  if (!has_task_config_) {
    return false;
  }

  // Record finish time
  finish_time_ = std::chrono::system_clock::now();
  injected_message_count_ = message_count;
  has_injected_message_count_ = true;

  // 1. Write sidecar envelope metadata (axon.sidecar)
  auto sidecar_meta = build_sidecar_metadata(writer.get_path());
  if (!writer.write_metadata(kSidecarMetadataRecord, sidecar_meta)) {
    return false;
  }

  // 2. Write task context metadata (axon.task)
  auto task_meta = build_task_metadata();
  if (!writer.write_metadata(kTaskMetadataRecord, task_meta)) {
    return false;
  }

  // 3. Write device metadata (axon.device)
  auto device_meta = build_device_metadata();
  if (!writer.write_metadata(kDeviceMetadataRecord, device_meta)) {
    return false;
  }

  // 4. Write recording metadata (axon.recording)
  auto recording_meta =
    build_recording_metadata(message_count, std::string(kFileSizeMetadataWidth, '0'));
  if (!writer.write_metadata(kRecordingMetadataRecord, recording_meta)) {
    return false;
  }

  // 5. Write deterministic topic summary metadata (axon.topics)
  auto topics_meta = build_topics_metadata();
  if (!writer.write_metadata(kTopicsMetadataRecord, topics_meta)) {
    return false;
  }

  return true;
}

bool MetadataInjector::update_mcap_file_size_metadata(
  const std::string& mcap_path, uint64_t actual_file_size
) const {
  const std::string replacement = format_file_size_metadata_value(actual_file_size);
  if (replacement.size() != kFileSizeMetadataWidth) {
    return false;
  }

  const std::string placeholder(kFileSizeMetadataWidth, '0');
  const std::string pattern = build_file_size_patch_pattern(placeholder);
  const size_t value_offset_in_pattern = pattern.size() - placeholder.size();

  std::fstream file(mcap_path, std::ios::in | std::ios::out | std::ios::binary);
  if (!file) {
    return false;
  }

  std::string carry;
  std::vector<char> buffer(kPatchBufferSize);
  uint64_t stream_offset = 0;

  while (file.read(buffer.data(), static_cast<std::streamsize>(buffer.size())) ||
         file.gcount() > 0) {
    const std::streamsize bytes_read = file.gcount();
    std::string window = carry;
    window.append(buffer.data(), static_cast<size_t>(bytes_read));

    const size_t found = window.find(pattern);
    if (found != std::string::npos) {
      const uint64_t window_start_offset = stream_offset - carry.size();
      const uint64_t patch_offset = window_start_offset + found + value_offset_in_pattern;
      file.clear();
      file.seekp(static_cast<std::streamoff>(patch_offset), std::ios::beg);
      file.write(replacement.data(), static_cast<std::streamsize>(replacement.size()));
      file.flush();
      return static_cast<bool>(file);
    }

    if (window.size() >= pattern.size() - 1) {
      carry = window.substr(window.size() - (pattern.size() - 1));
    } else {
      carry = window;
    }
    stream_offset += static_cast<uint64_t>(bytes_read);
  }

  return false;
}

bool MetadataInjector::generate_sidecar_json(
  const std::string& mcap_path, uint64_t actual_file_size
) {
  if (!has_task_config_) {
    return false;
  }

  if (finish_time_ == std::chrono::system_clock::time_point{}) {
    finish_time_ = std::chrono::system_clock::now();
  }

  // Compute SHA-256 checksum (always enabled)
  checksum_ = compute_sha256(mcap_path);
  if (checksum_.empty()) {
    return false;
  }

  const uint64_t total_messages =
    has_injected_message_count_ ? injected_message_count_ : sum_topic_message_count();

  auto sidecar_meta = build_sidecar_metadata(mcap_path);
  auto task_meta = build_task_metadata();
  auto device_meta = build_device_metadata();
  auto recording_meta =
    build_recording_metadata(total_messages, format_file_size_metadata_value(actual_file_size));
  auto topics_meta = build_topics_metadata();

  nlohmann::json sidecar =
    build_sidecar_from_metadata(sidecar_meta, task_meta, device_meta, recording_meta, topics_meta);
  sidecar["recording"]["checksum_sha256"] = checksum_;

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

std::unordered_map<std::string, std::string> MetadataInjector::build_sidecar_metadata(
  const std::string& mcap_path
) const {
  std::unordered_map<std::string, std::string> meta;

  meta["version"] = kSidecarVersion;
  meta["mcap_file"] = std::filesystem::path(mcap_path).filename().string();

  return meta;
}

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
    meta["skills"] = to_json_array_string(sanitized_skills);
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
  uint64_t message_count, const std::string& file_size_bytes
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
  meta[kFileSizeMetadataKey] = file_size_bytes;

  std::vector<std::string> topics;
  for (const auto& stats : sorted_topic_stats()) {
    topics.push_back(stats.topic);
  }
  meta["topics_recorded"] = to_json_array_string(topics);

  return meta;
}

std::unordered_map<std::string, std::string> MetadataInjector::build_topics_metadata() const {
  std::unordered_map<std::string, std::string> meta;
  nlohmann::json topics_summary = nlohmann::json::array();

  for (const auto& stats : sorted_topic_stats()) {
    nlohmann::json topic_info;
    topic_info["topic"] = stats.topic;
    topic_info["message_type"] = stats.message_type;
    topic_info["message_count"] = stats.message_count;
    topic_info["frequency_hz"] = stats.compute_frequency_hz();
    topics_summary.push_back(topic_info);
  }

  meta["topics_summary"] = topics_summary.dump();
  return meta;
}

std::vector<TopicStats> MetadataInjector::sorted_topic_stats() const {
  std::vector<TopicStats> stats;
  stats.reserve(topic_stats_.size());
  for (const auto& [topic, topic_stats] : topic_stats_) {
    (void)topic;
    stats.push_back(topic_stats);
  }

  std::sort(stats.begin(), stats.end(), [](const TopicStats& a, const TopicStats& b) {
    return a.topic < b.topic;
  });

  return stats;
}

uint64_t MetadataInjector::sum_topic_message_count() const {
  uint64_t total = 0;
  for (const auto& [topic, stats] : topic_stats_) {
    (void)topic;
    total += stats.message_count;
  }
  return total;
}

std::string MetadataInjector::format_file_size_metadata_value(uint64_t file_size) const {
  std::ostringstream ss;
  ss << std::setw(static_cast<int>(kFileSizeMetadataWidth)) << std::setfill('0') << file_size;
  std::string value = ss.str();
  if (value.size() > kFileSizeMetadataWidth) {
    return "";
  }
  return value;
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
