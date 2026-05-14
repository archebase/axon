// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "recorder.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <functional>
#include <iostream>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <unordered_set>

#include "../config/config_parser.hpp"
#include "../http/rpc_handlers.hpp"
#include "../http/ws_rpc_client.hpp"
#include "incident_debug_bundle.hpp"

// Logging infrastructure
#define AXON_LOG_COMPONENT "recorder"
#include <axon_log_init.hpp>
#include <axon_log_macros.hpp>

#include "latency_monitor.hpp"
#include "mcap_writer_wrapper.hpp"
#include "schema_resolver.hpp"

namespace axon {
namespace recorder {

namespace {

/// Middleware name advertised by the UDP JSON plugin shared library.
constexpr const char* kUdpMiddlewareName = "UDP";

// Serialize a BufferPool stats snapshot into the same JSON shape used by
// /rpc/status. Centralizing this keeps the two get_stats callbacks below in
// sync and gives dashboards a stable schema to monitor pool health
// (allocations per second, hit rate, resident footprint).
nlohmann::json buffer_pool_stats_json() {
  const auto pool_stats = axon::memory::BufferPool::instance().stats();
  nlohmann::json j;
  j["acquires"] = pool_stats.acquires;
  j["hits"] = pool_stats.hits;
  j["misses"] = pool_stats.misses;
  j["oversized"] = pool_stats.oversized;
  j["releases"] = pool_stats.releases;
  j["release_to_pool"] = pool_stats.release_to_pool;
  j["release_freed"] = pool_stats.release_freed;
  j["resident_bytes"] = pool_stats.resident_bytes;
  j["hit_rate"] = pool_stats.hit_rate();
  return j;
}

std::string to_lower_ascii(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

std::string severity_to_level_string(axon::logging::severity_level level) {
  switch (level) {
    case axon::logging::severity_level::debug:
      return "debug";
    case axon::logging::severity_level::info:
      return "info";
    case axon::logging::severity_level::warn:
      return "warn";
    case axon::logging::severity_level::error:
      return "error";
    case axon::logging::severity_level::fatal:
      return "fatal";
  }
  return "info";
}

std::string sidecar_path_for_output(const std::string& output_path) {
  if (output_path.empty()) {
    return "";
  }
  std::filesystem::path path(output_path);
  path.replace_extension(".json");
  return path.string();
}

std::string completion_marker_path_for_output(const std::string& output_path) {
  return output_path.empty() ? "" : output_path + ".done";
}

TaskConfig task_config_from_json(const nlohmann::json& config_json) {
  TaskConfig config;
  auto get_string = [&config_json](const char* key) -> std::string {
    return config_json.contains(key) ? config_json[key].get<std::string>() : "";
  };

  config.task_id = get_string("task_id");
  config.device_id = get_string("device_id");
  config.data_collector_id = get_string("data_collector_id");
  config.order_id = get_string("order_id");
  config.operator_name = get_string("operator_name");
  config.scene = get_string("scene");
  config.subscene = get_string("subscene");
  config.factory = get_string("factory");
  config.start_callback_url = get_string("start_callback_url");
  config.finish_callback_url = get_string("finish_callback_url");
  config.user_token = get_string("user_token");

  if (config_json.contains("skills")) {
    for (const auto& skill : config_json["skills"]) {
      config.skills.push_back(skill.get<std::string>());
    }
  }
  if (config_json.contains("topics")) {
    for (const auto& topic : config_json["topics"]) {
      config.topics.push_back(topic.get<std::string>());
    }
  }
  return config;
}

// C callback wrapper for plugin (ABI v1.0 / v1.1 — borrowed buffer, copy-out)
void message_callback(
  const char* topic_name, const uint8_t* message_data, size_t message_size,
  const char* message_type, uint64_t timestamp, void* user_data
) {
  if (!user_data) {
    return;
  }

  auto* recorder = static_cast<AxonRecorder*>(user_data);
  recorder->on_message(topic_name, message_data, message_size, message_type, timestamp);
}

// C callback wrapper for plugin (ABI v1.2 — ownership transfer, zero-copy)
void message_callback_v2(
  const char* topic_name, const uint8_t* message_data, size_t message_size,
  const char* message_type, uint64_t timestamp, void (*release_fn)(void*), void* release_opaque,
  void* user_data
) {
  if (!user_data) {
    if (release_fn) {
      release_fn(release_opaque);
    }
    return;
  }

  auto* recorder = static_cast<AxonRecorder*>(user_data);
  recorder->on_message_v2(
    topic_name, message_data, message_size, message_type, timestamp, release_fn, release_opaque
  );
}

}  // namespace

AxonRecorder::AxonRecorder()
    : running_(false)
    , last_rate_calc_time_(std::chrono::steady_clock::now())
    , last_bytes_received_(0)
    , last_bytes_written_(0) {
  // Register state transition callback
  state_manager_.register_transition_callback([this](RecorderState from, RecorderState to) {
    on_state_transition(from, to);
  });
}

AxonRecorder::~AxonRecorder() {
  stop();
  // Safety net: ensure ws_thread_ is joined before member destruction.
  // In normal program flow stop_ws_rpc_client() is called explicitly before
  // the recorder goes out of scope, making this a no-op.  If an exception
  // or early-exit path skips that call, the ws_ioc_ / ws_thread_ members
  // would be destroyed while still running, causing std::terminate().
  stop_ws_rpc_client();
  stop_http_server();
  shutdown_plugins();
}

bool AxonRecorder::initialize(const RecorderConfig& config) {
  config_ = config;
  last_session_sidecar_enabled_ = config_.recording.sidecar_json_enabled;
  last_session_incident_bundle_enabled_ = config_.incident_bundle.enabled;

  ::axon::logging::LoggingConfig log_config;
  convert_logging_config(config_.logging, log_config);
  ::axon::logging::reconfigure_logging(log_config);

  // Configure and create worker thread pool
  WorkerThreadPool::Config pool_config;
  pool_config.queue_capacity_per_topic = config_.queue_capacity;
  pool_config.worker_idle_sleep_us = 50;
  pool_config.use_adaptive_backoff = true;

  worker_pool_ = std::make_unique<WorkerThreadPool>(pool_config);

  // Register drop callback for message loss reporting
  worker_pool_->set_drop_callback(
    [this](const std::string& topic, const std::string& message_type, uint64_t total_dropped) {
      report_message_drop(topic, message_type, total_dropped);
    }
  );

  http_callback_client_ = std::make_shared<HttpCallbackClient>();

  latency_monitor_ = std::make_shared<LatencyMonitor>();

  // Initialize schema resolver if search paths are configured
  if (!config_.recording.schema_search_paths.empty()) {
    schema_resolver_ = std::make_unique<SchemaResolver>(config_.recording.schema_search_paths);
    AXON_LOG_INFO(
      "Schema resolver initialized with "
      << axon::logging::kv("paths_count", config_.recording.schema_search_paths.size())
      << " search paths"
    );
  }

  // Middleware plugins own process-lifetime state such as ROS1 roscpp/xmlrpcpp
  // singletons. Load and initialize them once instead of rebuilding that state
  // for every recording session.
  if (!config_.ordered_plugin_paths().empty() && !ensure_plugin_loaded()) {
    return false;
  }

  return true;
}

const AxonPluginDescriptor* AxonRecorder::get_ros_plugin_descriptor(std::string* out_name) const {
  for (const auto& name : plugin_startup_order_) {
    if (name != kUdpMiddlewareName) {
      if (out_name != nullptr) {
        *out_name = name;
      }
      return plugin_loader_.get_descriptor(name);
    }
  }
  return nullptr;
}

const AxonPluginDescriptor* AxonRecorder::get_udp_plugin_descriptor() const {
  return plugin_loader_.get_descriptor(kUdpMiddlewareName);
}

bool AxonRecorder::ensure_plugin_loaded() {
  const auto paths = config_.ordered_plugin_paths();
  if (paths.empty()) {
    set_error_helper("Plugin path is empty");
    return false;
  }

  auto unload_all_loaded = [this]() {
    const auto names = plugin_loader_.loaded_plugins();
    for (const auto& name : names) {
      plugin_loader_.unload(name);
    }
    plugin_startup_order_.clear();
    plugin_init_ok_.clear();
  };

  if (plugin_startup_order_.empty()) {
    for (const auto& path : paths) {
      auto plugin_name = plugin_loader_.load(path);
      if (!plugin_name.has_value()) {
        set_error_helper("Failed to load plugin: " + plugin_loader_.get_last_error());
        unload_all_loaded();
        return false;
      }
      plugin_startup_order_.push_back(plugin_name.value());
    }
  }

  const char* cfg = config_.plugin_config.c_str();
  for (const auto& name : plugin_startup_order_) {
    auto init_it = plugin_init_ok_.find(name);
    if (init_it != plugin_init_ok_.end() && init_it->second) {
      continue;
    }

    const auto* descriptor = plugin_loader_.get_descriptor(name);
    if (!descriptor || !descriptor->vtable || !descriptor->vtable->init) {
      set_error_helper("Invalid plugin descriptor: " + name);
      unload_all_loaded();
      return false;
    }

    AxonStatus status = descriptor->vtable->init(cfg);
    if (status != AXON_SUCCESS) {
      set_error_helper(
        std::string("Plugin init failed (") + name + "): " + status_to_string(status)
      );
      unload_all_loaded();
      return false;
    }
    plugin_init_ok_[name] = true;
  }

  plugins_shutting_down_ = false;
  return true;
}

DiskUsageLimitConfig AxonRecorder::make_disk_usage_limits() const {
  DiskUsageLimitConfig limits;
  limits.enabled = config_.recording.disk_usage.enabled;
  limits.warn_usage_bytes = gb_to_bytes(config_.recording.disk_usage.warn_usage_gb);
  limits.hard_limit_bytes = gb_to_bytes(config_.recording.disk_usage.hard_limit_gb);
  limits.max_task_size_bytes = gb_to_bytes(config_.recording.disk_usage.max_task_size_gb);
  return limits;
}

std::vector<DiskUsagePathConfig> AxonRecorder::make_disk_usage_paths() const {
  std::vector<DiskUsagePathConfig> paths;

  if (!config_.dataset.path.empty()) {
    paths.push_back({"recording_output", config_.dataset.path});
  }

  std::filesystem::path configured_output(config_.output_file);
  if (configured_output.is_absolute() && configured_output.has_parent_path()) {
    paths.push_back({"recording_output", configured_output.parent_path()});
  }

  if (config_.upload.enabled) {
    if (!config_.upload.state_db_path.empty()) {
      std::filesystem::path state_db_path(config_.upload.state_db_path);
      paths.push_back({"upload_state", state_db_path.parent_path()});
    }
    if (!config_.upload.failed_uploads_dir.empty()) {
      paths.push_back({"upload_failed", config_.upload.failed_uploads_dir});
    }
  }

  if (paths.empty()) {
    paths.push_back({"working_directory", "."});
  }

  return paths;
}

DiskUsageSnapshot AxonRecorder::collect_disk_usage_snapshot(uint64_t current_task_bytes) const {
  DiskUsageMonitor monitor(make_disk_usage_limits(), make_disk_usage_paths());
  return monitor.snapshot(current_task_bytes);
}

nlohmann::json AxonRecorder::get_disk_usage_status_json() const {
  uint64_t current_task_bytes = 0;
  {
    std::lock_guard<std::mutex> lock(recorder_mutex_);
    if (recording_session_) {
      current_task_bytes = recording_session_->get_stats().bytes_written;
    }
  }
  return collect_disk_usage_snapshot(current_task_bytes).to_json();
}

nlohmann::json AxonRecorder::get_keystone_time_gap_status_json() const {
  if (config_.rpc.mode != RpcMode::WS_CLIENT) {
    return nlohmann::json{
      {"enabled", false},
      {"status", "unavailable"},
      {"reliable", false},
      {"offset_ms", nullptr},
      {"absolute_offset_ms", nullptr},
      {"reason", "recorder is not running in WebSocket client mode"}
    };
  }

  if (!ws_rpc_client_) {
    return nlohmann::json{
      {"enabled", config_.rpc.ws_client.time_gap_check_enabled},
      {"status", "unavailable"},
      {"reliable", false},
      {"offset_ms", nullptr},
      {"absolute_offset_ms", nullptr},
      {"reason", "WebSocket RPC client is not running"}
    };
  }

  return ws_rpc_client_->get_time_gap_status_json();
}

nlohmann::json AxonRecorder::get_metadata_status_json() const {
  nlohmann::json j;
  j["output_path"] = last_session_output_path_.empty() ? nlohmann::json(nullptr)
                                                       : nlohmann::json(last_session_output_path_);
  j["sidecar_enabled"] = last_session_sidecar_enabled_;
  j["sidecar_generated"] = last_session_sidecar_generated_;
  j["sidecar_path"] = last_session_sidecar_path_.empty()
                        ? nlohmann::json(nullptr)
                        : nlohmann::json(last_session_sidecar_path_);
  j["checksum_sha256"] = last_session_checksum_.empty() ? nlohmann::json(nullptr)
                                                        : nlohmann::json(last_session_checksum_);
  j["incident_bundle_enabled"] = last_session_incident_bundle_enabled_;
  j["incident_bundle_created"] = last_session_incident_bundle_created_;
  j["incident_bundle_path"] = last_session_incident_bundle_path_.empty()
                                ? nlohmann::json(nullptr)
                                : nlohmann::json(last_session_incident_bundle_path_);
  j["incident_bundle_error"] = last_session_incident_bundle_error_.empty()
                                 ? nlohmann::json(nullptr)
                                 : nlohmann::json(last_session_incident_bundle_error_);
  return j;
}

nlohmann::json AxonRecorder::build_incident_diagnostic_snapshot() const {
  auto stats = get_statistics();
  nlohmann::json j;
  j["state"] = get_state_string();
  j["messages_received"] = stats.messages_received;
  j["messages_written"] = stats.messages_written;
  j["messages_dropped"] = stats.messages_dropped;
  j["bytes_written"] = stats.bytes_written;
  j["bytes_received"] = stats.bytes_received;
  j["duration_sec"] = stats.duration_sec;
  j["metadata"] = get_metadata_status_json();
  j["disk_usage"] = get_disk_usage_status_json();
  j["keystone_time_gap"] = get_keystone_time_gap_status_json();
  if (latency_monitor_) {
    j["latency"] = latency_monitor_->get_stats_json();
  }
  if (worker_pool_) {
    nlohmann::json topic_counts = nlohmann::json::object();
    for (const auto& topic_name : worker_pool_->get_topics()) {
      auto ts = worker_pool_->get_topic_stats(topic_name);
      nlohmann::json t;
      t["received"] = ts.received;
      t["written"] = ts.written;
      t["dropped"] = ts.dropped;
      topic_counts[topic_name] = t;
    }
    j["topic_message_counts"] = topic_counts;
  }
  j["buffer_pool"] = buffer_pool_stats_json();
  return j;
}

void AxonRecorder::maybe_create_incident_bundle(const std::string& output_path) {
  last_session_incident_bundle_enabled_ = config_.incident_bundle.enabled;
  last_session_incident_bundle_created_ = false;
  last_session_incident_bundle_path_.clear();
  last_session_incident_bundle_error_.clear();

  if (!config_.incident_bundle.enabled) {
    return;
  }
  if (cancel_in_progress_.load(std::memory_order_acquire)) {
    return;
  }

  IncidentDebugBundleRequest request;
  request.config = config_.incident_bundle;
  request.mcap_path = output_path;
  request.mcap_file_size = last_session_final_file_size_;
  request.checksum_sha256 = last_session_checksum_;
  request.sidecar_path = last_session_sidecar_path_;
  request.sidecar_enabled = last_session_sidecar_enabled_;
  request.sidecar_generated = last_session_sidecar_generated_;
  request.task_config = task_config_ ? &task_config_.value() : nullptr;
  request.recorder_config = &config_;
  request.diagnostic_snapshot = build_incident_diagnostic_snapshot();

  IncidentDebugBundleWriter writer;
  auto result = writer.create(request);
  last_session_incident_bundle_created_ = result.created;
  last_session_incident_bundle_path_ = result.bundle_path;
  if (!result.success) {
    last_session_incident_bundle_error_ = result.error_message;
    AXON_LOG_WARN(
      "Failed to create incident debug bundle" << axon::logging::kv("error", result.error_message)
    );
  } else if (result.created) {
    AXON_LOG_INFO("Incident debug bundle created" << axon::logging::kv("path", result.bundle_path));
  }
}

bool AxonRecorder::maybe_cleanup_disk_usage(
  const std::string& active_output_path, DiskUsageSnapshot& snapshot
) const {
  const auto& cleanup = config_.recording.disk_usage;
  if (!cleanup.cleanup_enabled || !snapshot.hard_limit_reached() ||
      snapshot.reason == "current task size reached hard limit") {
    return false;
  }

  uint64_t target_bytes = gb_to_bytes(cleanup.cleanup_target_gb);
  if (target_bytes == 0) {
    target_bytes = snapshot.warn_usage_bytes;
  }
  if (target_bytes == 0 && snapshot.hard_limit_bytes > 0) {
    target_bytes = snapshot.hard_limit_bytes * 8 / 10;
  }

  if (target_bytes == 0 || target_bytes >= snapshot.total_used_bytes) {
    return false;
  }

  std::vector<std::filesystem::path> roots;
  if (!config_.dataset.path.empty()) {
    roots.push_back(config_.dataset.path);
  }
  if (cleanup.cleanup_upload_backlog && config_.upload.enabled &&
      !config_.upload.failed_uploads_dir.empty()) {
    roots.push_back(config_.upload.failed_uploads_dir);
  }

  if (roots.empty()) {
    return false;
  }

  DiskUsageMonitor monitor(make_disk_usage_limits(), make_disk_usage_paths());
  auto result = monitor.cleanup_recording_files(
    roots, active_output_path, snapshot.total_used_bytes, target_bytes, cleanup.cleanup_min_age_sec
  );

  if (result.files_removed > 0) {
    AXON_LOG_WARN(
      "Disk cleanup removed completed recording files"
      << axon::logging::kv("files_removed", result.files_removed)
      << axon::logging::kv("bytes_removed", result.bytes_removed)
      << axon::logging::kv("target_bytes", target_bytes)
    );
    snapshot = collect_disk_usage_snapshot(snapshot.current_task_bytes);
    return true;
  }

  if (!result.errors.empty()) {
    AXON_LOG_WARN(
      "Disk cleanup could not remove files" << axon::logging::kv("errors", result.errors.size())
    );
  }
  return false;
}

void AxonRecorder::log_disk_warning_once(const DiskUsageSnapshot& snapshot) {
  bool expected = false;
  if (!disk_warn_logged_.compare_exchange_strong(expected, true)) {
    return;
  }

  AXON_LOG_WARN(
    "Disk usage warning threshold reached"
    << axon::logging::kv("state", disk_usage_state_to_string(snapshot.state))
    << axon::logging::kv("total_used_gb", bytes_to_gb(snapshot.total_used_bytes))
    << axon::logging::kv("warn_usage_gb", bytes_to_gb(snapshot.warn_usage_bytes))
    << axon::logging::kv("hard_limit_gb", bytes_to_gb(snapshot.hard_limit_bytes))
  );
}

void AxonRecorder::trip_disk_hard_limit(const DiskUsageSnapshot& snapshot) {
  const bool first_trip = !disk_hard_limit_reached_.exchange(true);
  if (first_trip) {
    set_error_helper("Disk hard limit reached: " + snapshot.reason);
    AXON_LOG_ERROR(
      "Disk hard limit reached"
      << axon::logging::kv("reason", snapshot.reason)
      << axon::logging::kv("total_used_gb", bytes_to_gb(snapshot.total_used_bytes))
      << axon::logging::kv("hard_limit_gb", bytes_to_gb(snapshot.hard_limit_bytes))
      << axon::logging::kv("current_task_gb", bytes_to_gb(snapshot.current_task_bytes))
      << axon::logging::kv("max_task_size_gb", bytes_to_gb(snapshot.max_task_size_bytes))
    );
  }

  if (get_state() == RecorderState::RECORDING) {
    std::string error_msg;
    if (state_manager_.transition(RecorderState::RECORDING, RecorderState::PAUSED, error_msg)) {
      mark_pause_started();
    } else if (first_trip) {
      AXON_LOG_ERROR(
        "Failed to pause recorder after disk hard limit" << axon::logging::kv("error", error_msg)
      );
    }
  }
}

bool AxonRecorder::ensure_disk_capacity_before_start(const std::string& output_file) {
  if (!config_.recording.disk_usage.enabled) {
    return true;
  }

  auto snapshot = collect_disk_usage_snapshot();
  if (snapshot.hard_limit_reached()) {
    maybe_cleanup_disk_usage(output_file, snapshot);
  }

  {
    std::lock_guard<std::mutex> lock(disk_usage_mutex_);
    last_disk_check_time_ = std::chrono::steady_clock::now();
    last_disk_usage_snapshot_ = snapshot;
  }

  if (snapshot.warn_reached()) {
    log_disk_warning_once(snapshot);
  }

  if (snapshot.hard_limit_reached()) {
    trip_disk_hard_limit(snapshot);
    set_error_helper("Cannot start recording: " + get_last_error());
    return false;
  }

  return true;
}

bool AxonRecorder::ensure_disk_capacity_before_write(size_t next_write_bytes) {
  if (!config_.recording.disk_usage.enabled) {
    return true;
  }
  if (disk_hard_limit_reached_.load(std::memory_order_acquire)) {
    return false;
  }

  uint64_t current_task_bytes = next_write_bytes;
  std::string active_output_path;
  if (recording_session_) {
    auto stats = recording_session_->get_stats();
    current_task_bytes += stats.bytes_written;
    active_output_path = recording_session_->get_path();
  }

  const auto limits = make_disk_usage_limits();
  if (limits.max_task_size_bytes > 0 && current_task_bytes >= limits.max_task_size_bytes) {
    auto snapshot = collect_disk_usage_snapshot(current_task_bytes);
    trip_disk_hard_limit(snapshot);
    return false;
  }

  const auto now = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(disk_usage_mutex_);
    if (last_disk_check_time_ != std::chrono::steady_clock::time_point{} &&
        now - last_disk_check_time_ < std::chrono::seconds(1)) {
      return true;
    }
    last_disk_check_time_ = now;
  }

  auto snapshot = collect_disk_usage_snapshot(current_task_bytes);
  if (snapshot.hard_limit_reached()) {
    maybe_cleanup_disk_usage(active_output_path, snapshot);
  }

  {
    std::lock_guard<std::mutex> lock(disk_usage_mutex_);
    last_disk_usage_snapshot_ = snapshot;
  }

  if (snapshot.warn_reached()) {
    log_disk_warning_once(snapshot);
  }

  if (snapshot.hard_limit_reached()) {
    trip_disk_hard_limit(snapshot);
    return false;
  }

  return true;
}

bool AxonRecorder::start() {
  std::string error_msg;

  auto current_state = state_manager_.get_state();

  // If in IDLE state, transition to READY first (for direct start() calls)
  // If already in READY state, proceed to RECORDING (for HTTP RPC mode)
  if (current_state == RecorderState::IDLE) {
    // Transition from IDLE to READY
    if (!state_manager_.transition(RecorderState::IDLE, RecorderState::READY, error_msg)) {
      set_error_helper("State transition to READY failed: " + error_msg);
      return false;
    }
  } else if (current_state != RecorderState::READY) {
    set_error_helper(
      "Cannot start recording from state: " + state_to_string(current_state) +
      ". Must be in IDLE or READY state."
    );
    return false;
  }

  if (!ensure_plugin_loaded()) {
    state_manager_.transition_to(RecorderState::IDLE, error_msg);
    return false;
  }

  disk_warn_logged_.store(false, std::memory_order_release);
  disk_hard_limit_reached_.store(false, std::memory_order_release);
  {
    std::lock_guard<std::mutex> lock(disk_usage_mutex_);
    last_disk_check_time_ = {};
    last_disk_usage_snapshot_ = DiskUsageSnapshot{};
  }

  reset_pause_tracking();
  reset_topic_monotonic_timestamps();
  last_session_active_duration_sec_ = 0.0;
  last_session_final_file_size_ = 0;
  last_session_close_time_ = std::chrono::system_clock::time_point{};
  last_session_output_path_.clear();
  last_session_sidecar_enabled_ = config_.recording.sidecar_json_enabled;
  last_session_sidecar_generated_ = false;
  last_session_sidecar_path_.clear();
  last_session_checksum_.clear();
  last_session_incident_bundle_enabled_ = config_.incident_bundle.enabled;
  last_session_incident_bundle_created_ = false;
  last_session_incident_bundle_path_.clear();
  last_session_incident_bundle_error_.clear();

  for (const auto& name : plugin_startup_order_) {
    const auto* pd = plugin_loader_.get_descriptor(name);
    if (!pd || !pd->vtable || !pd->vtable->start) {
      set_error_helper("Plugin missing start function: " + name);
      state_manager_.transition_to(RecorderState::IDLE, error_msg);
      return false;
    }
  }

  // Create recording session
  recording_session_ = std::make_unique<RecordingSession>();
  recording_session_->set_sidecar_json_enabled(config_.recording.sidecar_json_enabled);

  // Configure MCAP options
  mcap_wrapper::McapWriterOptions mcap_options;
  mcap_options.profile = config_.recording.profile;

  if (config_.recording.compression == "zstd") {
    mcap_options.compression = mcap_wrapper::Compression::Zstd;
  } else if (config_.recording.compression == "lz4") {
    mcap_options.compression = mcap_wrapper::Compression::Lz4;
  } else {
    mcap_options.compression = mcap_wrapper::Compression::None;
  }

  mcap_options.compression_level = config_.recording.compression_level;

  // Construct output file path
  // Priority:
  // 1. HTTP RPC mode with task_id: {dataset.path}/{task_id}.mcap
  // 2. Explicit output file (CLI --output): use as-is (with dataset.path if relative)
  // 3. Fallback: {dataset.path}/recording.mcap or default output_file
  std::string output_file;

  bool has_task_id = task_config_.has_value() && !task_config_.value().task_id.empty();

  if (has_task_id && !config_.dataset.path.empty()) {
    // HTTP RPC mode: always use task_id as filename
    std::string path = config_.dataset.path;
    if (!path.empty() && path.back() != '/') {
      path += '/';
    }
    output_file = path + task_config_.value().task_id + ".mcap";
  } else if (config_.output_file_is_explicit) {
    // Explicit output file (from CLI --output)
    bool is_absolute = !config_.output_file.empty() && config_.output_file[0] == '/';
    if (is_absolute) {
      output_file = config_.output_file;
    } else if (!config_.dataset.path.empty()) {
      std::string path = config_.dataset.path;
      if (!path.empty() && path.back() != '/') {
        path += '/';
      }
      output_file = path + config_.output_file;
    } else {
      output_file = config_.output_file;
    }
  } else if (!config_.dataset.path.empty()) {
    // Fallback with dataset path but no task_id
    std::string path = config_.dataset.path;
    if (!path.empty() && path.back() != '/') {
      path += '/';
    }
    output_file = path + "recording.mcap";
  } else {
    // Final fallback
    output_file = config_.output_file;
  }

  if (!ensure_disk_capacity_before_start(output_file)) {
    state_manager_.transition_to(RecorderState::IDLE, error_msg);
    recording_session_.reset();
    return false;
  }

  // Open MCAP file via recording session
  if (!recording_session_->open(output_file, mcap_options)) {
    set_error_helper(
      "Failed to open MCAP file: " + output_file + " - " + recording_session_->get_last_error()
    );
    state_manager_.transition_to(RecorderState::IDLE, error_msg);
    recording_session_.reset();
    return false;
  }

  // Inject config files from cache (if enabled)
  if (!recording_session_->inject_config()) {
    // Config injection failure is not fatal, log warning and continue
    // inject_config() returns true when disabled or successful, false only on error
  }

  // Set task config if available
  if (task_config_.has_value()) {
    recording_session_->set_task_config(task_config_.value());
  }

  // Register topics with MCAP and create topic workers
  if (!register_topics()) {
    set_error_helper("Failed to register topics");
    state_manager_.transition_to(RecorderState::IDLE, error_msg);
    recording_session_.reset();
    return false;
  }

  // Setup subscriptions via plugin
  if (!setup_subscriptions()) {
    set_error_helper("Failed to setup subscriptions: " + get_last_error());
    state_manager_.transition_to(RecorderState::IDLE, error_msg);
    recording_session_.reset();
    return false;
  }

  // Transition from READY to RECORDING
  if (!state_manager_.transition(RecorderState::READY, RecorderState::RECORDING, error_msg)) {
    set_error_helper("State transition to RECORDING failed: " + error_msg);
    state_manager_.transition_to(RecorderState::IDLE, error_msg);
    recording_session_.reset();
    return false;
  }

  if (task_config_.has_value() && http_callback_client_ &&
      !task_config_->start_callback_url.empty()) {
    StartCallbackPayload payload;
    payload.task_id = task_config_->task_id;
    payload.device_id = task_config_->device_id;
    payload.status = "recording";
    payload.started_at = HttpCallbackClient::get_iso8601_timestamp();
    payload.topics = task_config_->topics;

    http_callback_client_->post_start_callback_async(*task_config_, payload);
  }

  // Start all middleware plugins (ROS + UDP, etc.) in config load order
  for (const auto& name : plugin_startup_order_) {
    const auto* pd = plugin_loader_.get_descriptor(name);
    AxonStatus st = pd->vtable->start();
    if (st != AXON_SUCCESS) {
      set_error_helper(std::string("Plugin start failed (") + name + "): " + status_to_string(st));
      state_manager_.transition_to(RecorderState::IDLE, error_msg);
      recording_session_.reset();
      return false;
    }
  }

  // Start worker thread pool
  worker_pool_->start();
  running_.store(true);

  return true;
}

void AxonRecorder::stop() {
  if (!running_.load()) {
    return;
  }

  AXON_LOG_INFO("Recorder stop requested");

  std::string error_msg;

  // Shutdown ordering rationale:
  //
  // ABI v1.2 zero-copy messages land in per-topic queues wrapped in
  // PooledBuffer::adopt(), whose external_release_ is a function pointer
  // into the producing plugin's .text section. The plugin now remains loaded
  // for the AxonRecorder process lifetime, so ordinary finish only stops
  // producers/subscriptions and drains queues. Final dlclose happens from
  // shutdown_plugins() during destruction.
  //
  // To guarantee zero data loss while preserving process-lifetime middleware
  // state, shutdown proceeds in this order:
  //
  //   1. Halt every plugin's producer loop (e.g. the ROS2 executor).
  //      After plugin->stop() returns, no callback is in flight and no
  //      further MessageItems will be enqueued.
  //   2. Stop the worker pool. Each worker exits its main loop and then
  //      runs its internal drain, dispatching every queued item into the
  //      still-open recording_session_ — so the messages that legitimately
  //      arrived before step (1) are all persisted to MCAP.
  //   3. Call drain_remaining_sync() as defense-in-depth. Under a healthy
  //      sequence this is a no-op; if a future change ever reintroduces a
  //      producer/consumer ordering bug, this catches residual items on
  //      the calling thread while the plugin is still mapped in, so the
  //      adopted release_fn is still valid.
  //   4. Close the MCAP session (flush, write metadata, emit sidecar).
  //   5. Keep plugins loaded. This preserves process-lifetime middleware
  //      state (notably ROS1 roscpp/xmlrpcpp) across sessions.

  // 1. Stop per-session plugin producers so no more messages can be enqueued.
  const std::vector<std::string> stop_order =
    !plugin_startup_order_.empty() ? plugin_startup_order_ : plugin_loader_.loaded_plugins();
  for (const auto& plugin_name : stop_order) {
    const auto* descriptor = plugin_loader_.get_descriptor(plugin_name);
    if (!descriptor || !descriptor->vtable) {
      continue;
    }

    AxonStatus status = AXON_SUCCESS;
    AxonSessionStopFn session_stop = plugin_session_stop(descriptor);
    AXON_LOG_INFO(
      "Stopping plugin session" << axon::logging::kv("plugin", plugin_name)
                                << axon::logging::kv("has_session_stop", session_stop != nullptr)
    );
    if (session_stop) {
      status = session_stop();
    } else if (descriptor->vtable->stop) {
      // Backward compatibility for plugins that do not yet expose the v1.3
      // session-stop slot. ROS1 should use session_stop so its process-wide
      // roscpp/xmlrpcpp state survives between sessions.
      status = descriptor->vtable->stop();
      if (status == AXON_SUCCESS) {
        plugin_init_ok_[plugin_name] = false;
      }
    }

    if (status != AXON_SUCCESS) {
      AXON_LOG_ERROR(
        "Plugin session stop failed" << axon::logging::kv("plugin", plugin_name)
                                     << axon::logging::kv("status", status_to_string(status))
      );
    } else {
      AXON_LOG_INFO("Plugin session stopped" << axon::logging::kv("plugin", plugin_name));
    }
  }

  // 2. Stop worker thread pool (workers drain remaining messages into the
  //    still-open recording_session_ before exiting).
  if (worker_pool_) {
    AXON_LOG_INFO("Stopping worker thread pool");
    worker_pool_->stop();
    AXON_LOG_INFO("Worker thread pool stopped");
  }

  // 3. Defense-in-depth: synchronously drain any residual items that
  //    slipped past worker_thread_func's own drain loop. Must run before
  //    recording_session_->close() because the handler writes via the
  //    session. Normally a no-op.
  if (worker_pool_) {
    AXON_LOG_INFO("Draining remaining queued messages");
    const size_t drained = worker_pool_->drain_remaining_sync();
    AXON_LOG_INFO("Remaining queued messages drained" << axon::logging::kv("count", drained));
  }

  // 4. Close recording session (injects metadata, generates sidecar).
  if (recording_session_) {
    AXON_LOG_INFO("Closing recording session");
    const std::string output_path = recording_session_->get_path();
    const double wall_duration_sec = recording_session_->get_duration_sec();
    const double paused_duration_sec = static_cast<double>(current_pause_offset_ns()) / 1e9;
    last_session_active_duration_sec_ =
      wall_duration_sec > paused_duration_sec ? wall_duration_sec - paused_duration_sec : 0.0;

    recording_session_->close();
    AXON_LOG_INFO("Recording session closed");

    last_session_final_file_size_ = recording_session_->get_final_file_size();
    last_session_close_time_ = recording_session_->get_close_time();
    last_session_output_path_ = output_path;
    last_session_sidecar_enabled_ = recording_session_->sidecar_json_enabled();
    last_session_sidecar_generated_ = recording_session_->was_sidecar_generated();
    last_session_sidecar_path_ = recording_session_->get_sidecar_path();
    last_session_checksum_ = recording_session_->get_checksum();

    maybe_create_incident_bundle(output_path);

    recording_session_.reset();
  }

  // 5. Keep plugins loaded until AxonRecorder destruction.

  // Reset worker pool statistics and drop report state for next session.
  if (worker_pool_) {
    worker_pool_->reset_stats();
  }
  {
    std::lock_guard<std::mutex> lock(drop_report_mutex_);
    drop_report_states_.clear();
  }

  // Transition back to IDLE
  state_manager_.transition_to(RecorderState::IDLE, error_msg);

  reset_pause_tracking();

  running_.store(false);
  AXON_LOG_INFO("Recorder stop completed");
}

void AxonRecorder::shutdown_plugins() {
  if (plugins_shutting_down_) {
    return;
  }
  plugins_shutting_down_ = true;

  auto plugins = plugin_loader_.loaded_plugins();
  for (const auto& plugin_name : plugins) {
    const auto* descriptor = plugin_loader_.get_descriptor(plugin_name);
    if (descriptor && descriptor->vtable && descriptor->vtable->stop) {
      auto plugin = plugin_loader_.get_plugin(plugin_name);
      if (plugin) {
        plugin->running = false;
      }

      AxonStatus status = descriptor->vtable->stop();
      if (status != AXON_SUCCESS) {
        AXON_LOG_ERROR(
          "Plugin final stop failed" << axon::logging::kv("plugin", plugin_name)
                                     << axon::logging::kv("status", status_to_string(status))
        );
      }
    }
  }

  auto loaded = plugin_loader_.loaded_plugins();
  for (const auto& plugin_name : loaded) {
    plugin_loader_.unload(plugin_name);
  }
  plugin_startup_order_.clear();
  plugin_init_ok_.clear();
}

bool AxonRecorder::is_running() const {
  return running_.load();
}

RecorderState AxonRecorder::get_state() const {
  return state_manager_.get_state();
}

std::string AxonRecorder::get_state_string() const {
  return state_manager_.get_state_string();
}

RpcCallbacks AxonRecorder::make_rpc_callbacks() {
  RpcCallbacks callbacks;

  callbacks.get_state = [this]() -> std::string {
    return this->get_state_string();
  };

  callbacks.get_last_error = [this]() -> std::string {
    return this->get_last_error();
  };

  callbacks.get_stats = [this]() -> nlohmann::json {
    auto stats = this->get_statistics();
    nlohmann::json j;
    j["state"] = this->get_state_string();
    j["messages_received"] = stats.messages_received;
    j["messages_written"] = stats.messages_written;
    j["messages_dropped"] = stats.messages_dropped;
    j["bytes_written"] = stats.bytes_written;
    j["bytes_received"] = stats.bytes_received;
    j["receive_rate_mbps"] = stats.receive_rate_mbps;
    j["write_rate_mbps"] = stats.write_rate_mbps;
    j["duration_sec"] = stats.duration_sec;
    j["message_count"] = stats.messages_written;

    if (worker_pool_) {
      auto depths = worker_pool_->get_queue_depths();
      nlohmann::json queue_depths = nlohmann::json::object();
      for (const auto& [topic, info] : depths) {
        nlohmann::json q;
        q["depth"] = info.depth;
        q["capacity"] = info.capacity;
        q["utilization"] =
          info.capacity > 0 ? static_cast<double>(info.depth) / info.capacity * 100.0 : 0.0;
        queue_depths[topic] = q;
      }
      j["queue_depths"] = queue_depths;

      nlohmann::json topic_counts = nlohmann::json::object();
      for (const auto& topic_name : worker_pool_->get_topics()) {
        auto ts = worker_pool_->get_topic_stats(topic_name);
        nlohmann::json t;
        t["received"] = ts.received;
        t["written"] = ts.written;
        t["dropped"] = ts.dropped;
        topic_counts[topic_name] = t;
      }
      j["topic_message_counts"] = topic_counts;
    }

    j["buffer_pool"] = buffer_pool_stats_json();
    j["disk_usage"] = this->get_disk_usage_status_json();
    j["metadata"] = this->get_metadata_status_json();
    j["keystone_time_gap"] = this->get_keystone_time_gap_status_json();
    return j;
  };

  callbacks.get_drop_stats = [this]() -> nlohmann::json {
    auto stats = this->get_statistics();
    nlohmann::json j;
    j["state"] = this->get_state_string();
    j["messages_received"] = stats.messages_received;
    j["messages_dropped"] = stats.messages_dropped;
    nlohmann::json topics = nlohmann::json::object();
    if (worker_pool_) {
      for (const auto& topic_name : worker_pool_->get_topics()) {
        auto ts = worker_pool_->get_topic_stats(topic_name);
        nlohmann::json t;
        t["received"] = ts.received;
        t["dropped"] = ts.dropped;
        t["written"] = ts.written;
        topics[topic_name] = t;
      }
    }
    j["topics"] = topics;
    return j;
  };

  callbacks.get_latency_stats = [this]() -> nlohmann::json {
    if (latency_monitor_) {
      return latency_monitor_->get_stats_json();
    }
    return nlohmann::json::object();
  };

  callbacks.get_task_config = [this]() -> const TaskConfig* {
    return this->get_task_config();
  };

  callbacks.set_config =
    [this](const std::string& task_id, const nlohmann::json& config_json) -> bool {
    try {
      TaskConfig config = task_config_from_json(config_json);
      if (task_id != config.task_id) {
        set_error_helper(
          "task_id mismatch: expected '" + config.task_id + "' but got '" + task_id + "'"
        );
        return false;
      }

      auto current_state = this->get_state();
      if (current_state == RecorderState::RECORDING || current_state == RecorderState::PAUSED) {
        set_error_helper(
          "Cannot set task configuration from state: " + state_to_string(current_state) +
          ". Finish or cancel the active recording first."
        );
        return false;
      }

      this->set_task_config(config);

      if (http_server_) {
        http_server_->broadcast_config_change(&config);
      }
      if (ws_rpc_client_) {
        ws_rpc_client_->send_config_update(config);
      }

      if (current_state == RecorderState::IDLE) {
        std::string error_msg;
        if (!this->transition_to(RecorderState::READY, error_msg)) {
          set_error_helper("State transition to READY failed: " + error_msg);
          return false;
        }
      }

      return true;
    } catch (const std::exception& e) {
      set_error_helper(std::string("Failed to parse task config: ") + e.what());
      return false;
    }
  };

  callbacks.begin_recording = [this](const std::string& task_id) -> bool {
    const TaskConfig* task_config = this->get_task_config();
    if (!task_config || task_config->task_id.empty()) {
      set_error_helper("No task configuration cached. Call /rpc/config first.");
      return false;
    }

    if (task_id != task_config->task_id) {
      set_error_helper(
        "task_id mismatch: expected '" + task_config->task_id + "' but got '" + task_id + "'"
      );
      return false;
    }

    if (this->get_state() != RecorderState::READY) {
      set_error_helper(
        "Cannot start recording from state: " + this->get_state_string() +
        ". Must be in READY state."
      );
      return false;
    }

    return this->start();
  };

  callbacks.finish_recording = [this](const std::string& task_id) -> bool {
    const TaskConfig* current_task_config = this->get_task_config();
    if (!current_task_config || current_task_config->task_id.empty()) {
      set_error_helper("No task configuration cached. Call /rpc/config first.");
      return false;
    }
    TaskConfig task_config = *current_task_config;

    if (task_id != task_config.task_id) {
      set_error_helper(
        "task_id mismatch: expected '" + task_config.task_id + "' but got '" + task_id + "'"
      );
      return false;
    }

    auto current_state = this->get_state();
    if (current_state != RecorderState::RECORDING && current_state != RecorderState::PAUSED) {
      set_error_helper(
        "Cannot finish recording from state: " + state_to_string(current_state) +
        ". Must be in RECORDING or PAUSED state."
      );
      return false;
    }

    std::string output_path;
    std::chrono::system_clock::time_point start_time;
    if (recording_session_) {
      output_path = recording_session_->get_path();
      start_time = recording_session_->get_start_time();
    }

    std::string started_at;
    if (start_time != std::chrono::system_clock::time_point{}) {
      started_at = HttpCallbackClient::get_iso8601_timestamp(start_time);
    }

    if (!this->is_running()) {
      set_error_helper("Recorder is not running");
      return false;
    }

    auto pre_stop_stats = this->get_statistics();
    this->stop();

    int64_t file_size_bytes = (last_session_final_file_size_ > 0)
                                ? static_cast<int64_t>(last_session_final_file_size_)
                                : static_cast<int64_t>(pre_stop_stats.bytes_written);

    std::string finished_at;
    double duration_sec = 0.0;
    if (last_session_close_time_ != std::chrono::system_clock::time_point{}) {
      finished_at = HttpCallbackClient::get_iso8601_timestamp(last_session_close_time_);
      if (last_session_active_duration_sec_ > 0.0) {
        duration_sec = last_session_active_duration_sec_;
      } else if (start_time != std::chrono::system_clock::time_point{}) {
        duration_sec = std::chrono::duration<double>(last_session_close_time_ - start_time).count();
      }
    } else {
      finished_at = HttpCallbackClient::get_iso8601_timestamp();
    }

    if (http_callback_client_ && !task_config.finish_callback_url.empty()) {
      FinishCallbackPayload payload;
      payload.task_id = task_id;
      payload.device_id = task_config.device_id;
      payload.status = "finished";
      payload.started_at = started_at;
      payload.finished_at = finished_at;
      payload.duration_sec = duration_sec;
      payload.message_count = static_cast<int64_t>(pre_stop_stats.messages_written);
      payload.file_size_bytes = file_size_bytes;
      payload.output_path = output_path;
      payload.sidecar_path = last_session_sidecar_path_;
      payload.sidecar_enabled = last_session_sidecar_enabled_;
      payload.sidecar_generated = last_session_sidecar_generated_;
      payload.topics = task_config.topics;
      payload.metadata.scene = task_config.scene;
      payload.metadata.subscene = task_config.subscene;
      payload.metadata.skills = task_config.skills;
      payload.metadata.factory = task_config.factory;

      http_callback_client_->post_finish_callback_async(task_config, payload);
    }

    return true;
  };

  callbacks.cancel_recording = [this]() -> bool {
    return this->cancel_current_recording();
  };

  callbacks.pause_recording = [this]() -> bool {
    return this->pause_recording();
  };

  callbacks.resume_recording = [this]() -> bool {
    return this->resume_recording();
  };

  callbacks.clear_config = [this]() -> bool {
    return this->clear_task_config();
  };

  callbacks.get_log_levels = [this]() -> nlohmann::json {
    return this->get_log_levels_json();
  };

  callbacks.set_log_levels = [this](const nlohmann::json& params, nlohmann::json& result) -> bool {
    return this->set_log_levels_from_rpc(params, result);
  };

  callbacks.quit = [this]() -> void {
    this->request_shutdown();
  };

  return callbacks;
}

bool AxonRecorder::start_http_server(const std::string& host, uint16_t port) {
  if (http_server_) {
    set_error_helper("HTTP server already running");
    return false;
  }

  http_server_ = std::make_unique<HttpServer>(host, port);

  HttpServer::Callbacks callbacks = make_rpc_callbacks();
  http_server_->register_callbacks(callbacks);

  if (!http_server_->start()) {
    set_error_helper("Failed to start HTTP server: " + http_server_->get_last_error());
    http_server_.reset();
    return false;
  }

  return true;
}

void AxonRecorder::stop_http_server() {
  if (http_server_) {
    http_server_->stop();
    http_server_.reset();
  }
}

bool AxonRecorder::is_http_server_running() const {
  return http_server_ && http_server_->is_running();
}

bool AxonRecorder::transition_to(RecorderState to, std::string& error_msg) {
  return state_manager_.transition_to(to, error_msg);
}

bool AxonRecorder::clear_task_config() {
  if (get_state() != RecorderState::READY) {
    set_error_helper(
      "Cannot clear task configuration from state: " + get_state_string() +
      ". Must be in READY state."
    );
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(recorder_mutex_);
    task_config_.reset();
  }

  std::string error_msg;
  if (!transition_to(RecorderState::IDLE, error_msg)) {
    set_error_helper("State transition to IDLE failed: " + error_msg);
    return false;
  }

  if (http_server_) {
    http_server_->broadcast_config_change(nullptr);
  }

  return true;
}

bool AxonRecorder::discard_recording_artifacts(
  const std::string& output_path, const std::string& sidecar_path
) {
  bool success = true;
  auto remove_if_present = [this, &success](const std::string& path, const char* label) {
    if (path.empty()) {
      return;
    }

    std::error_code ec;
    const bool exists = std::filesystem::exists(path, ec);
    if (ec) {
      set_error_helper(
        std::string("Failed to inspect cancelled ") + label + ": " + path + " - " + ec.message()
      );
      success = false;
      return;
    }
    if (!exists) {
      return;
    }

    std::filesystem::remove(path, ec);
    if (ec) {
      set_error_helper(
        std::string("Failed to delete cancelled ") + label + ": " + path + " - " + ec.message()
      );
      success = false;
      return;
    }

    AXON_LOG_INFO(
      "Deleted cancelled recording artifact" << axon::logging::kv("path", path)
                                             << axon::logging::kv("type", label)
    );
  };

  remove_if_present(output_path, "mcap");
  const std::string completion_marker_path = completion_marker_path_for_output(output_path);
  remove_if_present(completion_marker_path, "completion_marker");
  if (!completion_marker_path.empty()) {
    remove_if_present(completion_marker_path + ".tmp", "completion_marker_tmp");
  }
  remove_if_present(sidecar_path, "sidecar");
  if (!sidecar_path.empty()) {
    remove_if_present(sidecar_path + ".tmp", "sidecar_tmp");
  }
  return success;
}

bool AxonRecorder::cancel_current_recording() {
  auto current_state = get_state();
  if (current_state != RecorderState::RECORDING && current_state != RecorderState::PAUSED) {
    set_error_helper(
      "Cannot cancel recording from state: " + state_to_string(current_state) +
      ". Must be in RECORDING or PAUSED state."
    );
    return false;
  }

  if (!is_running()) {
    set_error_helper("Recorder is not running");
    return false;
  }

  std::string output_path;
  if (recording_session_) {
    output_path = recording_session_->get_path();
  }
  const std::string sidecar_path = sidecar_path_for_output(output_path);

  cancel_in_progress_.store(true, std::memory_order_release);
  this->stop();
  cancel_in_progress_.store(false, std::memory_order_release);

  const bool discarded = discard_recording_artifacts(output_path, sidecar_path);
  {
    std::lock_guard<std::mutex> lock(recorder_mutex_);
    task_config_.reset();
  }
  if (http_server_) {
    http_server_->broadcast_config_change(nullptr);
  }

  return discarded;
}

nlohmann::json AxonRecorder::get_log_levels_json() const {
  ::axon::logging::LoggingConfig effective;
  convert_logging_config(config_.logging, effective);
  ::axon::logging::apply_env_overrides(effective);

  nlohmann::json j;
  j["console_enabled"] = config_.logging.console_enabled;
  j["file_enabled"] = config_.logging.file_enabled;
  j["console_level"] = to_lower_ascii(config_.logging.console_level);
  j["file_level"] = to_lower_ascii(config_.logging.file_level);
  j["effective_console_level"] = severity_to_level_string(effective.console_level);
  j["effective_file_level"] = severity_to_level_string(effective.file_level);
  j["logging_initialized"] = ::axon::logging::is_logging_initialized();
  return j;
}

bool AxonRecorder::set_log_levels_from_rpc(const nlohmann::json& params, nlohmann::json& result) {
  nlohmann::json request = params.is_object() ? params : nlohmann::json::object();
  if (request.contains("console") && request["console"].is_object() &&
      request["console"].contains("level")) {
    request["console_level"] = request["console"]["level"];
  }
  if (request.contains("file") && request["file"].is_object() &&
      request["file"].contains("level")) {
    request["file_level"] = request["file"]["level"];
  }

  std::vector<ValidationError> errors;
  bool has_update = false;
  std::string console_level = to_lower_ascii(config_.logging.console_level);
  std::string file_level = to_lower_ascii(config_.logging.file_level);

  auto parse_level = [&](const char* field, std::string& out_level) {
    if (!request.contains(field)) {
      return;
    }
    has_update = true;
    if (!request[field].is_string()) {
      errors.push_back({field, "invalid_type", std::string(field) + " must be a string"});
      return;
    }
    const std::string raw = request[field].get<std::string>();
    auto parsed = ::axon::logging::parse_severity_level(raw);
    if (!parsed.has_value()) {
      errors.push_back(
        {field,
         "invalid_value",
         std::string(field) + " must be one of debug, info, warn, error, fatal"}
      );
      return;
    }
    out_level = severity_to_level_string(parsed.value());
  };

  parse_level("console_level", console_level);
  parse_level("file_level", file_level);

  if (!has_update) {
    errors.push_back(
      {"log_levels", "missing_required", "At least one of console_level or file_level is required"}
    );
  }

  if (!errors.empty()) {
    result["message"] = "Log level validation failed";
    result["validation_errors"] = nlohmann::json::array();
    for (const auto& error : errors) {
      result["validation_errors"].push_back(error.to_json());
    }
    return false;
  }

  config_.logging.console_level = console_level;
  config_.logging.file_level = file_level;

  ::axon::logging::LoggingConfig log_config;
  convert_logging_config(config_.logging, log_config);
  ::axon::logging::reconfigure_logging(log_config);

  result = get_log_levels_json();
  return true;
}

bool AxonRecorder::pause_recording() {
  if (get_state() != RecorderState::RECORDING) {
    set_error_helper(
      "Cannot pause recording from state: " + get_state_string() + ". Must be in RECORDING state."
    );
    return false;
  }

  std::string error_msg;
  const bool success =
    state_manager_.transition(RecorderState::RECORDING, RecorderState::PAUSED, error_msg);
  if (success) {
    mark_pause_started();
  } else {
    set_error_helper("State transition to PAUSED failed: " + error_msg);
  }
  return success;
}

bool AxonRecorder::resume_recording() {
  if (get_state() != RecorderState::PAUSED) {
    set_error_helper(
      "Cannot resume recording from state: " + get_state_string() + ". Must be in PAUSED state."
    );
    return false;
  }

  mark_pause_finished();

  std::string error_msg;
  const bool success =
    state_manager_.transition(RecorderState::PAUSED, RecorderState::RECORDING, error_msg);
  if (!success && get_state() == RecorderState::PAUSED) {
    mark_pause_started();
    set_error_helper("State transition to RECORDING failed: " + error_msg);
  }
  return success;
}

void AxonRecorder::reset_pause_tracking() {
  std::lock_guard<std::mutex> lock(pause_time_mutex_);
  pause_started_at_.reset();
  total_pause_offset_ns_.store(0, std::memory_order_release);
}

void AxonRecorder::mark_pause_started() {
  std::lock_guard<std::mutex> lock(pause_time_mutex_);
  if (!pause_started_at_.has_value()) {
    pause_started_at_ = std::chrono::steady_clock::now();
  }
}

void AxonRecorder::mark_pause_finished() {
  std::lock_guard<std::mutex> lock(pause_time_mutex_);
  if (!pause_started_at_.has_value()) {
    return;
  }

  const auto elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now() - pause_started_at_.value()
  )
                            .count();
  if (elapsed_ns > 0) {
    total_pause_offset_ns_.fetch_add(static_cast<uint64_t>(elapsed_ns), std::memory_order_acq_rel);
  }
  pause_started_at_.reset();
}

uint64_t AxonRecorder::current_pause_offset_ns() const {
  std::lock_guard<std::mutex> lock(pause_time_mutex_);
  uint64_t offset_ns = total_pause_offset_ns_.load(std::memory_order_acquire);
  if (pause_started_at_.has_value()) {
    const auto active_pause_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                   std::chrono::steady_clock::now() - pause_started_at_.value()
    )
                                   .count();
    if (active_pause_ns > 0) {
      offset_ns += static_cast<uint64_t>(active_pause_ns);
    }
  }

  return offset_ns;
}

uint64_t AxonRecorder::adjust_message_timestamp(uint64_t timestamp_ns) const {
  const uint64_t offset_ns = total_pause_offset_ns_.load(std::memory_order_acquire);
  return timestamp_ns > offset_ns ? timestamp_ns - offset_ns : 0;
}

void AxonRecorder::reset_topic_monotonic_timestamps() {
  std::lock_guard<std::mutex> lock(topic_monotonic_mutex_);
  last_written_log_time_ns_by_topic_.clear();
}

uint64_t AxonRecorder::apply_monotonic_timestamp_for_topic(
  const std::string& topic, uint64_t stamp_ns
) {
  if (!config_.recording.enforce_monotonic_timestamps_per_topic) {
    return stamp_ns;
  }

  std::lock_guard<std::mutex> lock(topic_monotonic_mutex_);
  uint64_t& last = last_written_log_time_ns_by_topic_[topic];
  if (stamp_ns <= last) {
    if (last == UINT64_MAX) {
      return UINT64_MAX;
    }
    stamp_ns = last + 1;
  }
  last = stamp_ns;
  return stamp_ns;
}

void AxonRecorder::set_task_config(const TaskConfig& config) {
  std::lock_guard<std::mutex> lock(recorder_mutex_);
  task_config_ = config;
}

const TaskConfig* AxonRecorder::get_task_config() const {
  std::lock_guard<std::mutex> lock(recorder_mutex_);
  return task_config_.has_value() ? &task_config_.value() : nullptr;
}

RecordingSession* AxonRecorder::get_recording_session() {
  std::lock_guard<std::mutex> lock(recorder_mutex_);
  return recording_session_.get();
}

std::string AxonRecorder::get_last_error() const {
  std::lock_guard<std::mutex> lock(error_mutex_);
  return last_error_;
}

AxonRecorder::Statistics AxonRecorder::get_statistics() const {
  std::lock_guard<std::mutex> lock(recorder_mutex_);
  Statistics stats{};  // Value-initialize all fields to 0

  // Get aggregate statistics from worker pool
  if (worker_pool_) {
    auto pool_stats = worker_pool_->get_aggregate_stats();
    stats.messages_received = pool_stats.total_received;
    stats.messages_written = pool_stats.total_written;
    stats.messages_dropped = pool_stats.total_dropped;
    stats.bytes_received = pool_stats.total_bytes_received;
    stats.bytes_written = pool_stats.total_bytes_written;
  }

  // Calculate bandwidth rates
  {
    std::lock_guard<std::mutex> qos_lock(qos_mutex_);
    auto now = std::chrono::steady_clock::now();
    auto elapsed =
      std::chrono::duration_cast<std::chrono::milliseconds>(now - last_rate_calc_time_).count();

    if (elapsed > 0) {
      double elapsed_sec = elapsed / 1000.0;
      uint64_t bytes_recv_delta = stats.bytes_received - last_bytes_received_;
      uint64_t bytes_write_delta = stats.bytes_written - last_bytes_written_;

      stats.receive_rate_mbps = (bytes_recv_delta / elapsed_sec) / (1024.0 * 1024.0);
      stats.write_rate_mbps = (bytes_write_delta / elapsed_sec) / (1024.0 * 1024.0);

      last_rate_calc_time_ = now;
      last_bytes_received_ = stats.bytes_received;
      last_bytes_written_ = stats.bytes_written;
    }
  }

  // Get statistics from recording session
  if (recording_session_) {
    auto session_stats = recording_session_->get_stats();
    stats.messages_written = session_stats.messages_written;
    stats.bytes_written = session_stats.bytes_written;
    // Expose running session duration so /rpc/stats and /rpc/state can
    // report active recording time alongside the message/byte counters.
    const double wall_duration_sec = recording_session_->get_duration_sec();
    const double paused_duration_sec = static_cast<double>(current_pause_offset_ns()) / 1e9;
    stats.duration_sec =
      wall_duration_sec > paused_duration_sec ? wall_duration_sec - paused_duration_sec : 0.0;
  }

  return stats;
}

namespace {
inline uint64_t get_steady_clock_ns() {
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                 std::chrono::steady_clock::now().time_since_epoch()
  )
                                 .count());
}
}  // namespace

void AxonRecorder::on_message(
  const char* topic_name, const uint8_t* message_data, size_t message_size,
  const char* message_type, uint64_t timestamp
) {
  // Only enqueue messages when in RECORDING state
  // Messages received in IDLE, READY, or PAUSED states are discarded
  if (!state_manager_.is_state(RecorderState::RECORDING) ||
      disk_hard_limit_reached_.load(std::memory_order_acquire)) {
    return;
  }

  uint64_t receive_time_ns = get_steady_clock_ns();

  // Acquire a pool-backed buffer sized for this payload. On the steady-state
  // hot path this avoids `operator new` per message (bucket reuse).
  MessageItem item;
  const uint64_t adjusted_timestamp = config_.recording.subtract_pause_duration_from_timestamps
                                        ? adjust_message_timestamp(timestamp)
                                        : timestamp;
  item.timestamp_ns = static_cast<int64_t>(adjusted_timestamp);
  item.publish_time_ns = adjusted_timestamp;
  item.receive_time_ns = receive_time_ns;
  item.message_type = message_type;
  item.raw_data =
    axon::memory::BufferPool::instance().acquire(message_size == 0 ? 1 : message_size);
  item.raw_data.assign(message_data, message_size);

  if (worker_pool_) {
    worker_pool_->try_push(topic_name, std::move(item));
  }
  // If queue is full or topic not found, the PooledBuffer returns to the pool
  // on MessageItem destruction. Statistics tracked by WorkerThreadPool.
}

void AxonRecorder::on_message_v2(
  const char* topic_name, const uint8_t* message_data, size_t message_size,
  const char* message_type, uint64_t timestamp, void (*release_fn)(void*), void* release_opaque
) {
  // Drop-and-release if we're not recording: the plugin gave us ownership,
  // so we must still call the release function even though we don't enqueue.
  if (!state_manager_.is_state(RecorderState::RECORDING) ||
      disk_hard_limit_reached_.load(std::memory_order_acquire)) {
    if (release_fn) {
      release_fn(release_opaque);
    }
    return;
  }

  uint64_t receive_time_ns = get_steady_clock_ns();

  MessageItem item;
  const uint64_t adjusted_timestamp = config_.recording.subtract_pause_duration_from_timestamps
                                        ? adjust_message_timestamp(timestamp)
                                        : timestamp;
  item.timestamp_ns = static_cast<int64_t>(adjusted_timestamp);
  item.publish_time_ns = adjusted_timestamp;
  item.receive_time_ns = receive_time_ns;
  item.message_type = message_type;

  if (release_fn != nullptr && message_data != nullptr) {
    // Zero-copy path: adopt the plugin's buffer; PooledBuffer will call
    // release_fn(release_opaque) exactly once when MessageItem is destroyed
    // (either after MCAP write or on queue-full drop).
    item.raw_data = axon::memory::PooledBuffer::adopt(
      const_cast<uint8_t*>(message_data), message_size, release_fn, release_opaque
    );
  } else {
    // Fallback: plugin advertised v1.2 but emitted a borrowed buffer with no
    // release hook. Treat as v1.x — copy into a pooled buffer.
    item.raw_data =
      axon::memory::BufferPool::instance().acquire(message_size == 0 ? 1 : message_size);
    item.raw_data.assign(message_data, message_size);
  }

  if (worker_pool_) {
    worker_pool_->try_push(topic_name, std::move(item));
  }
  // On queue-full or unknown-topic drop, MessageItem destruction triggers the
  // adopted release (or returns the pool slab) — symmetric with v1.1 path.
}

bool AxonRecorder::message_handler(
  const std::string& topic, const std::string& message_type, int64_t timestamp_ns,
  const uint8_t* data, size_t data_size, uint32_t sequence
) {
  if (!recording_session_) {
    return false;  // No active recording session
  }

  if (!ensure_disk_capacity_before_write(data_size)) {
    return false;
  }

  const uint64_t msg_time_ns = static_cast<uint64_t>(timestamp_ns);
  const uint64_t mcap_time_ns = apply_monotonic_timestamp_for_topic(topic, msg_time_ns);
  uint64_t write_time_ns = get_steady_clock_ns();

  uint16_t channel_id = recording_session_->get_or_create_channel(topic, message_type);
  if (channel_id == 0) {
    return false;
  }

  bool success =
    recording_session_->write(channel_id, sequence, mcap_time_ns, mcap_time_ns, data, data_size);

  if (success) {
    const SubscriptionConfig* sub_config = get_subscription_config(topic);
    if (sub_config) {
      recording_session_->update_topic_stats(topic, sub_config->message_type);
    }

    if (latency_monitor_) {
      LatencyRecord record;
      record.topic = topic;
      record.publish_time_ns = msg_time_ns;
      record.receive_time_ns = 0;
      record.enqueue_time_ns = 0;
      record.dequeue_time_ns = 0;
      record.write_time_ns = write_time_ns;
      record.record_time_ns = write_time_ns;
      latency_monitor_->record(record);
    }
  }

  return success;
}

bool AxonRecorder::latency_message_handler(
  const std::string& topic, const std::string& message_type, int64_t timestamp_ns,
  const uint8_t* data, size_t data_size, uint32_t sequence, uint64_t publish_time_ns,
  uint64_t receive_time_ns, uint64_t enqueue_time_ns, uint64_t dequeue_time_ns
) {
  if (!recording_session_) {
    return false;
  }

  if (!ensure_disk_capacity_before_write(data_size)) {
    return false;
  }

  const uint64_t msg_time_ns = static_cast<uint64_t>(timestamp_ns);
  const uint64_t mcap_time_ns = apply_monotonic_timestamp_for_topic(topic, msg_time_ns);
  uint64_t write_time_ns = get_steady_clock_ns();

  uint16_t channel_id = recording_session_->get_or_create_channel(topic, message_type);
  if (channel_id == 0) {
    return false;
  }

  bool success =
    recording_session_->write(channel_id, sequence, mcap_time_ns, mcap_time_ns, data, data_size);

  if (success) {
    const SubscriptionConfig* sub_config = get_subscription_config(topic);
    if (sub_config) {
      recording_session_->update_topic_stats(topic, sub_config->message_type);
    }

    if (latency_monitor_) {
      LatencyRecord record;
      record.topic = topic;
      record.publish_time_ns = (publish_time_ns > 0) ? publish_time_ns : msg_time_ns;
      record.receive_time_ns = receive_time_ns;
      record.enqueue_time_ns = enqueue_time_ns;
      record.dequeue_time_ns = dequeue_time_ns;
      record.write_time_ns = write_time_ns;
      record.record_time_ns = write_time_ns;
      latency_monitor_->record(record);
    }
  }

  return success;
}

bool AxonRecorder::register_topics() {
  if (!recording_session_) {
    set_error_helper("No active recording session");
    return false;
  }

  std::string ros_mw_name;
  get_ros_plugin_descriptor(&ros_mw_name);

  std::unordered_set<std::string> seen_subscription_keys;

  // Register schemas and channels for all subscriptions
  for (const auto& sub : config_.subscriptions) {
    std::string sub_key = sub.topic_name;
    sub_key.push_back('\0');
    sub_key += sub.message_type;
    if (seen_subscription_keys.count(sub_key)) {
      AXON_LOG_WARN(
        "Duplicate subscription entry" << axon::logging::kv("topic", sub.topic_name)
                                       << axon::logging::kv("message_type", sub.message_type)
      );
    }
    seen_subscription_keys.insert(sub_key);

    // Determine schema encoding based on message type
    bool is_json_type = (sub.message_type == "axon_udp/json");
    std::string schema_encoding =
      is_json_type ? "jsonschema" : (config_.recording.profile == "ros1" ? "ros1msg" : "ros2msg");
    std::string channel_encoding =
      is_json_type ? "json" : (config_.recording.profile == "ros1" ? "ros1" : "cdr");

    // Resolve schema definition from .msg files (if resolver is configured)
    std::string schema_definition;
    if (!is_json_type && schema_resolver_) {
      schema_definition = schema_resolver_->resolve(sub.message_type);
      if (schema_definition.empty()) {
        AXON_LOG_WARN(
          "Could not resolve schema for "
          << axon::logging::kv("type", sub.message_type) << ": "
          << axon::logging::kv("error", schema_resolver_->get_last_error())
          << ". MCAP will have empty schema data."
        );
      }
    }

    // Register schema (use message type as schema name)
    uint16_t schema_id =
      recording_session_->register_schema(sub.message_type, schema_encoding, schema_definition);

    if (schema_id == 0) {
      set_error_helper(
        "Failed to register schema for: " + sub.message_type + " - " +
        recording_session_->get_last_error()
      );
      return false;
    }

    std::unordered_map<std::string, std::string> ch_meta;
    if (is_json_type) {
      ch_meta["axon.source"] = "udp";
      ch_meta["axon.plugin"] = "UDP";
    } else {
      ch_meta["axon.source"] = config_.recording.profile;
      ch_meta["axon.plugin"] = ros_mw_name.empty() ? "ros" : ros_mw_name;
    }

    // Register channel using composite key (topic + message_type)
    // This allows the same topic to have different message types (e.g., Image and CompressedImage)
    uint16_t channel_id = recording_session_->register_channel(
      sub.topic_name, sub.message_type, channel_encoding, schema_id, ch_meta
    );

    if (channel_id == 0) {
      set_error_helper(
        "Failed to register channel for: " + sub.topic_name + " - " +
        recording_session_->get_last_error()
      );
      return false;
    }

    // Create topic worker in the pool with latency tracking
    WorkerThreadPool::LatencyMessageHandler handler = [this](
                                                        const std::string& topic,
                                                        const std::string& message_type,
                                                        int64_t timestamp_ns,
                                                        const uint8_t* data,
                                                        size_t data_size,
                                                        uint32_t sequence,
                                                        uint64_t publish_time_ns,
                                                        uint64_t receive_time_ns,
                                                        uint64_t enqueue_time_ns,
                                                        uint64_t dequeue_time_ns
                                                      ) -> bool {
      return this->latency_message_handler(
        topic,
        message_type,
        timestamp_ns,
        data,
        data_size,
        sequence,
        publish_time_ns,
        receive_time_ns,
        enqueue_time_ns,
        dequeue_time_ns
      );
    };

    // Plumb batch_size / flush_interval_ms from SubscriptionConfig into the worker.
    // This actually activates the batching fields that were previously parsed but unused.
    WorkerThreadPool::BatchConfig batch_cfg;
    batch_cfg.batch_size = sub.batch_size;
    batch_cfg.flush_interval_ms = sub.flush_interval_ms;

    if (!worker_pool_->create_topic_worker(sub.topic_name, handler, batch_cfg)) {
      set_error_helper("Failed to create worker for topic: " + sub.topic_name);
      return false;
    }
  }

  return true;
}

bool AxonRecorder::setup_subscriptions() {
  if (plugin_startup_order_.empty()) {
    set_error_helper("No plugins loaded");
    return false;
  }

  std::string ros_plugin_name;
  const AxonPluginDescriptor* ros_descriptor = get_ros_plugin_descriptor(&ros_plugin_name);
  const AxonPluginDescriptor* udp_descriptor = get_udp_plugin_descriptor();

  bool need_ros = false;
  bool need_udp = false;
  for (const auto& sub : config_.subscriptions) {
    if (sub.message_type == "axon_udp/json") {
      need_udp = true;
    } else {
      need_ros = true;
    }
  }

  if (need_ros) {
    if (!ros_descriptor || !ros_descriptor->vtable || !ros_descriptor->vtable->subscribe) {
      set_error_helper(
        "ROS middleware plugin required for subscriptions with non-UDP message types "
        "(load a ROS1/ROS2 plugin before UDP in `plugins` / plugin.path)"
      );
      return false;
    }

    AxonSubscribeV2Fn ros_subscribe_v2 = plugin_subscribe_v2(ros_descriptor);
    if (ros_subscribe_v2 != nullptr) {
      AXON_LOG_INFO(
        "Plugin '" << ros_plugin_name
                   << "' supports ABI v1.2 zero-copy callback; using subscribe_v2"
      );
    }

    for (const auto& sub : config_.subscriptions) {
      if (sub.message_type == "axon_udp/json") {
        continue;
      }

      std::string options_json;
      if (sub.depth_compression.enabled) {
        nlohmann::json opts;
        opts["depth_compression"]["enabled"] = sub.depth_compression.enabled;
        opts["depth_compression"]["level"] = sub.depth_compression.level;
        options_json = opts.dump();
      }

      AxonStatus status = AXON_ERROR_INTERNAL;
      if (ros_subscribe_v2 != nullptr) {
        status = ros_subscribe_v2(
          sub.topic_name.c_str(),
          sub.message_type.c_str(),
          options_json.empty() ? nullptr : options_json.c_str(),
          message_callback_v2,
          this
        );
      } else {
        status = ros_descriptor->vtable->subscribe(
          sub.topic_name.c_str(),
          sub.message_type.c_str(),
          options_json.empty() ? nullptr : options_json.c_str(),
          message_callback,
          this
        );
      }

      if (status != AXON_SUCCESS) {
        set_error_helper(
          "Failed to subscribe to " + sub.topic_name + " (" + sub.message_type +
          "): " + status_to_string(status)
        );
        return false;
      }
    }
  }

  if (need_udp) {
    if (!udp_descriptor || !udp_descriptor->vtable || !udp_descriptor->vtable->subscribe) {
      set_error_helper(
        "axon_udp/json subscriptions require the UDP plugin .so "
        "(add `libaxon_udp` to `plugins` / plugin.auxiliary_paths)"
      );
      return false;
    }

    const SubscriptionConfig* first_udp = nullptr;
    for (const auto& sub : config_.subscriptions) {
      if (sub.message_type == "axon_udp/json") {
        first_udp = &sub;
        break;
      }
    }
    if (first_udp == nullptr) {
      set_error_helper("Internal error: UDP subscription flag without axon_udp/json entry");
      return false;
    }

    AxonStatus ustatus = udp_descriptor->vtable->subscribe(
      first_udp->topic_name.c_str(),
      first_udp->message_type.c_str(),
      nullptr,
      message_callback,
      this
    );
    if (ustatus != AXON_SUCCESS) {
      set_error_helper(std::string("UDP plugin subscribe failed: ") + status_to_string(ustatus));
      return false;
    }
  }

  return true;
}

const SubscriptionConfig* AxonRecorder::get_subscription_config(const std::string& topic_name
) const {
  for (const auto& sub : config_.subscriptions) {
    if (sub.topic_name == topic_name) {
      return &sub;
    }
  }
  return nullptr;
}

const char* AxonRecorder::status_to_string(AxonStatus status) {
  switch (status) {
    case AXON_SUCCESS:
      return "Success";
    case AXON_ERROR_INVALID_ARGUMENT:
      return "Invalid argument";
    case AXON_ERROR_NOT_INITIALIZED:
      return "Not initialized";
    case AXON_ERROR_ALREADY_INITIALIZED:
      return "Already initialized";
    case AXON_ERROR_NOT_STARTED:
      return "Not started";
    case AXON_ERROR_ALREADY_STARTED:
      return "Already started";
    case AXON_ERROR_INTERNAL:
      return "Internal error";
    default:
      return "Unknown error";
  }
}

void AxonRecorder::set_error_helper(const std::string& error) {
  std::lock_guard<std::mutex> lock(error_mutex_);
  last_error_ = error;
}

void AxonRecorder::report_message_drop(
  const std::string& topic, const std::string& message_type, uint64_t total_dropped
) {
  // Rate limit: at most one log line per topic per second.
  // Between two log lines, unreported drops are counted and included
  // in the next log as `recent_drops` (drops since last report).
  constexpr auto kMinInterval = std::chrono::seconds(1);

  auto now = std::chrono::steady_clock::now();
  uint64_t recent_drops = 0;

  {
    std::lock_guard<std::mutex> lock(drop_report_mutex_);
    auto& state = drop_report_states_[topic];

    if (now - state.last_report_time < kMinInterval) {
      ++state.suppressed_count;
      return;
    }

    // This drop + previously unreported drops = recent_drops
    recent_drops = state.suppressed_count + 1;
    state.suppressed_count = 0;
    state.last_report_time = now;
  }

  // Log outside the lock to avoid holding it during I/O
  //
  // Fields:
  //   topic         - topic name where the drop occurred
  //   message_type  - ROS message type
  //   total_dropped - cumulative drop count for this topic since recording started
  //   recent_drops  - number of drops since the last log line (>1 when rate-limited)
  if (recent_drops > 1) {
    AXON_LOG_WARN(
      "Message dropped (queue full)" << axon::logging::kv("topic", topic)
                                     << axon::logging::kv("message_type", message_type)
                                     << axon::logging::kv("total_dropped", total_dropped)
                                     << axon::logging::kv("recent_drops", recent_drops)
    );
  } else {
    AXON_LOG_WARN(
      "Message dropped (queue full)" << axon::logging::kv("topic", topic)
                                     << axon::logging::kv("message_type", message_type)
                                     << axon::logging::kv("total_dropped", total_dropped)
    );
  }
}

void AxonRecorder::on_state_transition(RecorderState from, RecorderState to) {
  std::string task_id;
  if (task_config_.has_value()) {
    task_id = task_config_->task_id;
  }

  if (http_server_) {
    http_server_->broadcast_state_change(from, to, task_id);

    // Broadcast log for important state transitions
    const bool cancelled = cancel_in_progress_.load(std::memory_order_acquire);
    if (to == RecorderState::RECORDING) {
      http_server_->broadcast_log("info", "Recording started");
    } else if (to == RecorderState::PAUSED) {
      http_server_->broadcast_log("info", "Recording paused");
    } else if (to == RecorderState::IDLE &&
               (from == RecorderState::RECORDING || from == RecorderState::PAUSED)) {
      http_server_->broadcast_log("info", cancelled ? "Recording cancelled" : "Recording finished");
    } else if (to == RecorderState::READY) {
      http_server_->broadcast_log("info", "Configuration set, ready to record");
    }
  }

  AXON_LOG_INFO("State transition: " << state_to_string(from) << " -> " << state_to_string(to));
  if (ws_rpc_client_) {
    AXON_LOG_INFO("Sending state update to WebSocket RPC server");
    ws_rpc_client_->send_state_update(from, to, task_id);
  }
}

void AxonRecorder::request_shutdown() {
  // Call the shutdown callback if registered
  if (shutdown_callback_) {
    shutdown_callback_();
  }
}

void AxonRecorder::set_shutdown_callback(ShutdownCallback callback) {
  shutdown_callback_ = callback;
}

AxonRecorder::ShutdownCallback AxonRecorder::get_shutdown_callback() const {
  return shutdown_callback_;
}

bool AxonRecorder::start_ws_rpc_client(const WsClientConfig& config) {
  if (ws_rpc_client_) {
    set_error_helper("WebSocket RPC client already running");
    return false;
  }

  if (config.url.empty()) {
    set_error_helper("WebSocket URL is empty");
    return false;
  }

  AXON_LOG_INFO("Starting WebSocket RPC client, url=" << config.url);

  RpcCallbacks callbacks = make_rpc_callbacks();

  // Create WS RPC client and run io_context on a dedicated thread
  ws_rpc_client_ = std::make_unique<WsRpcClient>(ws_ioc_, config);
  ws_rpc_client_->register_callbacks(callbacks);
  ws_rpc_client_->start();

  // Start the io_context thread so async operations are driven
  ws_thread_ = std::thread([this]() {
    ws_ioc_.run();
  });

  return true;
}

void AxonRecorder::stop_ws_rpc_client() {
  if (ws_rpc_client_) {
    ws_rpc_client_->stop();
    ws_rpc_client_.reset();
  }

  // Stop the io_context
  ws_ioc_.stop();

  // Wait for thread to finish
  if (ws_thread_.joinable()) {
    ws_thread_.join();
  }
}

bool AxonRecorder::is_ws_rpc_client_running() const {
  return ws_rpc_client_ && ws_rpc_client_->is_connected();
}

}  // namespace recorder
}  // namespace axon
