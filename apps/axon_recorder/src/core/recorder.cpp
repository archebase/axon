// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "recorder.hpp"

#include <nlohmann/json.hpp>

#include <chrono>
#include <cstring>
#include <ctime>
#include <functional>
#include <iostream>
#include <mutex>
#include <thread>

#include "../http/rpc_handlers.hpp"
#include "../http/ws_rpc_client.hpp"

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
// C callback wrapper for plugin
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
}

bool AxonRecorder::initialize(const RecorderConfig& config) {
  config_ = config;

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
      "Schema resolver initialized with " <<
      axon::logging::kv("paths_count", config_.recording.schema_search_paths.size())
      << " search paths"
    );
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

  // Load plugin
  auto plugin_name = plugin_loader_.load(config_.plugin_path);
  if (!plugin_name.has_value()) {
    set_error_helper("Failed to load plugin: " + plugin_loader_.get_last_error());
    state_manager_.transition_to(RecorderState::IDLE, error_msg);
    return false;
  }

  // Get plugin descriptor
  const auto* descriptor = plugin_loader_.get_descriptor(plugin_name.value());
  if (!descriptor || !descriptor->vtable) {
    set_error_helper("Invalid plugin descriptor");
    state_manager_.transition_to(RecorderState::IDLE, error_msg);
    return false;
  }

  // Initialize plugin
  if (descriptor->vtable->init) {
    AxonStatus status = descriptor->vtable->init(config_.plugin_config.c_str());
    if (status != AXON_SUCCESS) {
      set_error_helper(std::string("Plugin init failed: ") + status_to_string(status));
      state_manager_.transition_to(RecorderState::IDLE, error_msg);
      return false;
    }
  }

  // Create recording session
  recording_session_ = std::make_unique<RecordingSession>();

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

  // Start plugin
  if (descriptor->vtable->start) {
    AxonStatus status = descriptor->vtable->start();
    if (status != AXON_SUCCESS) {
      set_error_helper(std::string("Plugin start failed: ") + status_to_string(status));
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

  std::string error_msg;

  // Stop worker thread pool (drains remaining messages)
  if (worker_pool_) {
    worker_pool_->stop();
  }

  // Stop plugin
  auto plugins = plugin_loader_.loaded_plugins();
  for (const auto& plugin_name : plugins) {
    const auto* descriptor = plugin_loader_.get_descriptor(plugin_name);
    if (descriptor && descriptor->vtable && descriptor->vtable->stop) {
      descriptor->vtable->stop();
    }
  }

  // Close recording session (injects metadata, generates sidecar)
  if (recording_session_) {
    recording_session_->close();

    last_session_final_file_size_ = recording_session_->get_final_file_size();
    last_session_close_time_ = recording_session_->get_close_time();

    recording_session_.reset();
  }

  // Unload plugins
  plugin_loader_.unload_all();

  // Reset worker pool statistics and drop report state for next session
  if (worker_pool_) {
    worker_pool_->reset_stats();
  }
  {
    std::lock_guard<std::mutex> lock(drop_report_mutex_);
    drop_report_states_.clear();
  }

  // Transition back to IDLE
  state_manager_.transition_to(RecorderState::IDLE, error_msg);

  running_.store(false);
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

bool AxonRecorder::start_http_server(const std::string& host, uint16_t port) {
  if (http_server_) {
    set_error_helper("HTTP server already running");
    return false;
  }

  http_server_ = std::make_unique<HttpServer>(host, port);

  // Register callbacks with the HTTP server
  HttpServer::Callbacks callbacks;
  callbacks.get_state = [this]() -> std::string {
    return this->get_state_string();
  };

  callbacks.get_stats = [this]() -> nlohmann::json {
    auto stats = this->get_statistics();
    nlohmann::json j;
    j["messages_received"] = stats.messages_received;
    j["messages_written"] = stats.messages_written;
    j["messages_dropped"] = stats.messages_dropped;
    j["bytes_written"] = stats.bytes_written;
    j["bytes_received"] = stats.bytes_received;
    j["receive_rate_mbps"] = stats.receive_rate_mbps;
    j["write_rate_mbps"] = stats.write_rate_mbps;

    // Add queue depth monitoring
    if (worker_pool_) {
      auto depths = worker_pool_->get_queue_depths();
      nlohmann::json queue_depths = nlohmann::json::object();
      for (const auto& [topic, info] : depths) {
        nlohmann::json q;
        q["depth"] = info.depth;
        q["capacity"] = info.capacity;
        q["utilization"] = info.capacity > 0 ?
          static_cast<double>(info.depth) / info.capacity * 100.0 : 0.0;
        queue_depths[topic] = q;
      }
      j["queue_depths"] = queue_depths;
    }

    return j;
  };

  callbacks.get_drop_stats = [this]() -> nlohmann::json {
    std::lock_guard<std::mutex> lock(recorder_mutex_);
    nlohmann::json j;
    j["state"] = this->get_state_string();
    auto stats = this->get_statistics();
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
    std::lock_guard<std::mutex> lock(recorder_mutex_);
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
      TaskConfig config;

      // Parse the config JSON
      if (config_json.contains("task_id")) {
        config.task_id = config_json["task_id"];
      }
      if (config_json.contains("device_id")) {
        config.device_id = config_json["device_id"];
      }
      if (config_json.contains("data_collector_id")) {
        config.data_collector_id = config_json["data_collector_id"];
      }
      if (config_json.contains("order_id")) {
        config.order_id = config_json["order_id"];
      }
      if (config_json.contains("operator_name")) {
        config.operator_name = config_json["operator_name"];
      }
      if (config_json.contains("scene")) {
        config.scene = config_json["scene"];
      }
      if (config_json.contains("subscene")) {
        config.subscene = config_json["subscene"];
      }
      if (config_json.contains("skills")) {
        for (const auto& skill : config_json["skills"]) {
          config.skills.push_back(skill);
        }
      }
      if (config_json.contains("factory")) {
        config.factory = config_json["factory"];
      }
      if (config_json.contains("topics")) {
        for (const auto& topic : config_json["topics"]) {
          config.topics.push_back(topic);
        }
      }
      if (config_json.contains("start_callback_url")) {
        config.start_callback_url = config_json["start_callback_url"];
      }
      if (config_json.contains("finish_callback_url")) {
        config.finish_callback_url = config_json["finish_callback_url"];
      }
      if (config_json.contains("user_token")) {
        config.user_token = config_json["user_token"];
      }

      // Set the task config
      this->set_task_config(config);

      // Broadcast config change via WebSocket
      if (http_server_) {
        http_server_->broadcast_config_change(&config);
      }

      // Transition state from IDLE to READY
      auto current_state = this->get_state();
      if (current_state == RecorderState::IDLE) {
        std::string error_msg;
        if (!this->transition_to(RecorderState::READY, error_msg)) {
          return false;
        }
      }

      return true;
    } catch (const std::exception& e) {
      return false;
    }
  };

  callbacks.begin_recording = [this](const std::string& task_id) -> bool {
    // Verify task_id matches
    const TaskConfig* task_config = this->get_task_config();
    if (!task_config || task_config->task_id.empty()) {
      return false;
    }

    if (task_id != task_config->task_id) {
      return false;
    }

    // Check if we're in READY state
    if (this->get_state() != RecorderState::READY) {
      return false;
    }

    // Start recording
    return this->start();
  };

  callbacks.finish_recording = [this](const std::string& task_id) -> bool {
    // Verify task_id matches
    const TaskConfig* task_config = this->get_task_config();
    if (!task_config || task_config->task_id.empty()) {
      return false;
    }

    if (task_id != task_config->task_id) {
      return false;
    }

    // Check if we're in RECORDING or PAUSED state
    auto current_state = this->get_state();
    if (current_state != RecorderState::RECORDING && current_state != RecorderState::PAUSED) {
      return false;
    }

    std::string output_path;
    std::string sidecar_path;
    std::chrono::system_clock::time_point start_time;

    if (recording_session_) {
      output_path = recording_session_->get_path();
      sidecar_path = recording_session_->get_sidecar_path();
      start_time = recording_session_->get_start_time();
    }

    // Convert start_time to ISO8601 string
    std::string started_at;
    if (start_time != std::chrono::system_clock::time_point{}) {
      started_at = HttpCallbackClient::get_iso8601_timestamp(start_time);
    }

    // Stop recording
    if (this->is_running()) {
      this->stop();

      auto stats = this->get_statistics();
      int64_t file_size_bytes = (last_session_final_file_size_ > 0)
                                  ? static_cast<int64_t>(last_session_final_file_size_)
                                  : static_cast<int64_t>(stats.bytes_written);

      std::string finished_at;
      double duration_sec = 0.0;
      if (last_session_close_time_ != std::chrono::system_clock::time_point{}) {
        finished_at = HttpCallbackClient::get_iso8601_timestamp(last_session_close_time_);
        if (start_time != std::chrono::system_clock::time_point{}) {
          duration_sec =
            std::chrono::duration<double>(last_session_close_time_ - start_time).count();
        }
      } else {
        finished_at = HttpCallbackClient::get_iso8601_timestamp();
      }

      // Send finish callback if URL is configured
      if (http_callback_client_ && !task_config->finish_callback_url.empty()) {
        FinishCallbackPayload payload;
        payload.task_id = task_id;
        payload.device_id = task_config->device_id;
        payload.status = "finished";
        payload.started_at = started_at;
        payload.finished_at = finished_at;
        payload.duration_sec = duration_sec;
        payload.message_count = static_cast<int64_t>(stats.messages_written);
        payload.file_size_bytes = file_size_bytes;
        payload.output_path = output_path;
        payload.sidecar_path = sidecar_path;
        payload.topics = task_config->topics;

        // Map TaskConfig fields to CallbackMetadata
        payload.metadata.scene = task_config->scene;
        payload.metadata.subscene = task_config->subscene;
        payload.metadata.skills = task_config->skills;
        payload.metadata.factory = task_config->factory;

        http_callback_client_->post_finish_callback_async(*task_config, payload);
      }

      return true;
    }
    return false;
  };

  callbacks.cancel_recording = [this]() -> bool {
    // Check if we're in RECORDING or PAUSED state
    auto current_state = this->get_state();
    if (current_state != RecorderState::RECORDING && current_state != RecorderState::PAUSED) {
      return false;
    }

    // Stop recording (TODO: add cleanup to discard partial recording)
    if (this->is_running()) {
      this->stop();
      return true;
    }
    return false;
  };

  callbacks.pause_recording = [this]() -> bool {
    // Check if we're in RECORDING state
    if (this->get_state() != RecorderState::RECORDING) {
      return false;
    }

    std::string error_msg;
    return this->transition_to(RecorderState::PAUSED, error_msg);
  };

  callbacks.resume_recording = [this]() -> bool {
    // Check if we're in PAUSED state
    if (this->get_state() != RecorderState::PAUSED) {
      return false;
    }

    std::string error_msg;
    return this->transition_to(RecorderState::RECORDING, error_msg);
  };

  callbacks.clear_config = [this]() -> bool {
    // Check if we're in READY state
    if (this->get_state() != RecorderState::READY) {
      return false;
    }

    // TODO: Implement clear to reset task config and return to IDLE
    std::string error_msg;
    return this->transition_to(RecorderState::IDLE, error_msg);
  };

  callbacks.get_task_config = [this]() -> const TaskConfig* {
    return this->get_task_config();
  };

  callbacks.quit = [this]() -> void {
    this->request_shutdown();
  };

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
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_rate_calc_time_).count();

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
  }

  return stats;
}

namespace {
inline uint64_t get_steady_clock_ns() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now()
                                                              .time_since_epoch())
          .count());
}
}  // namespace

void AxonRecorder::on_message(
  const char* topic_name, const uint8_t* message_data, size_t message_size,
  const char* message_type, uint64_t timestamp
) {
  // Only enqueue messages when in RECORDING state
  // Messages received in IDLE, READY, or PAUSED states are discarded
  if (!state_manager_.is_state(RecorderState::RECORDING)) {
    return;
  }

  uint64_t receive_time_ns = get_steady_clock_ns();

  // Create MessageItem and push to worker pool
  MessageItem item;
  item.timestamp_ns = static_cast<int64_t>(timestamp);
  item.publish_time_ns = timestamp;
  item.receive_time_ns = receive_time_ns;
  item.message_type = message_type;  // Save the message type
  item.raw_data.assign(message_data, message_data + message_size);

  // Push to the topic's queue (worker pool handles this)
  if (worker_pool_) {
    worker_pool_->try_push(topic_name, std::move(item));
  }
  // If queue is full or topic not found, message is dropped
  // Statistics are tracked by WorkerThreadPool
}

bool AxonRecorder::message_handler(
  const std::string& topic, const std::string& message_type, int64_t timestamp_ns,
  const uint8_t* data, size_t data_size, uint32_t sequence
) {
  if (!recording_session_) {
    return false;  // No active recording session
  }

  uint64_t log_time_ns = static_cast<uint64_t>(timestamp_ns);
  uint64_t write_time_ns = get_steady_clock_ns();

  uint16_t channel_id = recording_session_->get_or_create_channel(topic, message_type);
  if (channel_id == 0) {
    return false;
  }

  bool success =
    recording_session_->write(channel_id, sequence, log_time_ns, timestamp_ns, data, data_size);

  if (success) {
    const SubscriptionConfig* sub_config = get_subscription_config(topic);
    if (sub_config) {
      recording_session_->update_topic_stats(topic, sub_config->message_type);
    }

    if (latency_monitor_) {
      LatencyRecord record;
      record.topic = topic;
      record.publish_time_ns = log_time_ns;
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

  uint64_t log_time_ns = static_cast<uint64_t>(timestamp_ns);
  uint64_t write_time_ns = get_steady_clock_ns();

  uint16_t channel_id = recording_session_->get_or_create_channel(topic, message_type);
  if (channel_id == 0) {
    return false;
  }

  bool success =
    recording_session_->write(channel_id, sequence, log_time_ns, timestamp_ns, data, data_size);

  if (success) {
    const SubscriptionConfig* sub_config = get_subscription_config(topic);
    if (sub_config) {
      recording_session_->update_topic_stats(topic, sub_config->message_type);
    }

    if (latency_monitor_) {
      LatencyRecord record;
      record.topic = topic;
      record.publish_time_ns = (publish_time_ns > 0) ? publish_time_ns : log_time_ns;
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

  // Register schemas and channels for all subscriptions
  for (const auto& sub : config_.subscriptions) {
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
          "Could not resolve schema for " << axon::logging::kv("type", sub.message_type)
          << ": " << axon::logging::kv("error", schema_resolver_->get_last_error())
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

    // Register channel using composite key (topic + message_type)
    // This allows the same topic to have different message types (e.g., Image and CompressedImage)
    uint16_t channel_id = recording_session_->register_channel(
      sub.topic_name, sub.message_type, channel_encoding, schema_id
    );

    if (channel_id == 0) {
      set_error_helper(
        "Failed to register channel for: " + sub.topic_name + " - " +
        recording_session_->get_last_error()
      );
      return false;
    }

    // Create topic worker in the pool with latency tracking
    WorkerThreadPool::LatencyMessageHandler handler =
      [this](
        const std::string& topic, const std::string& message_type, int64_t timestamp_ns,
        const uint8_t* data, size_t data_size, uint32_t sequence, uint64_t publish_time_ns,
        uint64_t receive_time_ns, uint64_t enqueue_time_ns, uint64_t dequeue_time_ns) -> bool {
        return this->latency_message_handler(
          topic, message_type, timestamp_ns, data, data_size, sequence,
          publish_time_ns, receive_time_ns, enqueue_time_ns, dequeue_time_ns);
      };

    if (!worker_pool_->create_topic_worker(sub.topic_name, handler)) {
      set_error_helper("Failed to create worker for topic: " + sub.topic_name);
      return false;
    }
  }

  return true;
}

bool AxonRecorder::setup_subscriptions() {
  auto plugins = plugin_loader_.loaded_plugins();
  if (plugins.empty()) {
    set_error_helper("No plugins loaded");
    return false;
  }

  const auto* descriptor = plugin_loader_.get_descriptor(plugins[0]);
  if (!descriptor || !descriptor->vtable) {
    set_error_helper("Invalid plugin descriptor or vtable");
    return false;
  }

  if (!descriptor->vtable->subscribe) {
    set_error_helper("Plugin does not support subscribe (null function pointer)");
    return false;
  }

  // Setup subscriptions via plugin
  for (const auto& sub : config_.subscriptions) {
    // Build options JSON for depth compression (if enabled)
    std::string options_json;
    if (sub.depth_compression.enabled) {
      nlohmann::json opts;
      opts["depth_compression"]["enabled"] = sub.depth_compression.enabled;
      opts["depth_compression"]["level"] = sub.depth_compression.level;
      options_json = opts.dump();
    }

    AxonStatus status = descriptor->vtable->subscribe(
      sub.topic_name.c_str(),
      sub.message_type.c_str(),
      options_json.empty() ? nullptr : options_json.c_str(),
      message_callback,
      this
    );

    if (status != AXON_SUCCESS) {
      set_error_helper(
        "Failed to subscribe to " + sub.topic_name + " (" + sub.message_type +
        "): " + status_to_string(status)
      );
      return false;
    }
  }

  return true;
}

const SubscriptionConfig* AxonRecorder::get_subscription_config(
  const std::string& topic_name
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
      "Message dropped (queue full)"
      << axon::logging::kv("topic", topic)
      << axon::logging::kv("message_type", message_type)
      << axon::logging::kv("total_dropped", total_dropped)
      << axon::logging::kv("recent_drops", recent_drops)
    );
  } else {
    AXON_LOG_WARN(
      "Message dropped (queue full)"
      << axon::logging::kv("topic", topic)
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
    if (to == RecorderState::RECORDING) {
      http_server_->broadcast_log("info", "Recording started");
    } else if (to == RecorderState::PAUSED) {
      http_server_->broadcast_log("info", "Recording paused");
    } else if (to == RecorderState::IDLE && from == RecorderState::RECORDING) {
      http_server_->broadcast_log("info", "Recording finished");
    } else if (to == RecorderState::IDLE && from == RecorderState::PAUSED) {
      http_server_->broadcast_log("info", "Recording cancelled");
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

  // Create RpcCallbacks structure (same as HTTP server)
  RpcCallbacks callbacks;
  callbacks.get_state = [this]() -> std::string {
    return this->get_state_string();
  };

  callbacks.get_stats = [this]() -> nlohmann::json {
    auto stats = this->get_statistics();
    nlohmann::json j;
    j["messages_received"] = stats.messages_received;
    j["messages_written"] = stats.messages_written;
    j["messages_dropped"] = stats.messages_dropped;
    j["bytes_written"] = stats.bytes_written;
    j["bytes_received"] = stats.bytes_received;
    j["receive_rate_mbps"] = stats.receive_rate_mbps;
    j["write_rate_mbps"] = stats.write_rate_mbps;

    // Add queue depth monitoring
    if (worker_pool_) {
      auto depths = worker_pool_->get_queue_depths();
      nlohmann::json queue_depths = nlohmann::json::object();
      for (const auto& [topic, info] : depths) {
        nlohmann::json q;
        q["depth"] = info.depth;
        q["capacity"] = info.capacity;
        q["utilization"] = info.capacity > 0 ?
          static_cast<double>(info.depth) / info.capacity * 100.0 : 0.0;
        queue_depths[topic] = q;
      }
      j["queue_depths"] = queue_depths;
    }

    return j;
  };

  callbacks.get_drop_stats = [this]() -> nlohmann::json {
    std::lock_guard<std::mutex> lock(recorder_mutex_);
    nlohmann::json j;
    j["state"] = this->get_state_string();
    auto stats = this->get_statistics();
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
    std::lock_guard<std::mutex> lock(recorder_mutex_);
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
      TaskConfig config;

      // Parse the config JSON
      if (config_json.contains("task_id")) {
        config.task_id = config_json["task_id"];
      }
      if (config_json.contains("device_id")) {
        config.device_id = config_json["device_id"];
      }
      if (config_json.contains("data_collector_id")) {
        config.data_collector_id = config_json["data_collector_id"];
      }
      if (config_json.contains("order_id")) {
        config.order_id = config_json["order_id"];
      }
      if (config_json.contains("operator_name")) {
        config.operator_name = config_json["operator_name"];
      }
      if (config_json.contains("scene")) {
        config.scene = config_json["scene"];
      }
      if (config_json.contains("subscene")) {
        config.subscene = config_json["subscene"];
      }
      if (config_json.contains("skills")) {
        for (const auto& skill : config_json["skills"]) {
          config.skills.push_back(skill);
        }
      }
      if (config_json.contains("factory")) {
        config.factory = config_json["factory"];
      }
      if (config_json.contains("topics")) {
        for (const auto& topic : config_json["topics"]) {
          config.topics.push_back(topic);
        }
      }
      if (config_json.contains("start_callback_url")) {
        config.start_callback_url = config_json["start_callback_url"];
      }
      if (config_json.contains("finish_callback_url")) {
        config.finish_callback_url = config_json["finish_callback_url"];
      }
      if (config_json.contains("user_token")) {
        config.user_token = config_json["user_token"];
      }

      // Set the task config
      this->set_task_config(config);

      // Transition state from IDLE to READY
      auto current_state = this->get_state();
      if (current_state == RecorderState::IDLE) {
        std::string error_msg;
        if (!this->transition_to(RecorderState::READY, error_msg)) {
          return false;
        }
      }

      return true;
    } catch (const std::exception& e) {
      return false;
    }
  };

  callbacks.begin_recording = [this](const std::string& task_id) -> bool {
    // Verify task_id matches
    const TaskConfig* task_config = this->get_task_config();
    if (!task_config || task_config->task_id.empty()) {
      return false;
    }

    if (task_id != task_config->task_id) {
      return false;
    }

    // Check if we're in READY state
    if (this->get_state() != RecorderState::READY) {
      return false;
    }

    // Start recording
    return this->start();
  };

  callbacks.finish_recording = [this](const std::string& task_id) -> bool {
    // Verify task_id matches
    const TaskConfig* task_config = this->get_task_config();
    if (!task_config || task_config->task_id.empty()) {
      return false;
    }

    if (task_id != task_config->task_id) {
      return false;
    }

    // Check if we're in RECORDING or PAUSED state
    auto current_state = this->get_state();
    if (current_state != RecorderState::RECORDING && current_state != RecorderState::PAUSED) {
      return false;
    }

    std::string output_path;
    std::string sidecar_path;
    std::chrono::system_clock::time_point start_time;

    if (recording_session_) {
      output_path = recording_session_->get_path();
      sidecar_path = recording_session_->get_sidecar_path();
      start_time = recording_session_->get_start_time();
    }

    // Convert start_time to ISO8601 string
    std::string started_at;
    if (start_time != std::chrono::system_clock::time_point{}) {
      started_at = HttpCallbackClient::get_iso8601_timestamp(start_time);
    }

    // Stop recording
    if (this->is_running()) {
      this->stop();

      auto stats = this->get_statistics();
      int64_t file_size_bytes = (last_session_final_file_size_ > 0)
                                  ? static_cast<int64_t>(last_session_final_file_size_)
                                  : static_cast<int64_t>(stats.bytes_written);

      std::string finished_at;
      double duration_sec = 0.0;
      if (last_session_close_time_ != std::chrono::system_clock::time_point{}) {
        finished_at = HttpCallbackClient::get_iso8601_timestamp(last_session_close_time_);
        if (start_time != std::chrono::system_clock::time_point{}) {
          duration_sec =
            std::chrono::duration<double>(last_session_close_time_ - start_time).count();
        }
      } else {
        finished_at = HttpCallbackClient::get_iso8601_timestamp();
      }

      // Send finish callback if URL is configured
      if (http_callback_client_ && !task_config->finish_callback_url.empty()) {
        FinishCallbackPayload payload;
        payload.task_id = task_id;
        payload.device_id = task_config->device_id;
        payload.status = "finished";
        payload.started_at = started_at;
        payload.finished_at = finished_at;
        payload.duration_sec = duration_sec;
        payload.message_count = static_cast<int64_t>(stats.messages_written);
        payload.file_size_bytes = file_size_bytes;
        payload.output_path = output_path;
        payload.sidecar_path = sidecar_path;
        payload.topics = task_config->topics;

        // Map TaskConfig fields to CallbackMetadata
        payload.metadata.scene = task_config->scene;
        payload.metadata.subscene = task_config->subscene;
        payload.metadata.skills = task_config->skills;
        payload.metadata.factory = task_config->factory;

        http_callback_client_->post_finish_callback_async(*task_config, payload);
      }

      return true;
    }
    return false;
  };

  callbacks.cancel_recording = [this]() -> bool {
    // Check if we're in RECORDING or PAUSED state
    auto current_state = this->get_state();
    if (current_state != RecorderState::RECORDING && current_state != RecorderState::PAUSED) {
      return false;
    }

    // Stop recording
    if (this->is_running()) {
      this->stop();
      return true;
    }
    return false;
  };

  callbacks.pause_recording = [this]() -> bool {
    // Check if we're in RECORDING state
    if (this->get_state() != RecorderState::RECORDING) {
      return false;
    }

    std::string error_msg;
    return this->transition_to(RecorderState::PAUSED, error_msg);
  };

  callbacks.resume_recording = [this]() -> bool {
    // Check if we're in PAUSED state
    if (this->get_state() != RecorderState::PAUSED) {
      return false;
    }

    std::string error_msg;
    return this->transition_to(RecorderState::RECORDING, error_msg);
  };

  callbacks.clear_config = [this]() -> bool {
    // Check if we're in READY state
    if (this->get_state() != RecorderState::READY) {
      return false;
    }

    std::string error_msg;
    return this->transition_to(RecorderState::IDLE, error_msg);
  };

  callbacks.quit = [this]() -> void {
    this->request_shutdown();
  };

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
