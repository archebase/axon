#include "recorder.hpp"

#include <chrono>
#include <cstring>
#include <ctime>
#include <functional>
#include <iostream>
#include <mutex>
#include <thread>

#include "mcap_writer_wrapper.hpp"

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
    : running_(false) {
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

  return true;
}

bool AxonRecorder::start() {
  std::string error_msg;

  // Transition from IDLE to READY
  if (!state_manager_.transition(RecorderState::IDLE, RecorderState::READY, error_msg)) {
    set_error_helper("State transition to READY failed: " + error_msg);
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
    AxonStatus status = descriptor->vtable->init("");
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
  mcap_options.profile = config_.profile;

  if (config_.compression == "zstd") {
    mcap_options.compression = mcap_wrapper::Compression::Zstd;
  } else if (config_.compression == "lz4") {
    mcap_options.compression = mcap_wrapper::Compression::Lz4;
  } else {
    mcap_options.compression = mcap_wrapper::Compression::None;
  }

  mcap_options.compression_level = config_.compression_level;

  // Open MCAP file via recording session
  if (!recording_session_->open(config_.output_file, mcap_options)) {
    set_error_helper(
      "Failed to open MCAP file: " + config_.output_file + " - " +
      recording_session_->get_last_error()
    );
    state_manager_.transition_to(RecorderState::IDLE, error_msg);
    recording_session_.reset();
    return false;
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
    set_error_helper("Failed to setup subscriptions");
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
    recording_session_.reset();
  }

  // Unload plugins
  plugin_loader_.unload_all();

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

  http_server_ = std::make_unique<HttpServer>(*this, host, port);

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

void AxonRecorder::set_task_config(const TaskConfig& config) {
  task_config_ = config;
}

const TaskConfig* AxonRecorder::get_task_config() const {
  return task_config_.has_value() ? &task_config_.value() : nullptr;
}

RecordingSession* AxonRecorder::get_recording_session() {
  return recording_session_.get();
}

std::string AxonRecorder::get_last_error() const {
  std::lock_guard<std::mutex> lock(error_mutex_);
  return last_error_;
}

AxonRecorder::Statistics AxonRecorder::get_statistics() const {
  Statistics stats;

  // Get aggregate statistics from worker pool
  if (worker_pool_) {
    auto pool_stats = worker_pool_->get_aggregate_stats();
    stats.messages_received = pool_stats.total_received;
    stats.messages_written = pool_stats.total_written;
    stats.messages_dropped = pool_stats.total_dropped;
  }

  // Get statistics from recording session
  if (recording_session_) {
    auto session_stats = recording_session_->get_stats();
    stats.messages_written = session_stats.messages_written;
    stats.bytes_written = session_stats.bytes_written;
  }

  return stats;
}

void AxonRecorder::on_message(
  const char* topic_name, const uint8_t* message_data, size_t message_size,
  const char* message_type, uint64_t timestamp
) {
  // Create MessageItem and push to worker pool
  MessageItem item;
  item.timestamp_ns = static_cast<int64_t>(timestamp);
  item.raw_data.assign(message_data, message_data + message_size);

  // Push to the topic's queue (worker pool handles this)
  if (worker_pool_) {
    worker_pool_->try_push(topic_name, std::move(item));
  }
  // If queue is full or topic not found, message is dropped
  // Statistics are tracked by WorkerThreadPool
}

bool AxonRecorder::message_handler(
  const std::string& topic, int64_t timestamp_ns, const uint8_t* data, size_t data_size,
  uint32_t sequence
) {
  if (!recording_session_) {
    return false;  // No active recording session
  }

  // Get channel ID for this topic from recording session
  uint16_t channel_id = recording_session_->get_channel_id(topic);
  if (channel_id == 0) {
    return false;  // Channel not found
  }

  uint64_t log_time_ns = static_cast<uint64_t>(timestamp_ns);

  // Write message to MCAP via recording session
  return recording_session_->write(
    channel_id, sequence, log_time_ns, timestamp_ns, data, data_size
  );
}

bool AxonRecorder::register_topics() {
  if (!recording_session_) {
    set_error_helper("No active recording session");
    return false;
  }

  // Register schemas and channels for all subscriptions
  for (const auto& sub : config_.subscriptions) {
    // Register schema (use message type as schema name)
    uint16_t schema_id = recording_session_->register_schema(
      sub.message_type, config_.profile == "ros1" ? "ros1msg" : "ros2msg", ""
    );

    if (schema_id == 0) {
      set_error_helper(
        "Failed to register schema for: " + sub.message_type + " - " +
        recording_session_->get_last_error()
      );
      return false;
    }

    // Register channel
    uint16_t channel_id = recording_session_->register_channel(
      sub.topic_name, config_.profile == "ros1" ? "ros1" : "cdr", schema_id
    );

    if (channel_id == 0) {
      set_error_helper(
        "Failed to register channel for: " + sub.topic_name + " - " +
        recording_session_->get_last_error()
      );
      return false;
    }

    // Create topic worker in the pool
    // Use std::bind to bind the message handler member function
    auto handler = std::bind(
      &AxonRecorder::message_handler,
      this,
      std::placeholders::_1,
      std::placeholders::_2,
      std::placeholders::_3,
      std::placeholders::_4,
      std::placeholders::_5
    );

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
  if (!descriptor || !descriptor->vtable || !descriptor->vtable->subscribe) {
    set_error_helper("Plugin does not support subscribe");
    return false;
  }

  // Setup subscriptions via plugin
  for (const auto& sub : config_.subscriptions) {
    AxonStatus status = descriptor->vtable->subscribe(
      sub.topic_name.c_str(), sub.message_type.c_str(), message_callback, this
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

void AxonRecorder::on_state_transition(RecorderState from, RecorderState to) {
  // Handle state transitions
  // This callback is invoked after each successful state transition
  // Can be used for logging, metrics, or triggering side effects
}

}  // namespace recorder
}  // namespace axon
