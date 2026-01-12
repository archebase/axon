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
    : running_(false) {}

AxonRecorder::~AxonRecorder() {
  stop();
}

bool AxonRecorder::initialize(const RecorderConfig& config) {
  config_ = config;

  // Create MCAP writer
  mcap_writer_ = std::make_unique<mcap_wrapper::McapWriterWrapper>();

  // Configure MCAP writer
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

  // Open MCAP file
  if (!mcap_writer_->open(config_.output_file, mcap_options)) {
    set_error_helper(
      std::string("Failed to open MCAP file: ") + config_.output_file + " - " +
      mcap_writer_->get_last_error()
    );
    return false;
  }

  // Configure and create worker thread pool
  WorkerThreadPool::Config pool_config;
  pool_config.queue_capacity_per_topic = config_.queue_capacity;
  pool_config.worker_idle_sleep_us = 50;
  pool_config.use_adaptive_backoff = true;

  worker_pool_ = std::make_unique<WorkerThreadPool>(pool_config);

  return true;
}

bool AxonRecorder::start() {
  if (running_.load()) {
    set_error_helper("Recorder already running");
    return false;
  }

  // Load plugin
  auto plugin_name = plugin_loader_.load(config_.plugin_path);
  if (!plugin_name.has_value()) {
    set_error_helper("Failed to load plugin: " + plugin_loader_.get_last_error());
    return false;
  }

  // Get plugin descriptor
  const auto* descriptor = plugin_loader_.get_descriptor(plugin_name.value());
  if (!descriptor || !descriptor->vtable) {
    set_error_helper("Invalid plugin descriptor");
    return false;
  }

  // Initialize plugin
  if (descriptor->vtable->init) {
    AxonStatus status = descriptor->vtable->init("");
    if (status != AXON_SUCCESS) {
      set_error_helper(std::string("Plugin init failed: ") + status_to_string(status));
      return false;
    }
  }

  // Register topics with MCAP and create topic workers
  if (!register_topics()) {
    set_error_helper("Failed to register topics");
    return false;
  }

  // Setup subscriptions via plugin
  if (!setup_subscriptions()) {
    set_error_helper("Failed to setup subscriptions");
    return false;
  }

  // Start plugin
  if (descriptor->vtable->start) {
    AxonStatus status = descriptor->vtable->start();
    if (status != AXON_SUCCESS) {
      set_error_helper(std::string("Plugin start failed: ") + status_to_string(status));
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

  // Close MCAP writer
  if (mcap_writer_) {
    mcap_writer_->close();
  }

  // Unload plugins
  plugin_loader_.unload_all();

  running_.store(false);
}

bool AxonRecorder::is_running() const {
  return running_.load();
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
  stats.bytes_written = mcap_writer_ ? mcap_writer_->get_statistics().bytes_written : 0;

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
  // Find channel ID for this topic
  auto it = channel_ids_.find(topic);
  if (it == channel_ids_.end()) {
    return false;  // Channel not found
  }

  uint16_t channel_id = it->second;
  uint64_t log_time_ns = static_cast<uint64_t>(timestamp_ns);

  // Write message to MCAP
  if (mcap_writer_->write(channel_id, log_time_ns, timestamp_ns, data, data_size)) {
    return true;
  }

  return false;
}

bool AxonRecorder::register_topics() {
  // Register schemas and channels for all subscriptions
  for (const auto& sub : config_.subscriptions) {
    // Register schema (use message type as schema name)
    uint16_t schema_id = mcap_writer_->register_schema(
      sub.message_type, config_.profile == "ros1" ? "ros1msg" : "ros2msg", ""
    );

    if (schema_id == 0) {
      set_error_helper(
        "Failed to register schema for: " + sub.message_type + " - " +
        mcap_writer_->get_last_error()
      );
      return false;
    }

    // Register channel
    uint16_t channel_id = mcap_writer_->register_channel(
      sub.topic_name, config_.profile == "ros1" ? "ros1" : "cdr", schema_id
    );

    if (channel_id == 0) {
      set_error_helper(
        "Failed to register channel for: " + sub.topic_name + " - " + mcap_writer_->get_last_error()
      );
      return false;
    }

    // Store channel ID
    channel_ids_[sub.topic_name] = channel_id;

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

}  // namespace recorder
}  // namespace axon
