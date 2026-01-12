#include "recorder.hpp"

#include <chrono>
#include <cstring>
#include <ctime>
#include <functional>
#include <iostream>
#include <mutex>
#include <thread>

#include "mcap_writer_wrapper.hpp"
#include "spsc_queue.hpp"

namespace axon {
namespace recorder {

namespace {
// Global callback context (simplified for example)
struct CallbackContext {
  AxonRecorder* recorder;
  const std::string* topic_name;
  const std::string* message_type;
};

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
    , should_stop_(false)
    , messages_received_(0)
    , messages_written_(0)
    , messages_dropped_(0) {}

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

  // Register topics with MCAP
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

  // Start worker thread
  should_stop_.store(false);
  worker_thread_ = std::thread(&AxonRecorder::worker_thread, this);
  running_.store(true);

  return true;
}

void AxonRecorder::stop() {
  if (!running_.load()) {
    return;
  }

  // Signal worker thread to stop
  should_stop_.store(true);

  // Stop plugin
  auto plugins = plugin_loader_.loaded_plugins();
  for (const auto& plugin_name : plugins) {
    const auto* descriptor = plugin_loader_.get_descriptor(plugin_name);
    if (descriptor && descriptor->vtable && descriptor->vtable->stop) {
      descriptor->vtable->stop();
    }
  }

  // Wait for worker thread to finish
  if (worker_thread_.joinable()) {
    worker_thread_.join();
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
  stats.messages_received = messages_received_.load();
  stats.messages_written = messages_written_.load();
  stats.messages_dropped = messages_dropped_.load();
  stats.bytes_written = mcap_writer_ ? mcap_writer_->get_statistics().bytes_written : 0;

  // Calculate total queue size across all topics
  size_t total_size = 0;
  {
    std::lock_guard<std::mutex> lock(topic_queues_mutex_);
    for (const auto& [topic, queue] : topic_queues_) {
      total_size += queue->size();
    }
  }
  stats.queue_size = total_size;

  return stats;
}

void AxonRecorder::on_message(
  const char* topic_name, const uint8_t* message_data, size_t message_size,
  const char* message_type, uint64_t timestamp
) {
  messages_received_.fetch_add(1);

  // Find the queue for this topic
  std::string topic_key(topic_name);
  SPSCQueue<QueuedMessage>* queue = nullptr;

  {
    std::lock_guard<std::mutex> lock(topic_queues_mutex_);
    auto it = topic_queues_.find(topic_key);
    if (it != topic_queues_.end()) {
      queue = it->second.get();
    }
  }

  if (!queue) {
    // Queue not found for this topic
    messages_dropped_.fetch_add(1);
    return;
  }

  // Create queued message
  QueuedMessage msg(topic_name, message_data, message_size, message_type, timestamp);

  // Push to topic's queue (drop if full)
  if (!queue->try_push(std::move(msg))) {
    messages_dropped_.fetch_add(1);
  }
}

void AxonRecorder::worker_thread() {
  // Round-robin through all topic queues
  std::vector<std::string> topic_names;

  while (!should_stop_.load()) {
    // Get current topic list
    {
      std::lock_guard<std::mutex> lock(topic_queues_mutex_);
      topic_names.clear();
      topic_names.reserve(topic_queues_.size());
      for (const auto& [topic, queue] : topic_queues_) {
        topic_names.push_back(topic);
      }
    }

    bool processed = false;
    auto now = std::chrono::steady_clock::now();

    // Round-robin through all topic queues
    for (const auto& topic_name : topic_names) {
      SPSCQueue<QueuedMessage>* queue = nullptr;

      {
        std::lock_guard<std::mutex> lock(topic_queues_mutex_);
        auto it = topic_queues_.find(topic_name);
        if (it != topic_queues_.end()) {
          queue = it->second.get();
        }
      }

      if (!queue) {
        continue;
      }

      // Try to pop message from this topic's queue
      QueuedMessage msg;
      if (queue->try_pop(msg)) {
        // Add to batch
        std::lock_guard<std::mutex> lock(topic_batches_mutex_);
        auto& batch = topic_batches_[msg.topic_name];
        batch.messages.push_back(std::move(msg));
        batch.last_flush = now;
        processed = true;

        // Get subscription config for this topic
        const SubscriptionConfig* sub_config = get_subscription_config(msg.topic_name);
        size_t batch_size = sub_config ? sub_config->batch_size : 1;
        int flush_interval_ms = sub_config ? sub_config->flush_interval_ms : 100;

        // Check if we should flush this batch
        bool should_flush = false;

        // Flush if batch size reached
        if (batch.messages.size() >= batch_size) {
          should_flush = true;
        }

        // Also check flush interval for all batches
        if (!should_flush) {
          auto elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - batch.last_flush).count();
          if (elapsed >= flush_interval_ms) {
            should_flush = true;
          }
        }

        if (should_flush) {
          // Write all messages in batch to MCAP
          for (const auto& batch_msg : batch.messages) {
            auto it = channel_ids_.find(batch_msg.topic_name);
            if (it != channel_ids_.end()) {
              uint16_t channel_id = it->second;
              uint64_t log_time_ns = batch_msg.timestamp_ns;

              if (mcap_writer_->write(
                    channel_id,
                    log_time_ns,
                    batch_msg.timestamp_ns,
                    batch_msg.data.data(),
                    batch_msg.data.size()
                  )) {
                messages_written_.fetch_add(1);
              } else {
                messages_dropped_.fetch_add(1);
              }
            } else {
              messages_dropped_.fetch_add(1);
            }
          }
          batch.messages.clear();
          batch.last_flush = now;
        }
      }
    }

    // Check for timed out batches on all topics
    {
      std::lock_guard<std::mutex> lock(topic_batches_mutex_);
      for (auto& [topic_name, batch] : topic_batches_) {
        if (batch.messages.empty()) {
          continue;
        }

        const SubscriptionConfig* sub_config = get_subscription_config(topic_name);
        int flush_interval_ms = sub_config ? sub_config->flush_interval_ms : 100;
        auto elapsed =
          std::chrono::duration_cast<std::chrono::milliseconds>(now - batch.last_flush).count();

        if (elapsed >= flush_interval_ms) {
          // Flush this batch
          for (const auto& batch_msg : batch.messages) {
            auto it = channel_ids_.find(batch_msg.topic_name);
            if (it != channel_ids_.end()) {
              uint16_t channel_id = it->second;
              uint64_t log_time_ns = batch_msg.timestamp_ns;

              if (mcap_writer_->write(
                    channel_id,
                    log_time_ns,
                    batch_msg.timestamp_ns,
                    batch_msg.data.data(),
                    batch_msg.data.size()
                  )) {
                messages_written_.fetch_add(1);
              } else {
                messages_dropped_.fetch_add(1);
              }
            } else {
              messages_dropped_.fetch_add(1);
            }
          }
          batch.messages.clear();
          batch.last_flush = now;
        }
      }
    }

    // If no messages were processed, sleep a bit
    if (!processed) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  // Flush all remaining batches before exiting
  {
    std::lock_guard<std::mutex> lock(topic_batches_mutex_);
    for (auto& [topic_name, batch] : topic_batches_) {
      for (const auto& batch_msg : batch.messages) {
        auto it = channel_ids_.find(batch_msg.topic_name);
        if (it != channel_ids_.end()) {
          uint16_t channel_id = it->second;
          uint64_t log_time_ns = batch_msg.timestamp_ns;

          if (mcap_writer_->write(
                channel_id,
                log_time_ns,
                batch_msg.timestamp_ns,
                batch_msg.data.data(),
                batch_msg.data.size()
              )) {
            messages_written_.fetch_add(1);
          }
        }
      }
      batch.messages.clear();
    }
  }

  // Flush MCAP before exiting
  if (mcap_writer_) {
    mcap_writer_->flush();
  }
}

bool AxonRecorder::register_topics() {
  // For simplicity, we register schemas with empty definitions
  // In a real implementation, you would fetch message definitions from ROS

  for (const auto& sub : config_.subscriptions) {
    // Create SPSC queue for this topic
    try {
      auto queue = std::make_unique<SPSCQueue<QueuedMessage>>(config_.queue_capacity);
      std::lock_guard<std::mutex> lock(topic_queues_mutex_);
      topic_queues_[sub.topic_name] = std::move(queue);
    } catch (const std::exception& e) {
      set_error_helper("Failed to create queue for topic " + sub.topic_name + ": " + e.what());
      return false;
    }

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
