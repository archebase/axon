#include "recorder_node.hpp"

#include <axon/lance_bridge.h>

#include <arrow/builder.h>   // BinaryBuilder for worker thread conversion
#include <arrow/c/bridge.h>  // Arrow C Data Interface (ExportSchema, ExportRecordBatch)

#include "batch_manager.hpp"
#include "config_parser.hpp"
#include "message_converter.hpp"
#include "message_introspection.hpp"
#include "ros_interface.hpp"
#include "ros_introspection.hpp"
#include "schema_merger.hpp"

#if defined(AXON_ROS1)
#include <ros/package.h>
#include <ros/param.h>
#include <ros/ros.h>
#elif defined(AXON_ROS2)
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#endif

#include <atomic>
#include <chrono>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>
#include <unordered_map>
#include <vector>

namespace axon {
namespace recorder {

// =============================================================================
// Constructor / Destructor
// =============================================================================

RecorderNode::RecorderNode()
    : dataset_handle_(0)
    , recording_(false)
    , shutdown_requested_(false) {}

RecorderNode::~RecorderNode() {
  shutdown();
}

// =============================================================================
// Worker Thread Management
// =============================================================================

void RecorderNode::start_worker_threads() {
  // Start per-topic worker threads
  for (auto& [topic_name, context] : topic_contexts_) {
    if (context->running.load()) {
      continue;
    }

    context->running.store(true);
    context->worker_thread = std::thread(&RecorderNode::worker_thread_func, this, topic_name);

    ros_interface_->log_info("Started worker thread for topic: " + topic_name);
  }
}

void RecorderNode::stop_worker_threads() {
  // Signal all workers to stop
  for (auto& [topic_name, context] : topic_contexts_) {
    context->running.store(false);
  }

  // Wait for all workers to finish
  for (auto& [topic_name, context] : topic_contexts_) {
    if (context->worker_thread.joinable()) {
      context->worker_thread.join();
    }
  }

  std::cerr << "[axon_recorder] All worker threads stopped" << std::endl;
}

/**
 * Per-topic worker thread function
 *
 * This function runs in a dedicated thread for each topic, draining the
 * lock-free queue and writing to the BatchManager.
 *
 * Design rationale:
 * - One thread per topic eliminates contention between topics
 * - Lock-free queue provides wait-free push from ROS callback
 * - Tight spin loop with brief sleeps balances latency and CPU usage
 */
void RecorderNode::worker_thread_func(const std::string& topic_name) {
  auto context_it = topic_contexts_.find(topic_name);
  if (context_it == topic_contexts_.end()) {
    return;
  }

  auto& context = context_it->second;
  auto batch_mgr_it = batch_managers_.find(topic_name);

  if (batch_mgr_it == batch_managers_.end()) {
    std::cerr << "[Worker " << topic_name << "] No batch manager found!" << std::endl;
    return;
  }

  auto& batch_mgr = batch_mgr_it->second;
  MessageItem item;
  size_t consecutive_empty = 0;

  while (context->running.load(std::memory_order_acquire)) {
    // Try to pop from lock-free queue
    if (context->queue->try_pop(item)) {
      consecutive_empty = 0;

      try {
        // Direct binary append with ownership transfer
        batch_mgr->add_row_with_raw_data(item.timestamp_ns, topic_name, std::move(item.raw_data));

        // Update statistics
        context->written.fetch_add(1, std::memory_order_relaxed);
        messages_written_.fetch_add(1, std::memory_order_relaxed);

      } catch (const std::exception& e) {
        std::cerr << "[Worker " << topic_name << "] Exception: " << e.what() << std::endl;
      }
    } else {
      // Queue empty - use adaptive backoff
      ++consecutive_empty;

      if (consecutive_empty > 100) {
        // Long idle - sleep longer to save CPU
        std::this_thread::sleep_for(std::chrono::microseconds(500));
      } else if (consecutive_empty > 10) {
        // Brief idle - short sleep
        std::this_thread::sleep_for(std::chrono::microseconds(WORKER_IDLE_SLEEP_US));
      }
      // else: tight spin for first few empty iterations (low latency)
    }
  }

  // Drain remaining items before exit
  while (context->queue->try_pop(item)) {
    try {
      batch_mgr->add_row_with_raw_data(item.timestamp_ns, topic_name, std::move(item.raw_data));
      context->written.fetch_add(1, std::memory_order_relaxed);
      messages_written_.fetch_add(1, std::memory_order_relaxed);
    } catch (const std::exception& e) {
      std::cerr << "[Worker " << topic_name << "] Drain exception: " << e.what() << std::endl;
    }
  }

  std::cerr << "[Worker " << topic_name << "] Stopped. Written: " << context->written.load()
            << std::endl;
}

bool RecorderNode::initialize(int argc, char** argv) {
  // Create ROS interface (implementation selected at compile time)
  ros_interface_ = RosInterfaceFactory::create();
  if (!ros_interface_) {
    std::cerr << "Failed to create ROS interface" << std::endl;
    return false;
  }

  if (!ros_interface_->init(argc, argv, "axon_recorder")) {
    std::cerr << "Failed to initialize ROS" << std::endl;
    return false;
  }

  // Load configuration
  if (!load_configuration()) {
    ros_interface_->log_error("Failed to load configuration");
    return false;
  }

  // Register ROS introspector factory for message conversion
  // This allows MessageConverterFactory to create converters for any ROS message type
  core::MessageIntrospectorFactory::set_factory([]() {
    return RosIntrospectorFactory::create();
  });

  // Collect schemas from all topics
  if (!collect_topic_schemas()) {
    ros_interface_->log_error("Failed to collect topic schemas");
    return false;
  }

  // Merge schemas and initialize dataset
  if (!initialize_dataset()) {
    ros_interface_->log_error("Failed to initialize dataset");
    return false;
  }

  // Setup batch managers for each topic
  for (const auto& topic_config : config_.topics) {
    setup_topic_recording(topic_config);
  }

  // Setup services
  setup_services();

  return true;
}

void RecorderNode::run() {
  if (config_.recording.auto_start) {
    start_recording();
  }

  std::cerr << "[axon_recorder] Entering spin()..." << std::endl;
  ros_interface_->spin();
  std::cerr << "[axon_recorder] spin() returned" << std::endl;
}

void RecorderNode::shutdown() {
  // Prevent double shutdown
  if (shutdown_requested_.exchange(true)) {
    return;
  }

  std::cerr << "[axon_recorder] shutdown() called" << std::endl;

  // Stop recording (drains queues, flushes batches)
  stop_recording();

  // Unsubscribe all topics FIRST to stop new messages
  std::cerr << "[axon_recorder] Unsubscribing topics..." << std::endl;
  for (auto& pair : subscriptions_) {
    if (ros_interface_) {
      ros_interface_->unsubscribe(pair.second);
    }
  }
  subscriptions_.clear();

  // Clear topic contexts (queues already drained by stop_recording)
  std::cerr << "[axon_recorder] Clearing topic contexts..." << std::endl;
  topic_contexts_.clear();

  // Stop all batch managers and clear BEFORE closing dataset
  // This ensures all pending Arrow operations complete
  std::cerr << "[axon_recorder] Stopping batch managers..." << std::endl;
  for (auto& pair : batch_managers_) {
    pair.second->stop();
  }
  batch_managers_.clear();

  // Clear converters (may hold Arrow schema references)
  converters_.clear();
  topic_schemas_.clear();
  merged_schema_.reset();

  // CRITICAL: Wait for all pending async writes to complete BEFORE closing dataset
  // This ensures Rust's Arrow resources are released while C++ Arrow memory pool is still valid
  std::cerr << "[axon_recorder] Flushing pending Lance writes..." << std::endl;
  lance_flush();
  std::cerr << "[axon_recorder] Lance flush complete" << std::endl;

  // Close dataset AFTER all Arrow operations are done
  if (dataset_handle_ > 0) {
    std::cerr << "[axon_recorder] Closing dataset..." << std::endl;
    close_dataset(dataset_handle_);
    dataset_handle_ = 0;
  }

  // Shutdown ROS interface LAST and release it
  if (ros_interface_) {
    std::cerr << "[axon_recorder] Shutting down ROS interface..." << std::endl;
    ros_interface_->shutdown();
    ros_interface_.reset();  // Explicitly release to avoid static destruction issues
  }

  std::cerr << "[axon_recorder] Shutdown complete" << std::endl;
}

bool RecorderNode::load_configuration() {
  std::string config_path = get_config_path();
  config_ = core::RecorderConfig::from_yaml(config_path);

  if (!config_.validate()) {
    ros_interface_->log_error("Invalid configuration");
    return false;
  }

  ros_interface_->log_info("Configuration loaded: " + config_.to_string());
  return true;
}

std::string RecorderNode::get_config_path() {
#if defined(AXON_ROS1)
  // Get from ROS parameter server
  std::string config_path;
  ros::NodeHandle nh;

  if (nh.getParam("config_path", config_path)) {
    return config_path;
  }

  // Try to get from package path
  std::string package_path = ros::package::getPath("axon_recorder");
  if (!package_path.empty()) {
    return package_path + "/config/default_config.yaml";
  }

  // Fallback to default
  return "config/default_config.yaml";
#elif defined(AXON_ROS2)
  // ROS 2: Try several locations for config file

  // Try workspace source path first (for development)
  std::string source_config = "/workspace/axon/ros/axon_recorder/config/default_config.yaml";
  std::ifstream test_file(source_config);
  if (test_file.good()) {
    return source_config;
  }

  // Try install share directory (for installed package)
  std::string install_config =
    ament_index_cpp::get_package_share_directory("axon_recorder") + "/config/default_config.yaml";
  std::ifstream test_install(install_config);
  if (test_install.good()) {
    return install_config;
  }

  // Fallback to relative path
  return "config/default_config.yaml";
#else
  return "config/default_config.yaml";
#endif
}

bool RecorderNode::collect_topic_schemas() {
  // Create converters for all topics to get their schemas
  for (const auto& topic_config : config_.topics) {
    auto converter = core::MessageConverterFactory::create(topic_config.message_type);
    if (!converter) {
      ros_interface_->log_warn("No converter for message type: " + topic_config.message_type);
      continue;
    }

    converters_[topic_config.name] = std::move(converter);
    topic_schemas_[topic_config.name] = converters_[topic_config.name]->get_schema();
  }

  return !topic_schemas_.empty();
}

bool RecorderNode::initialize_dataset() {
  dataset_path_ = config_.dataset.path;

  // Merge schemas from all topics
  // Use simple recording schema: timestamp, topic, message_data
  // This allows efficient storage of any message type as binary
  merged_schema_ = core::SchemaMerger::create_recording_schema(topic_schemas_);

  if (!merged_schema_) {
    ros_interface_->log_error("Failed to create recording schema");
    return false;
  }

  ros_interface_->log_info(
    "Recording schema has " + std::to_string(merged_schema_->num_fields()) + " fields"
  );

  // Export schema to C interface
  struct ArrowSchema c_schema;
  arrow::Status status = arrow::ExportSchema(*merged_schema_, &c_schema);
  if (!status.ok()) {
    ros_interface_->log_error("Failed to export schema: " + status.ToString());
    return false;
  }

  dataset_handle_ = create_or_open_dataset(dataset_path_.c_str(), &c_schema);

  // Release schema
  c_schema.release(&c_schema);

  if (dataset_handle_ <= 0) {
    const char* error = lance_get_last_error();
    if (error) {
      ros_interface_->log_error("Failed to create dataset: " + std::string(error));
    }
    return false;
  }

  return true;
}

void RecorderNode::setup_topic_recording(const core::TopicConfig& topic_config) {
  // Get converter (already created in collect_topic_schemas)
  auto converter_it = converters_.find(topic_config.name);
  if (converter_it == converters_.end()) {
    ros_interface_->log_warn("No converter for topic: " + topic_config.name);
    return;
  }

  // =========================================================================
  // Step 1: Create BatchManager for this topic
  // =========================================================================
  auto write_callback =
    [this](
      const std::shared_ptr<arrow::RecordBatch>& batch, const std::string& path, int64_t handle
    ) -> bool {
    struct ArrowArray c_array;
    struct ArrowSchema c_schema;
    arrow::Status status = arrow::ExportRecordBatch(*batch, &c_array, &c_schema);

    if (!status.ok()) {
      ros_interface_->log_error("Failed to export batch: " + status.ToString());
      return false;
    }

    int32_t result = write_batch(handle, &c_array, &c_schema);

    // Release schema only - Rust takes ownership of array via Arrow C Data Interface
    if (c_schema.release) {
      c_schema.release(&c_schema);
    }

    if (result != LANCE_SUCCESS) {
      const char* error = lance_get_last_error();
      if (error) {
        ros_interface_->log_error("Write failed: " + std::string(error));
      }
    }

    return result == LANCE_SUCCESS;
  };

  auto memory_pool = arrow::default_memory_pool();

  auto batch_mgr = std::make_unique<core::BatchManager>(
    topic_config.batch_size, topic_config.flush_interval_ms, write_callback, memory_pool
  );

  batch_mgr->initialize_schema(merged_schema_);
  batch_mgr->set_dataset(dataset_path_, dataset_handle_);
  batch_mgr->start();

  batch_managers_[topic_config.name] = std::move(batch_mgr);

  // =========================================================================
  // Step 2: Create lock-free queue and topic context
  // =========================================================================
  auto context = std::make_unique<TopicContext>();
  context->queue = std::make_unique<core::SPSCQueue<MessageItem>>(QUEUE_CAPACITY_PER_TOPIC);

  // Store raw pointer for capture in callback (safe: context outlives subscription)
  auto* context_ptr = context.get();
  topic_contexts_[topic_config.name] = std::move(context);

  // =========================================================================
  // Step 3: Create zero-copy subscription
  // =========================================================================
  // Configure QoS based on message type
  // CRITICAL: Match publisher's QoS depth to prevent DDS reliable protocol drops
  SubscriptionConfig sub_config;

  // Use larger history to match or exceed publisher's buffer (1000)
  // This prevents DDS reliable ACK delays from causing drops
  if (topic_config.message_type.find("Image") != std::string::npos) {
    sub_config = SubscriptionConfig::high_throughput();
    sub_config.history_depth = 1000;  // Match publisher depth
  } else if (topic_config.message_type.find("Imu") != std::string::npos) {
    sub_config = SubscriptionConfig::high_throughput();
    sub_config.history_depth = 5000;  // IMU is high frequency - need more buffer
  } else {
    sub_config = SubscriptionConfig::high_throughput();
    sub_config.history_depth = 1000;  // Match publisher default
  }

  // Zero-copy callback: minimal work, just enqueue
  auto zero_copy_callback =
    [this, topic_name = topic_config.name, context_ptr](SerializedMessageData&& msg_data) {
      // Fast path: check recording state
      if (!recording_.load(std::memory_order_acquire)) {
        return;
      }

      // Update statistics
      context_ptr->received.fetch_add(1, std::memory_order_relaxed);
      messages_received_.fetch_add(1, std::memory_order_relaxed);

      // Create message item with ownership transfer (zero-copy from here)
      MessageItem item(msg_data.receive_time_ns, std::move(msg_data.data));

      // Try to enqueue to lock-free queue
      if (!context_ptr->queue->try_push(std::move(item))) {
        // Queue full - drop message
        context_ptr->dropped.fetch_add(1, std::memory_order_relaxed);
        messages_dropped_.fetch_add(1, std::memory_order_relaxed);
      }
    };

  // Use the new zero-copy subscription API
  void* sub = ros_interface_->subscribe_zero_copy(
    topic_config.name, topic_config.message_type, zero_copy_callback, sub_config
  );

  if (sub) {
    subscriptions_[topic_config.name] = sub;
    ros_interface_->log_info("Zero-copy subscription created for topic: " + topic_config.name);
  } else {
    ros_interface_->log_error("Failed to subscribe to topic: " + topic_config.name);
  }
}

void RecorderNode::setup_services() {
  // Full service implementation would go here
  ros_interface_->log_info("Services available via ROS interface");
}

void RecorderNode::start_recording() {
  if (recording_.load(std::memory_order_acquire)) {
    ros_interface_->log_warn("Recording already started");
    return;
  }

  // Start per-topic worker threads
  start_worker_threads();

  // Enable message acceptance
  recording_.store(true, std::memory_order_release);

  ros_interface_->log_info(
    "Recording started with " + std::to_string(topic_contexts_.size()) + " topics"
  );
}

void RecorderNode::stop_recording() {
  // Use std::cerr for shutdown messages since ROS logging may be disabled after rclcpp::shutdown()
  std::cerr << "[axon_recorder] stop_recording() called" << std::endl;

  // Disable new message acceptance
  recording_.store(false, std::memory_order_release);

  // Brief sleep to allow in-flight callbacks to complete
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Stop worker threads (will drain remaining queue items)
  std::cerr << "[axon_recorder] Stopping worker threads..." << std::endl;
  stop_worker_threads();

  // Flush all batch managers
  std::cerr << "[axon_recorder] Flushing batch managers..." << std::endl;
  for (auto& pair : batch_managers_) {
    pair.second->flush();
  }
  std::cerr << "[axon_recorder] Batch managers flushed" << std::endl;

  // Write statistics file for performance monitoring
  write_stats_file();

  std::cerr << "[axon_recorder] Recording stopped" << std::endl;
  if (ros_interface_) {
    ros_interface_->log_info("Recording stopped");
  }
}

RecorderNode::RecordingStats RecorderNode::get_stats() const {
  RecordingStats stats;
  stats.messages_received = messages_received_.load(std::memory_order_relaxed);
  stats.messages_dropped = messages_dropped_.load(std::memory_order_relaxed);
  stats.messages_written = messages_written_.load(std::memory_order_relaxed);

  if (stats.messages_received > 0) {
    stats.drop_rate_percent = 100.0 * static_cast<double>(stats.messages_dropped) /
                              static_cast<double>(stats.messages_received);
  } else {
    stats.drop_rate_percent = 0.0;
  }

  return stats;
}

void RecorderNode::write_stats_file() {
  std::ofstream stats_file(STATS_FILE_PATH);
  if (!stats_file.is_open()) {
    std::cerr << "[axon_recorder] Could not write stats file: " << STATS_FILE_PATH << std::endl;
    return;
  }

  auto stats = get_stats();

  // Also write per-topic statistics
  stats_file << "{\n"
             << "  \"messages_received\": " << stats.messages_received << ",\n"
             << "  \"messages_dropped\": " << stats.messages_dropped << ",\n"
             << "  \"messages_written\": " << stats.messages_written << ",\n"
             << "  \"drop_rate_percent\": " << std::fixed << std::setprecision(2)
             << stats.drop_rate_percent << ",\n"
             << "  \"per_topic_stats\": {\n";

  bool first = true;
  for (const auto& [topic_name, context] : topic_contexts_) {
    if (!first) stats_file << ",\n";
    first = false;

    uint64_t topic_received = context->received.load();
    uint64_t topic_dropped = context->dropped.load();
    uint64_t topic_written = context->written.load();
    double topic_drop_rate = (topic_received > 0) ? (100.0 * topic_dropped / topic_received) : 0.0;

    stats_file << "    \"" << topic_name << "\": {"
               << "\"received\": " << topic_received << ", "
               << "\"dropped\": " << topic_dropped << ", "
               << "\"written\": " << topic_written << ", "
               << "\"drop_rate\": " << std::fixed << std::setprecision(2) << topic_drop_rate << "}";
  }

  stats_file << "\n  }\n}\n";
  stats_file.close();

  std::cerr << "[axon_recorder] Stats written: received=" << stats.messages_received
            << ", dropped=" << stats.messages_dropped << ", written=" << stats.messages_written
            << ", drop_rate=" << stats.drop_rate_percent << "%" << std::endl;
}

}  // namespace recorder
}  // namespace axon

// ============================================================================
// Main Entry Point
// ============================================================================
// Note: ROS2's rclcpp::init() installs default signal handlers for SIGINT
// and SIGTERM that call rclcpp::shutdown(). We rely on these handlers.
// After spin() returns, we call node.shutdown() to write stats.

// Global flag to track if we received a signal
static std::atomic<bool> g_signal_received{false};

// Signal handler to log signal reception
static void signal_handler(int signum) {
  std::cerr << "[axon_recorder] Received signal " << signum << " (SIGTERM=" << SIGTERM
            << ", SIGINT=" << SIGINT << ")" << std::endl;
  g_signal_received.store(true);

  // Call rclcpp::shutdown() to trigger executor exit
#if defined(AXON_ROS2)
  std::cerr << "[axon_recorder] Calling rclcpp::shutdown()..." << std::endl;
  rclcpp::shutdown();
  std::cerr << "[axon_recorder] rclcpp::shutdown() called" << std::endl;
#elif defined(AXON_ROS1)
  ros::shutdown();
#endif
}

int main(int argc, char** argv) {
  std::cerr << "[axon_recorder] main() starting" << std::endl;

  // Use a block scope to ensure RecorderNode is fully destroyed before main() returns
  // This prevents static destruction order issues with Arrow's memory pool
  {
    axon::recorder::RecorderNode node;

    if (!node.initialize(argc, argv)) {
      std::cerr << "[axon_recorder] Initialization failed" << std::endl;
      return 1;
    }

    // Install custom signal handlers AFTER rclcpp::init() to override ROS defaults
    std::cerr << "[axon_recorder] Installing signal handlers..." << std::endl;
    std::signal(SIGTERM, signal_handler);
    std::signal(SIGINT, signal_handler);

    std::cerr << "[axon_recorder] Initialization complete, calling run()" << std::endl;

    // run() blocks until rclcpp::shutdown() is called (by signal or otherwise)
    node.run();

    std::cerr << "[axon_recorder] run() returned, calling shutdown()" << std::endl;

    // Ensure shutdown is called - this writes the stats file
    // This runs AFTER spin() returns due to SIGTERM/SIGINT
    node.shutdown();

    std::cerr << "[axon_recorder] Node destroyed" << std::endl;
  }  // RecorderNode destructor runs here, before main() exits

  // Final safety flush - ensure ALL Rust async tasks complete before C++ exits
  // This is critical to prevent Arrow memory pool access after destruction
  std::cerr << "[axon_recorder] Final Lance flush before exit..." << std::endl;
  lance_flush();

  // Brief pause to allow any background cleanup to complete
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  std::cerr << "[axon_recorder] main() exiting normally" << std::endl;
  return 0;
}
