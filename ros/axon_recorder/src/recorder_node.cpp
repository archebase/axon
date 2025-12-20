#include "recorder_node.hpp"

#include "config_parser.hpp"
#include "http_callback_client.hpp"
#include "mcap_writer_wrapper.hpp"
#include "ros_interface.hpp"
#include "service_adapter.hpp"
#include "state_machine.hpp"
#include "task_config.hpp"

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
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <thread>
#include <unordered_map>
#include <vector>

namespace axon {
namespace recorder {

// =============================================================================
// Constructor / Destructor
// =============================================================================

RecorderNode::RecorderNode()
    : recording_(false)
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
 * lock-free queue and writing directly to MCAP.
 *
 * Design rationale:
 * - One thread per topic eliminates contention between topics
 * - Lock-free queue provides wait-free push from ROS callback
 * - Tight spin loop with brief sleeps balances latency and CPU usage
 * - Direct MCAP write (no Arrow conversion) reduces CPU overhead
 */
void RecorderNode::worker_thread_func(const std::string& topic_name) {
  auto context_it = topic_contexts_.find(topic_name);
  if (context_it == topic_contexts_.end()) {
    return;
  }

  auto& context = context_it->second;

  // Get channel ID for this topic
  auto channel_it = topic_channel_ids_.find(topic_name);
  if (channel_it == topic_channel_ids_.end()) {
    std::cerr << "[Worker " << topic_name << "] No channel ID found!" << std::endl;
    return;
  }

  uint16_t channel_id = channel_it->second;
  MessageItem item;
  size_t consecutive_empty = 0;

  while (context->running.load(std::memory_order_acquire)) {
    // Try to pop from lock-free queue
    if (context->queue->try_pop(item)) {
      consecutive_empty = 0;

      try {
        // Get sequence number for this message
        uint32_t seq = context->sequence.fetch_add(1, std::memory_order_relaxed);

        // Write directly to MCAP (no Arrow conversion!)
        bool success = mcap_writer_->write(
          channel_id, seq, static_cast<uint64_t>(item.timestamp_ns),
          static_cast<uint64_t>(item.timestamp_ns),  // publish_time = log_time
          item.raw_data.data(), item.raw_data.size()
        );

        if (success) {
          // Update statistics
          context->written.fetch_add(1, std::memory_order_relaxed);
          messages_written_.fetch_add(1, std::memory_order_relaxed);
        } else {
          std::cerr << "[Worker " << topic_name
                    << "] Write failed: " << mcap_writer_->get_last_error() << std::endl;
        }

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
      uint32_t seq = context->sequence.fetch_add(1, std::memory_order_relaxed);
      bool success = mcap_writer_->write(
        channel_id, seq, static_cast<uint64_t>(item.timestamp_ns),
        static_cast<uint64_t>(item.timestamp_ns), item.raw_data.data(), item.raw_data.size()
      );

      if (success) {
        context->written.fetch_add(1, std::memory_order_relaxed);
        messages_written_.fetch_add(1, std::memory_order_relaxed);
      }
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

  // Initialize MCAP writer
  if (!initialize_mcap_writer()) {
    ros_interface_->log_error("Failed to initialize MCAP writer");
    return false;
  }

  // Register schemas for all message types
  if (!register_topic_schemas()) {
    ros_interface_->log_error("Failed to register topic schemas");
    return false;
  }

  // Setup subscriptions for each topic
  for (const auto& topic_config : config_.topics) {
    setup_topic_recording(topic_config);
  }

  // Setup services
  setup_services();

  return true;
}

void RecorderNode::run() {
  // Recording is started via service calls, not automatically
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

  // Stop recording (drains queues)
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

  // Close MCAP writer (writes footer and finalizes file)
  if (mcap_writer_ && mcap_writer_->is_open()) {
    std::cerr << "[axon_recorder] Closing MCAP writer..." << std::endl;
    auto stats = mcap_writer_->get_statistics();
    std::cerr << "[axon_recorder] MCAP stats: " << stats.messages_written << " messages, "
              << stats.bytes_written << " bytes" << std::endl;
    mcap_writer_->close();
  }
  mcap_writer_.reset();

  // Shutdown service adapter before ROS interface
  if (service_adapter_) {
    std::cerr << "[axon_recorder] Shutting down service adapter..." << std::endl;
    service_adapter_->shutdown();
    service_adapter_.reset();
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

std::string RecorderNode::generate_output_path() const {
  // Generate timestamped filename
  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);
  std::tm tm = *std::localtime(&time_t);

  std::ostringstream oss;
  oss << config_.dataset.path;

  // Ensure path ends with /
  if (!config_.dataset.path.empty() && config_.dataset.path.back() != '/') {
    oss << "/";
  }

  oss << "recording_" << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".mcap";

  return oss.str();
}

bool RecorderNode::initialize_mcap_writer() {
  mcap_writer_ = std::make_unique<mcap_wrapper::McapWriterWrapper>();

  output_path_ = generate_output_path();

  // Configure MCAP options
  mcap_wrapper::McapWriterOptions options;

#if defined(AXON_ROS1)
  options.profile = "ros1";
#else
  options.profile = "ros2";
#endif

  options.compression = mcap_wrapper::Compression::Zstd;
  options.compression_level = 3;   // Fast compression
  options.chunk_size = 4 * 1024 * 1024;  // 4MB chunks

  if (!mcap_writer_->open(output_path_, options)) {
    ros_interface_->log_error("Failed to open MCAP file: " + mcap_writer_->get_last_error());
    return false;
  }

  ros_interface_->log_info("MCAP file opened: " + output_path_);
  return true;
}

std::string RecorderNode::get_schema_encoding() {
#if defined(AXON_ROS1)
  return "ros1msg";
#else
  return "ros2msg";
#endif
}

std::string RecorderNode::get_message_encoding() {
#if defined(AXON_ROS1)
  return "ros1";
#else
  return "cdr";
#endif
}

std::string RecorderNode::get_message_definition(const std::string& message_type) {
  // Get message definition from ROS interface
  // This will be implemented in schema-extraction task
  return ros_interface_->get_message_definition(message_type);
}

bool RecorderNode::register_topic_schemas() {
  std::string schema_encoding = get_schema_encoding();
  std::string message_encoding = get_message_encoding();

  // Register schemas for all unique message types
  for (const auto& topic_config : config_.topics) {
    const std::string& msg_type = topic_config.message_type;

    // Skip if already registered
    if (message_type_schema_ids_.find(msg_type) != message_type_schema_ids_.end()) {
      continue;
    }

    // Get message definition
    std::string definition = get_message_definition(msg_type);

    // Register schema
    uint16_t schema_id = mcap_writer_->register_schema(msg_type, schema_encoding, definition);

    if (schema_id == 0) {
      ros_interface_->log_warn("Failed to register schema for: " + msg_type);
      // Continue anyway - MCAP can work without schema
    }

    message_type_schema_ids_[msg_type] = schema_id;
    ros_interface_->log_info(
      "Registered schema: " + msg_type + " (id=" + std::to_string(schema_id) + ")"
    );
  }

  // Register channels for all topics
  for (const auto& topic_config : config_.topics) {
    uint16_t schema_id = message_type_schema_ids_[topic_config.message_type];

    // Build metadata (QoS, frame_id, etc.)
    std::unordered_map<std::string, std::string> metadata;
    // Add QoS profile metadata for ROS 2 compatibility
    metadata["offered_qos_profiles"] = "- history: keep_last\n  depth: 10\n  reliability: reliable";

    uint16_t channel_id =
      mcap_writer_->register_channel(topic_config.name, message_encoding, schema_id, metadata);

    if (channel_id == 0) {
      ros_interface_->log_error("Failed to register channel for: " + topic_config.name);
      return false;
    }

    topic_channel_ids_[topic_config.name] = channel_id;
    ros_interface_->log_info(
      "Registered channel: " + topic_config.name + " (id=" + std::to_string(channel_id) + ")"
    );
  }

  return true;
}

void RecorderNode::setup_topic_recording(const core::TopicConfig& topic_config) {
  // =========================================================================
  // Step 1: Create lock-free queue and topic context
  // =========================================================================
  auto context = std::make_unique<TopicContext>();
  context->queue = std::make_unique<core::SPSCQueue<MessageItem>>(QUEUE_CAPACITY_PER_TOPIC);

  // Store raw pointer for capture in callback (safe: context outlives subscription)
  auto* context_ptr = context.get();
  topic_contexts_[topic_config.name] = std::move(context);

  // =========================================================================
  // Step 2: Create zero-copy subscription
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

      // Check if paused - drop messages while paused
      if (paused_.load(std::memory_order_acquire)) {
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
  // Create and register service adapter
  service_adapter_ = std::make_unique<ServiceAdapter>(this, ros_interface_.get());

  if (service_adapter_->register_services()) {
    ros_interface_->log_info("Recording services registered successfully");
  } else {
    ros_interface_->log_error("Failed to register recording services");
  }
}

void RecorderNode::start_recording() {
  if (recording_.load(std::memory_order_acquire)) {
    ros_interface_->log_warn("Recording already started");
    return;
  }

  // Start per-topic worker threads
  start_worker_threads();

  // Record start time for duration calculation
  recording_start_time_ = std::chrono::system_clock::now();

  // Enable message acceptance
  recording_.store(true, std::memory_order_release);
  paused_.store(false, std::memory_order_release);

  ros_interface_->log_info(
    "Recording started with " + std::to_string(topic_contexts_.size()) + " topics to " +
    output_path_
  );

  // Send start callback if configured
  auto config_opt = task_config_cache_.get();
  if (config_opt && config_opt->has_callbacks()) {
    StartCallbackPayload payload;
    payload.task_id = config_opt->task_id;
    payload.device_id = config_opt->device_id;
    payload.status = "recording";
    payload.started_at = HttpCallbackClient::get_iso8601_timestamp(recording_start_time_);
    payload.topics = config_opt->topics;

    http_callback_client_.post_start_callback_async(*config_opt, payload);
  }
}

void RecorderNode::pause_recording() {
  if (!recording_.load(std::memory_order_acquire)) {
    ros_interface_->log_warn("Cannot pause: not recording");
    return;
  }

  if (paused_.load(std::memory_order_acquire)) {
    ros_interface_->log_warn("Recording already paused");
    return;
  }

  paused_.store(true, std::memory_order_release);
  ros_interface_->log_info("Recording paused");
}

void RecorderNode::resume_recording() {
  if (!recording_.load(std::memory_order_acquire)) {
    ros_interface_->log_warn("Cannot resume: not recording");
    return;
  }

  if (!paused_.load(std::memory_order_acquire)) {
    ros_interface_->log_warn("Recording not paused");
    return;
  }

  paused_.store(false, std::memory_order_release);
  ros_interface_->log_info("Recording resumed");
}

void RecorderNode::cancel_recording() {
  std::cerr << "[axon_recorder] cancel_recording() called" << std::endl;

  // Stop accepting new messages
  recording_.store(false, std::memory_order_release);
  paused_.store(false, std::memory_order_release);

  // Brief sleep to allow in-flight callbacks to complete
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Stop worker threads (will drain remaining queue items)
  stop_worker_threads();

  // Send finish callback with cancelled status if configured
  auto config_opt = task_config_cache_.get();
  if (config_opt && config_opt->has_callbacks()) {
    auto now = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - recording_start_time_);

    FinishCallbackPayload payload;
    payload.task_id = config_opt->task_id;
    payload.device_id = config_opt->device_id;
    payload.status = "cancelled";
    payload.started_at = HttpCallbackClient::get_iso8601_timestamp(recording_start_time_);
    payload.finished_at = HttpCallbackClient::get_iso8601_timestamp(now);
    payload.duration_sec = duration.count() / 1000.0;
    payload.message_count = messages_written_.load();
    payload.file_size_bytes = 0;  // Not finalized
    payload.output_path = output_path_;
    payload.topics = config_opt->topics;
    payload.error = "Recording cancelled";

    http_callback_client_.post_finish_callback_async(*config_opt, payload);
  }

  // Clear the task config
  task_config_cache_.clear();

  std::cerr << "[axon_recorder] Recording cancelled" << std::endl;
}

void RecorderNode::stop_recording() {
  // Use std::cerr for shutdown messages since ROS logging may be disabled after rclcpp::shutdown()
  std::cerr << "[axon_recorder] stop_recording() called" << std::endl;

  // Disable new message acceptance
  recording_.store(false, std::memory_order_release);
  paused_.store(false, std::memory_order_release);

  // Brief sleep to allow in-flight callbacks to complete
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Stop worker threads (will drain remaining queue items)
  std::cerr << "[axon_recorder] Stopping worker threads..." << std::endl;
  stop_worker_threads();

  // Flush MCAP writer to ensure all data is written
  if (mcap_writer_ && mcap_writer_->is_open()) {
    std::cerr << "[axon_recorder] Flushing MCAP writer..." << std::endl;
    mcap_writer_->flush();
  }
  std::cerr << "[axon_recorder] MCAP writer flushed" << std::endl;

  // Send finish callback if configured
  auto config_opt = task_config_cache_.get();
  if (config_opt && config_opt->has_callbacks()) {
    auto now = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - recording_start_time_);
    
    auto mcap_stats = mcap_writer_ ? mcap_writer_->get_statistics()
                                   : mcap_wrapper::McapWriterWrapper::Statistics{};

    FinishCallbackPayload payload;
    payload.task_id = config_opt->task_id;
    payload.device_id = config_opt->device_id;
    payload.status = "finished";
    payload.started_at = HttpCallbackClient::get_iso8601_timestamp(recording_start_time_);
    payload.finished_at = HttpCallbackClient::get_iso8601_timestamp(now);
    payload.duration_sec = duration.count() / 1000.0;
    payload.message_count = messages_written_.load();
    payload.file_size_bytes = mcap_stats.bytes_written;
    payload.output_path = output_path_;
    payload.topics = config_opt->topics;
    payload.error = "";

    http_callback_client_.post_finish_callback_async(*config_opt, payload);
  }

  // Clear the task config
  task_config_cache_.clear();

  // Write statistics file for performance monitoring
  write_stats_file();

  std::cerr << "[axon_recorder] Recording stopped" << std::endl;
  if (ros_interface_) {
    ros_interface_->log_info("Recording stopped");
  }
}

bool RecorderNode::configure_from_task_config(const TaskConfig& config) {
  // This method reconfigures the recorder based on task config
  // It's called when starting recording with cached config

  // If topics are specified in the config, use them instead of the default config
  if (!config.topics.empty()) {
    // For now, we validate that the topics exist in the default config
    // Future enhancement: dynamically subscribe to new topics
    ros_interface_->log_info(
      "Task config specifies " + std::to_string(config.topics.size()) + " topics"
    );
  }

  // Update output path to use task_id
  if (!config.task_id.empty()) {
    // Generate new output path with task_id
    std::string base_path = config_.dataset.path;
    if (!base_path.empty() && base_path.back() != '/') {
      base_path += "/";
    }
    output_path_ = base_path + config.generate_output_filename();
    ros_interface_->log_info("Output path set to: " + output_path_);
  }

  return true;
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
  auto mcap_stats = mcap_writer_ ? mcap_writer_->get_statistics()
                                 : mcap_wrapper::McapWriterWrapper::Statistics{};

  // Also write per-topic statistics
  stats_file << "{\n"
             << "  \"output_file\": \"" << output_path_ << "\",\n"
             << "  \"format\": \"mcap\",\n"
             << "  \"messages_received\": " << stats.messages_received << ",\n"
             << "  \"messages_dropped\": " << stats.messages_dropped << ",\n"
             << "  \"messages_written\": " << stats.messages_written << ",\n"
             << "  \"bytes_written\": " << mcap_stats.bytes_written << ",\n"
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
