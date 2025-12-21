#include "recorder_node.hpp"

#include "config_parser.hpp"
#include "http_callback_client.hpp"
#include "recording_session.hpp"
#include "ros_interface.hpp"
#include "service_adapter.hpp"
#include "state_machine.hpp"
#include "task_config.hpp"
#include "topic_manager.hpp"
#include "worker_thread_pool.hpp"

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
// Factory Method
// =============================================================================

std::shared_ptr<RecorderNode> RecorderNode::create() {
  // Can't use make_shared with private constructor
  return std::shared_ptr<RecorderNode>(new RecorderNode());
}

// =============================================================================
// Constructor / Destructor
// =============================================================================

RecorderNode::RecorderNode()
    : recording_session_(std::make_unique<RecordingSession>())
    , worker_pool_(std::make_unique<WorkerThreadPool>())
    , http_callback_client_(std::make_shared<HttpCallbackClient>())
    , shutdown_requested_(false) {}

RecorderNode::~RecorderNode() {
  shutdown();
}

// =============================================================================
// Worker Thread Pool Setup
// =============================================================================

void RecorderNode::setup_worker_pool() {
  // Worker pool is already initialized in constructor
  // This method sets up message handlers for each topic
  ros_interface_->log_info("Worker pool configured with " + 
                           std::to_string(worker_pool_->topic_count()) + " topics");
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

  // Create topic manager (requires ros_interface_)
  topic_manager_ = std::make_unique<TopicManager>(ros_interface_.get());

  // Load configuration
  if (!load_configuration()) {
    ros_interface_->log_error("Failed to load configuration");
    return false;
  }

  // Initialize recording session (opens MCAP file)
  if (!initialize_mcap_writer()) {
    ros_interface_->log_error("Failed to initialize recording session");
    return false;
  }

  // Register schemas for all message types
  if (!register_topic_schemas()) {
    ros_interface_->log_error("Failed to register topic schemas");
    return false;
  }

  // Setup subscriptions and worker threads for each topic
  for (const auto& topic_config : config_.topics) {
    setup_topic_recording(topic_config);
  }

  // Setup worker pool
  setup_worker_pool();

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

  // Stop recording (drains queues, injects metadata, closes session)
  stop_recording();

  // Unsubscribe all topics FIRST to stop new messages
  std::cerr << "[axon_recorder] Unsubscribing topics..." << std::endl;
  if (topic_manager_) {
    topic_manager_->unsubscribe_all();
  }

  // Stop and clear worker pool (queues already drained by stop_recording)
  std::cerr << "[axon_recorder] Clearing worker pool..." << std::endl;
  if (worker_pool_) {
    worker_pool_->stop();
  }

  // Recording session already closed by stop_recording(), just reset
  if (recording_session_) {
    auto stats = recording_session_->get_stats();
    std::cerr << "[axon_recorder] Final recording stats: " << stats.messages_written << " messages"
              << std::endl;
  }
  recording_session_.reset();

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

  if (!recording_session_->open(output_path_, options)) {
    ros_interface_->log_error("Failed to open recording session: " + recording_session_->get_last_error());
    return false;
  }

  ros_interface_->log_info("Recording session opened: " + output_path_);
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
  std::unordered_map<std::string, uint16_t> registered_schemas;
  for (const auto& topic_config : config_.topics) {
    const std::string& msg_type = topic_config.message_type;

    // Skip if already registered
    if (registered_schemas.find(msg_type) != registered_schemas.end()) {
      continue;
    }

    // Get message definition
    std::string definition = get_message_definition(msg_type);

    // Register schema with recording session
    uint16_t schema_id = recording_session_->register_schema(msg_type, schema_encoding, definition);

    if (schema_id == 0) {
      ros_interface_->log_warn("Failed to register schema for: " + msg_type);
      // Continue anyway - MCAP can work without schema
    }

    registered_schemas[msg_type] = schema_id;
    ros_interface_->log_info(
      "Registered schema: " + msg_type + " (id=" + std::to_string(schema_id) + ")"
    );
  }

  // Register channels for all topics
  for (const auto& topic_config : config_.topics) {
    uint16_t schema_id = registered_schemas[topic_config.message_type];

    // Build metadata (QoS, frame_id, etc.)
    std::unordered_map<std::string, std::string> metadata;
    // Add QoS profile metadata for ROS 2 compatibility
    metadata["offered_qos_profiles"] = "- history: keep_last\n  depth: 10\n  reliability: reliable";

    uint16_t channel_id =
      recording_session_->register_channel(topic_config.name, message_encoding, schema_id, metadata);

    if (channel_id == 0) {
      ros_interface_->log_error("Failed to register channel for: " + topic_config.name);
      return false;
    }

    ros_interface_->log_info(
      "Registered channel: " + topic_config.name + " (id=" + std::to_string(channel_id) + ")"
    );
  }

  return true;
}

void RecorderNode::setup_topic_recording(const core::TopicConfig& topic_config) {
  const std::string& topic_name = topic_config.name;

  // =========================================================================
  // Step 1: Get channel ID for this topic from recording session
  // =========================================================================
  uint16_t channel_id = recording_session_->get_channel_id(topic_name);
  if (channel_id == 0) {
    ros_interface_->log_error("No channel ID found for topic: " + topic_name);
    return;
  }

  // =========================================================================
  // Step 2: Create worker thread with message handler
  // =========================================================================
  // Capture recording_session_ pointer for use in handler
  RecordingSession* session_ptr = recording_session_.get();
  const std::string& msg_type = topic_config.message_type;

  WorkerThreadPool::MessageHandler handler = 
    [session_ptr, channel_id, topic_name, msg_type](const std::string& topic, int64_t timestamp_ns,
                              const uint8_t* data, size_t data_size, uint32_t sequence) -> bool {
      bool success = session_ptr->write(channel_id, sequence, 
                                static_cast<uint64_t>(timestamp_ns),
                                static_cast<uint64_t>(timestamp_ns),  // publish_time = log_time
                                data, data_size);
      if (success) {
        // Track topic stats for metadata injection
        session_ptr->update_topic_stats(topic_name, msg_type);
      }
      return success;
    };

  if (!worker_pool_->create_topic_worker(topic_name, handler)) {
    ros_interface_->log_error("Failed to create worker for topic: " + topic_name);
    return;
  }

  // =========================================================================
  // Step 3: Create subscription via TopicManager
  // =========================================================================
  // Configure QoS based on message type
  SubscriptionConfig sub_config;
  if (topic_config.message_type.find("Image") != std::string::npos) {
    sub_config = SubscriptionConfig::high_throughput();
    sub_config.history_depth = 1000;  // Match publisher depth
  } else if (topic_config.message_type.find("Imu") != std::string::npos) {
    sub_config = SubscriptionConfig::high_throughput();
    sub_config.history_depth = 5000;  // IMU is high frequency
  } else {
    sub_config = SubscriptionConfig::high_throughput();
    sub_config.history_depth = 1000;
  }

  // Callback routes messages to worker pool
  TopicManager::MessageCallback callback = 
    [this, topic_name](const std::string& topic, int64_t timestamp_ns, std::vector<uint8_t>&& data) {
      // Fast path: check recording state
      if (!is_actively_recording()) {
        return;
      }

      // Create message item and push to worker pool
      MessageItem item(timestamp_ns, std::move(data));
      worker_pool_->try_push(topic, std::move(item));
    };

  if (!topic_manager_->subscribe(topic_name, topic_config.message_type, callback, sub_config)) {
    ros_interface_->log_error("Failed to subscribe to topic: " + topic_name);
    return;
  }

  ros_interface_->log_info("Topic recording setup complete for: " + topic_name);
}

void RecorderNode::setup_services() {
  // Create and register service adapter
  // Use shared_from_this() to pass a shared_ptr to the ServiceAdapter
  service_adapter_ = std::make_unique<ServiceAdapter>(shared_from_this(), ros_interface_.get());

  if (service_adapter_->register_services()) {
    ros_interface_->log_info("Recording services registered successfully");
  } else {
    ros_interface_->log_error("Failed to register recording services");
  }
}

bool RecorderNode::start_recording() {
  // Note: State validation is done by RecordingServiceImpl before calling this method
  // StateManager is the single source of truth for recording state

  // Start worker thread pool
  worker_pool_->start();

  // Record start time for duration calculation
  recording_start_time_ = std::chrono::system_clock::now();

  ros_interface_->log_info(
    "Recording started with " + std::to_string(worker_pool_->topic_count()) + " topics to " +
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

    http_callback_client_->post_start_callback_async(*config_opt, payload);
  }

  return true;
}

void RecorderNode::pause_recording() {
  // Note: State validation is done by RecordingServiceImpl before calling this method
  // StateManager is the single source of truth for recording state
  ros_interface_->log_info("Recording paused");
}

void RecorderNode::resume_recording() {
  // Note: State validation is done by RecordingServiceImpl before calling this method
  // StateManager is the single source of truth for recording state
  ros_interface_->log_info("Recording resumed");
}

void RecorderNode::cancel_recording() {
  std::cerr << "[axon_recorder] cancel_recording() called" << std::endl;

  // Note: State is managed by StateManager - no need to set flags here
  // Brief sleep to allow in-flight callbacks to complete
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Stop worker pool (will drain remaining queue items)
  worker_pool_->stop();

  // Get stats before clearing
  auto aggregate_stats = worker_pool_->get_aggregate_stats();

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
    payload.message_count = aggregate_stats.total_written;
    payload.file_size_bytes = 0;  // Not finalized
    payload.output_path = output_path_;
    payload.sidecar_path = "";  // Not generated for cancelled recording
    payload.topics = config_opt->topics;
    // Fill metadata summary
    payload.metadata.scene = config_opt->scene;
    payload.metadata.subscene = config_opt->subscene;
    payload.metadata.skills = config_opt->skills;
    payload.metadata.factory = config_opt->factory;
    payload.error = "Recording cancelled";

    http_callback_client_->post_finish_callback_async(*config_opt, payload);
  }

  // Clear the task config
  task_config_cache_.clear();

  std::cerr << "[axon_recorder] Recording cancelled" << std::endl;
}

void RecorderNode::stop_recording() {
  // Use std::cerr for shutdown messages since ROS logging may be disabled after rclcpp::shutdown()
  std::cerr << "[axon_recorder] stop_recording() called" << std::endl;

  // Note: State is managed by StateManager - no need to set flags here
  // Brief sleep to allow in-flight callbacks to complete
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Stop worker pool (will drain remaining queue items)
  std::cerr << "[axon_recorder] Stopping worker pool..." << std::endl;
  worker_pool_->stop();

  // Flush recording session to ensure all data is written
  if (recording_session_ && recording_session_->is_open()) {
    std::cerr << "[axon_recorder] Flushing recording session..." << std::endl;
    recording_session_->flush();
  }
  std::cerr << "[axon_recorder] Recording session flushed" << std::endl;

  // Get stats before closing (close will inject metadata and generate sidecar)
  auto aggregate_stats = worker_pool_->get_aggregate_stats();
  auto session_stats = recording_session_ ? recording_session_->get_stats()
                                          : RecordingSession::Stats{};

  // Close recording session (this injects metadata and generates sidecar JSON)
  std::string sidecar_path;
  if (recording_session_ && recording_session_->is_open()) {
    std::cerr << "[axon_recorder] Closing recording session (metadata injection)..." << std::endl;
    recording_session_->close();
    sidecar_path = recording_session_->get_sidecar_path();
    std::cerr << "[axon_recorder] Sidecar generated: " << sidecar_path << std::endl;
  }

  // Send finish callback if configured
  auto config_opt = task_config_cache_.get();
  if (config_opt && config_opt->has_callbacks()) {
    auto now = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - recording_start_time_);

    FinishCallbackPayload payload;
    payload.task_id = config_opt->task_id;
    payload.device_id = config_opt->device_id;
    payload.status = "finished";
    payload.started_at = HttpCallbackClient::get_iso8601_timestamp(recording_start_time_);
    payload.finished_at = HttpCallbackClient::get_iso8601_timestamp(now);
    payload.duration_sec = duration.count() / 1000.0;
    payload.message_count = aggregate_stats.total_written;
    payload.file_size_bytes = session_stats.bytes_written;
    payload.output_path = output_path_;
    payload.sidecar_path = sidecar_path;
    payload.topics = config_opt->topics;
    // Fill metadata summary
    payload.metadata.scene = config_opt->scene;
    payload.metadata.subscene = config_opt->subscene;
    payload.metadata.skills = config_opt->skills;
    payload.metadata.factory = config_opt->factory;
    payload.error = "";

    http_callback_client_->post_finish_callback_async(*config_opt, payload);
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

  // Pass task config to recording session for metadata injection
  if (recording_session_) {
    recording_session_->set_task_config(config);
    ros_interface_->log_info("Task config set for metadata injection");
  }

  return true;
}

RecordingStats RecorderNode::get_stats() const {
  auto aggregate = worker_pool_->get_aggregate_stats();

  RecordingStats stats;
  stats.messages_received = aggregate.total_received;
  stats.messages_dropped = aggregate.total_dropped;
  stats.messages_written = aggregate.total_written;

  if (stats.messages_received > 0) {
    stats.drop_rate_percent = 100.0 * static_cast<double>(stats.messages_dropped) /
                              static_cast<double>(stats.messages_received);
  } else {
    stats.drop_rate_percent = 0.0;
  }

  return stats;
}

double RecorderNode::get_recording_duration_sec() const {
  // Use StateManager to check if recording is active
  if (!is_recording()) {
    return 0.0;
  }
  auto now = std::chrono::system_clock::now();
  return std::chrono::duration<double>(now - recording_start_time_).count();
}

void RecorderNode::write_stats_file() {
  std::ofstream stats_file(STATS_FILE_PATH);
  if (!stats_file.is_open()) {
    std::cerr << "[axon_recorder] Could not write stats file: " << STATS_FILE_PATH << std::endl;
    return;
  }

  auto stats = get_stats();
  auto session_stats = recording_session_ ? recording_session_->get_stats()
                                          : RecordingSession::Stats{};

  // Also write per-topic statistics
  stats_file << "{\n"
             << "  \"output_file\": \"" << output_path_ << "\",\n"
             << "  \"format\": \"mcap\",\n"
             << "  \"messages_received\": " << stats.messages_received << ",\n"
             << "  \"messages_dropped\": " << stats.messages_dropped << ",\n"
             << "  \"messages_written\": " << stats.messages_written << ",\n"
             << "  \"bytes_written\": " << session_stats.bytes_written << ",\n"
             << "  \"drop_rate_percent\": " << std::fixed << std::setprecision(2)
             << stats.drop_rate_percent << ",\n"
             << "  \"per_topic_stats\": {\n";

  bool first = true;
  auto topics = worker_pool_->get_topics();
  for (const auto& topic_name : topics) {
    if (!first) stats_file << ",\n";
    first = false;

    auto topic_stats = worker_pool_->get_topic_stats(topic_name);
    double topic_drop_rate = (topic_stats.received > 0) 
                             ? (100.0 * topic_stats.dropped / topic_stats.received) 
                             : 0.0;

    stats_file << "    \"" << topic_name << "\": {"
               << "\"received\": " << topic_stats.received << ", "
               << "\"dropped\": " << topic_stats.dropped << ", "
               << "\"written\": " << topic_stats.written << ", "
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
