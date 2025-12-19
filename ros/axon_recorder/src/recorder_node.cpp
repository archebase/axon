#include "recorder_node.hpp"
#include "ros_interface.hpp"
#include "ros_introspection.hpp"
#include "batch_manager.hpp"
#include "config_parser.hpp"
#include "message_converter.hpp"
#include "message_introspection.hpp"
#include "schema_merger.hpp"
#include <arrow/c/bridge.h>  // Arrow C Data Interface (ExportSchema, ExportRecordBatch)
#include <arrow/builder.h>   // BinaryBuilder for worker thread conversion
#include <axon/lance_bridge.h>

#if defined(AXON_ROS1)
#include <ros/ros.h>
#include <ros/param.h>
#include <ros/package.h>
#elif defined(AXON_ROS2)
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#endif

#include <fstream>
#include <iomanip>
#include <memory>
#include <unordered_map>
#include <vector>
#include <thread>
#include <atomic>
#include <iostream>
#include <csignal>

namespace axon {
namespace recorder {

RecorderNode::RecorderNode() 
    : recording_(false)
    , dataset_handle_(0)
    , workers_running_(false)
{
}

RecorderNode::~RecorderNode() {
    shutdown();
}

void RecorderNode::start_worker_threads() {
    if (workers_running_.load()) {
        return;
    }
    
    workers_running_.store(true);
    worker_threads_.reserve(NUM_WORKER_THREADS);
    
    for (size_t i = 0; i < NUM_WORKER_THREADS; ++i) {
        worker_threads_.emplace_back(&RecorderNode::worker_thread_func, this);
    }
}

void RecorderNode::stop_worker_threads() {
    if (!workers_running_.load()) {
        return;
    }
    
    workers_running_.store(false);
    
    // Notify all per-topic queues
    for (auto& [topic_name, topic_queue] : topic_queues_) {
        topic_queue->cv.notify_all();
    }
    
    for (auto& thread : worker_threads_) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    worker_threads_.clear();
}

void RecorderNode::worker_thread_func() {
    while (workers_running_.load()) {
        // Round-robin through all topic queues to avoid starvation
        bool processed_any = false;
        
        for (auto& [topic_name, topic_queue] : topic_queues_) {
            if (!workers_running_.load()) break;
            
            MessageItem item;
            bool got_item = false;
            
            // Try to get an item from this topic's queue (non-blocking)
            {
                std::unique_lock<std::mutex> lock(topic_queue->mutex, std::try_to_lock);
                if (lock.owns_lock() && !topic_queue->queue.empty()) {
                    item = std::move(topic_queue->queue.front());
                    topic_queue->queue.pop();
                    topic_queue->size.fetch_sub(1, std::memory_order_relaxed);
                    got_item = true;
                }
            }
            
            if (!got_item) continue;
            processed_any = true;
            
            // Process message outside of lock
            auto batch_mgr_it = batch_managers_.find(topic_name);
            if (batch_mgr_it == batch_managers_.end()) continue;
            
            try {
                // Direct binary append with ownership transfer (zero-copy in BatchManager)
                batch_mgr_it->second->add_row_with_raw_data(
                    item.timestamp_ns, topic_name, std::move(item.raw_data));
                
                // Track successful writes
                messages_written_.fetch_add(1, std::memory_order_relaxed);
            } catch (const std::exception& e) {
                if (ros_interface_) {
                    ros_interface_->log_error(std::string("Worker exception: ") + e.what());
                }
            }
        }
        
        // If no work was found, sleep briefly to avoid busy-waiting
        if (!processed_any) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }
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
    core::MessageIntrospectorFactory::set_factory(
        []() { return RosIntrospectorFactory::create(); }
    );
    
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
    std::cerr << "[axon_recorder] shutdown() called" << std::endl;
    
    stop_recording();
    
    // Stop all batch managers
    std::cerr << "[axon_recorder] Stopping batch managers..." << std::endl;
    for (auto& pair : batch_managers_) {
        pair.second->stop();
    }
    batch_managers_.clear();
    
    // Unsubscribe all
    std::cerr << "[axon_recorder] Unsubscribing topics..." << std::endl;
    for (auto& pair : subscriptions_) {
        ros_interface_->unsubscribe(pair.second);
    }
    subscriptions_.clear();
    
    if (dataset_handle_ > 0) {
        std::cerr << "[axon_recorder] Closing dataset..." << std::endl;
        close_dataset(dataset_handle_);
        dataset_handle_ = 0;
    }
    
    if (ros_interface_) {
        std::cerr << "[axon_recorder] Shutting down ROS interface..." << std::endl;
        ros_interface_->shutdown();
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
    std::string install_config = ament_index_cpp::get_package_share_directory("axon_recorder") + "/config/default_config.yaml";
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

    ros_interface_->log_info("Recording schema has " +
                             std::to_string(merged_schema_->num_fields()) + " fields");
    
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
    
    // Create batch manager with merged schema
    auto write_callback = [this](const std::shared_ptr<arrow::RecordBatch>& batch,
                                 const std::string& path,
                                 int64_t handle) -> bool {
        struct ArrowArray c_array;
        struct ArrowSchema c_schema;
        arrow::Status status = arrow::ExportRecordBatch(*batch, &c_array, &c_schema);
        
        if (!status.ok()) {
            ros_interface_->log_error("Failed to export batch: " + status.ToString());
            return false;
        }
        
        int32_t result = write_batch(handle, &c_array, &c_schema);
        
        // Release schema only - Rust takes ownership of array via Arrow C Data Interface
        // The array's release callback will be called by Rust when it's done with the data.
        // Calling c_array.release() here would cause a double-free.
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
    
    // Create memory pool for this batch manager
    auto memory_pool = arrow::default_memory_pool();
    
    auto batch_mgr = std::make_unique<core::BatchManager>(
        topic_config.batch_size,
        topic_config.flush_interval_ms,
        write_callback,
        memory_pool
    );
    
    // Initialize with merged schema
    batch_mgr->initialize_schema(merged_schema_);
    batch_mgr->set_dataset(dataset_path_, dataset_handle_);
    batch_mgr->start();
    
    batch_managers_[topic_config.name] = std::move(batch_mgr);
    
    // Create per-topic queue for this topic
    topic_queues_[topic_config.name] = std::make_unique<TopicQueue>();
    auto* topic_queue = topic_queues_[topic_config.name].get();
    
    // Subscribe to topic - fast path: just copy raw bytes and queue
    auto callback = [this, topic_name = topic_config.name, topic_queue](const void* msg_ptr) {
        if (!recording_.load()) {
            return;
        }
        
        // Capture timestamp immediately for accuracy
        int64_t timestamp_ns = ros_interface_->now_nsec();
        
        // Get raw message bytes - fast pointer access
#if defined(AXON_ROS2)
        const rclcpp::SerializedMessage* serialized_msg = 
            static_cast<const rclcpp::SerializedMessage*>(msg_ptr);
        if (!serialized_msg || serialized_msg->size() == 0) {
            return;
        }
        
        // Increment received counter
        messages_received_.fetch_add(1, std::memory_order_relaxed);
        
        // Check queue size without lock (approximate, but fast)
        if (topic_queue->size.load(std::memory_order_relaxed) >= MAX_QUEUE_SIZE_PER_TOPIC) {
            messages_dropped_.fetch_add(1, std::memory_order_relaxed);
            return;  // Drop message to prevent memory buildup
        }
        
        // Fast enqueue to per-topic queue - minimal contention
        {
            std::lock_guard<std::mutex> lock(topic_queue->mutex);
            
            MessageItem item;
            item.timestamp_ns = timestamp_ns;
            
            // Copy raw bytes - unavoidable but faster than Arrow conversion
            const auto& rcl_msg = serialized_msg->get_rcl_serialized_message();
            item.raw_data.assign(rcl_msg.buffer, rcl_msg.buffer + rcl_msg.buffer_length);
            
            topic_queue->queue.push(std::move(item));
            topic_queue->size.fetch_add(1, std::memory_order_relaxed);
        }
#endif
    };
    
    void* sub = ros_interface_->subscribe(
        topic_config.name,
        topic_config.message_type,
        callback
    );
    
    if (sub) {
        subscriptions_[topic_config.name] = sub;
        ros_interface_->log_info("Subscribed to topic: " + topic_config.name);
    } else {
        ros_interface_->log_error("Failed to subscribe to topic: " + topic_config.name);
    }
}

void RecorderNode::setup_services() {
    // Full service implementation would go here
    ros_interface_->log_info("Services available via ROS interface");
}

void RecorderNode::start_recording() {
    // Start worker threads for async message processing
    start_worker_threads();
    recording_ = true;
    ros_interface_->log_info("Recording started");
}

void RecorderNode::stop_recording() {
    // Use std::cerr for shutdown messages since ROS logging may be disabled after rclcpp::shutdown()
    std::cerr << "[axon_recorder] stop_recording() called" << std::endl;
    
    recording_ = false;
    
    // Stop worker threads (will drain remaining queue items)
    std::cerr << "[axon_recorder] Stopping worker threads..." << std::endl;
    stop_worker_threads();
    std::cerr << "[axon_recorder] Worker threads stopped" << std::endl;
    
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

void RecorderNode::write_stats_file() {
    std::ofstream stats_file(STATS_FILE_PATH);
    if (!stats_file.is_open()) {
        ros_interface_->log_warn("Could not write stats file: " + std::string(STATS_FILE_PATH));
        return;
    }
    
    uint64_t received = messages_received_.load();
    uint64_t dropped = messages_dropped_.load();
    uint64_t written = messages_written_.load();
    double drop_rate = (received > 0) ? (100.0 * dropped / received) : 0.0;
    
    stats_file << "{\n"
               << "  \"messages_received\": " << received << ",\n"
               << "  \"messages_dropped\": " << dropped << ",\n"
               << "  \"messages_written\": " << written << ",\n"
               << "  \"drop_rate_percent\": " << std::fixed << std::setprecision(2) << drop_rate << "\n"
               << "}\n";
    
    stats_file.close();
    
    ros_interface_->log_info("Stats written: received=" + std::to_string(received) + 
                            ", dropped=" + std::to_string(dropped) +
                            ", written=" + std::to_string(written) +
                            ", drop_rate=" + std::to_string(drop_rate) + "%");
}

} // namespace recorder
} // namespace axon

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
    std::cerr << "[axon_recorder] Received signal " << signum 
              << " (SIGTERM=" << SIGTERM << ", SIGINT=" << SIGINT << ")" << std::endl;
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
    
    std::cerr << "[axon_recorder] main() exiting normally" << std::endl;
    return 0;
}
