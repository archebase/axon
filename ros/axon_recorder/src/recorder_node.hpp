#ifndef AXON_RECORDER_NODE_HPP
#define AXON_RECORDER_NODE_HPP

#include <atomic>
#include <chrono>
#include <memory>
#include <string>

#include "config_parser.hpp"
#include "http_callback_client.hpp"
#include "recorder_context.hpp"
#include "recording_session.hpp"
#include "ros_interface.hpp"
#include "state_machine.hpp"
#include "task_config.hpp"
#include "topic_manager.hpp"
#include "worker_thread_pool.hpp"

// Forward declarations
namespace axon {
namespace recorder {
class ServiceAdapter;
}
}  // namespace axon

namespace axon {
namespace recorder {

/**
 * High-Performance Recorder Node for Axon
 *
 * Architecture Overview:
 * ----------------------
 * This recorder is designed for high-throughput edge recording scenarios with
 * multiple high-frequency sensor streams (cameras, IMU, lidar, etc.).
 *
 * Key Design Principles:
 * 1. Zero-copy message path: ROS subscription -> Lock-free queue -> MCAP file
 * 2. Per-topic parallelism: Each topic has dedicated resources to prevent interference
 * 3. Lock-free queues: Eliminates mutex contention in the hot path
 * 4. Deferred processing: Minimal work in callbacks, heavy lifting in worker threads
 *
 * Data Flow (MCAP Backend):
 * -------------------------
 * [ROS DDS] -> [Generic Subscription] -> [Zero-copy Callback] -> [SPSC Queue]
 *                                                                      |
 *                                                                      v
 * [MCAP File] <----------------------- [McapWriterWrapper] <- [Worker Thread]
 *
 * Threading Model:
 * ----------------
 * - ROS executor threads: Handle subscription callbacks (configurable)
 * - Per-topic worker threads: Drain queues and write to MCAP
 * - MCAP writer handles chunking and compression internally
 *
 * Memory Management:
 * ------------------
 * - Pre-allocated lock-free queues per topic
 * - Ownership transfer (move semantics) throughout the pipeline
 * - Direct serialized message storage (no Arrow conversion needed)
 *
 * This class implements IRecorderContext interface, providing:
 * - Dependency injection for services (RecordingServiceImpl, ServiceAdapter)
 * - Single source of truth for recording state (via StateManager)
 * - Testable interface for mock implementations
 */
class RecorderNode : public IRecorderContext,
                     public std::enable_shared_from_this<RecorderNode> {
public:
  /**
   * Create a RecorderNode instance.
   * Use this factory method instead of constructor for proper shared_ptr usage.
   */
  static std::shared_ptr<RecorderNode> create();

  ~RecorderNode();

  // Non-copyable, non-movable (contains threads and atomics)
  RecorderNode(const RecorderNode&) = delete;
  RecorderNode& operator=(const RecorderNode&) = delete;
  RecorderNode(RecorderNode&&) = delete;
  RecorderNode& operator=(RecorderNode&&) = delete;

  /**
   * Initialize the recorder node (loads config from file)
   * Sets up ROS interface, loads config, creates subscriptions
   */
  bool initialize(int argc, char** argv);

  /**
   * Initialize the recorder node with injected configuration
   * Allows bypassing file-based config lookup for testing and programmatic setup.
   * @param argc Command line argument count
   * @param argv Command line arguments
   * @param config Pre-configured RecorderConfig to use
   * @return true if initialization succeeded
   */
  bool initialize(int argc, char** argv, const RecorderConfig& config);

  /**
   * Run the recorder (blocking)
   * Starts ROS spin loop, returns when shutdown is requested
   */
  void run();

  /**
   * Shutdown the recorder
   * Drains queues, flushes batches, writes stats
   */
  void shutdown();

  // =========================================================================
  // IRecorderContext Implementation - Recording Operations
  // =========================================================================

  /**
   * Start recording
   * Enables message processing from queues
   */
  bool start_recording() override;

  /**
   * Stop recording
   * Disables new message acceptance, drains existing queues
   */
  void stop_recording() override;

  /**
   * Pause recording
   * Temporarily stops accepting new messages
   */
  void pause_recording() override;

  /**
   * Resume recording
   * Resumes accepting messages after pause
   */
  void resume_recording() override;

  /**
   * Cancel recording
   * Stops recording and cleans up without finalizing
   */
  void cancel_recording() override;

  /**
   * Check if recording is active (RECORDING or PAUSED state).
   * Derived from StateManager - single source of truth.
   */
  bool is_recording() const {
    auto state = state_manager_.get_state();
    return state == RecorderState::RECORDING || state == RecorderState::PAUSED;
  }

  /**
   * Check if recording is paused.
   * Derived from StateManager - single source of truth.
   */
  bool is_paused() const {
    return state_manager_.is_state(RecorderState::PAUSED);
  }

  /**
   * Check if actively recording (not paused).
   * Derived from StateManager - single source of truth.
   */
  bool is_actively_recording() const {
    return state_manager_.is_state(RecorderState::RECORDING);
  }

  // =========================================================================
  // IRecorderContext Implementation - State Management
  // =========================================================================

  /**
   * Get the state manager for external access
   */
  StateManager& get_state_manager() override {
    return state_manager_;
  }

  /**
   * Get the task config cache for external access
   */
  TaskConfigCache& get_task_config_cache() override {
    return task_config_cache_;
  }

  /**
   * Get the HTTP callback client for external access
   */
  std::shared_ptr<HttpCallbackClient> get_http_callback_client() override {
    return http_callback_client_;
  }

  // =========================================================================
  // IRecorderContext Implementation - Configuration and Status
  // =========================================================================

  /**
   * Configure topics from task config (used when starting from cached config)
   */
  bool configure_from_task_config(const TaskConfig& config) override;

  /**
   * Get the current output path
   */
  std::string get_output_path() const override {
    return output_path_;
  }

  /**
   * Get current statistics
   */
  RecordingStats get_stats() const override;

  /**
   * Get recording duration in seconds.
   * Returns 0.0 if not currently recording.
   */
  double get_recording_duration_sec() const override;

  // =========================================================================
  // IRecorderContext Implementation - Logging
  // =========================================================================

  /**
   * Log an informational message.
   */
  void log_info(const std::string& msg) override {
    if (ros_interface_) {
      ros_interface_->log_info(msg);
    }
  }

  /**
   * Log a warning message.
   */
  void log_warn(const std::string& msg) override {
    if (ros_interface_) {
      ros_interface_->log_warn(msg);
    }
  }

  /**
   * Log an error message.
   */
  void log_error(const std::string& msg) override {
    if (ros_interface_) {
      ros_interface_->log_error(msg);
    }
  }

  // =========================================================================
  // Additional Public Methods (not part of IRecorderContext)
  // =========================================================================

  /**
   * Get the ROS interface for direct access (internal use)
   */
  RosInterface* get_ros_interface() {
    return ros_interface_.get();
  }

private:
  // Private constructor - use create() factory method
  RecorderNode();

  // =========================================================================
  // Initialization
  // =========================================================================
  
  /**
   * Common initialization logic after ROS and config are ready.
   * Called by both initialize() overloads.
   */
  bool complete_initialization();
  
  bool load_configuration();
  bool initialize_mcap_writer();
  bool register_topic_schemas();
  void setup_topic_recording(const TopicConfig& topic_config);
  void setup_services();
  std::string get_config_path();
  std::string generate_output_path() const;

  // =========================================================================
  // Schema helpers
  // =========================================================================
  std::string get_message_definition(const std::string& message_type);
  std::string get_schema_encoding();
  std::string get_message_encoding();

  // =========================================================================
  // Worker Thread Management (delegated to WorkerThreadPool)
  // =========================================================================
  void setup_worker_pool();

  // =========================================================================
  // Statistics
  // =========================================================================
  void write_stats_file();

  // =========================================================================
  // Core Components
  // =========================================================================
  std::unique_ptr<RosInterface> ros_interface_;
  std::unique_ptr<ServiceAdapter> service_adapter_;
  RecorderConfig config_;

  // Recording session (encapsulates MCAP lifecycle)
  std::unique_ptr<RecordingSession> recording_session_;
  std::string output_path_;

  // Topic manager (handles ROS subscriptions)
  std::unique_ptr<TopicManager> topic_manager_;

  // Worker thread pool (handles per-topic queues and workers)
  std::unique_ptr<WorkerThreadPool> worker_pool_;

  // =========================================================================
  // High-Performance Message Queue Architecture
  // =========================================================================
  // Note: MessageItem and TopicContext are now in WorkerThreadPool
  // Worker threads and queues are managed by worker_pool_

  // =========================================================================
  // State Machine & Task Management
  // =========================================================================
  StateManager state_manager_;
  TaskConfigCache task_config_cache_;
  std::shared_ptr<HttpCallbackClient> http_callback_client_;

  // =========================================================================
  // Edge Uploader (optional - uploads MCAP files to S3)
  // =========================================================================
  // Note: EdgeUploader is conditionally compiled. When AXON_HAS_UPLOADER is
  // defined, the recorder will upload MCAP files to S3 after finalization.
  // The uploader_ member is managed via void* to avoid incomplete type issues
  // with forward declarations. See recorder_node.cpp for the actual type.
  void* uploader_ = nullptr;  // Actually uploader::EdgeUploader* when enabled

  // Recording start time (for duration calculation)
  std::chrono::system_clock::time_point recording_start_time_;

  // =========================================================================
  // State
  // =========================================================================
  // Note: recording_ and paused_ flags removed - use StateManager as single source of truth
  // Use is_recording(), is_paused(), is_actively_recording() methods instead
  std::atomic<bool> shutdown_requested_{false};

  // =========================================================================
  // Configuration Constants
  // =========================================================================
  // Statistics file path
  static constexpr const char* STATS_FILE_PATH = "/data/recordings/recorder_stats.json";
};

}  // namespace recorder
}  // namespace axon

#endif  // AXON_RECORDER_NODE_HPP
