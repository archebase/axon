#ifndef AXON_RECORDER_NODE_HPP
#define AXON_RECORDER_NODE_HPP

#include <axon_utils/worker_thread_pool.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>

#include "common_types.hpp"
#include "http_client.hpp"
#include "recorder_adapter.hpp"
#include "recording_session.hpp"
#include "ros_interface.hpp"
#include "topic_manager.hpp"

// Forward declarations
namespace axon {
namespace recorder {
class ServiceAdapter;
}
namespace uploader {
class EdgeUploader;
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
 * - Single source of truth for recording state (via StateMachine)
 * - Testable interface for mock implementations
 */
class RecorderNode : public IRecorderContext, public std::enable_shared_from_this<RecorderNode> {
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
  // IRecorderContext Implementation
  // =========================================================================

  /**
   * Get the current state as a string.
   */
  std::string get_state_string() const override;

  /**
   * Check if recording is active (RECORDING or PAUSED).
   */
  bool is_recording_active() const override;

  /**
   * Get the cached task configuration.
   */
  std::optional<TaskConfig> get_cached_config() const override;

  /**
   * Get recording metrics.
   */
  bool get_recording_metrics(
    std::string& output_path, double& disk_usage_gb, double& duration_sec, int64_t& message_count,
    double& throughput_mb_sec, std::string& last_error
  ) const override;

  /**
   * Cache a new task configuration.
   */
  bool cache_task_config(const TaskConfig& config, std::string& error_msg) override;

  /**
   * Start recording
   * Enables message processing from queues
   */
  bool start_recording(std::string& error_msg) override;

  /**
   * Pause recording
   * Temporarily stops accepting new messages
   */
  bool pause_recording(std::string& error_msg) override;

  /**
   * Resume recording
   * Resumes accepting messages after pause
   */
  bool resume_recording(std::string& error_msg) override;

  /**
   * Cancel recording
   * Stops recording and cleans up without finalizing
   */
  bool cancel_recording(std::string& error_msg) override;

  /**
   * Finish recording
   * Stops recording and finalizes file
   */
  bool finish_recording(std::string& error_msg) override;

  /**
   * Clear cached configuration.
   */
  bool clear_config(std::string& error_msg) override;

  // =========================================================================
  // Additional Methods (not part of IRecorderContext)
  // =========================================================================

  /**
   * Stop recording (internal use, calls finish_recording)
   */
  void stop_recording();

  /**
   * Check if recording is active (RECORDING or PAUSED state).
   * Internal helper method.
   */
  bool is_recording() const {
    auto state = state_machine_.get_state();
    return state == RecorderState::RECORDING || state == RecorderState::PAUSED;
  }

  /**
   * Check if recording is paused.
   * Internal helper method.
   */
  bool is_paused() const {
    return state_machine_.is_state(RecorderState::PAUSED);
  }

  /**
   * Check if actively recording (not paused).
   * Internal helper method.
   */
  bool is_actively_recording() const {
    return state_machine_.is_state(RecorderState::RECORDING);
  }

  /**
   * Configure topics from task config (used when starting from cached config)
   * Internal helper method.
   */
  bool configure_from_task_config(const TaskConfig& config);

  /**
   * Get current statistics (internal helper, not part of IRecorderContext)
   */
  struct RecordingStats {
    uint64_t messages_received = 0;
    uint64_t messages_dropped = 0;
    uint64_t messages_written = 0;
    double drop_rate_percent = 0.0;
  };
  RecordingStats get_stats() const;

  /**
   * Get recording duration in seconds (internal helper, not part of IRecorderContext)
   */
  double get_recording_duration_sec() const;

  // =========================================================================
  // Logging Methods (not part of IRecorderContext interface)
  // =========================================================================

  /**
   * Log an informational message.
   */
  void log_info(const std::string& msg) {
    if (ros_interface_) {
      ros_interface_->log_info(msg);
    }
  }

  /**
   * Log a warning message.
   */
  void log_warn(const std::string& msg) {
    if (ros_interface_) {
      ros_interface_->log_warn(msg);
    }
  }

  /**
   * Log an error message.
   */
  void log_error(const std::string& msg) {
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
  std::unique_ptr<axon::utils::WorkerThreadPool> worker_pool_;

  // =========================================================================
  // High-Performance Message Queue Architecture
  // =========================================================================
  // Note: MessageItem and TopicContext are now in WorkerThreadPool
  // Worker threads and queues are managed by worker_pool_

  // =========================================================================
  // State Machine & Task Management
  // =========================================================================
  StateMachine state_machine_;
  TaskConfigCache task_config_cache_;
  std::shared_ptr<HttpClient> http_callback_client_;

  // =========================================================================
  // Edge Uploader (optional - uploads MCAP files to S3)
  // =========================================================================
  // Note: EdgeUploader is conditionally compiled. When AXON_HAS_UPLOADER is
  // defined, the recorder will upload MCAP files to S3 after finalization.
  // Uses unique_ptr with custom deleter to handle incomplete type properly.
  struct UploaderDeleter {
    void operator()(uploader::EdgeUploader* ptr) const;
  };
  std::unique_ptr<uploader::EdgeUploader, UploaderDeleter> uploader_;

  // Recording start time (for duration calculation)
  std::chrono::system_clock::time_point recording_start_time_;

  // =========================================================================
  // State
  // =========================================================================
  // Note: recording_ and paused_ flags removed - use StateManager as single source of truth
  // Use is_recording(), is_paused(), is_actively_recording() methods instead
  std::atomic<bool> shutdown_requested_{false};
};

}  // namespace recorder
}  // namespace axon

#endif  // AXON_RECORDER_NODE_HPP
