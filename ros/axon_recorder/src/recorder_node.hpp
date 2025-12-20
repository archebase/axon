#ifndef AXON_RECORDER_NODE_HPP
#define AXON_RECORDER_NODE_HPP

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "config_parser.hpp"
#include "http_callback_client.hpp"
#include "mcap_writer_wrapper.hpp"
#include "ros_interface.hpp"
#include "spsc_queue.hpp"
#include "state_machine.hpp"
#include "task_config.hpp"

// Forward declaration
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
 */
class RecorderNode {
public:
  RecorderNode();
  ~RecorderNode();

  // Non-copyable, non-movable (contains threads and atomics)
  RecorderNode(const RecorderNode&) = delete;
  RecorderNode& operator=(const RecorderNode&) = delete;
  RecorderNode(RecorderNode&&) = delete;
  RecorderNode& operator=(RecorderNode&&) = delete;

  /**
   * Initialize the recorder node
   * Sets up ROS interface, loads config, creates subscriptions
   */
  bool initialize(int argc, char** argv);

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

  /**
   * Start recording
   * Enables message processing from queues
   */
  void start_recording();

  /**
   * Stop recording
   * Disables new message acceptance, drains existing queues
   */
  void stop_recording();

  /**
   * Pause recording
   * Temporarily stops accepting new messages
   */
  void pause_recording();

  /**
   * Resume recording
   * Resumes accepting messages after pause
   */
  void resume_recording();

  /**
   * Cancel recording
   * Stops recording and cleans up without finalizing
   */
  void cancel_recording();

  /**
   * Check if recording is active (recording or paused)
   */
  bool is_recording() const {
    return recording_.load(std::memory_order_acquire);
  }

  /**
   * Check if recording is paused
   */
  bool is_paused() const {
    return paused_.load(std::memory_order_acquire);
  }

  /**
   * Get the state manager for external access
   */
  StateManager& get_state_manager() {
    return state_manager_;
  }

  /**
   * Get the task config cache for external access
   */
  TaskConfigCache& get_task_config_cache() {
    return task_config_cache_;
  }

  /**
   * Get the HTTP callback client for external access
   */
  HttpCallbackClient& get_http_callback_client() {
    return http_callback_client_;
  }

  /**
   * Configure topics from task config (used when starting from cached config)
   */
  bool configure_from_task_config(const TaskConfig& config);

  /**
   * Get the current output path
   */
  std::string get_output_path() const {
    return output_path_;
  }

  /**
   * Get current statistics
   */
  struct RecordingStats {
    uint64_t messages_received;
    uint64_t messages_dropped;
    uint64_t messages_written;
    double drop_rate_percent;
  };
  RecordingStats get_stats() const;

private:
  // =========================================================================
  // Initialization
  // =========================================================================
  bool load_configuration();
  bool initialize_mcap_writer();
  bool register_topic_schemas();
  void setup_topic_recording(const core::TopicConfig& topic_config);
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
  // Worker Thread Management
  // =========================================================================
  void start_worker_threads();
  void stop_worker_threads();
  void worker_thread_func(const std::string& topic_name);

  // =========================================================================
  // Statistics
  // =========================================================================
  void write_stats_file();

  // =========================================================================
  // Core Components
  // =========================================================================
  std::unique_ptr<RosInterface> ros_interface_;
  std::unique_ptr<ServiceAdapter> service_adapter_;
  core::RecorderConfig config_;

  // MCAP writer (replaces Lance dataset)
  std::unique_ptr<mcap_wrapper::McapWriterWrapper> mcap_writer_;
  std::string output_path_;

  // Channel IDs for each topic (assigned by MCAP writer)
  std::unordered_map<std::string, uint16_t> topic_channel_ids_;

  // Schema IDs for each message type (assigned by MCAP writer)
  std::unordered_map<std::string, uint16_t> message_type_schema_ids_;

  // Subscriptions (handles for cleanup)
  std::unordered_map<std::string, void*> subscriptions_;

  // =========================================================================
  // High-Performance Message Queue Architecture
  // =========================================================================

  /**
   * Message item for the lock-free queue
   * Uses move-only semantics for zero-copy transfer
   */
  struct MessageItem {
    int64_t timestamp_ns;
    std::vector<uint8_t> raw_data;  // Owned raw message bytes

    MessageItem()
        : timestamp_ns(0) {}

    MessageItem(int64_t ts, std::vector<uint8_t>&& data)
        : timestamp_ns(ts)
        , raw_data(std::move(data)) {}

    // Move-only for zero-copy
    MessageItem(MessageItem&&) = default;
    MessageItem& operator=(MessageItem&&) = default;
    MessageItem(const MessageItem&) = delete;
    MessageItem& operator=(const MessageItem&) = delete;
  };

  /**
   * Per-topic queue context
   * Contains the lock-free queue and its dedicated worker thread
   */
  struct TopicContext {
    std::unique_ptr<core::SPSCQueue<MessageItem>> queue;
    std::thread worker_thread;
    std::atomic<bool> running{false};

    // Statistics for this topic
    std::atomic<uint64_t> received{0};
    std::atomic<uint64_t> dropped{0};
    std::atomic<uint64_t> written{0};

    // Sequence number for MCAP messages
    std::atomic<uint32_t> sequence{0};

    TopicContext() = default;

    // Non-copyable, non-movable
    TopicContext(const TopicContext&) = delete;
    TopicContext& operator=(const TopicContext&) = delete;
    TopicContext(TopicContext&&) = delete;
    TopicContext& operator=(TopicContext&&) = delete;
  };

  // Per-topic contexts (one per subscribed topic)
  std::unordered_map<std::string, std::unique_ptr<TopicContext>> topic_contexts_;

  // =========================================================================
  // State Machine & Task Management
  // =========================================================================
  StateManager state_manager_;
  TaskConfigCache task_config_cache_;
  HttpCallbackClient http_callback_client_;

  // Recording start time (for duration calculation)
  std::chrono::system_clock::time_point recording_start_time_;

  // =========================================================================
  // State
  // =========================================================================
  std::atomic<bool> recording_{false};
  std::atomic<bool> paused_{false};
  std::atomic<bool> shutdown_requested_{false};

  // =========================================================================
  // Global Statistics (aggregated from per-topic stats)
  // =========================================================================
  std::atomic<uint64_t> messages_received_{0};
  std::atomic<uint64_t> messages_dropped_{0};
  std::atomic<uint64_t> messages_written_{0};

  // =========================================================================
  // Configuration Constants
  // =========================================================================
  // Queue capacity per topic - large enough to handle bursts
  static constexpr size_t QUEUE_CAPACITY_PER_TOPIC = 4096;

  // Worker thread sleep time when queue is empty (microseconds)
  static constexpr int WORKER_IDLE_SLEEP_US = 50;

  // Statistics file path
  static constexpr const char* STATS_FILE_PATH = "/data/recordings/recorder_stats.json";
};

}  // namespace recorder
}  // namespace axon

#endif  // AXON_RECORDER_NODE_HPP
