#ifndef AXON_RECORDER_NODE_HPP
#define AXON_RECORDER_NODE_HPP

#include <arrow/api.h>

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "batch_manager.hpp"
#include "config_parser.hpp"
#include "message_converter.hpp"
#include "ros_interface.hpp"
#include "spsc_queue.hpp"

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
 * 1. Zero-copy message path: ROS subscription -> Lock-free queue -> Arrow batch
 * 2. Per-topic parallelism: Each topic has dedicated resources to prevent interference
 * 3. Lock-free queues: Eliminates mutex contention in the hot path
 * 4. Deferred processing: Minimal work in callbacks, heavy lifting in worker threads
 *
 * Data Flow:
 * ----------
 * [ROS DDS] -> [Generic Subscription] -> [Zero-copy Callback] -> [SPSC Queue]
 *                                                                      |
 *                                                                      v
 * [Lance Dataset] <- [Arrow Batch] <- [BatchManager] <- [Worker Thread]
 *
 * Threading Model:
 * ----------------
 * - ROS executor threads: Handle subscription callbacks (configurable)
 * - Per-topic worker threads: Drain queues and build Arrow batches
 * - BatchManager writer thread: Async batch writing to Lance
 *
 * Memory Management:
 * ------------------
 * - Pre-allocated lock-free queues per topic
 * - Ownership transfer (move semantics) throughout the pipeline
 * - Arrow memory pool for efficient batch construction
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
   * Check if recording is active
   */
  bool is_recording() const {
    return recording_.load(std::memory_order_acquire);
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
  bool collect_topic_schemas();
  bool initialize_dataset();
  void setup_topic_recording(const core::TopicConfig& topic_config);
  void setup_services();
  std::string get_config_path();

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
  core::RecorderConfig config_;
  int64_t dataset_handle_;
  std::string dataset_path_;

  // Schema management
  std::shared_ptr<arrow::Schema> merged_schema_;
  std::unordered_map<std::string, std::shared_ptr<arrow::Schema>> topic_schemas_;
  std::unordered_map<std::string, std::unique_ptr<core::MessageConverter>> converters_;

  // Per-topic batch managers (one per topic for isolation)
  std::unordered_map<std::string, std::unique_ptr<core::BatchManager>> batch_managers_;

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
  // State
  // =========================================================================
  std::atomic<bool> recording_{false};
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
