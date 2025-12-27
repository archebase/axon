#ifndef AXON_RECORDER_WORKER_THREAD_POOL_HPP
#define AXON_RECORDER_WORKER_THREAD_POOL_HPP

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "spsc_queue.hpp"

namespace axon {
namespace recorder {

/**
 * Message item for the lock-free queue.
 * Uses move-only semantics for zero-copy transfer.
 */
struct MessageItem {
  int64_t timestamp_ns = 0;
  std::vector<uint8_t> raw_data;  // Owned raw message bytes

  MessageItem() = default;

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
 * Per-topic statistics snapshot (copyable, for returning from functions).
 */
struct TopicStatsSnapshot {
  uint64_t received = 0;
  uint64_t dropped = 0;
  uint64_t written = 0;
  uint32_t sequence = 0;
};

/**
 * Per-topic statistics (internal, atomic for thread-safety).
 */
struct WorkerTopicStats {
  std::atomic<uint64_t> received{0};
  std::atomic<uint64_t> dropped{0};
  std::atomic<uint64_t> written{0};
  std::atomic<uint32_t> sequence{0};

  /// Create a copyable snapshot of the current values
  TopicStatsSnapshot snapshot() const {
    TopicStatsSnapshot s;
    s.received = received.load(std::memory_order_relaxed);
    s.dropped = dropped.load(std::memory_order_relaxed);
    s.written = written.load(std::memory_order_relaxed);
    s.sequence = sequence.load(std::memory_order_relaxed);
    return s;
  }
};

/**
 * WorkerThreadPool manages worker threads that process messages from per-topic queues.
 *
 * This class is responsible for:
 * - Managing per-topic lock-free SPSC queues
 * - Running worker threads that drain queues
 * - Tracking per-topic and aggregate statistics
 * - Graceful shutdown with queue draining
 *
 * Extracted from RecorderNode to follow Single Responsibility Principle.
 *
 * Threading Model:
 * - Each topic has a dedicated SPSC queue
 * - Worker threads are assigned to topics (can be 1:1 or N:M)
 * - Lock-free queues ensure minimal contention
 *
 * Thread Safety:
 * - try_push() is thread-safe from ROS callback threads
 * - Statistics are atomic and can be read from any thread
 * - start()/stop() should be called from a single control thread
 */
class WorkerThreadPool {
public:
  /**
   * Callback type for processing messages.
   * Called by worker threads when messages are dequeued.
   *
   * Parameters:
   * - topic: The topic name
   * - timestamp_ns: Message timestamp
   * - data: Serialized message data (pointer to queue item, valid until callback returns)
   * - data_size: Size of message data
   * - sequence: Message sequence number for this topic
   *
   * Returns: true if message was processed successfully
   */
  using MessageHandler = std::function<bool(
    const std::string& topic, int64_t timestamp_ns, const uint8_t* data, size_t data_size,
    uint32_t sequence
  )>;

  /**
   * Configuration for the thread pool.
   */
  struct Config {
    /// Queue capacity per topic (default: 4096 messages)
    size_t queue_capacity_per_topic;

    /// Worker thread sleep time when queue is empty (microseconds)
    int worker_idle_sleep_us;

    /// Whether to use adaptive backoff when idle
    bool use_adaptive_backoff;

    /// Default constructor with default values
    Config()
        : queue_capacity_per_topic(4096)
        , worker_idle_sleep_us(50)
        , use_adaptive_backoff(true) {}
  };

  explicit WorkerThreadPool(const Config& config = Config());
  ~WorkerThreadPool();

  // Non-copyable, non-movable
  WorkerThreadPool(const WorkerThreadPool&) = delete;
  WorkerThreadPool& operator=(const WorkerThreadPool&) = delete;
  WorkerThreadPool(WorkerThreadPool&&) = delete;
  WorkerThreadPool& operator=(WorkerThreadPool&&) = delete;

  /**
   * Create a queue and worker for a topic.
   *
   * @param topic Topic name
   * @param handler Message handler callback
   * @return true if queue was created successfully
   */
  bool create_topic_worker(const std::string& topic, MessageHandler handler);

  /**
   * Remove a topic's queue and stop its worker.
   *
   * @param topic Topic name
   */
  void remove_topic_worker(const std::string& topic);

  /**
   * Try to push a message to a topic's queue.
   * Thread-safe: can be called from ROS callback threads.
   *
   * @param topic Topic name
   * @param item Message item (will be moved)
   * @return true if message was queued, false if queue full or topic not found
   */
  bool try_push(const std::string& topic, MessageItem&& item);

  /**
   * Start all worker threads.
   */
  void start();

  /**
   * Stop all worker threads.
   * Drains remaining messages before stopping.
   */
  void stop();

  /**
   * Check if workers are running.
   */
  bool is_running() const;

  /**
   * Pause message processing (workers continue running but don't process).
   */
  void pause();

  /**
   * Resume message processing.
   */
  void resume();

  /**
   * Check if processing is paused.
   */
  bool is_paused() const;

  /**
   * Get statistics for a specific topic.
   *
   * @param topic Topic name
   * @return Statistics snapshot, or zeros if topic not found
   */
  TopicStatsSnapshot get_topic_stats(const std::string& topic) const;

  /**
   * Get aggregate statistics across all topics.
   */
  struct AggregateStats {
    uint64_t total_received = 0;
    uint64_t total_dropped = 0;
    uint64_t total_written = 0;
  };
  AggregateStats get_aggregate_stats() const;

  /**
   * Get list of active topics.
   */
  std::vector<std::string> get_topics() const;

  /**
   * Get number of active topic workers.
   */
  size_t topic_count() const;

private:
  /**
   * Per-topic context with queue and worker thread.
   */
  struct TopicContext {
    std::unique_ptr<SPSCQueue<MessageItem>> queue;
    std::thread worker_thread;
    std::atomic<bool> running{false};
    MessageHandler handler;
    WorkerTopicStats stats;

    TopicContext() = default;

    // Non-copyable, non-movable
    TopicContext(const TopicContext&) = delete;
    TopicContext& operator=(const TopicContext&) = delete;
    TopicContext(TopicContext&&) = delete;
    TopicContext& operator=(TopicContext&&) = delete;
  };

  /**
   * Worker thread function for a specific topic.
   */
  void worker_thread_func(const std::string& topic);

  Config config_;
  std::atomic<bool> running_{false};
  std::atomic<bool> paused_{false};

  mutable std::mutex contexts_mutex_;
  std::unordered_map<std::string, std::unique_ptr<TopicContext>> topic_contexts_;
};

}  // namespace recorder
}  // namespace axon

#endif  // AXON_RECORDER_WORKER_THREAD_POOL_HPP

