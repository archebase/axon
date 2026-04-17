// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_RECORDER_WORKER_THREAD_POOL_HPP
#define AXON_RECORDER_WORKER_THREAD_POOL_HPP

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <buffer_pool.hpp>

#include "spsc_queue.hpp"

namespace axon {
namespace recorder {

/**
 * Message item for the lock-free queue.
 * Uses move-only semantics for zero-copy transfer.
 *
 * Payload storage is a PooledBuffer (from core/axon_memory). This replaces
 * the previous std::vector<uint8_t>, eliminating the per-message
 * `operator new` on the hot path when the selected size class is already
 * warmed up in the pool.
 *
 * The legacy `std::vector<uint8_t>` constructor is kept for test convenience;
 * it delegates to a heap allocation via a detached PooledBuffer.
 */
struct MessageItem {
  int64_t timestamp_ns = 0;
  uint64_t publish_time_ns = 0;
  uint64_t receive_time_ns = 0;
  uint64_t enqueue_time_ns = 0;
  uint64_t dequeue_time_ns = 0;
  std::string message_type;             // ROS2 message type (e.g., "sensor_msgs::msg::Image")
  axon::memory::PooledBuffer raw_data;  // Owned raw message bytes (pooled)

  MessageItem() = default;

  // Convenience constructor used by tests: acquires a pool buffer and copies
  // the given bytes in. Prefer direct PooledBuffer assignment on the hot path.
  MessageItem(int64_t ts, const std::string& type, std::vector<uint8_t>&& data)
      : timestamp_ns(ts)
      , message_type(type) {
    raw_data = axon::memory::BufferPool::instance().acquire(data.empty() ? 1 : data.size());
    raw_data.assign(data.data(), data.size());
  }

  MessageItem(int64_t ts, uint64_t pub_time, uint64_t recv_time, const std::string& type,
              std::vector<uint8_t>&& data)
      : timestamp_ns(ts)
      , publish_time_ns(pub_time)
      , receive_time_ns(recv_time)
      , message_type(type) {
    raw_data = axon::memory::BufferPool::instance().acquire(data.empty() ? 1 : data.size());
    raw_data.assign(data.data(), data.size());
  }

  // Move-only for zero-copy (PooledBuffer is also noexcept-movable)
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
  uint64_t bytes_received = 0;
  uint64_t bytes_written = 0;
  uint32_t sequence = 0;
  size_t queue_depth = 0;
  size_t queue_capacity = 0;
};

/**
 * Per-topic statistics (internal, atomic for thread-safety).
 */
struct WorkerTopicStats {
  std::atomic<uint64_t> received{0};
  std::atomic<uint64_t> dropped{0};
  std::atomic<uint64_t> written{0};
  std::atomic<uint64_t> bytes_received{0};
  std::atomic<uint64_t> bytes_written{0};
  std::atomic<uint32_t> sequence{0};

  /// Create a copyable snapshot of the current values
  TopicStatsSnapshot snapshot() const {
    TopicStatsSnapshot s;
    s.received = received.load(std::memory_order_relaxed);
    s.dropped = dropped.load(std::memory_order_relaxed);
    s.written = written.load(std::memory_order_relaxed);
    s.bytes_received = bytes_received.load(std::memory_order_relaxed);
    s.bytes_written = bytes_written.load(std::memory_order_relaxed);
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
    const std::string& topic, const std::string& message_type, int64_t timestamp_ns,
    const uint8_t* data, size_t data_size, uint32_t sequence
  )>;

  /**
   * Callback type for processing messages with latency timing info.
   * Called by worker threads when messages are dequeued.
   *
   * Parameters:
   * - topic: The topic name
   * - timestamp_ns: Message timestamp
   * - data: Serialized message data (pointer to queue item, valid until callback returns)
   * - data_size: Size of message data
   * - sequence: Message sequence number for this topic
   * - publish_time_ns: Original message publish time (from ROS header)
   * - receive_time_ns: Time when plugin received the message
   * - enqueue_time_ns: Time when message was enqueued to SPSC queue
   * - dequeue_time_ns: Time when message was dequeued from SPSC queue
   *
   * Returns: true if message was processed successfully
   */
  using LatencyMessageHandler = std::function<bool(
    const std::string& topic, const std::string& message_type, int64_t timestamp_ns,
    const uint8_t* data, size_t data_size, uint32_t sequence, uint64_t publish_time_ns,
    uint64_t receive_time_ns, uint64_t enqueue_time_ns, uint64_t dequeue_time_ns
  )>;

  /**
   * Callback type for message drop notifications.
   * Called by try_push() when a message is dropped due to queue full.
   * Must be lightweight — invoked on the plugin callback thread (hot path).
   *
   * Parameters:
   * - topic: The topic name
   * - message_type: The ROS message type (e.g., "sensor_msgs::msg::Image")
   * - total_dropped: Total number of messages dropped for this topic so far
   */
  using DropCallback = std::function<void(
    const std::string& topic, const std::string& message_type, uint64_t total_dropped
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

  /**
   * Per-topic batching configuration.
   *
   * Controls how the worker thread groups messages before invoking the user
   * handler. Batching reduces per-message dispatch / mutex overhead and
   * enables reuse of a pre-sized scratch vector (preallocation strategy).
   *
   * - batch_size == 1: dispatch immediately, no batching (default, backward-compatible).
   * - batch_size  > 1 + flush_interval_ms > 0: accumulate up to batch_size items
   *   OR force-flush when the oldest buffered item has aged past flush_interval_ms.
   *
   * Note: dispatch still happens one-message-at-a-time from the batch; batching
   * is about amortizing the accumulate/clear/reserve cost and enabling future
   * batch-write APIs, not about changing per-message semantics.
   */
  struct BatchConfig {
    size_t batch_size;
    int flush_interval_ms;

    // Explicit default constructor: required because this struct is used as
    // a default argument for create_topic_worker() while still being a nested
    // type; NSDMI would not be available yet at that parse point.
    BatchConfig()
        : batch_size(1)
        , flush_interval_ms(100) {}

    BatchConfig(size_t size, int interval_ms)
        : batch_size(size)
        , flush_interval_ms(interval_ms) {}
  };

  explicit WorkerThreadPool(const Config& config = Config());
  ~WorkerThreadPool();

  /**
   * Set callback for message drop notifications.
   * Must be called before start(). Not thread-safe with try_push().
   */
  void set_drop_callback(DropCallback callback);

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
   * @param batch Batching configuration (default: immediate dispatch)
   * @return true if queue was created successfully
   */
  bool create_topic_worker(
    const std::string& topic, MessageHandler handler, BatchConfig batch = BatchConfig{}
  );

  /**
   * Create a queue and worker for a topic with latency timing support.
   *
   * @param topic Topic name
   * @param handler Latency message handler callback
   * @param batch Batching configuration (default: immediate dispatch)
   * @return true if queue was created successfully
   */
  bool create_topic_worker(
    const std::string& topic, LatencyMessageHandler handler, BatchConfig batch = BatchConfig{}
  );

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
    uint64_t total_bytes_received = 0;
    uint64_t total_bytes_written = 0;
  };
  AggregateStats get_aggregate_stats() const;

  /**
   * Get per-topic queue depths (for monitoring).
   * Returns a map from topic name to queue depth and capacity.
   */
  struct QueueDepthInfo {
    size_t depth = 0;
    size_t capacity = 0;
  };
  std::unordered_map<std::string, QueueDepthInfo> get_queue_depths() const;

  /**
   * Reset all statistics to zero.
   * Called after recording finishes to clear stats for the next session.
   */
  void reset_stats();

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
   * Uses shared_ptr to prevent use-after-free when worker threads access context.
   */
  struct TopicContext : public std::enable_shared_from_this<TopicContext> {
    std::unique_ptr<SPSCQueue<MessageItem>> queue;
    std::thread worker_thread;
    std::atomic<bool> running{false};
    MessageHandler handler;
    LatencyMessageHandler latency_handler;
    BatchConfig batch_config;  // Per-topic batching configuration
    WorkerTopicStats stats;
    std::string topic_name;  // Store topic name for logging
    std::mutex push_mutex;   // Serializes pushes to maintain SPSC single-producer guarantee

    TopicContext() = default;

    // Non-copyable, non-movable
    TopicContext(const TopicContext&) = delete;
    TopicContext& operator=(const TopicContext&) = delete;
    TopicContext(TopicContext&&) = delete;
    TopicContext& operator=(TopicContext&&) = delete;
  };

  /**
   * Worker thread function for a specific topic.
   * Takes shared_ptr to context to prevent use-after-free.
   */
  void worker_thread_func(std::shared_ptr<TopicContext> context);

  Config config_;
  DropCallback drop_callback_;
  std::atomic<bool> running_{false};
  std::atomic<bool> paused_{false};
  std::atomic<bool> stopping_{false};  // Prevent concurrent stop() calls

  // Condition variable for pause/resume signaling
  mutable std::mutex pause_mutex_;
  std::condition_variable pause_cv_;

  // Use shared_mutex to allow concurrent reads in try_push while preventing
  // modifications during iteration
  mutable std::shared_mutex contexts_mutex_;
  std::unordered_map<std::string, std::shared_ptr<TopicContext>> topic_contexts_;
};

}  // namespace recorder
}  // namespace axon

#endif  // AXON_RECORDER_WORKER_THREAD_POOL_HPP
