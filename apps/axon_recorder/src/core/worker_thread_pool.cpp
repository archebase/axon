// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "worker_thread_pool.hpp"

#include <chrono>

// Clock utilities
namespace {
inline uint64_t get_steady_clock_ns() {
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                 std::chrono::steady_clock::now().time_since_epoch()
  )
                                 .count());
}
}  // namespace

// Logging infrastructure
#define AXON_LOG_COMPONENT "worker_thread_pool"
#include <axon_log_macros.hpp>

namespace axon {
namespace recorder {

WorkerThreadPool::WorkerThreadPool(const Config& config)
    : config_(config) {}

WorkerThreadPool::~WorkerThreadPool() {
  stop();
}

void WorkerThreadPool::set_drop_callback(DropCallback callback) {
  drop_callback_ = std::move(callback);
}

namespace {
// Reject obviously invalid BatchConfig values; fall back to defaults.
WorkerThreadPool::BatchConfig normalize_batch_config(WorkerThreadPool::BatchConfig cfg) {
  if (cfg.batch_size == 0) {
    cfg.batch_size = 1;
  }
  if (cfg.flush_interval_ms < 0) {
    cfg.flush_interval_ms = 0;
  }
  return cfg;
}
}  // namespace

bool WorkerThreadPool::create_topic_worker(
  const std::string& topic, MessageHandler handler, BatchConfig batch
) {
  batch = normalize_batch_config(batch);
  std::unique_lock<std::shared_mutex> lock(contexts_mutex_);

  // Check if topic already exists
  auto it = topic_contexts_.find(topic);
  if (it != topic_contexts_.end()) {
    auto context = it->second;

    // If worker is already running, this is a no-op (idempotent)
    if (context->running.load(std::memory_order_acquire)) {
      AXON_LOG_DEBUG("Topic worker already running" << axon::logging::kv("topic", topic));
      return true;
    }

    // Update handler and restart the worker
    context->handler = std::move(handler);
    context->batch_config = batch;

    // Start the worker thread
    context->running.store(true, std::memory_order_release);
    context->worker_thread = std::thread(&WorkerThreadPool::worker_thread_func, this, context);

    AXON_LOG_INFO("Restarted topic worker" << axon::logging::kv("topic", topic));
    return true;
  }

  // Create context using shared_ptr
  auto context = std::make_shared<TopicContext>();
  context->queue = std::make_unique<SPSCQueue<MessageItem>>(config_.queue_capacity_per_topic);
  context->handler = std::move(handler);
  context->batch_config = batch;
  context->topic_name = topic;

  // If pool is already running, start the worker immediately
  if (running_.load(std::memory_order_acquire)) {
    context->running.store(true, std::memory_order_release);
    // Pass shared_ptr to worker thread to prevent use-after-free
    context->worker_thread = std::thread(&WorkerThreadPool::worker_thread_func, this, context);
  }

  topic_contexts_[topic] = context;
  return true;
}

bool WorkerThreadPool::create_topic_worker(
  const std::string& topic, LatencyMessageHandler handler, BatchConfig batch
) {
  batch = normalize_batch_config(batch);
  std::unique_lock<std::shared_mutex> lock(contexts_mutex_);

  auto it = topic_contexts_.find(topic);
  if (it != topic_contexts_.end()) {
    auto context = it->second;

    if (context->running.load(std::memory_order_acquire)) {
      AXON_LOG_DEBUG("Topic worker already running" << axon::logging::kv("topic", topic));
      return true;
    }

    context->latency_handler = std::move(handler);
    context->batch_config = batch;
    context->running.store(true, std::memory_order_release);
    context->worker_thread = std::thread(&WorkerThreadPool::worker_thread_func, this, context);

    AXON_LOG_INFO(
      "Restarted topic worker with latency tracking" << axon::logging::kv("topic", topic)
    );
    return true;
  }

  auto context = std::make_shared<TopicContext>();
  context->queue = std::make_unique<SPSCQueue<MessageItem>>(config_.queue_capacity_per_topic);
  context->latency_handler = std::move(handler);
  context->batch_config = batch;
  context->topic_name = topic;

  if (running_.load(std::memory_order_acquire)) {
    context->running.store(true, std::memory_order_release);
    context->worker_thread = std::thread(&WorkerThreadPool::worker_thread_func, this, context);
  }

  topic_contexts_[topic] = context;
  return true;
}

void WorkerThreadPool::remove_topic_worker(const std::string& topic) {
  std::shared_ptr<TopicContext> context;
  std::thread worker_thread;

  {
    std::unique_lock<std::shared_mutex> lock(contexts_mutex_);
    auto it = topic_contexts_.find(topic);
    if (it == topic_contexts_.end()) {
      return;
    }

    context = it->second;
    topic_contexts_.erase(it);
  }

  // Stop the worker thread outside the lock
  // The shared_ptr ensures context stays alive until thread completes
  if (context) {
    context->running.store(false, std::memory_order_release);

    // Wake up paused workers
    {
      std::lock_guard<std::mutex> pause_lock(pause_mutex_);
      pause_cv_.notify_all();
    }

    if (context->worker_thread.joinable()) {
      context->worker_thread.join();
    }
  }
}

bool WorkerThreadPool::try_push(const std::string& topic, MessageItem&& item) {
  // Use shared_lock for concurrent read access to topic_contexts_ map
  // This prevents race with remove_topic_worker erasing the topic
  std::shared_lock<std::shared_mutex> lock(contexts_mutex_);

  auto it = topic_contexts_.find(topic);
  if (it == topic_contexts_.end()) {
    return false;
  }

  auto context = it->second;

  // Per-topic mutex serializes pushes to maintain SPSC single-producer guarantee
  // Different topics can push concurrently, but same topic is serialized
  std::lock_guard<std::mutex> push_lock(context->push_mutex);

  context->stats.received.fetch_add(1, std::memory_order_relaxed);
  context->stats.bytes_received.fetch_add(item.raw_data.size(), std::memory_order_relaxed);

  item.enqueue_time_ns = get_steady_clock_ns();

  if (context->queue->try_push(std::move(item))) {
    return true;
  }

  // Queue full - message dropped
  // Note: item is still valid here — SPSCQueue::try_push() returns false
  // before moving the item when the queue is full.
  uint64_t total_dropped = context->stats.dropped.fetch_add(1, std::memory_order_relaxed) + 1;
  if (drop_callback_) {
    drop_callback_(topic, item.message_type, total_dropped);
  }
  return false;
}

void WorkerThreadPool::start() {
  // Prevent concurrent start
  bool expected = false;
  if (!running_.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
    return;  // Already running
  }

  paused_.store(false, std::memory_order_release);
  stopping_.store(false, std::memory_order_release);

  std::unique_lock<std::shared_mutex> lock(contexts_mutex_);

  for (auto& [topic, context] : topic_contexts_) {
    if (context->running.load(std::memory_order_acquire)) {
      continue;
    }

    context->running.store(true, std::memory_order_release);
    // Pass shared_ptr to worker thread to prevent use-after-free
    context->worker_thread = std::thread(&WorkerThreadPool::worker_thread_func, this, context);
  }
}

void WorkerThreadPool::stop() {
  // Prevent concurrent stop calls using atomic flag
  bool expected = false;
  if (!stopping_.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
    return;  // Another thread is already stopping
  }

  if (!running_.load(std::memory_order_acquire)) {
    stopping_.store(false, std::memory_order_release);
    return;
  }

  running_.store(false, std::memory_order_release);

  // Collect all contexts and signal them to stop under lock
  std::vector<std::shared_ptr<TopicContext>> contexts_to_join;
  {
    std::unique_lock<std::shared_mutex> lock(contexts_mutex_);
    contexts_to_join.reserve(topic_contexts_.size());

    for (auto& [topic, context] : topic_contexts_) {
      context->running.store(false, std::memory_order_release);
      contexts_to_join.push_back(context);
    }
  }

  // Wake up any paused workers
  {
    std::lock_guard<std::mutex> pause_lock(pause_mutex_);
    pause_cv_.notify_all();
  }

  // Wait for all workers to finish outside the lock
  // shared_ptr keeps contexts alive until join completes
  for (auto& context : contexts_to_join) {
    if (context->worker_thread.joinable()) {
      context->worker_thread.join();
    }
  }

  stopping_.store(false, std::memory_order_release);
}

size_t WorkerThreadPool::drain_remaining_sync() {
  // Precondition: workers have been joined (stop() already called) and no
  // producer is still active (the source plugin has been halted). Under
  // those assumptions we are the sole consumer and the sole reader of each
  // context's queue, so lock-free try_pop() is safe here.
  //
  // If this precondition is violated — e.g. drain runs while a worker is
  // still calling try_pop on the same queue — we'd have two concurrent
  // consumers on an SPSC queue, silently corrupting its indices. Guard
  // against that at the entry point rather than debugging the fallout.
  if (running_.load(std::memory_order_acquire)) {
    AXON_LOG_ERROR(
      "drain_remaining_sync called while worker pool is still running; "
      "this violates the SPSC single-consumer invariant. Caller must stop() first."
    );
    return 0;
  }

  // We hold a shared_lock because we're only iterating topic_contexts_ and
  // not mutating the map itself; concurrent get_topic_stats / get_topics
  // readers remain allowed.
  std::shared_lock<std::shared_mutex> lock(contexts_mutex_);

  size_t total_drained = 0;

  for (auto& [topic, context] : topic_contexts_) {
    if (!context || !context->queue) {
      continue;
    }

    size_t drained_here = 0;
    MessageItem item;
    while (context->queue->try_pop(item)) {
      // Residual items were enqueued during a prior shutdown window, possibly
      // far earlier than "now". Setting dequeue_time_ns to the current clock
      // would inject a huge spike into LatencyMonitor's queueing-delay
      // histogram that reflects shutdown timing rather than real backpressure,
      // triggering spurious alerts. Collapse the apparent queue time to zero
      // by mirroring enqueue_time_ns so downstream stats treat these items as
      // instantaneous pass-through.
      item.dequeue_time_ns = item.enqueue_time_ns;
      const uint32_t seq = context->stats.sequence.fetch_add(1, std::memory_order_relaxed);
      bool success = false;

      try {
        if (context->latency_handler) {
          success = context->latency_handler(
            topic,
            item.message_type,
            item.timestamp_ns,
            item.raw_data.data(),
            item.raw_data.size(),
            seq,
            item.publish_time_ns,
            item.receive_time_ns,
            item.enqueue_time_ns,
            item.dequeue_time_ns
          );
        } else if (context->handler) {
          success = context->handler(
            topic,
            item.message_type,
            item.timestamp_ns,
            item.raw_data.data(),
            item.raw_data.size(),
            seq
          );
        }
      } catch (const std::exception& e) {
        AXON_LOG_WARN(
          "Exception during sync drain" << axon::logging::kv("topic", topic)
                                        << axon::logging::kv("error", e.what())
        );
      } catch (...) {
        AXON_LOG_WARN("Unknown exception during sync drain" << axon::logging::kv("topic", topic));
      }

      if (success) {
        context->stats.written.fetch_add(1, std::memory_order_relaxed);
        context->stats.bytes_written.fetch_add(item.raw_data.size(), std::memory_order_relaxed);
      }

      ++drained_here;
      // `item` (and its raw_data PooledBuffer) is destroyed at loop-iteration
      // end, triggering any adopted external_release_ here — on the caller's
      // thread, while the producing plugin is still loaded.
    }

    if (drained_here > 0) {
      // Residual items indicate that worker_thread_func's own drain loop did
      // not flush this queue — typically because messages were enqueued
      // AFTER workers exited but BEFORE this drain ran (i.e., a producer
      // wasn't stopped before the worker pool). Warn so the regression is
      // visible in operations.
      AXON_LOG_WARN(
        "Drained residual items after worker stop" << axon::logging::kv("topic", topic)
                                                   << axon::logging::kv("count", drained_here)
      );
      total_drained += drained_here;
    }
  }

  return total_drained;
}

bool WorkerThreadPool::is_running() const {
  return running_.load(std::memory_order_acquire);
}

void WorkerThreadPool::pause() {
  paused_.store(true, std::memory_order_release);
}

void WorkerThreadPool::resume() {
  {
    std::lock_guard<std::mutex> lock(pause_mutex_);
    paused_.store(false, std::memory_order_release);
  }
  // Wake up all waiting workers
  pause_cv_.notify_all();
}

bool WorkerThreadPool::is_paused() const {
  return paused_.load(std::memory_order_acquire);
}

TopicStatsSnapshot WorkerThreadPool::get_topic_stats(const std::string& topic) const {
  std::shared_lock<std::shared_mutex> lock(contexts_mutex_);

  auto it = topic_contexts_.find(topic);
  if (it == topic_contexts_.end()) {
    return TopicStatsSnapshot{};
  }

  TopicStatsSnapshot snapshot = it->second->stats.snapshot();
  snapshot.queue_depth = it->second->queue->size();
  snapshot.queue_capacity = it->second->queue->capacity();
  return snapshot;
}

WorkerThreadPool::AggregateStats WorkerThreadPool::get_aggregate_stats() const {
  std::shared_lock<std::shared_mutex> lock(contexts_mutex_);

  AggregateStats aggregate;
  for (const auto& [topic, context] : topic_contexts_) {
    aggregate.total_received += context->stats.received.load(std::memory_order_relaxed);
    aggregate.total_dropped += context->stats.dropped.load(std::memory_order_relaxed);
    aggregate.total_written += context->stats.written.load(std::memory_order_relaxed);
    aggregate.total_bytes_received += context->stats.bytes_received.load(std::memory_order_relaxed);
    aggregate.total_bytes_written += context->stats.bytes_written.load(std::memory_order_relaxed);
  }

  return aggregate;
}

std::unordered_map<std::string, WorkerThreadPool::QueueDepthInfo>
WorkerThreadPool::get_queue_depths() const {
  std::shared_lock<std::shared_mutex> lock(contexts_mutex_);

  std::unordered_map<std::string, QueueDepthInfo> depths;
  for (const auto& [topic, context] : topic_contexts_) {
    QueueDepthInfo info;
    info.depth = context->queue->size();
    info.capacity = context->queue->capacity();
    depths[topic] = info;
  }

  return depths;
}

void WorkerThreadPool::reset_stats() {
  std::unique_lock<std::shared_mutex> lock(contexts_mutex_);

  for (auto& [topic, context] : topic_contexts_) {
    context->stats.received.store(0, std::memory_order_relaxed);
    context->stats.dropped.store(0, std::memory_order_relaxed);
    context->stats.written.store(0, std::memory_order_relaxed);
    context->stats.bytes_received.store(0, std::memory_order_relaxed);
    context->stats.bytes_written.store(0, std::memory_order_relaxed);
  }
}

std::vector<std::string> WorkerThreadPool::get_topics() const {
  std::shared_lock<std::shared_mutex> lock(contexts_mutex_);

  std::vector<std::string> topics;
  topics.reserve(topic_contexts_.size());

  for (const auto& [topic, context] : topic_contexts_) {
    topics.push_back(topic);
  }

  return topics;
}

size_t WorkerThreadPool::topic_count() const {
  std::shared_lock<std::shared_mutex> lock(contexts_mutex_);
  return topic_contexts_.size();
}

void WorkerThreadPool::worker_thread_func(std::shared_ptr<TopicContext> context) {
  // Context is kept alive by shared_ptr even if removed from map
  if (!context) {
    return;
  }

  const std::string& topic = context->topic_name;
  const BatchConfig batch_cfg = context->batch_config;

  // Reusable batch scratch buffer — allocated once, cleared between flushes.
  // Reserve up-front to avoid reallocation during steady state.
  std::vector<MessageItem> batch;
  batch.reserve(batch_cfg.batch_size);

  auto dispatch_one = [&](MessageItem& item) {
    item.dequeue_time_ns = get_steady_clock_ns();
    uint32_t seq = context->stats.sequence.fetch_add(1, std::memory_order_relaxed);
    bool success = false;
    try {
      if (context->latency_handler) {
        success = context->latency_handler(
          topic,
          item.message_type,
          item.timestamp_ns,
          item.raw_data.data(),
          item.raw_data.size(),
          seq,
          item.publish_time_ns,
          item.receive_time_ns,
          item.enqueue_time_ns,
          item.dequeue_time_ns
        );
      } else if (context->handler) {
        success = context->handler(
          topic,
          item.message_type,
          item.timestamp_ns,
          item.raw_data.data(),
          item.raw_data.size(),
          seq
        );
      }
    } catch (const std::exception& e) {
      AXON_LOG_ERROR(
        "Worker exception" << axon::logging::kv("topic", topic)
                           << axon::logging::kv("error", e.what())
      );
    } catch (...) {
      AXON_LOG_ERROR("Worker unknown exception" << axon::logging::kv("topic", topic));
    }

    if (success) {
      context->stats.written.fetch_add(1, std::memory_order_relaxed);
      context->stats.bytes_written.fetch_add(item.raw_data.size(), std::memory_order_relaxed);
    }
  };

  // Flush the accumulated batch, then clear (keep capacity for reuse).
  auto flush_batch = [&]() {
    for (auto& item : batch) {
      dispatch_one(item);
    }
    batch.clear();
  };

  size_t consecutive_empty = 0;
  auto last_flush_time = std::chrono::steady_clock::now();

  while (context->running.load(std::memory_order_acquire)) {
    // Check if paused - use condition variable for efficient waiting
    if (paused_.load(std::memory_order_acquire)) {
      // Drain anything already buffered before blocking on pause.
      if (!batch.empty()) {
        flush_batch();
        last_flush_time = std::chrono::steady_clock::now();
      }
      std::unique_lock<std::mutex> pause_lock(pause_mutex_);
      pause_cv_.wait(pause_lock, [this, &context]() {
        return !paused_.load(std::memory_order_acquire) ||
               !context->running.load(std::memory_order_acquire);
      });
      continue;
    }

    MessageItem item;
    if (context->queue->try_pop(item)) {
      consecutive_empty = 0;
      batch.emplace_back(std::move(item));

      // Size-triggered flush.
      if (batch.size() >= batch_cfg.batch_size) {
        flush_batch();
        last_flush_time = std::chrono::steady_clock::now();
      }
    } else {
      // Queue empty.
      // Time-triggered flush: if we have partial batch older than flush_interval_ms.
      if (!batch.empty() && batch_cfg.flush_interval_ms > 0) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed_ms =
          std::chrono::duration_cast<std::chrono::milliseconds>(now - last_flush_time).count();
        if (elapsed_ms >= batch_cfg.flush_interval_ms) {
          flush_batch();
          last_flush_time = now;
        }
      } else if (!batch.empty() && batch_cfg.flush_interval_ms == 0) {
        // No time budget: flush as soon as queue drains.
        flush_batch();
        last_flush_time = std::chrono::steady_clock::now();
      }

      ++consecutive_empty;

      if (config_.use_adaptive_backoff) {
        if (consecutive_empty > 100) {
          std::this_thread::sleep_for(std::chrono::microseconds(500));
        } else if (consecutive_empty > 10) {
          std::this_thread::sleep_for(std::chrono::microseconds(config_.worker_idle_sleep_us));
        } else {
          std::this_thread::yield();
        }
      } else {
        std::this_thread::sleep_for(std::chrono::microseconds(config_.worker_idle_sleep_us));
      }
    }
  }

  // Drain remaining messages before exiting: flush any buffered batch, then
  // consume anything left in the queue.
  flush_batch();
  MessageItem drain_item;
  while (context->queue->try_pop(drain_item)) {
    try {
      dispatch_one(drain_item);
    } catch (const std::exception& e) {
      AXON_LOG_WARN(
        "Exception during drain" << axon::logging::kv("topic", topic)
                                 << axon::logging::kv("error", e.what())
      );
    } catch (...) {
      AXON_LOG_WARN("Unknown exception during drain" << axon::logging::kv("topic", topic));
    }
  }
}

}  // namespace recorder
}  // namespace axon
