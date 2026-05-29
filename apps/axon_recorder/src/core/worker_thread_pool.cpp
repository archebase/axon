// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "worker_thread_pool.hpp"

#include <algorithm>
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

void WorkerThreadPool::set_latency_batch_handler(LatencyBatchMessageHandler callback) {
  latency_batch_handler_ = std::move(callback);
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

template<typename T>
void update_atomic_max(std::atomic<T>& target, T value) {
  T current = target.load(std::memory_order_relaxed);
  while (current < value && !target.compare_exchange_weak(
                              current, value, std::memory_order_relaxed, std::memory_order_relaxed
                            )) {
  }
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

  if (config_.use_writer_batching && latency_batch_handler_) {
    writer_running_.store(true, std::memory_order_release);
    writer_thread_ = std::thread(&WorkerThreadPool::writer_thread_func, this);
  } else if (config_.use_writer_batching && !latency_batch_handler_) {
    AXON_LOG_WARN("Writer batching requested but no batch handler was installed");
  }

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

  if (writer_running_.load(std::memory_order_acquire)) {
    writer_running_.store(false, std::memory_order_release);
    writer_queue_cv_.notify_all();
  }
  if (writer_thread_.joinable()) {
    writer_thread_.join();
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

WorkerThreadPool::WriterBatchStatsSnapshot WorkerThreadPool::get_writer_batch_stats() const {
  WriterBatchStatsSnapshot stats;
  stats.enabled = config_.use_writer_batching;
  stats.batches_written = writer_batches_written_.load(std::memory_order_relaxed);
  stats.messages_written = writer_messages_written_.load(std::memory_order_relaxed);
  stats.bytes_written = writer_bytes_written_.load(std::memory_order_relaxed);
  stats.partial_failures = writer_partial_failures_.load(std::memory_order_relaxed);
  stats.queue_overflows = writer_queue_overflows_.load(std::memory_order_relaxed);
  stats.flush_by_messages = writer_flush_by_messages_.load(std::memory_order_relaxed);
  stats.flush_by_bytes = writer_flush_by_bytes_.load(std::memory_order_relaxed);
  stats.flush_by_time = writer_flush_by_time_.load(std::memory_order_relaxed);
  stats.flush_by_stop = writer_flush_by_stop_.load(std::memory_order_relaxed);
  stats.total_write_duration_ns = writer_total_write_duration_ns_.load(std::memory_order_relaxed);
  stats.max_write_duration_ns = writer_max_write_duration_ns_.load(std::memory_order_relaxed);
  stats.last_batch_messages = writer_last_batch_messages_.load(std::memory_order_relaxed);
  stats.last_batch_bytes = writer_last_batch_bytes_.load(std::memory_order_relaxed);
  stats.max_batch_messages = writer_max_batch_messages_.load(std::memory_order_relaxed);
  stats.max_batch_bytes = writer_max_batch_bytes_.load(std::memory_order_relaxed);
  stats.queue_capacity = std::max<size_t>(1, config_.writer_queue_capacity);
  stats.max_queue_depth = writer_max_queue_depth_.load(std::memory_order_relaxed);
  {
    std::lock_guard<std::mutex> lock(writer_queue_mutex_);
    stats.queue_depth = writer_queue_.size();
  }
  return stats;
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
  reset_writer_batch_stats();
}

void WorkerThreadPool::reset_writer_batch_stats() {
  writer_batches_written_.store(0, std::memory_order_relaxed);
  writer_messages_written_.store(0, std::memory_order_relaxed);
  writer_bytes_written_.store(0, std::memory_order_relaxed);
  writer_partial_failures_.store(0, std::memory_order_relaxed);
  writer_queue_overflows_.store(0, std::memory_order_relaxed);
  writer_flush_by_messages_.store(0, std::memory_order_relaxed);
  writer_flush_by_bytes_.store(0, std::memory_order_relaxed);
  writer_flush_by_time_.store(0, std::memory_order_relaxed);
  writer_flush_by_stop_.store(0, std::memory_order_relaxed);
  writer_total_write_duration_ns_.store(0, std::memory_order_relaxed);
  writer_max_write_duration_ns_.store(0, std::memory_order_relaxed);
  writer_last_batch_messages_.store(0, std::memory_order_relaxed);
  writer_last_batch_bytes_.store(0, std::memory_order_relaxed);
  writer_max_batch_messages_.store(0, std::memory_order_relaxed);
  writer_max_batch_bytes_.store(0, std::memory_order_relaxed);
  writer_max_queue_depth_.store(0, std::memory_order_relaxed);
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

bool WorkerThreadPool::submit_to_writer_batcher(
  const std::shared_ptr<TopicContext>& context, MessageItem&& item, uint32_t sequence
) {
  if (!context) {
    return false;
  }

  const size_t queue_capacity = std::max<size_t>(1, config_.writer_queue_capacity);
  size_t queue_depth = 0;
  {
    std::unique_lock<std::mutex> lock(writer_queue_mutex_);

    if (!writer_running_.load(std::memory_order_acquire)) {
      return false;
    }

    queue_depth = writer_queue_.size();
    if (queue_depth >= queue_capacity) {
      writer_queue_overflows_.fetch_add(1, std::memory_order_relaxed);
      queue_depth = writer_queue_.size();
    } else {
      WriterBatchItem queued;
      queued.context = context;
      queued.item = std::move(item);
      queued.sequence = sequence;
      writer_queue_.push_back(std::move(queued));
      update_atomic_max(writer_max_queue_depth_, writer_queue_.size());
      lock.unlock();
      writer_queue_cv_.notify_one();
      return true;
    }
  }

  const uint64_t total_dropped = context->stats.dropped.fetch_add(1, std::memory_order_relaxed) + 1;
  AXON_LOG_ERROR(
    "Writer queue full, dropping message" << axon::logging::kv("topic", context->topic_name)
                                          << axon::logging::kv("queue_depth", queue_depth)
                                          << axon::logging::kv("queue_capacity", queue_capacity)
                                          << axon::logging::kv("topic_total_dropped", total_dropped)
  );
  if (drop_callback_) {
    drop_callback_(context->topic_name, item.message_type, total_dropped);
  }
  writer_queue_cv_.notify_one();
  return false;
}

void WorkerThreadPool::flush_writer_batch(
  std::vector<WriterBatchItem>& batch, WriterFlushReason reason
) {
  if (batch.empty()) {
    return;
  }

  size_t batch_bytes = 0;
  for (const auto& queued : batch) {
    batch_bytes += queued.item.raw_data.size();
  }

  std::vector<BatchMessageView> views;
  views.reserve(batch.size());
  for (const auto& queued : batch) {
    if (!queued.context) {
      continue;
    }
    BatchMessageView view;
    view.topic = &queued.context->topic_name;
    view.message_type = &queued.item.message_type;
    view.timestamp_ns = queued.item.timestamp_ns;
    view.data = queued.item.raw_data.data();
    view.data_size = queued.item.raw_data.size();
    view.sequence = queued.sequence;
    view.publish_time_ns = queued.item.publish_time_ns;
    view.receive_time_ns = queued.item.receive_time_ns;
    view.enqueue_time_ns = queued.item.enqueue_time_ns;
    view.dequeue_time_ns = queued.item.dequeue_time_ns;
    views.push_back(view);
  }

  size_t written_count = 0;
  const uint64_t write_start_ns = get_steady_clock_ns();
  try {
    if (latency_batch_handler_) {
      written_count = latency_batch_handler_(views);
    }
  } catch (const std::exception& e) {
    AXON_LOG_ERROR("Writer batch exception" << axon::logging::kv("error", e.what()));
  } catch (...) {
    AXON_LOG_ERROR("Writer batch unknown exception");
  }
  const uint64_t write_duration_ns = get_steady_clock_ns() - write_start_ns;

  written_count = std::min(written_count, batch.size());
  writer_batches_written_.fetch_add(1, std::memory_order_relaxed);
  writer_messages_written_.fetch_add(written_count, std::memory_order_relaxed);
  writer_total_write_duration_ns_.fetch_add(write_duration_ns, std::memory_order_relaxed);
  writer_last_batch_messages_.store(batch.size(), std::memory_order_relaxed);
  writer_last_batch_bytes_.store(batch_bytes, std::memory_order_relaxed);
  update_atomic_max(writer_max_write_duration_ns_, write_duration_ns);
  update_atomic_max(writer_max_batch_messages_, batch.size());
  update_atomic_max(writer_max_batch_bytes_, batch_bytes);

  switch (reason) {
    case WriterFlushReason::Messages:
      writer_flush_by_messages_.fetch_add(1, std::memory_order_relaxed);
      break;
    case WriterFlushReason::Bytes:
      writer_flush_by_bytes_.fetch_add(1, std::memory_order_relaxed);
      break;
    case WriterFlushReason::Time:
      writer_flush_by_time_.fetch_add(1, std::memory_order_relaxed);
      break;
    case WriterFlushReason::Stop:
      writer_flush_by_stop_.fetch_add(1, std::memory_order_relaxed);
      break;
  }

  for (size_t i = 0; i < written_count; ++i) {
    auto& queued = batch[i];
    if (!queued.context) {
      continue;
    }
    queued.context->stats.written.fetch_add(1, std::memory_order_relaxed);
    const size_t item_bytes = queued.item.raw_data.size();
    queued.context->stats.bytes_written.fetch_add(item_bytes, std::memory_order_relaxed);
    writer_bytes_written_.fetch_add(item_bytes, std::memory_order_relaxed);
  }

  if (written_count < batch.size()) {
    writer_partial_failures_.fetch_add(1, std::memory_order_relaxed);
    AXON_LOG_WARN(
      "Writer batch partially written" << axon::logging::kv("written", written_count)
                                       << axon::logging::kv("batch_size", batch.size())
    );
  }

  batch.clear();
}

void WorkerThreadPool::writer_thread_func() {
  const size_t max_messages = std::max<size_t>(1, config_.writer_batch_max_messages);
  const size_t max_bytes = config_.writer_batch_max_bytes;
  const auto flush_interval =
    std::chrono::milliseconds(std::max(0, config_.writer_batch_flush_interval_ms));

  std::vector<WriterBatchItem> batch;
  batch.reserve(max_messages);
  size_t batch_bytes = 0;
  auto first_item_time = std::chrono::steady_clock::now();

  auto can_take = [&](const WriterBatchItem& item) {
    if (batch.empty()) {
      return true;
    }
    if (batch.size() >= max_messages) {
      return false;
    }
    return max_bytes == 0 || batch_bytes + item.item.raw_data.size() <= max_bytes;
  };

  auto take_available = [&]() {
    bool took = false;
    while (!writer_queue_.empty() && can_take(writer_queue_.front())) {
      if (batch.empty()) {
        first_item_time = std::chrono::steady_clock::now();
      }
      batch_bytes += writer_queue_.front().item.raw_data.size();
      batch.push_back(std::move(writer_queue_.front()));
      writer_queue_.pop_front();
      took = true;
    }
  };

  while (true) {
    bool should_flush = false;
    WriterFlushReason flush_reason = WriterFlushReason::Time;
    {
      std::unique_lock<std::mutex> lock(writer_queue_mutex_);
      if (!writer_running_.load(std::memory_order_acquire) && writer_queue_.empty() &&
          batch.empty()) {
        break;
      }

      if (batch.empty()) {
        writer_queue_cv_.wait(lock, [this]() {
          return !writer_running_.load(std::memory_order_acquire) || !writer_queue_.empty();
        });
        take_available();
      } else if (batch.size() < max_messages && (max_bytes == 0 || batch_bytes < max_bytes) &&
                 writer_running_.load(std::memory_order_acquire)) {
        const auto deadline = first_item_time + flush_interval;
        if (flush_interval.count() > 0 && writer_queue_.empty()) {
          writer_queue_cv_.wait_until(lock, deadline, [this]() {
            return !writer_running_.load(std::memory_order_acquire) || !writer_queue_.empty();
          });
        }
        take_available();
      }

      const auto now = std::chrono::steady_clock::now();
      if (!batch.empty()) {
        if (!writer_running_.load(std::memory_order_acquire)) {
          should_flush = true;
          flush_reason = WriterFlushReason::Stop;
        } else if (batch.size() >= max_messages) {
          should_flush = true;
          flush_reason = WriterFlushReason::Messages;
        } else if ((max_bytes > 0 && batch_bytes >= max_bytes) ||
                   (!writer_queue_.empty() && !can_take(writer_queue_.front()))) {
          should_flush = true;
          flush_reason = WriterFlushReason::Bytes;
        } else if (flush_interval.count() == 0 || now - first_item_time >= flush_interval) {
          should_flush = true;
          flush_reason = WriterFlushReason::Time;
        }
      }
    }

    if (should_flush) {
      flush_writer_batch(batch, flush_reason);
      batch_bytes = 0;
    }
  }
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
      if (config_.use_writer_batching && latency_batch_handler_ && context->latency_handler) {
        success = submit_to_writer_batcher(context, std::move(item), seq);
      } else if (context->latency_handler) {
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

    if (success &&
        !(config_.use_writer_batching && latency_batch_handler_ && context->latency_handler)) {
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
