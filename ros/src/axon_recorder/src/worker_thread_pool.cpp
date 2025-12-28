#include "worker_thread_pool.hpp"

#include <chrono>

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

bool WorkerThreadPool::create_topic_worker(const std::string& topic, MessageHandler handler) {
  std::unique_lock<std::shared_mutex> lock(contexts_mutex_);

  // Check if topic already exists
  if (topic_contexts_.find(topic) != topic_contexts_.end()) {
    AXON_LOG_WARN("Topic worker already exists" << axon::logging::kv("topic", topic));
    return false;
  }

  // Create context using shared_ptr
  auto context = std::make_shared<TopicContext>();
  context->queue = std::make_unique<SPSCQueue<MessageItem>>(config_.queue_capacity_per_topic);
  context->handler = std::move(handler);
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
  // Use shared_lock for concurrent read access - reduces lock contention
  std::shared_lock<std::shared_mutex> lock(contexts_mutex_);

  auto it = topic_contexts_.find(topic);
  if (it == topic_contexts_.end()) {
    return false;
  }

  // Get shared_ptr to context - safe even if context is removed after we release lock
  auto context = it->second;

  // Release lock before queue operations to minimize contention
  lock.unlock();

  context->stats.received.fetch_add(1, std::memory_order_relaxed);

  if (context->queue->try_push(std::move(item))) {
    return true;
  }

  // Queue full - message dropped
  context->stats.dropped.fetch_add(1, std::memory_order_relaxed);
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

  return it->second->stats.snapshot();
}

WorkerThreadPool::AggregateStats WorkerThreadPool::get_aggregate_stats() const {
  std::shared_lock<std::shared_mutex> lock(contexts_mutex_);

  AggregateStats aggregate;
  for (const auto& [topic, context] : topic_contexts_) {
    aggregate.total_received += context->stats.received.load(std::memory_order_relaxed);
    aggregate.total_dropped += context->stats.dropped.load(std::memory_order_relaxed);
    aggregate.total_written += context->stats.written.load(std::memory_order_relaxed);
  }

  return aggregate;
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
  MessageItem item;
  size_t consecutive_empty = 0;

  while (context->running.load(std::memory_order_acquire)) {
    // Check if paused - use condition variable for efficient waiting
    if (paused_.load(std::memory_order_acquire)) {
      std::unique_lock<std::mutex> pause_lock(pause_mutex_);
      // Re-check condition after acquiring lock to avoid spurious wakeups
      pause_cv_.wait(pause_lock, [this, &context]() {
        return !paused_.load(std::memory_order_acquire) ||
               !context->running.load(std::memory_order_acquire);
      });
      continue;
    }

    // Try to pop from lock-free queue
    if (context->queue->try_pop(item)) {
      consecutive_empty = 0;

      // Get sequence number
      uint32_t seq = context->stats.sequence.fetch_add(1, std::memory_order_relaxed);

      // Call the message handler
      bool success = false;
      try {
        success = context->handler(topic, item.timestamp_ns, item.raw_data.data(),
                                   item.raw_data.size(), seq);
      } catch (const std::exception& e) {
        AXON_LOG_ERROR("Worker exception" << axon::logging::kv("topic", topic) << axon::logging::kv("error", e.what()));
      } catch (...) {
        AXON_LOG_ERROR("Worker unknown exception" << axon::logging::kv("topic", topic));
      }

      if (success) {
        context->stats.written.fetch_add(1, std::memory_order_relaxed);
      }

    } else {
      // Queue empty - use adaptive backoff
      ++consecutive_empty;

      if (config_.use_adaptive_backoff) {
        if (consecutive_empty > 100) {
          // Long idle - sleep longer to save CPU
          std::this_thread::sleep_for(std::chrono::microseconds(500));
        } else if (consecutive_empty > 10) {
          // Brief idle - short sleep
          std::this_thread::sleep_for(std::chrono::microseconds(config_.worker_idle_sleep_us));
        } else {
          // Very brief idle - yield to reduce CPU spinning
          std::this_thread::yield();
        }
      } else {
        std::this_thread::sleep_for(std::chrono::microseconds(config_.worker_idle_sleep_us));
      }
    }
  }

  // Drain remaining messages before exiting
  while (context->queue->try_pop(item)) {
    uint32_t seq = context->stats.sequence.fetch_add(1, std::memory_order_relaxed);
    try {
      if (context->handler(topic, item.timestamp_ns, item.raw_data.data(), item.raw_data.size(),
                           seq)) {
        context->stats.written.fetch_add(1, std::memory_order_relaxed);
      }
    } catch (const std::exception& e) {
      AXON_LOG_WARN("Exception during drain" << axon::logging::kv("topic", topic) << axon::logging::kv("error", e.what()));
    } catch (...) {
      AXON_LOG_WARN("Unknown exception during drain" << axon::logging::kv("topic", topic));
    }
  }
}

}  // namespace recorder
}  // namespace axon
