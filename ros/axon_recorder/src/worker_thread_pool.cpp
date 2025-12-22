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
  std::lock_guard<std::mutex> lock(contexts_mutex_);

  // Check if topic already exists
  if (topic_contexts_.find(topic) != topic_contexts_.end()) {
    AXON_LOG_WARN("Topic worker already exists" << axon::logging::kv("topic", topic));
    return false;
  }

  // Create context
  auto context = std::make_unique<TopicContext>();
  context->queue = std::make_unique<core::SPSCQueue<MessageItem>>(config_.queue_capacity_per_topic);
  context->handler = std::move(handler);

  // If pool is already running, start the worker immediately
  if (running_.load(std::memory_order_acquire)) {
    context->running.store(true);
    context->worker_thread = std::thread(&WorkerThreadPool::worker_thread_func, this, topic);
  }

  topic_contexts_[topic] = std::move(context);
  return true;
}

void WorkerThreadPool::remove_topic_worker(const std::string& topic) {
  std::unique_ptr<TopicContext> context;

  {
    std::lock_guard<std::mutex> lock(contexts_mutex_);
    auto it = topic_contexts_.find(topic);
    if (it == topic_contexts_.end()) {
      return;
    }

    context = std::move(it->second);
    topic_contexts_.erase(it);
  }

  // Stop the worker thread outside the lock
  if (context) {
    context->running.store(false);
    if (context->worker_thread.joinable()) {
      context->worker_thread.join();
    }
  }
}

bool WorkerThreadPool::try_push(const std::string& topic, MessageItem&& item) {
  std::lock_guard<std::mutex> lock(contexts_mutex_);

  auto it = topic_contexts_.find(topic);
  if (it == topic_contexts_.end()) {
    return false;
  }

  auto& context = it->second;
  context->stats.received.fetch_add(1, std::memory_order_relaxed);

  if (context->queue->try_push(std::move(item))) {
    return true;
  }

  // Queue full - message dropped
  context->stats.dropped.fetch_add(1, std::memory_order_relaxed);
  return false;
}

void WorkerThreadPool::start() {
  if (running_.load(std::memory_order_acquire)) {
    return;
  }

  running_.store(true, std::memory_order_release);
  paused_.store(false, std::memory_order_release);

  std::lock_guard<std::mutex> lock(contexts_mutex_);

  for (auto& [topic, context] : topic_contexts_) {
    if (context->running.load()) {
      continue;
    }

    context->running.store(true);
    context->worker_thread = std::thread(&WorkerThreadPool::worker_thread_func, this, topic);
  }
}

void WorkerThreadPool::stop() {
  if (!running_.load(std::memory_order_acquire)) {
    return;
  }

  running_.store(false, std::memory_order_release);

  // Signal all workers to stop
  {
    std::lock_guard<std::mutex> lock(contexts_mutex_);
    for (auto& [topic, context] : topic_contexts_) {
      context->running.store(false);
    }
  }

  // Wait for all workers to finish (outside the lock)
  std::vector<std::thread*> threads_to_join;
  {
    std::lock_guard<std::mutex> lock(contexts_mutex_);
    for (auto& [topic, context] : topic_contexts_) {
      if (context->worker_thread.joinable()) {
        threads_to_join.push_back(&context->worker_thread);
      }
    }
  }

  for (auto* thread : threads_to_join) {
    thread->join();
  }
}

bool WorkerThreadPool::is_running() const {
  return running_.load(std::memory_order_acquire);
}

void WorkerThreadPool::pause() {
  paused_.store(true, std::memory_order_release);
}

void WorkerThreadPool::resume() {
  paused_.store(false, std::memory_order_release);
}

bool WorkerThreadPool::is_paused() const {
  return paused_.load(std::memory_order_acquire);
}

TopicStatsSnapshot WorkerThreadPool::get_topic_stats(const std::string& topic) const {
  std::lock_guard<std::mutex> lock(contexts_mutex_);

  auto it = topic_contexts_.find(topic);
  if (it == topic_contexts_.end()) {
    return TopicStatsSnapshot{};
  }

  return it->second->stats.snapshot();
}

WorkerThreadPool::AggregateStats WorkerThreadPool::get_aggregate_stats() const {
  std::lock_guard<std::mutex> lock(contexts_mutex_);

  AggregateStats aggregate;
  for (const auto& [topic, context] : topic_contexts_) {
    aggregate.total_received += context->stats.received.load(std::memory_order_relaxed);
    aggregate.total_dropped += context->stats.dropped.load(std::memory_order_relaxed);
    aggregate.total_written += context->stats.written.load(std::memory_order_relaxed);
  }

  return aggregate;
}

std::vector<std::string> WorkerThreadPool::get_topics() const {
  std::lock_guard<std::mutex> lock(contexts_mutex_);

  std::vector<std::string> topics;
  topics.reserve(topic_contexts_.size());

  for (const auto& [topic, context] : topic_contexts_) {
    topics.push_back(topic);
  }

  return topics;
}

size_t WorkerThreadPool::topic_count() const {
  std::lock_guard<std::mutex> lock(contexts_mutex_);
  return topic_contexts_.size();
}

void WorkerThreadPool::worker_thread_func(const std::string& topic) {
  TopicContext* context = nullptr;

  {
    std::lock_guard<std::mutex> lock(contexts_mutex_);
    auto it = topic_contexts_.find(topic);
    if (it == topic_contexts_.end()) {
      return;
    }
    context = it->second.get();
  }

  if (!context) {
    return;
  }

  MessageItem item;
  size_t consecutive_empty = 0;

  while (context->running.load(std::memory_order_acquire)) {
    // Check if paused
    if (paused_.load(std::memory_order_acquire)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
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
        }
        // Otherwise, tight spin for low latency
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
    } catch (...) {
      // Ignore exceptions during drain
    }
  }
}

}  // namespace recorder
}  // namespace axon

