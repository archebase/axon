// Copyright (c) 2026 ArcheBase
// Axon is licensed under Mulan PSL v2.
// You can use this software according to the terms and conditions of the Mulan PSL v2.
// You may obtain a copy of Mulan PSL v2 at:
//          http://license.coscl.org.cn/MulanPSL2
// THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
// EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
// MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
// See the Mulan PSL v2 for more details.

#ifndef AXON_SPSC_QUEUE_HPP
#define AXON_SPSC_QUEUE_HPP

#include <atomic>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <new>
#include <type_traits>
#include <utility>
#include <vector>

namespace axon {
namespace recorder {

/**
 * Lock-free Single-Producer Single-Consumer (SPSC) Queue
 *
 * This queue is optimized for the common pattern in recording systems where:
 * - One thread (ROS callback) produces messages
 * - One or more worker threads consume and process messages
 *
 * Key features:
 * - Lock-free for both push and pop operations
 * - Cache-line aligned to prevent false sharing
 * - Fixed capacity with bounded memory usage
 * - Move-only semantics for zero-copy message handling
 * - Supports movable-only types (perfect for ownership transfer)
 *
 * Memory ordering:
 * - Uses acquire-release semantics for thread synchronization
 * - Ensures proper visibility of written data between threads
 *
 * @tparam T Element type (must be move-constructible)
 */
template<typename T>
class SPSCQueue {
  static_assert(
    std::is_move_constructible<T>::value, "SPSCQueue requires move-constructible element type"
  );

public:
  /**
   * Create queue with specified capacity
   * @param capacity Maximum number of elements (will be rounded up to power of 2)
   */
  explicit SPSCQueue(size_t capacity)
      : capacity_(next_power_of_2(capacity))
      , mask_(capacity_ - 1)
      , head_(0)
      , tail_(0) {
    // Allocate storage with cache-line alignment
    storage_ = static_cast<Storage*>(aligned_alloc(CACHE_LINE_SIZE, sizeof(Storage) * capacity_));

    if (!storage_) {
      throw std::bad_alloc();
    }

    // Initialize storage (but don't construct T objects yet)
    for (size_t i = 0; i < capacity_; ++i) {
      storage_[i].occupied.store(false, std::memory_order_relaxed);
    }
  }

  ~SPSCQueue() {
    // Destroy any remaining elements
    size_t head = head_.load(std::memory_order_relaxed);
    size_t tail = tail_.load(std::memory_order_relaxed);

    while (head != tail) {
      size_t idx = head & mask_;
      if (storage_[idx].occupied.load(std::memory_order_relaxed)) {
        reinterpret_cast<T*>(&storage_[idx].data)->~T();
      }
      ++head;
    }

    free(storage_);
  }

  // Non-copyable, non-movable (contains raw pointers)
  SPSCQueue(const SPSCQueue&) = delete;
  SPSCQueue& operator=(const SPSCQueue&) = delete;
  SPSCQueue(SPSCQueue&&) = delete;
  SPSCQueue& operator=(SPSCQueue&&) = delete;

  /**
   * Try to push an element (producer thread only)
   *
   * @param value Element to push (will be moved)
   * @return true if successful, false if queue is full
   */
  bool try_push(T&& value) {
    const size_t tail = tail_.load(std::memory_order_relaxed);
    const size_t head = head_.load(std::memory_order_acquire);

    // Check if full
    if (tail - head >= capacity_) {
      return false;
    }

    const size_t idx = tail & mask_;

    // Construct element in-place
    new (&storage_[idx].data) T(std::move(value));
    storage_[idx].occupied.store(true, std::memory_order_release);

    // Publish the write
    tail_.store(tail + 1, std::memory_order_release);

    return true;
  }

  /**
   * Try to pop an element (consumer thread only)
   *
   * @param value Output parameter for popped element
   * @return true if successful, false if queue is empty
   */
  bool try_pop(T& value) {
    const size_t head = head_.load(std::memory_order_relaxed);
    const size_t tail = tail_.load(std::memory_order_acquire);

    // Check if empty
    if (head >= tail) {
      return false;
    }

    const size_t idx = head & mask_;

    // Wait for element to be fully written
    if (!storage_[idx].occupied.load(std::memory_order_acquire)) {
      return false;
    }

    // Move element out
    value = std::move(*reinterpret_cast<T*>(&storage_[idx].data));

    // Destroy the moved-from object
    reinterpret_cast<T*>(&storage_[idx].data)->~T();
    storage_[idx].occupied.store(false, std::memory_order_release);

    // Publish the read
    head_.store(head + 1, std::memory_order_release);

    return true;
  }

  /**
   * Check if queue is empty (approximate, for monitoring)
   */
  bool empty() const {
    return size() == 0;
  }

  /**
   * Get approximate size (for monitoring, not exact during concurrent access)
   */
  size_t size() const {
    const size_t head = head_.load(std::memory_order_acquire);
    const size_t tail = tail_.load(std::memory_order_acquire);
    return (tail >= head) ? (tail - head) : 0;
  }

  /**
   * Get capacity
   */
  size_t capacity() const {
    return capacity_;
  }

private:
  static constexpr size_t CACHE_LINE_SIZE = 64;

  /**
   * Round up to next power of 2 for efficient modulo via masking
   */
  static size_t next_power_of_2(size_t n) {
    if (n == 0) return 1;
    --n;
    n |= n >> 1;
    n |= n >> 2;
    n |= n >> 4;
    n |= n >> 8;
    n |= n >> 16;
    n |= n >> 32;
    return n + 1;
  }

  /**
   * Storage slot with cache-line padding
   */
  struct alignas(CACHE_LINE_SIZE) Storage {
    std::atomic<bool> occupied;
    typename std::aligned_storage<sizeof(T), alignof(T)>::type data;
  };

  const size_t capacity_;
  const size_t mask_;

  // Producer state (written by producer, read by consumer)
  alignas(CACHE_LINE_SIZE) std::atomic<size_t> tail_;

  // Consumer state (written by consumer, read by producer)
  alignas(CACHE_LINE_SIZE) std::atomic<size_t> head_;

  Storage* storage_;
};

/**
 * Multi-Producer Single-Consumer (MPSC) Queue
 *
 * Lock-free queue supporting multiple producers and a single consumer.
 * Built on top of multiple SPSC queues with round-robin distribution.
 *
 * Use case: Multiple ROS callback threads writing to a single worker thread.
 *
 * @tparam T Element type (must be move-constructible)
 */
template<typename T>
class MPSCQueue {
public:
  /**
   * Create MPSC queue with specified capacity
   * @param capacity_per_producer Capacity for each producer slot
   * @param num_producers Maximum number of producer threads
   */
  explicit MPSCQueue(size_t capacity_per_producer, size_t num_producers = 8)
      : num_queues_(num_producers)
      , queues_()
      , current_push_queue_(0)
      , current_pop_queue_(0) {
    queues_.reserve(num_producers);
    for (size_t i = 0; i < num_producers; ++i) {
      queues_.push_back(std::make_unique<SPSCQueue<T>>(capacity_per_producer));
    }
  }

  /**
   * Try to push an element (any producer thread)
   * Uses thread-local queue assignment to maintain SPSC guarantee.
   * Each producer thread is pinned to a dedicated queue on first call.
   */
  bool try_push(T&& value) {
    // Thread-local queue index - each thread gets assigned to one queue on first use
    // This maintains the single-producer guarantee for each underlying SPSC queue
    thread_local size_t my_queue =
      current_push_queue_.fetch_add(1, std::memory_order_relaxed) % num_queues_;

    // Only use assigned queue - maintains SPSC contract (no concurrent producers)
    return queues_[my_queue]->try_push(std::move(value));
  }

  /**
   * Try to pop an element (single consumer thread)
   * Round-robins through queues for fairness.
   */
  bool try_pop(T& value) {
    size_t start = current_pop_queue_.load(std::memory_order_relaxed);

    for (size_t i = 0; i < num_queues_; ++i) {
      size_t idx = (start + i) % num_queues_;
      if (queues_[idx]->try_pop(value)) {
        current_pop_queue_.store((idx + 1) % num_queues_, std::memory_order_relaxed);
        return true;
      }
    }

    return false;
  }

  /**
   * Check if all queues are empty
   */
  bool empty() const {
    for (const auto& q : queues_) {
      if (!q->empty()) return false;
    }
    return true;
  }

  /**
   * Get total approximate size
   */
  size_t size() const {
    size_t total = 0;
    for (const auto& q : queues_) {
      total += q->size();
    }
    return total;
  }

private:
  size_t num_queues_;
  std::vector<std::unique_ptr<SPSCQueue<T>>> queues_;
  std::atomic<size_t> current_push_queue_;
  std::atomic<size_t> current_pop_queue_;
};

}  // namespace recorder
}  // namespace axon

#endif  // AXON_SPSC_QUEUE_HPP
