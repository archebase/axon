// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_UPLOAD_QUEUE_HPP
#define AXON_UPLOAD_QUEUE_HPP

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <vector>

namespace axon {
namespace uploader {

/**
 * Item to be uploaded to S3
 */
struct UploadItem {
  std::string mcap_path;        // Local MCAP file path
  std::string json_path;        // Local sidecar JSON path
  std::string s3_key_prefix;    // S3 key prefix (factory/device/date)
  std::string task_id;          // Task identifier
  std::string factory_id;       // Factory identifier
  std::string device_id;        // Device identifier
  std::string checksum_sha256;  // Pre-computed checksum from recorder
  uint64_t file_size_bytes;     // File size for progress tracking
  int retry_count = 0;          // Number of retry attempts
  std::chrono::steady_clock::time_point created_at;
  std::chrono::steady_clock::time_point next_retry_at;

  // Default constructor
  UploadItem()
      : file_size_bytes(0)
      , retry_count(0)
      , created_at(std::chrono::steady_clock::now()) {}

  // Convenience constructor
  UploadItem(
    const std::string& mcap, const std::string& json, const std::string& task,
    const std::string& factory, const std::string& device, const std::string& checksum,
    uint64_t size = 0
  )
      : mcap_path(mcap)
      , json_path(json)
      , task_id(task)
      , factory_id(factory)
      , device_id(device)
      , checksum_sha256(checksum)
      , file_size_bytes(size)
      , retry_count(0)
      , created_at(std::chrono::steady_clock::now()) {}
};

/**
 * Comparison functor for retry queue (min-heap by next_retry_at)
 */
struct RetryItemComparator {
  bool operator()(const UploadItem& a, const UploadItem& b) const {
    // Min-heap: earlier retry time has higher priority
    return a.next_retry_at > b.next_retry_at;
  }
};

/**
 * Thread-safe upload queue with retry support
 *
 * This queue supports:
 * - Single producer (recorder) adding items
 * - Multiple consumers (upload workers) dequeuing items
 * - Retry queue with delayed processing
 *
 * The queue uses mutex-based synchronization for simplicity and correctness.
 * For higher throughput, a lock-free SPSC queue could be used for the main queue.
 */
class UploadQueue {
public:
  /**
   * Create an upload queue with specified capacity
   *
   * @param capacity Maximum number of items in the main queue (0 = unlimited)
   */
  explicit UploadQueue(size_t capacity = 0);
  ~UploadQueue();

  // Non-copyable, non-movable
  UploadQueue(const UploadQueue&) = delete;
  UploadQueue& operator=(const UploadQueue&) = delete;
  UploadQueue(UploadQueue&&) = delete;
  UploadQueue& operator=(UploadQueue&&) = delete;

  /**
   * Add an item to the queue
   *
   * Thread-safe. Called by the recorder after finalization.
   *
   * @param item Upload item to enqueue
   * @return true if enqueued, false if queue is full
   */
  bool enqueue(UploadItem item);

  /**
   * Remove and return an item from the queue
   *
   * Thread-safe. Called by upload workers.
   * Blocks until an item is available or shutdown is requested.
   *
   * @return Item if available, std::nullopt if queue is shutting down
   */
  std::optional<UploadItem> dequeue();

  /**
   * Remove and return an item with timeout
   *
   * @param timeout Maximum time to wait for an item
   * @return Item if available within timeout, std::nullopt otherwise
   */
  std::optional<UploadItem> dequeue_with_timeout(std::chrono::milliseconds timeout);

  /**
   * Re-queue an item for retry
   *
   * The item will be available for dequeue after its next_retry_at time.
   *
   * @param item Item to re-queue (should have updated retry_count and next_retry_at)
   * @return true if re-queued, false if retry limit exceeded
   */
  bool requeue_for_retry(UploadItem item);

  /**
   * Get the current size of the main queue
   */
  size_t size() const;

  /**
   * Get the current size of the retry queue
   */
  size_t retry_size() const;

  /**
   * Check if the queue is empty (both main and retry)
   */
  bool empty() const;

  /**
   * Get total bytes pending in the queue
   */
  uint64_t pending_bytes() const;

  /**
   * Signal shutdown - dequeue() will return nullopt
   */
  void shutdown();

  /**
   * Check if shutdown was requested
   */
  bool is_shutdown() const;

private:
  // Move ready retry items to main queue
  void process_retry_queue();

  mutable std::mutex mutex_;
  std::condition_variable cv_;

  // Main queue for new items
  std::queue<UploadItem> main_queue_;

  // Retry queue (priority queue by next_retry_at)
  std::priority_queue<UploadItem, std::vector<UploadItem>, RetryItemComparator> retry_queue_;

  size_t capacity_;
  std::atomic<bool> shutdown_{false};
  std::atomic<uint64_t> pending_bytes_{0};
};

}  // namespace uploader
}  // namespace axon

#endif  // AXON_UPLOAD_QUEUE_HPP
