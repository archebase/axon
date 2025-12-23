#include "upload_queue.hpp"

namespace axon {
namespace uploader {

UploadQueue::UploadQueue(size_t capacity) : capacity_(capacity) {}

UploadQueue::~UploadQueue() { shutdown(); }

bool UploadQueue::enqueue(UploadItem item) {
  std::lock_guard<std::mutex> lock(mutex_);

  // Check capacity (0 = unlimited)
  if (capacity_ > 0 && main_queue_.size() >= capacity_) {
    return false;
  }

  pending_bytes_ += item.file_size_bytes;
  main_queue_.push(std::move(item));
  cv_.notify_one();

  return true;
}

std::optional<UploadItem> UploadQueue::dequeue() {
  std::unique_lock<std::mutex> lock(mutex_);

  while (true) {
    // Check for shutdown
    if (shutdown_) {
      return std::nullopt;
    }

    // Process retry queue - move ready items to main queue
    process_retry_queue();

    // Return item from main queue if available
    if (!main_queue_.empty()) {
      UploadItem item = std::move(main_queue_.front());
      main_queue_.pop();
      pending_bytes_ -= item.file_size_bytes;
      return item;
    }

    // Calculate wait time based on retry queue
    if (!retry_queue_.empty()) {
      auto now = std::chrono::steady_clock::now();
      auto next_retry = retry_queue_.top().next_retry_at;

      if (next_retry <= now) {
        // Item is ready, process immediately
        continue;
      }

      // Wait until next retry time or new item arrives
      cv_.wait_until(lock, next_retry);
    } else {
      // Wait for new item or shutdown
      cv_.wait(lock);
    }
  }
}

std::optional<UploadItem> UploadQueue::dequeue_with_timeout(std::chrono::milliseconds timeout) {
  std::unique_lock<std::mutex> lock(mutex_);
  auto deadline = std::chrono::steady_clock::now() + timeout;

  while (true) {
    // Check for shutdown
    if (shutdown_) {
      return std::nullopt;
    }

    // Process retry queue
    process_retry_queue();

    // Return item from main queue if available
    if (!main_queue_.empty()) {
      UploadItem item = std::move(main_queue_.front());
      main_queue_.pop();
      pending_bytes_ -= item.file_size_bytes;
      return item;
    }

    // Check timeout
    auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      return std::nullopt;
    }

    // Calculate wait time
    auto wait_until = deadline;
    if (!retry_queue_.empty()) {
      auto next_retry = retry_queue_.top().next_retry_at;
      if (next_retry < wait_until) {
        wait_until = next_retry;
      }
    }

    cv_.wait_until(lock, wait_until);
  }
}

bool UploadQueue::requeue_for_retry(UploadItem item) {
  std::lock_guard<std::mutex> lock(mutex_);

  // Validate retry timing is set
  if (item.next_retry_at <= std::chrono::steady_clock::now()) {
    // If no delay, add to main queue directly
    pending_bytes_ += item.file_size_bytes;
    main_queue_.push(std::move(item));
    cv_.notify_one();
    return true;
  }

  // Add to retry queue
  pending_bytes_ += item.file_size_bytes;
  retry_queue_.push(std::move(item));
  cv_.notify_one();

  return true;
}

size_t UploadQueue::size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return main_queue_.size();
}

size_t UploadQueue::retry_size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return retry_queue_.size();
}

bool UploadQueue::empty() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return main_queue_.empty() && retry_queue_.empty();
}

uint64_t UploadQueue::pending_bytes() const { return pending_bytes_.load(); }

void UploadQueue::shutdown() {
  shutdown_ = true;
  cv_.notify_all();
}

bool UploadQueue::is_shutdown() const { return shutdown_.load(); }

void UploadQueue::process_retry_queue() {
  // Must be called with mutex held
  auto now = std::chrono::steady_clock::now();

  while (!retry_queue_.empty()) {
    const auto& top = retry_queue_.top();
    if (top.next_retry_at > now) {
      // No more ready items
      break;
    }

    // Copy the item from priority queue (avoids const_cast)
    // Items are relatively small (strings + metadata), so copy is acceptable
    UploadItem item = retry_queue_.top();
    retry_queue_.pop();
    main_queue_.push(std::move(item));
  }
}

}  // namespace uploader
}  // namespace axon

