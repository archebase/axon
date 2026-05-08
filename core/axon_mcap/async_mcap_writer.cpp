// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "async_mcap_writer.hpp"

#include <algorithm>
#include <cstring>
#include <utility>

namespace axon {
namespace mcap_wrapper {

AsyncMcapWriter::AsyncMcapWriter() = default;

AsyncMcapWriter::~AsyncMcapWriter() {
  close();
}

bool AsyncMcapWriter::open(
  const std::string& path, const McapWriterOptions& options, const AsyncWriterConfig& async_config
) {
  if (open_.load(std::memory_order_acquire)) {
    return false;
  }
  async_config_ = async_config;
  if (async_config_.queue_capacity == 0) {
    async_config_.queue_capacity = 1;
  }
  if (async_config_.batch_size == 0) {
    async_config_.batch_size = 1;
  }

  if (!writer_.open(path, options)) {
    return false;
  }

  // Reset queue + counters for a fresh session.
  {
    std::lock_guard<std::mutex> lk(q_mu_);
    q_.clear();
    enqueued_ = 0;
    dequeued_ = 0;
    dropped_full_ = 0;
    batches_written_ = 0;
    write_failures_ = 0;
    peak_depth_ = 0;
    in_flight_ = 0;
  }
  stop_requested_.store(false, std::memory_order_release);
  running_.store(true, std::memory_order_release);
  open_.store(true, std::memory_order_release);

  // Start worker last so it only sees a fully-initialized writer.
  worker_ = std::thread(&AsyncMcapWriter::worker_loop, this);
  return true;
}

void AsyncMcapWriter::close() {
  // Mark closed first so producers stop enqueueing.
  bool was_open = open_.exchange(false, std::memory_order_acq_rel);
  if (!was_open) {
    return;
  }

  // Signal worker to drain and stop.
  {
    std::lock_guard<std::mutex> lk(q_mu_);
    stop_requested_.store(true, std::memory_order_release);
  }
  not_empty_.notify_all();
  not_full_.notify_all();
  drained_.notify_all();

  if (worker_.joinable()) {
    worker_.join();
  }
  running_.store(false, std::memory_order_release);
  writer_.close();
}

bool AsyncMcapWriter::is_open() const {
  return open_.load(std::memory_order_acquire);
}

uint16_t AsyncMcapWriter::register_schema(
  const std::string& name, const std::string& encoding, const std::string& data
) {
  return writer_.register_schema(name, encoding, data);
}

uint16_t AsyncMcapWriter::register_channel(
  const std::string& topic, const std::string& message_encoding, uint16_t schema_id
) {
  return writer_.register_channel(topic, message_encoding, schema_id);
}

bool AsyncMcapWriter::write(
  uint16_t channel_id, uint64_t log_time_ns, uint64_t publish_time_ns, const void* data,
  size_t data_size
) {
  return write(channel_id, 0, log_time_ns, publish_time_ns, data, data_size);
}

bool AsyncMcapWriter::write(
  uint16_t channel_id, uint32_t sequence, uint64_t log_time_ns, uint64_t publish_time_ns,
  const void* data, size_t data_size
) {
  if (!open_.load(std::memory_order_acquire)) {
    return false;
  }
  QueuedMsg m;
  m.channel_id = channel_id;
  m.sequence = sequence;
  m.log_time_ns = log_time_ns;
  m.publish_time_ns = publish_time_ns;
  if (data_size > 0) {
    m.data.resize(data_size);
    if (data != nullptr) {
      std::memcpy(m.data.data(), data, data_size);
    }
  }
  return enqueue_(std::move(m));
}

bool AsyncMcapWriter::enqueue_(QueuedMsg&& msg) {
  std::unique_lock<std::mutex> lk(q_mu_);

  // Fast path: space available.
  if (q_.size() >= async_config_.queue_capacity) {
    if (async_config_.drop_oldest_on_full) {
      // Drop oldest to make room. O(1) pop_front.
      q_.pop_front();
      ++dropped_full_;
    } else {
      // Block until space appears or we're told to stop.
      not_full_.wait(lk, [&] {
        return q_.size() < async_config_.queue_capacity || !open_.load(std::memory_order_acquire) ||
               stop_requested_.load(std::memory_order_acquire);
      });
      if (!open_.load(std::memory_order_acquire) ||
          stop_requested_.load(std::memory_order_acquire)) {
        return false;
      }
    }
  }

  q_.push_back(std::move(msg));
  ++enqueued_;
  if (q_.size() > peak_depth_) {
    peak_depth_ = q_.size();
  }
  // Notify consumer; use notify_one (single worker).
  lk.unlock();
  not_empty_.notify_one();
  return true;
}

void AsyncMcapWriter::worker_loop() {
  std::vector<QueuedMsg> local_batch;
  std::vector<BatchItem> items;
  local_batch.reserve(async_config_.batch_size);
  items.reserve(async_config_.batch_size);

  while (true) {
    // Wait for work or stop.
    {
      std::unique_lock<std::mutex> lk(q_mu_);
      not_empty_.wait(lk, [&] {
        return !q_.empty() || stop_requested_.load(std::memory_order_acquire);
      });

      if (q_.empty() && stop_requested_.load(std::memory_order_acquire)) {
        return;
      }

      // Pull up to batch_size messages under the queue lock.
      size_t take = std::min(async_config_.batch_size, q_.size());
      local_batch.clear();
      local_batch.reserve(take);
      for (size_t i = 0; i < take; ++i) {
        local_batch.push_back(std::move(q_.front()));
        q_.pop_front();
      }
      in_flight_ += local_batch.size();
      // Wake any blocked producers waiting for space.
      lk.unlock();
      not_full_.notify_all();
    }

    if (local_batch.empty()) {
      continue;
    }

    items.clear();
    items.reserve(local_batch.size());
    for (const auto& m : local_batch) {
      BatchItem it;
      it.channel_id = m.channel_id;
      it.sequence = m.sequence;
      it.log_time_ns = m.log_time_ns;
      it.publish_time_ns = m.publish_time_ns;
      it.data = m.data.data();
      it.data_size = m.data.size();
      items.push_back(it);
    }

    size_t written = 0;
    bool ok = writer_.write_batch(items.data(), items.size(), &written);

    {
      std::lock_guard<std::mutex> lk(q_mu_);
      dequeued_ += written;
      ++batches_written_;
      if (!ok) {
        ++write_failures_;
      }
      in_flight_ -= local_batch.size();
    }
    not_full_.notify_all();
    drained_.notify_all();
    // Any messages after `written` in this batch failed; they're dropped
    // silently (the underlying writer logs rate-limited errors). This
    // matches the best-effort contract documented on the class.
  }
}

void AsyncMcapWriter::wait_until_empty() {
  std::unique_lock<std::mutex> lk(q_mu_);
  drained_.wait(lk, [&] {
    return (q_.empty() && in_flight_ == 0) || !running_.load(std::memory_order_acquire);
  });
}

AsyncWriterStats AsyncMcapWriter::get_async_stats() const {
  std::lock_guard<std::mutex> lk(q_mu_);
  AsyncWriterStats s;
  s.enqueued = enqueued_;
  s.dequeued = dequeued_;
  s.dropped_full = dropped_full_;
  s.batches_written = batches_written_;
  s.write_failures = write_failures_;
  s.current_depth = q_.size();
  s.peak_depth = peak_depth_;
  return s;
}

McapWriterWrapper::Statistics AsyncMcapWriter::get_statistics() const {
  return writer_.get_statistics();
}

std::string AsyncMcapWriter::get_last_error() const {
  return writer_.get_last_error();
}

std::string AsyncMcapWriter::get_path() const {
  return writer_.get_path();
}

}  // namespace mcap_wrapper
}  // namespace axon
