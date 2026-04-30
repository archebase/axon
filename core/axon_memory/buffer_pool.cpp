// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "buffer_pool.hpp"

#include <algorithm>
#include <cstdlib>
#include <cstring>

namespace axon {
namespace memory {

// ---------------------------------------------------------------------------
// PooledBuffer
// ---------------------------------------------------------------------------

void PooledBuffer::assign(const uint8_t* src, size_t n) {
  // Adopted buffers don't own the allocation, so any assign() has to detach
  // from the external owner and fall through to the heap/pool path below.
  if (external_release_ != nullptr) {
    release();
  }

  if (n <= capacity_ && data_ != nullptr) {
    if (n > 0) {
      std::memcpy(data_, src, n);
    }
    size_ = n;
    return;
  }

  // Need a larger slab. Try the global pool first; fall back to heap.
  BufferPool* owning_pool = pool_;
  if (owning_pool == nullptr) {
    owning_pool = &BufferPool::instance();
  }

  PooledBuffer bigger = owning_pool->acquire(n);
  release();
  *this = std::move(bigger);
  if (n > 0 && data_ != nullptr) {
    std::memcpy(data_, src, n);
  }
  size_ = n;
}

void PooledBuffer::release() noexcept {
  if (data_ == nullptr) {
    // Adopted buffers may have non-null release_ even if data_ is null;
    // external_release_ is always paired with data_ so this is fine.
    return;
  }
  if (external_release_ != nullptr) {
    // Adopted: hand back to the external owner.
    external_release_(external_opaque_);
  } else if (pool_ != nullptr) {
    pool_->release_buffer(data_, capacity_);
  } else {
    std::free(data_);
  }
  pool_ = nullptr;
  external_release_ = nullptr;
  external_opaque_ = nullptr;
  data_ = nullptr;
  capacity_ = 0;
  size_ = 0;
}

// ---------------------------------------------------------------------------
// BufferPool
// ---------------------------------------------------------------------------

BufferPool::BufferPool(BufferPoolConfig config)
    : config_(std::move(config)) {
  // Normalize: sort size_classes ascending and dedupe.
  std::sort(config_.size_classes.begin(), config_.size_classes.end());
  config_.size_classes.erase(
    std::unique(config_.size_classes.begin(), config_.size_classes.end()),
    config_.size_classes.end()
  );
  buckets_ = std::vector<Bucket>(config_.size_classes.size());
}

BufferPool::~BufferPool() {
  drain();
}

int BufferPool::find_class_index(size_t min_size) const noexcept {
  for (size_t i = 0; i < config_.size_classes.size(); ++i) {
    if (config_.size_classes[i] >= min_size) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

PooledBuffer BufferPool::acquire(size_t min_size) {
  if (min_size == 0) {
    min_size = 1;  // treat 0 as 1 to always return a valid non-null pointer
  }

  int class_idx = find_class_index(min_size);

  // Oversized: heap-allocate exactly what was asked, bypass pool.
  if (class_idx < 0) {
    uint8_t* mem = static_cast<uint8_t*>(std::malloc(min_size));
    {
      std::lock_guard<std::mutex> lk(metrics_mu_);
      stats_.acquires++;
      stats_.oversized++;
    }
    if (!mem) {
      return PooledBuffer();
    }
    // pool_ = nullptr so release() frees via std::free.
    return PooledBuffer(nullptr, mem, min_size);
  }

  size_t cap = config_.size_classes[class_idx];
  Bucket& bucket = buckets_[class_idx];

  uint8_t* mem = nullptr;
  {
    std::lock_guard<std::mutex> lk(bucket.mu);
    if (!bucket.free_list.empty()) {
      mem = bucket.free_list.back();
      bucket.free_list.pop_back();
    }
  }

  bool hit = (mem != nullptr);
  if (!hit) {
    mem = static_cast<uint8_t*>(std::malloc(cap));
    if (!mem) {
      std::lock_guard<std::mutex> lk(metrics_mu_);
      stats_.acquires++;
      return PooledBuffer();
    }
  }

  {
    std::lock_guard<std::mutex> lk(metrics_mu_);
    stats_.acquires++;
    if (hit) {
      stats_.hits++;
      // Buffer was already counted in resident_bytes while cached;
      // subtract because it's now checked out.
      stats_.resident_bytes -= cap;
    } else {
      stats_.misses++;
    }
  }

  return PooledBuffer(this, mem, cap);
}

void BufferPool::release_buffer(uint8_t* data, size_t capacity) noexcept {
  if (data == nullptr) {
    return;
  }

  int class_idx = find_class_index(capacity);
  // Sanity check: capacity must match a known class exactly; otherwise free.
  // (Oversized buffers should not reach here because their pool_ is nullptr.)
  if (class_idx < 0 || config_.size_classes[static_cast<size_t>(class_idx)] != capacity) {
    std::free(data);
    std::lock_guard<std::mutex> lk(metrics_mu_);
    stats_.releases++;
    stats_.release_freed++;
    return;
  }

  Bucket& bucket = buckets_[class_idx];

  bool cached = false;
  {
    std::lock_guard<std::mutex> bucket_lk(bucket.mu);
    std::lock_guard<std::mutex> stats_lk(metrics_mu_);
    stats_.releases++;

    bool class_cap_ok = bucket.free_list.size() < config_.max_buffers_per_class;
    bool resident_ok = (stats_.resident_bytes + capacity) <= config_.max_resident_bytes;
    if (class_cap_ok && resident_ok) {
      bucket.free_list.push_back(data);
      stats_.release_to_pool++;
      stats_.resident_bytes += capacity;
      cached = true;
    } else {
      stats_.release_freed++;
    }
  }

  if (!cached) {
    std::free(data);
  }
}

BufferPoolStats BufferPool::stats() const {
  std::lock_guard<std::mutex> lk(metrics_mu_);
  return stats_;
}

void BufferPool::reset_stats() {
  std::lock_guard<std::mutex> lk(metrics_mu_);
  // Keep resident_bytes: it reflects current cached state, not a counter.
  uint64_t resident = stats_.resident_bytes;
  stats_ = BufferPoolStats{};
  stats_.resident_bytes = resident;
}

void BufferPool::drain() {
  for (size_t i = 0; i < buckets_.size(); ++i) {
    Bucket& bucket = buckets_[i];
    size_t cap = config_.size_classes[i];
    std::vector<uint8_t*> taken;
    {
      std::lock_guard<std::mutex> lk(bucket.mu);
      taken.swap(bucket.free_list);
    }
    for (uint8_t* mem : taken) {
      std::free(mem);
    }
    if (!taken.empty()) {
      std::lock_guard<std::mutex> lk(metrics_mu_);
      uint64_t freed = static_cast<uint64_t>(taken.size()) * static_cast<uint64_t>(cap);
      if (stats_.resident_bytes >= freed) {
        stats_.resident_bytes -= freed;
      } else {
        stats_.resident_bytes = 0;
      }
    }
  }
}

BufferPool& BufferPool::instance() {
  static BufferPool g_pool;
  return g_pool;
}

}  // namespace memory
}  // namespace axon
