// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_MEMORY_BUFFER_POOL_HPP
#define AXON_MEMORY_BUFFER_POOL_HPP

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

namespace axon {
namespace memory {

/**
 * Configuration for the BufferPool.
 */
struct BufferPoolConfig {
  /**
   * Size classes in bytes, in ascending order.
   * Each allocation rounds up to the smallest class >= requested size.
   * Requests larger than the biggest class fall back to heap allocation
   * (tracked in stats as `oversized`).
   *
   * Default covers a wide range from small control-plane messages up to
   * HD images / mid-sized point clouds. Very large (>= 4 MiB) buffers are
   * treated as oversized to avoid pinning large resident memory.
   */
  std::vector<size_t> size_classes = {
    1024,             // 1 KiB  - IMU / Odom / TF
    4 * 1024,         // 4 KiB  - JointState / small control msgs
    16 * 1024,        // 16 KiB - LaserScan
    64 * 1024,        // 64 KiB - small compressed image / metadata
    256 * 1024,       // 256 KiB - small PointCloud2
    1024 * 1024,      // 1 MiB  - mid-resolution image
    4 * 1024 * 1024,  // 4 MiB - large image / full PointCloud2
  };

  /// Maximum number of buffers cached per size class.
  /// Beyond this, returned buffers are freed to heap to cap memory growth.
  size_t max_buffers_per_class = 64;

  /// Hard upper bound on resident bytes across all classes.
  /// Returns that would exceed this ceiling free the buffer instead of caching.
  size_t max_resident_bytes = 256 * 1024 * 1024;  // 256 MiB
};

/**
 * Pool statistics snapshot.
 */
struct BufferPoolStats {
  uint64_t acquires = 0;         // total acquire() calls
  uint64_t hits = 0;             // acquires that reused a pooled buffer
  uint64_t misses = 0;           // acquires that allocated a new buffer (fit a class)
  uint64_t oversized = 0;        // acquires that exceeded the biggest class
  uint64_t releases = 0;         // total release() calls (buffer returned)
  uint64_t release_to_pool = 0;  // releases that were cached back into the pool
  uint64_t release_freed = 0;    // releases that freed to heap (class cap / ceiling)
  uint64_t resident_bytes = 0;   // current bytes held by the pool

  /// hit_rate() in [0, 1], or 0 when no acquires yet.
  double hit_rate() const {
    if (acquires == 0) {
      return 0.0;
    }
    return static_cast<double>(hits) / static_cast<double>(acquires);
  }
};

class BufferPool;  // forward

/**
 * External release function for adopted buffers.
 * Called with the `opaque` token passed at adopt time.
 */
using ExternalReleaseFn = void (*)(void* opaque);

/**
 * RAII owner of a message byte buffer.
 *
 * Supports three ownership modes (mutually exclusive, detected at destruction):
 *   1. Pool-owned: pool_ != nullptr, returned to pool on destruction.
 *   2. Heap-owned (oversized): pool_ == nullptr && external_release_ == nullptr,
 *      freed via std::free on destruction.
 *   3. Adopted from external owner: external_release_ != nullptr,
 *      external_release_(external_opaque_) called on destruction (zero-copy
 *      ABI v1.2 path — pointer retained, no copy at the ABI boundary).
 *
 * - Move-only, noexcept-movable (safe to store in std::vector and move across
 *   SPSC queue slots without triggering copies).
 *
 * For pool-owned / heap-owned buffers, the underlying storage capacity is the
 * pool size-class that was selected at acquire() time, NOT necessarily the
 * requested min_size. Code should use size() for logical payload length and
 * capacity() for the underlying slab.
 *
 * For adopted buffers, capacity() == size() (the external provider knows the
 * exact length and we cannot grow the allocation without copying out).
 */
class PooledBuffer {
public:
  PooledBuffer() noexcept = default;

  PooledBuffer(PooledBuffer&& other) noexcept
      : pool_(other.pool_)
      , external_release_(other.external_release_)
      , external_opaque_(other.external_opaque_)
      , data_(other.data_)
      , size_(other.size_)
      , capacity_(other.capacity_) {
    other.pool_ = nullptr;
    other.external_release_ = nullptr;
    other.external_opaque_ = nullptr;
    other.data_ = nullptr;
    other.size_ = 0;
    other.capacity_ = 0;
  }

  PooledBuffer& operator=(PooledBuffer&& other) noexcept {
    if (this != &other) {
      release();
      pool_ = other.pool_;
      external_release_ = other.external_release_;
      external_opaque_ = other.external_opaque_;
      data_ = other.data_;
      size_ = other.size_;
      capacity_ = other.capacity_;
      other.pool_ = nullptr;
      other.external_release_ = nullptr;
      other.external_opaque_ = nullptr;
      other.data_ = nullptr;
      other.size_ = 0;
      other.capacity_ = 0;
    }
    return *this;
  }

  PooledBuffer(const PooledBuffer&) = delete;
  PooledBuffer& operator=(const PooledBuffer&) = delete;

  ~PooledBuffer() {
    release();
  }

  /**
   * Construct a PooledBuffer that adopts an externally-owned buffer.
   *
   * Used by ABI v1.2 zero-copy callbacks: the plugin retains logical
   * ownership of the buffer and provides a release function that the
   * recorder invokes when it is done (after MCAP write / drop).
   *
   * The adopted buffer is NOT returned to the global BufferPool; it is
   * released via the provided function. assign() on an adopted buffer
   * that requires more space will detach from the external owner
   * (calling release_fn) and reacquire from the given pool (or heap).
   */
  static PooledBuffer adopt(
    uint8_t* data, size_t size, ExternalReleaseFn release_fn, void* opaque
  ) noexcept {
    PooledBuffer b;
    b.pool_ = nullptr;
    b.external_release_ = release_fn;
    b.external_opaque_ = opaque;
    b.data_ = data;
    b.size_ = size;
    b.capacity_ = size;
    return b;
  }

  /**
   * Copy `n` bytes from `src` into this buffer, resizing to exactly `n`.
   * If current capacity is insufficient, a new (larger) buffer is acquired
   * from the owning pool (or heap-allocated if no pool is attached).
   *
   * Must not be called while the buffer has no owning pool AND `n > capacity`.
   * In practice PooledBuffer is always obtained via BufferPool::acquire().
   */
  void assign(const uint8_t* src, size_t n);

  uint8_t* data() noexcept {
    return data_;
  }
  const uint8_t* data() const noexcept {
    return data_;
  }
  size_t size() const noexcept {
    return size_;
  }
  size_t capacity() const noexcept {
    return capacity_;
  }
  bool empty() const noexcept {
    return size_ == 0;
  }
  void clear() noexcept {
    size_ = 0;
  }

  /// Release the underlying buffer back to the pool (or heap).
  void release() noexcept;

private:
  friend class BufferPool;

  PooledBuffer(BufferPool* pool, uint8_t* data, size_t capacity) noexcept
      : pool_(pool)
      , data_(data)
      , size_(0)
      , capacity_(capacity) {}

  BufferPool* pool_ = nullptr;
  // If non-null, this buffer is "adopted" from an external owner (ABI v1.2
  // zero-copy). On release, invoke external_release_(external_opaque_)
  // instead of returning to the pool.
  ExternalReleaseFn external_release_ = nullptr;
  void* external_opaque_ = nullptr;
  uint8_t* data_ = nullptr;
  size_t size_ = 0;
  size_t capacity_ = 0;
};

/**
 * Tiered free-list pool for message byte buffers.
 *
 * Why: the recorder hot path (`on_message`) performs a full `std::vector`
 * allocation per ROS message, plus the ROS2 subscription wrapper does another
 * allocation earlier. Under high-rate point-cloud / image topics this causes
 * per-message `operator new` contention against glibc's malloc and page
 * faulting as the heap grows and shrinks.
 *
 * This pool amortizes that cost by recycling allocated byte slabs per size
 * class. It does NOT attempt to be a general-purpose allocator; it is tuned
 * for the single "acquire / fill / dispatch / release" pattern of the
 * recorder worker pipeline.
 *
 * Thread-safety: acquire() and release() are both thread-safe; uses a
 * per-size-class mutex to minimize cross-topic contention. The metrics
 * counters are plain uint64_t guarded by the same mutex (snapshot-consistent).
 *
 * Concurrency note: this is a mutex-based pool, not lock-free. In benchmark
 * testing the critical section is ~O(1) std::vector::pop_back and satisfies
 * the zero-allocation property which is what dominates the wall-clock cost
 * of the hot path. If contention ever shows up in profiles, a future
 * iteration can shard by thread or class.
 */
class BufferPool {
public:
  explicit BufferPool(BufferPoolConfig config = BufferPoolConfig{});
  ~BufferPool();

  BufferPool(const BufferPool&) = delete;
  BufferPool& operator=(const BufferPool&) = delete;

  /**
   * Acquire a buffer with at least `min_size` bytes of capacity.
   * Returned PooledBuffer starts with size() == 0.
   *
   * If `min_size` exceeds the largest configured class, the buffer is
   * heap-allocated with exactly `min_size` capacity (tracked as oversized).
   */
  PooledBuffer acquire(size_t min_size);

  /**
   * Snapshot current statistics. Safe to call from any thread.
   */
  BufferPoolStats stats() const;

  /**
   * Reset counters (resident_bytes is NOT reset; it reflects current state).
   */
  void reset_stats();

  /**
   * Drop all cached buffers. Does not affect in-flight PooledBuffers.
   * Primarily useful for tests and shutdown.
   */
  void drain();

  /**
   * Global default instance. Lazily initialized with default config on first
   * call. Callers needing custom config should instantiate their own pool.
   */
  static BufferPool& instance();

  const BufferPoolConfig& config() const noexcept {
    return config_;
  }

private:
  friend class PooledBuffer;

  // Called by PooledBuffer::release() / dtor.
  void release_buffer(uint8_t* data, size_t capacity) noexcept;

  // Find the index of the smallest class that fits `min_size`, or -1 if none.
  int find_class_index(size_t min_size) const noexcept;

  struct Bucket {
    // Protects `free_list`.
    mutable std::mutex mu;
    // Cached buffers for this size class.
    std::vector<uint8_t*> free_list;
  };

  BufferPoolConfig config_;
  std::vector<Bucket> buckets_;  // one per size class, parallel to config_.size_classes

  // Global metrics — guarded by metrics_mu_ for consistent snapshots.
  mutable std::mutex metrics_mu_;
  BufferPoolStats stats_{};
};

}  // namespace memory
}  // namespace axon

#endif  // AXON_MEMORY_BUFFER_POOL_HPP
