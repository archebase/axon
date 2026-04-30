// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

// -----------------------------------------------------------------------------
// Standalone micro-benchmark: acquire / assign / release throughput.
//
// This is intentionally dependency-free (no Google Benchmark) because we only
// need to spot-check the pool vs std::vector baseline on a given target host
// when investigating regressions.
//
// Two scenarios are run so the pool is evaluated both in its worst case and
// in the case it was actually designed for:
//
//   1. Single-thread tight loop.   Here glibc's per-thread tcache turns
//      same-size alloc+free into a bump allocator, which is extremely hard
//      to beat — do NOT expect the pool to win this scenario. It is included
//      only to calibrate the raw overhead.
//   2. Cross-thread producer/consumer.  One producer allocates+fills, a
//      second consumer frees. This models the recorder (plugin thread enqueues
//      MessageItem; worker thread dispatches and releases). It is the actual
//      hot path the pool was designed to amortize.
//
// Run with:
//
//   ./bench_buffer_pool [iterations] [payload_bytes] [threads]
//
// Defaults: 1_000_000 iterations, 32 KiB payload, 1 thread.
// -----------------------------------------------------------------------------

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "buffer_pool.hpp"

namespace {

double seconds_between(
  std::chrono::steady_clock::time_point a, std::chrono::steady_clock::time_point b
) {
  return std::chrono::duration<double>(b - a).count();
}

// Baseline: one fresh std::vector<uint8_t> allocation per "message".
void run_vector_baseline(size_t iters, size_t payload_bytes, int threads, const uint8_t* src) {
  auto worker = [&](size_t n) {
    for (size_t i = 0; i < n; ++i) {
      std::vector<uint8_t> v;
      v.assign(src, src + payload_bytes);
      // prevent the compiler from eliding
      if (v[0] == 0xFF && payload_bytes == 0) {
        std::fputs("unreachable\n", stderr);
      }
    }
  };

  std::vector<std::thread> pool;
  pool.reserve(threads);
  size_t per_thread = iters / static_cast<size_t>(threads);
  auto t0 = std::chrono::steady_clock::now();
  for (int t = 0; t < threads; ++t) {
    pool.emplace_back(worker, per_thread);
  }
  for (auto& th : pool) {
    th.join();
  }
  auto t1 = std::chrono::steady_clock::now();

  size_t total = per_thread * static_cast<size_t>(threads);
  double secs = seconds_between(t0, t1);
  std::printf(
    "[vector  ] iters=%zu payload=%zuB threads=%d  took=%.3fs  msgs/s=%.0f  bytes/s=%.2f MiB/s\n",
    total,
    payload_bytes,
    threads,
    secs,
    static_cast<double>(total) / secs,
    (static_cast<double>(total) * payload_bytes / (1024.0 * 1024.0)) / secs
  );
}

// Pool: acquire / assign / release one pooled buffer per "message".
void run_pool_bench(size_t iters, size_t payload_bytes, int threads, const uint8_t* src) {
  axon::memory::BufferPool pool;
  // Warm up: one acquire+release per size class so the pool has resident
  // buffers before the timed phase starts.
  {
    auto warm = pool.acquire(payload_bytes);
    warm.assign(src, payload_bytes);
  }

  auto worker = [&](size_t n) {
    for (size_t i = 0; i < n; ++i) {
      auto b = pool.acquire(payload_bytes);
      b.assign(src, payload_bytes);
      if (b.data() == nullptr && payload_bytes > 0) {
        std::fputs("acquire failed\n", stderr);
      }
    }
  };

  std::vector<std::thread> pool_threads;
  pool_threads.reserve(threads);
  size_t per_thread = iters / static_cast<size_t>(threads);
  auto t0 = std::chrono::steady_clock::now();
  for (int t = 0; t < threads; ++t) {
    pool_threads.emplace_back(worker, per_thread);
  }
  for (auto& th : pool_threads) {
    th.join();
  }
  auto t1 = std::chrono::steady_clock::now();

  size_t total = per_thread * static_cast<size_t>(threads);
  double secs = seconds_between(t0, t1);
  auto s = pool.stats();
  std::printf(
    "[pool    ] iters=%zu payload=%zuB threads=%d  took=%.3fs  msgs/s=%.0f  bytes/s=%.2f MiB/s\n",
    total,
    payload_bytes,
    threads,
    secs,
    static_cast<double>(total) / secs,
    (static_cast<double>(total) * payload_bytes / (1024.0 * 1024.0)) / secs
  );
  std::printf(
    "           hit_rate=%.4f  acquires=%lu hits=%lu misses=%lu oversized=%lu "
    "release_to_pool=%lu release_freed=%lu resident_bytes=%lu\n",
    s.hit_rate(),
    static_cast<unsigned long>(s.acquires),
    static_cast<unsigned long>(s.hits),
    static_cast<unsigned long>(s.misses),
    static_cast<unsigned long>(s.oversized),
    static_cast<unsigned long>(s.release_to_pool),
    static_cast<unsigned long>(s.release_freed),
    static_cast<unsigned long>(s.resident_bytes)
  );
}

// -----------------------------------------------------------------------------
// Cross-thread producer/consumer scenario.
//
// Producer allocates and fills a buffer, pushes onto a bounded queue, consumer
// pops and frees. This is the pattern the pool was designed for: alloc site
// and free site are on different threads, which defeats glibc's tcache.
// -----------------------------------------------------------------------------

constexpr size_t kQueueDepth = 64;

template<typename T>
class BoundedQueue {
public:
  bool push(T&& item) {
    std::unique_lock<std::mutex> lk(mu_);
    not_full_.wait(lk, [&] {
      return q_.size() < kQueueDepth || closed_;
    });
    if (closed_) {
      return false;
    }
    q_.push_back(std::move(item));
    not_empty_.notify_one();
    return true;
  }
  bool pop(T& out) {
    std::unique_lock<std::mutex> lk(mu_);
    not_empty_.wait(lk, [&] {
      return !q_.empty() || closed_;
    });
    if (q_.empty()) {
      return false;
    }
    out = std::move(q_.front());
    q_.pop_front();
    not_full_.notify_one();
    return true;
  }
  void close() {
    std::lock_guard<std::mutex> lk(mu_);
    closed_ = true;
    not_empty_.notify_all();
    not_full_.notify_all();
  }

private:
  std::mutex mu_;
  std::condition_variable not_full_;
  std::condition_variable not_empty_;
  std::deque<T> q_;
  bool closed_ = false;
};

void run_vector_crossthread(size_t iters, size_t payload_bytes, const uint8_t* src) {
  BoundedQueue<std::vector<uint8_t>> q;
  std::thread producer([&] {
    for (size_t i = 0; i < iters; ++i) {
      std::vector<uint8_t> v;
      v.assign(src, src + payload_bytes);
      q.push(std::move(v));
    }
    q.close();
  });
  auto t0 = std::chrono::steady_clock::now();
  std::vector<uint8_t> v;
  for (size_t i = 0; i < iters; ++i) {
    if (!q.pop(v)) break;
    // Explicit clear + release mimics worker finishing with a buffer.
    std::vector<uint8_t>().swap(v);
  }
  auto t1 = std::chrono::steady_clock::now();
  producer.join();

  double secs = seconds_between(t0, t1);
  std::printf(
    "[vector/xthread] iters=%zu payload=%zuB  took=%.3fs  msgs/s=%.0f  bytes/s=%.2f MiB/s\n",
    iters,
    payload_bytes,
    secs,
    static_cast<double>(iters) / secs,
    (static_cast<double>(iters) * payload_bytes / (1024.0 * 1024.0)) / secs
  );
}

void run_pool_crossthread(size_t iters, size_t payload_bytes, const uint8_t* src) {
  axon::memory::BufferPool pool;
  BoundedQueue<axon::memory::PooledBuffer> q;

  std::thread producer([&] {
    for (size_t i = 0; i < iters; ++i) {
      auto b = pool.acquire(payload_bytes);
      b.assign(src, payload_bytes);
      q.push(std::move(b));
    }
    q.close();
  });
  auto t0 = std::chrono::steady_clock::now();
  axon::memory::PooledBuffer b;
  for (size_t i = 0; i < iters; ++i) {
    if (!q.pop(b)) break;
    b.release();
  }
  auto t1 = std::chrono::steady_clock::now();
  producer.join();

  double secs = seconds_between(t0, t1);
  auto s = pool.stats();
  std::printf(
    "[pool/xthread  ] iters=%zu payload=%zuB  took=%.3fs  msgs/s=%.0f  bytes/s=%.2f MiB/s\n",
    iters,
    payload_bytes,
    secs,
    static_cast<double>(iters) / secs,
    (static_cast<double>(iters) * payload_bytes / (1024.0 * 1024.0)) / secs
  );
  std::printf(
    "                 hit_rate=%.4f  acquires=%lu hits=%lu misses=%lu oversized=%lu "
    "release_to_pool=%lu release_freed=%lu resident_bytes=%lu\n",
    s.hit_rate(),
    static_cast<unsigned long>(s.acquires),
    static_cast<unsigned long>(s.hits),
    static_cast<unsigned long>(s.misses),
    static_cast<unsigned long>(s.oversized),
    static_cast<unsigned long>(s.release_to_pool),
    static_cast<unsigned long>(s.release_freed),
    static_cast<unsigned long>(s.resident_bytes)
  );
}

}  // namespace

int main(int argc, char** argv) {
  size_t iters = 1'000'000;
  size_t payload = 32 * 1024;
  int threads = 1;

  if (argc > 1) iters = static_cast<size_t>(std::strtoull(argv[1], nullptr, 10));
  if (argc > 2) payload = static_cast<size_t>(std::strtoull(argv[2], nullptr, 10));
  if (argc > 3) threads = std::atoi(argv[3]);
  if (threads < 1) threads = 1;

  std::vector<uint8_t> src(payload == 0 ? 1 : payload, 0xAB);

  std::printf(
    "--- axon_memory/bench_buffer_pool ---\n"
    "iters=%zu payload=%zu bytes threads=%d\n",
    iters,
    payload,
    threads
  );

  std::printf("\n--- Scenario 1: single-thread tight loop (tcache-friendly) ---\n");
  run_vector_baseline(iters, payload, threads, src.data());
  run_pool_bench(iters, payload, threads, src.data());

  std::printf("\n--- Scenario 2: cross-thread producer/consumer (recorder pattern) ---\n");
  run_vector_crossthread(iters, payload, src.data());
  run_pool_crossthread(iters, payload, src.data());

  return 0;
}
