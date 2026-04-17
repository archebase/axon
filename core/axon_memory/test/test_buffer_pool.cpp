// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
#include <thread>
#include <vector>

#include "buffer_pool.hpp"

namespace axon {
namespace memory {
namespace {

TEST(BufferPoolTest, AcquireRoundsUpToSizeClass) {
  BufferPoolConfig cfg;
  cfg.size_classes = {64, 256, 1024};
  BufferPool pool(cfg);

  auto buf = pool.acquire(100);
  EXPECT_NE(buf.data(), nullptr);
  EXPECT_EQ(buf.capacity(), 256u);
  EXPECT_EQ(buf.size(), 0u);
}

TEST(BufferPoolTest, OversizedBypassesPool) {
  BufferPoolConfig cfg;
  cfg.size_classes = {64, 256};
  BufferPool pool(cfg);

  auto buf = pool.acquire(10000);
  EXPECT_NE(buf.data(), nullptr);
  EXPECT_EQ(buf.capacity(), 10000u);

  auto s = pool.stats();
  EXPECT_EQ(s.oversized, 1u);
  EXPECT_EQ(s.hits, 0u);
  EXPECT_EQ(s.misses, 0u);
}

TEST(BufferPoolTest, ReleaseReusesBuffer) {
  BufferPoolConfig cfg;
  cfg.size_classes = {64, 256};
  BufferPool pool(cfg);

  uint8_t* first_ptr = nullptr;
  {
    auto buf = pool.acquire(100);
    first_ptr = buf.data();
  }  // release on scope exit

  {
    auto buf = pool.acquire(100);
    EXPECT_EQ(buf.data(), first_ptr);
  }

  auto s = pool.stats();
  EXPECT_EQ(s.hits, 1u);
  EXPECT_EQ(s.misses, 1u);
  EXPECT_GT(s.hit_rate(), 0.0);
}

TEST(BufferPoolTest, AssignCopiesDataAndSetsSize) {
  BufferPool pool;
  auto buf = pool.acquire(32);
  const uint8_t payload[] = {1, 2, 3, 4, 5};
  buf.assign(payload, sizeof(payload));

  EXPECT_EQ(buf.size(), sizeof(payload));
  for (size_t i = 0; i < sizeof(payload); ++i) {
    EXPECT_EQ(buf.data()[i], payload[i]);
  }
}

TEST(BufferPoolTest, AssignBeyondCapacityReacquires) {
  BufferPoolConfig cfg;
  cfg.size_classes = {32, 256};
  BufferPool pool(cfg);

  auto buf = pool.acquire(16);
  EXPECT_EQ(buf.capacity(), 32u);

  std::vector<uint8_t> payload(200, 0xAB);
  buf.assign(payload.data(), payload.size());

  EXPECT_EQ(buf.size(), 200u);
  EXPECT_GE(buf.capacity(), 200u);
  for (uint8_t v : std::vector<uint8_t>(buf.data(), buf.data() + buf.size())) {
    EXPECT_EQ(v, 0xAB);
  }
}

TEST(BufferPoolTest, PoolCapPerClassFreesExcessReleases) {
  BufferPoolConfig cfg;
  cfg.size_classes = {64};
  cfg.max_buffers_per_class = 2;
  BufferPool pool(cfg);

  std::vector<PooledBuffer> bufs;
  for (int i = 0; i < 5; ++i) {
    bufs.push_back(pool.acquire(32));
  }
  bufs.clear();  // release all 5 at once

  auto s = pool.stats();
  EXPECT_EQ(s.releases, 5u);
  EXPECT_LE(s.release_to_pool, 2u);  // at most max_buffers_per_class cached
  EXPECT_GE(s.release_freed, 3u);    // the rest were freed
}

TEST(BufferPoolTest, ResidentCeilingEnforced) {
  BufferPoolConfig cfg;
  cfg.size_classes = {1024};
  cfg.max_buffers_per_class = 100;  // would otherwise allow lots
  cfg.max_resident_bytes = 2048;    // only 2 buffers can fit
  BufferPool pool(cfg);

  std::vector<PooledBuffer> bufs;
  for (int i = 0; i < 5; ++i) {
    bufs.push_back(pool.acquire(1024));
  }
  bufs.clear();

  auto s = pool.stats();
  EXPECT_LE(s.resident_bytes, cfg.max_resident_bytes);
  EXPECT_GE(s.release_freed, 3u);
}

TEST(BufferPoolTest, MoveTransfersOwnership) {
  BufferPool pool;
  auto a = pool.acquire(64);
  uint8_t* ptr = a.data();
  size_t cap = a.capacity();

  PooledBuffer b = std::move(a);
  EXPECT_EQ(b.data(), ptr);
  EXPECT_EQ(b.capacity(), cap);
  EXPECT_EQ(a.data(), nullptr);
  EXPECT_EQ(a.capacity(), 0u);
}

TEST(BufferPoolTest, DrainFreesCachedBuffers) {
  BufferPoolConfig cfg;
  cfg.size_classes = {64};
  BufferPool pool(cfg);

  { auto b = pool.acquire(32); }
  { auto b = pool.acquire(32); }

  auto before = pool.stats();
  EXPECT_GT(before.resident_bytes, 0u);

  pool.drain();

  auto after = pool.stats();
  EXPECT_EQ(after.resident_bytes, 0u);
}

TEST(BufferPoolTest, StatsReportAcquiresAndHitRate) {
  BufferPoolConfig cfg;
  cfg.size_classes = {128};
  BufferPool pool(cfg);

  // First acquire = miss; release, then acquire again = hit.
  { auto b = pool.acquire(64); }
  { auto b = pool.acquire(64); }

  auto s = pool.stats();
  EXPECT_EQ(s.acquires, 2u);
  EXPECT_EQ(s.hits, 1u);
  EXPECT_EQ(s.misses, 1u);
  EXPECT_DOUBLE_EQ(s.hit_rate(), 0.5);
}

TEST(BufferPoolTest, ConcurrentAcquireReleaseIsSafe) {
  BufferPool pool;
  constexpr int kThreads = 8;
  constexpr int kOpsPerThread = 1000;
  std::atomic<int> errors{0};

  std::vector<std::thread> threads;
  threads.reserve(kThreads);
  for (int t = 0; t < kThreads; ++t) {
    threads.emplace_back([&pool, &errors]() {
      for (int i = 0; i < kOpsPerThread; ++i) {
        size_t want = static_cast<size_t>(64 + (i % 2048));
        auto buf = pool.acquire(want);
        if (buf.data() == nullptr || buf.capacity() < want) {
          errors++;
          continue;
        }
        std::vector<uint8_t> src(want, static_cast<uint8_t>(i & 0xFF));
        buf.assign(src.data(), src.size());
        if (buf.size() != want) {
          errors++;
        }
      }
    });
  }
  for (auto& th : threads) {
    th.join();
  }

  EXPECT_EQ(errors.load(), 0);
  auto s = pool.stats();
  EXPECT_GE(s.acquires, static_cast<uint64_t>(kThreads * kOpsPerThread));
}

TEST(BufferPoolTest, InstanceReturnsSingleton) {
  auto& a = BufferPool::instance();
  auto& b = BufferPool::instance();
  EXPECT_EQ(&a, &b);
}

// ---------------------------------------------------------------------------
// Adopt / external-release path (ABI v1.2 zero-copy)
// ---------------------------------------------------------------------------

TEST(PooledBufferAdoptTest, ReleaseInvokesExternalReleaseExactlyOnce) {
  std::vector<uint8_t> external_storage(128, 0xAB);
  std::atomic<int> release_count{0};

  auto release_fn = +[](void* opaque) {
    auto* counter = static_cast<std::atomic<int>*>(opaque);
    counter->fetch_add(1);
  };

  {
    PooledBuffer b = PooledBuffer::adopt(
      external_storage.data(), external_storage.size(), release_fn, &release_count
    );
    EXPECT_EQ(b.data(), external_storage.data());
    EXPECT_EQ(b.size(), external_storage.size());
    EXPECT_EQ(b.capacity(), external_storage.size());
    EXPECT_EQ(release_count.load(), 0);
  }

  EXPECT_EQ(release_count.load(), 1);
}

TEST(PooledBufferAdoptTest, MoveTransfersOwnershipWithoutDoubleRelease) {
  uint8_t blob[16] = {};
  std::atomic<int> release_count{0};
  auto release_fn = +[](void* opaque) {
    static_cast<std::atomic<int>*>(opaque)->fetch_add(1);
  };

  {
    PooledBuffer src = PooledBuffer::adopt(blob, sizeof(blob), release_fn, &release_count);
    PooledBuffer dst(std::move(src));
    EXPECT_EQ(src.data(), nullptr);
    EXPECT_EQ(dst.data(), blob);
    EXPECT_EQ(release_count.load(), 0);
  }

  EXPECT_EQ(release_count.load(), 1);
}

TEST(PooledBufferAdoptTest, AssignDetachesFromExternalOwnerAndReleases) {
  uint8_t blob[32];
  for (size_t i = 0; i < sizeof(blob); ++i) {
    blob[i] = static_cast<uint8_t>(i);
  }
  std::atomic<int> release_count{0};
  auto release_fn = +[](void* opaque) {
    static_cast<std::atomic<int>*>(opaque)->fetch_add(1);
  };

  PooledBuffer b = PooledBuffer::adopt(blob, sizeof(blob), release_fn, &release_count);
  EXPECT_EQ(release_count.load(), 0);

  std::vector<uint8_t> replacement(64, 0x5A);
  b.assign(replacement.data(), replacement.size());

  // assign() must release the adopted buffer (so the external owner sees its
  // release callback) and then reacquire storage, this time from the heap /
  // pool — so data() now differs from blob and size matches replacement.
  EXPECT_EQ(release_count.load(), 1);
  EXPECT_NE(b.data(), blob);
  EXPECT_EQ(b.size(), replacement.size());
  for (size_t i = 0; i < replacement.size(); ++i) {
    EXPECT_EQ(b.data()[i], 0x5A);
  }

  // Destroying b should not re-invoke the external release.
  b.release();
  EXPECT_EQ(release_count.load(), 1);
}

TEST(PooledBufferAdoptTest, ReleaseOnDefaultConstructedIsNoOp) {
  PooledBuffer b;
  b.release();  // must not crash
  SUCCEED();
}

}  // namespace
}  // namespace memory
}  // namespace axon
