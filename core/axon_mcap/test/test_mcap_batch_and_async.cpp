// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

// Unit tests for McapWriterWrapper::write_batch() and AsyncMcapWriter.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <string>
#include <thread>
#include <vector>

#include "async_mcap_writer.hpp"
#include "mcap_writer_wrapper.hpp"

namespace fs = std::filesystem;

using axon::mcap_wrapper::AsyncMcapWriter;
using axon::mcap_wrapper::AsyncWriterConfig;
using axon::mcap_wrapper::BatchItem;
using axon::mcap_wrapper::Compression;
using axon::mcap_wrapper::CompressionLevel;
using axon::mcap_wrapper::McapWriterOptions;
using axon::mcap_wrapper::McapWriterWrapper;

namespace {

std::string unique_path(const std::string& prefix) {
  return "/tmp/" + prefix + "_" +
         std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".mcap";
}

}  // namespace

// ---------------------------------------------------------------------------
// write_batch tests
// ---------------------------------------------------------------------------

TEST(McapBatchWrite, EmptyBatchIsNoOp) {
  auto path = unique_path("batch_empty");
  McapWriterWrapper w;
  ASSERT_TRUE(w.open(path, McapWriterOptions{}));

  size_t written = 42;  // sentinel
  EXPECT_TRUE(w.write_batch(nullptr, 0, &written));
  EXPECT_EQ(written, 0U);

  BatchItem dummy;
  EXPECT_TRUE(w.write_batch(&dummy, 0, &written));
  EXPECT_EQ(written, 0U);

  w.close();
  fs::remove(path);
}

TEST(McapBatchWrite, WritesAllMessagesAndUpdatesStats) {
  auto path = unique_path("batch_all");
  McapWriterWrapper w;
  McapWriterOptions opts;
  opts.compression = Compression::None;
  ASSERT_TRUE(w.open(path, opts));

  uint16_t sid = w.register_schema("test/Data", "raw", "uint8[] data");
  uint16_t cid = w.register_channel("/test", "raw", sid);

  const size_t N = 128;
  std::vector<uint8_t> payload(256, 0xAB);
  std::vector<BatchItem> items(N);
  for (size_t i = 0; i < N; ++i) {
    items[i].channel_id = cid;
    items[i].sequence = static_cast<uint32_t>(i);
    items[i].log_time_ns = 1000000000ULL + i * 1000ULL;
    items[i].publish_time_ns = items[i].log_time_ns;
    items[i].data = payload.data();
    items[i].data_size = payload.size();
  }

  size_t written = 0;
  EXPECT_TRUE(w.write_batch(items.data(), items.size(), &written));
  EXPECT_EQ(written, N);

  auto s = w.get_statistics();
  EXPECT_EQ(s.messages_written, N);
  EXPECT_EQ(s.bytes_written, N * payload.size());

  w.close();
  EXPECT_TRUE(fs::exists(path));
  EXPECT_GT(fs::file_size(path), 0U);
  fs::remove(path);
}

TEST(McapBatchWrite, MixedBatchAndSingleProducesSameCount) {
  auto path = unique_path("batch_mixed");
  McapWriterWrapper w;
  ASSERT_TRUE(w.open(path, McapWriterOptions{}));

  uint16_t sid = w.register_schema("test/Data", "raw", "uint8[] data");
  uint16_t cid = w.register_channel("/test", "raw", sid);

  std::vector<uint8_t> payload(64, 0x55);
  // 10 singles
  for (int i = 0; i < 10; ++i) {
    ASSERT_TRUE(w.write(cid, 1000ULL + i, 1000ULL + i, payload.data(), payload.size()));
  }
  // one batch of 20
  std::vector<BatchItem> items(20);
  for (size_t i = 0; i < items.size(); ++i) {
    items[i].channel_id = cid;
    items[i].log_time_ns = 2000ULL + i;
    items[i].publish_time_ns = 2000ULL + i;
    items[i].data = payload.data();
    items[i].data_size = payload.size();
  }
  ASSERT_TRUE(w.write_batch(items.data(), items.size()));

  auto s = w.get_statistics();
  EXPECT_EQ(s.messages_written, 30U);
  EXPECT_EQ(s.bytes_written, 30U * payload.size());

  w.close();
  fs::remove(path);
}

TEST(McapBatchWrite, FailsWhenWriterClosed) {
  McapWriterWrapper w;
  BatchItem item;
  item.channel_id = 1;
  item.data = "abcd";
  item.data_size = 4;
  size_t written = 7;
  EXPECT_FALSE(w.write_batch(&item, 1, &written));
  EXPECT_EQ(written, 0U);
}

TEST(McapBatchWrite, LargeBatchWithCompression) {
  auto path = unique_path("batch_zstd");
  McapWriterWrapper w;
  McapWriterOptions opts;
  opts.compression = Compression::Zstd;
  opts.compression_preset = CompressionLevel::Fast;
  ASSERT_TRUE(w.open(path, opts));

  uint16_t sid = w.register_schema("test/Data", "raw", "uint8[] data");
  uint16_t cid = w.register_channel("/test", "raw", sid);

  const size_t N = 2000;
  std::vector<uint8_t> payload(4096, 0xCC);  // compressible
  std::vector<BatchItem> items(N);
  for (size_t i = 0; i < N; ++i) {
    items[i].channel_id = cid;
    items[i].log_time_ns = 1000ULL + i;
    items[i].publish_time_ns = 1000ULL + i;
    items[i].data = payload.data();
    items[i].data_size = payload.size();
  }
  size_t written = 0;
  EXPECT_TRUE(w.write_batch(items.data(), items.size(), &written));
  EXPECT_EQ(written, N);
  auto s = w.get_statistics();
  EXPECT_EQ(s.messages_written, N);
  w.close();
  // Compressed output should be smaller than logical bytes.
  auto fsz = fs::file_size(path);
  EXPECT_LT(fsz, N * payload.size());
  fs::remove(path);
}

// ---------------------------------------------------------------------------
// AsyncMcapWriter tests
// ---------------------------------------------------------------------------

TEST(AsyncWriter, OpenCloseEmpty) {
  auto path = unique_path("async_empty");
  AsyncMcapWriter w;
  ASSERT_TRUE(w.open(path, McapWriterOptions{}, AsyncWriterConfig{}));
  EXPECT_TRUE(w.is_open());
  w.close();
  EXPECT_FALSE(w.is_open());
  EXPECT_TRUE(fs::exists(path));
  fs::remove(path);
}

TEST(AsyncWriter, WritesAllMessages) {
  auto path = unique_path("async_all");
  AsyncMcapWriter w;
  McapWriterOptions opts;
  opts.compression = Compression::None;
  AsyncWriterConfig cfg;
  cfg.queue_capacity = 64;
  cfg.batch_size = 16;
  ASSERT_TRUE(w.open(path, opts, cfg));

  uint16_t sid = w.register_schema("test/Data", "raw", "uint8[] data");
  uint16_t cid = w.register_channel("/test", "raw", sid);

  const size_t N = 500;
  std::vector<uint8_t> payload(1024, 0x77);
  for (size_t i = 0; i < N; ++i) {
    ASSERT_TRUE(w.write(cid, i, 1000ULL + i, 1000ULL + i, payload.data(), payload.size()));
  }
  w.close();

  auto s = w.get_async_stats();
  EXPECT_EQ(s.enqueued, N);
  EXPECT_EQ(s.dequeued, N);
  EXPECT_EQ(s.dropped_full, 0U);
  EXPECT_GE(s.batches_written, 1U);

  auto ws = w.get_statistics();
  EXPECT_EQ(ws.messages_written, N);
  fs::remove(path);
}

TEST(AsyncWriter, DropOldestWhenFull) {
  // Tiny queue so dropping is guaranteed for back-to-back producers.
  auto path = unique_path("async_drop");
  AsyncMcapWriter w;
  McapWriterOptions opts;
  opts.compression = Compression::Zstd;  // slower -> helps back up the queue
  opts.compression_preset = CompressionLevel::Slowest;
  AsyncWriterConfig cfg;
  cfg.queue_capacity = 4;
  cfg.batch_size = 4;
  cfg.drop_oldest_on_full = true;
  ASSERT_TRUE(w.open(path, opts, cfg));

  uint16_t sid = w.register_schema("test/Data", "raw", "uint8[] data");
  uint16_t cid = w.register_channel("/test", "raw", sid);

  const size_t N = 2000;
  std::vector<uint8_t> payload(16 * 1024, 0xAB);
  for (size_t i = 0; i < N; ++i) {
    EXPECT_TRUE(w.write(cid, i, 1000ULL + i, 1000ULL + i, payload.data(), payload.size()));
  }
  w.close();

  auto s = w.get_async_stats();
  // Every call enqueues; older messages are displaced by dropped_full.
  EXPECT_EQ(s.enqueued, N);
  // After close() the queue is drained fully, so dequeued + dropped_full
  // must account for every enqueued message.
  EXPECT_EQ(s.dequeued + s.dropped_full, N);
  // With a tiny queue + slow compression, we expect at least one drop.
  EXPECT_GT(s.dropped_full, 0U);
  EXPECT_LE(s.peak_depth, cfg.queue_capacity);
  fs::remove(path);
}

TEST(AsyncWriter, BlocksWhenFullAndNoDrop) {
  auto path = unique_path("async_block");
  AsyncMcapWriter w;
  McapWriterOptions opts;
  opts.compression = Compression::None;
  AsyncWriterConfig cfg;
  cfg.queue_capacity = 8;
  cfg.batch_size = 4;
  cfg.drop_oldest_on_full = false;
  ASSERT_TRUE(w.open(path, opts, cfg));

  uint16_t sid = w.register_schema("test/Data", "raw", "uint8[] data");
  uint16_t cid = w.register_channel("/test", "raw", sid);

  const size_t N = 200;
  std::vector<uint8_t> payload(512, 0x42);
  for (size_t i = 0; i < N; ++i) {
    EXPECT_TRUE(w.write(cid, i, 1000ULL + i, 1000ULL + i, payload.data(), payload.size()));
  }
  w.close();

  auto s = w.get_async_stats();
  EXPECT_EQ(s.enqueued, N);
  EXPECT_EQ(s.dequeued, N);
  EXPECT_EQ(s.dropped_full, 0U);
  fs::remove(path);
}

TEST(AsyncWriter, MultiProducerNoLoss) {
  auto path = unique_path("async_multi");
  AsyncMcapWriter w;
  AsyncWriterConfig cfg;
  cfg.queue_capacity = 256;
  cfg.batch_size = 32;
  ASSERT_TRUE(w.open(path, McapWriterOptions{}, cfg));

  uint16_t sid = w.register_schema("test/Data", "raw", "uint8[] data");
  uint16_t cid = w.register_channel("/test", "raw", sid);

  const size_t threads = 4;
  const size_t per_thread = 500;
  std::vector<std::thread> ts;
  ts.reserve(threads);
  std::atomic<size_t> enqueued{0};
  std::vector<uint8_t> payload(256, 0x10);
  for (size_t t = 0; t < threads; ++t) {
    ts.emplace_back([&, t] {
      for (size_t i = 0; i < per_thread; ++i) {
        uint64_t ns = static_cast<uint64_t>(t) * 1000000ULL + i;
        if (w.write(cid, ns, ns, payload.data(), payload.size())) {
          enqueued.fetch_add(1, std::memory_order_relaxed);
        }
      }
    });
  }
  for (auto& th : ts) th.join();
  w.close();

  EXPECT_EQ(enqueued.load(), threads * per_thread);
  auto s = w.get_async_stats();
  EXPECT_EQ(s.enqueued, threads * per_thread);
  EXPECT_EQ(s.dequeued, threads * per_thread);
  EXPECT_EQ(s.dropped_full, 0U);
  fs::remove(path);
}

TEST(AsyncWriter, WaitUntilEmpty) {
  auto path = unique_path("async_wait");
  AsyncMcapWriter w;
  AsyncWriterConfig cfg;
  cfg.queue_capacity = 128;
  cfg.batch_size = 8;
  ASSERT_TRUE(w.open(path, McapWriterOptions{}, cfg));

  uint16_t sid = w.register_schema("test/Data", "raw", "uint8[] data");
  uint16_t cid = w.register_channel("/test", "raw", sid);

  std::vector<uint8_t> payload(128, 0x33);
  for (int i = 0; i < 300; ++i) {
    ASSERT_TRUE(w.write(cid, 1000ULL + i, 1000ULL + i, payload.data(), payload.size()));
  }
  w.wait_until_empty();
  auto s = w.get_async_stats();
  EXPECT_EQ(s.current_depth, 0U);
  EXPECT_EQ(s.enqueued, s.dequeued);
  w.close();
  fs::remove(path);
}

TEST(McapWriter, CompressionLevelPresetMapping) {
  // Verify that CompressionLevel preset reaches the writer without error
  // across Zstd / LZ4 and all levels.
  const Compression algos[] = {Compression::Zstd, Compression::Lz4};
  const CompressionLevel levels[] = {
    CompressionLevel::Fastest,
    CompressionLevel::Fast,
    CompressionLevel::Default,
    CompressionLevel::Slow,
    CompressionLevel::Slowest
  };
  for (auto c : algos) {
    for (auto lv : levels) {
      auto path = unique_path("level_map");
      McapWriterWrapper w;
      McapWriterOptions opts;
      opts.compression = c;
      opts.compression_preset = lv;
      ASSERT_TRUE(w.open(path, opts));
      uint16_t sid = w.register_schema("test/Data", "raw", "uint8[] data");
      uint16_t cid = w.register_channel("/test", "raw", sid);
      std::vector<uint8_t> p(1024, 0xEE);
      for (int i = 0; i < 10; ++i) {
        ASSERT_TRUE(w.write(cid, 1000ULL + i, 1000ULL + i, p.data(), p.size()));
      }
      w.close();
      EXPECT_TRUE(fs::exists(path));
      fs::remove(path);
    }
  }
}
