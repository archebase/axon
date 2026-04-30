// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

// -----------------------------------------------------------------------------
// Micro-benchmark: AsyncMcapWriter vs synchronous McapWriterWrapper on a
// compressed recording workload.
//
// Measures the producer-observed latency of write() — this is what the
// recorder's subscription thread actually sees. The async path should have
// very low per-call latency (memcpy + queue push) while the sync path is
// dominated by compression (Zstd/LZ4).
//
// Run with:
//   ./bench_async_writer [iterations] [payload_bytes] [queue_cap]
// Defaults: 20'000 iterations, 64 KiB payload, queue_cap 1024.
// -----------------------------------------------------------------------------

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

#include "async_mcap_writer.hpp"
#include "mcap_writer_wrapper.hpp"

namespace fs = std::filesystem;
using axon::mcap_wrapper::AsyncMcapWriter;
using axon::mcap_wrapper::AsyncWriterConfig;
using axon::mcap_wrapper::Compression;
using axon::mcap_wrapper::CompressionLevel;
using axon::mcap_wrapper::McapWriterOptions;
using axon::mcap_wrapper::McapWriterWrapper;

namespace {

double seconds_between(
  std::chrono::steady_clock::time_point a, std::chrono::steady_clock::time_point b
) {
  return std::chrono::duration<double>(b - a).count();
}

std::vector<uint8_t> make_payload(size_t bytes) {
  std::vector<uint8_t> v(bytes, 0);
  for (size_t i = 0; i < bytes; ++i) {
    v[i] = static_cast<uint8_t>((i * 31 + 7) & 0xFF);
  }
  return v;
}

struct SyncResult {
  double producer_seconds;
  double total_seconds;
  size_t file_bytes;
};

SyncResult run_sync(
  const std::string& path, Compression comp, CompressionLevel level, size_t iters,
  const std::vector<uint8_t>& payload
) {
  McapWriterOptions opts;
  opts.compression = comp;
  opts.compression_preset = level;

  McapWriterWrapper w;
  w.open(path, opts);
  uint16_t sid = w.register_schema("bench/Payload", "raw", "uint8[] data");
  uint16_t cid = w.register_channel("/bench/topic", "raw", sid);

  auto t0 = std::chrono::steady_clock::now();
  for (size_t i = 0; i < iters; ++i) {
    uint64_t ns = static_cast<uint64_t>(i) * 1000;
    w.write(cid, ns, ns, payload.data(), payload.size());
  }
  auto t_prod = std::chrono::steady_clock::now();
  w.close();
  auto t_end = std::chrono::steady_clock::now();

  SyncResult r;
  r.producer_seconds = seconds_between(t0, t_prod);
  r.total_seconds = seconds_between(t0, t_end);
  std::error_code ec;
  r.file_bytes = ec ? 0 : static_cast<size_t>(fs::file_size(path, ec));
  if (ec) r.file_bytes = 0;
  return r;
}

struct AsyncResult {
  double producer_seconds;
  double total_seconds;
  size_t file_bytes;
  uint64_t dequeued;
  uint64_t peak_depth;
  uint64_t dropped_full;
};

AsyncResult run_async(
  const std::string& path, Compression comp, CompressionLevel level, size_t iters, size_t queue_cap,
  const std::vector<uint8_t>& payload
) {
  McapWriterOptions opts;
  opts.compression = comp;
  opts.compression_preset = level;

  AsyncWriterConfig async_cfg;
  async_cfg.queue_capacity = queue_cap;
  async_cfg.batch_size = 64;
  async_cfg.drop_oldest_on_full = false;

  AsyncMcapWriter w;
  w.open(path, opts, async_cfg);
  uint16_t sid = w.register_schema("bench/Payload", "raw", "uint8[] data");
  uint16_t cid = w.register_channel("/bench/topic", "raw", sid);

  auto t0 = std::chrono::steady_clock::now();
  for (size_t i = 0; i < iters; ++i) {
    uint64_t ns = static_cast<uint64_t>(i) * 1000;
    w.write(cid, ns, ns, payload.data(), payload.size());
  }
  auto t_prod = std::chrono::steady_clock::now();
  w.close();  // waits for worker drain
  auto t_end = std::chrono::steady_clock::now();

  auto s = w.get_async_stats();

  AsyncResult r;
  r.producer_seconds = seconds_between(t0, t_prod);
  r.total_seconds = seconds_between(t0, t_end);
  std::error_code ec;
  r.file_bytes = ec ? 0 : static_cast<size_t>(fs::file_size(path, ec));
  if (ec) r.file_bytes = 0;
  r.dequeued = s.dequeued;
  r.peak_depth = s.peak_depth;
  r.dropped_full = s.dropped_full;
  return r;
}

const char* comp_name(Compression c) {
  switch (c) {
    case Compression::None:
      return "none";
    case Compression::Zstd:
      return "zstd";
    case Compression::Lz4:
      return "lz4 ";
  }
  return "?";
}

}  // namespace

int main(int argc, char** argv) {
  size_t iters = 20000;
  size_t payload_bytes = 64 * 1024;
  size_t queue_cap = 1024;
  if (argc > 1) iters = static_cast<size_t>(std::strtoull(argv[1], nullptr, 10));
  if (argc > 2) payload_bytes = static_cast<size_t>(std::strtoull(argv[2], nullptr, 10));
  if (argc > 3) queue_cap = static_cast<size_t>(std::strtoull(argv[3], nullptr, 10));
  if (iters == 0) iters = 1;
  if (payload_bytes == 0) payload_bytes = 1;
  if (queue_cap == 0) queue_cap = 1;

  auto payload = make_payload(payload_bytes);
  fs::path tmpdir = fs::temp_directory_path() / "axon_bench_async";
  fs::create_directories(tmpdir);

  std::printf(
    "--- axon_mcap/bench_async_writer ---\n"
    "iters=%zu payload=%zu bytes queue_cap=%zu\n",
    iters,
    payload_bytes,
    queue_cap
  );

  const Compression algos[] = {Compression::Zstd, Compression::Lz4};
  const CompressionLevel levels[] = {CompressionLevel::Fastest, CompressionLevel::Default};

  for (auto c : algos) {
    for (auto lv : levels) {
      fs::path sync_path = tmpdir / ("sync.mcap");
      fs::remove(sync_path);
      auto sr = run_sync(sync_path.string(), c, lv, iters, payload);
      double sync_prod_mbs =
        (static_cast<double>(iters) * payload_bytes / (1024.0 * 1024.0)) / sr.producer_seconds;
      std::printf(
        "[sync ] comp=%s level=%d  producer=%.3fs (%.1f MiB/s)  total=%.3fs  out=%.1f MiB\n",
        comp_name(c),
        static_cast<int>(lv),
        sr.producer_seconds,
        sync_prod_mbs,
        sr.total_seconds,
        static_cast<double>(sr.file_bytes) / (1024.0 * 1024.0)
      );
      fs::remove(sync_path);

      fs::path async_path = tmpdir / ("async.mcap");
      fs::remove(async_path);
      auto ar = run_async(async_path.string(), c, lv, iters, queue_cap, payload);
      double async_prod_mbs =
        (static_cast<double>(iters) * payload_bytes / (1024.0 * 1024.0)) / ar.producer_seconds;
      std::printf(
        "[async] comp=%s level=%d  producer=%.3fs (%.1f MiB/s)  total=%.3fs  out=%.1f MiB  "
        "peak_depth=%lu dropped=%lu\n",
        comp_name(c),
        static_cast<int>(lv),
        ar.producer_seconds,
        async_prod_mbs,
        ar.total_seconds,
        static_cast<double>(ar.file_bytes) / (1024.0 * 1024.0),
        static_cast<unsigned long>(ar.peak_depth),
        static_cast<unsigned long>(ar.dropped_full)
      );
      fs::remove(async_path);
    }
  }

  fs::remove_all(tmpdir);
  return 0;
}
