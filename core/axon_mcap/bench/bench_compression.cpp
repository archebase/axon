// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

// -----------------------------------------------------------------------------
// Standalone micro-benchmark: MCAP writer throughput by compression algorithm
// and compression level. Compares None / Zstd (Fastest..Slowest) / LZ4
// (Fastest..Slowest) on both compressible (sensor-like) and random payloads.
//
// Also exercises the batch write path (write_batch) vs the per-message
// write() path to quantify batching gains independently of compression.
//
// Run with:
//   ./bench_compression [iterations] [payload_bytes] [batch_size]
// Defaults: 20'000 iterations, 64 KiB payload, batch_size 64.
// -----------------------------------------------------------------------------

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <random>
#include <string>
#include <vector>

#include "mcap_writer_wrapper.hpp"

namespace fs = std::filesystem;
using axon::mcap_wrapper::BatchItem;
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

// "Compressible" sensor-like payload: sinusoidal int16 samples quantized to
// bytes. Mimics the spatial coherence typical of depth images / point cloud
// XYZ channels that the recorder actually sees.
std::vector<uint8_t> make_compressible(size_t bytes) {
  std::vector<uint8_t> v(bytes, 0);
  for (size_t i = 0; i < bytes; ++i) {
    double t = static_cast<double>(i) / 128.0;
    double s = 127.5 * (1.0 + 0.85 * std::sin(t * 0.07) + 0.10 * std::sin(t * 0.73));
    int x = static_cast<int>(s);
    if (x < 0) x = 0;
    if (x > 255) x = 255;
    v[i] = static_cast<uint8_t>(x);
  }
  return v;
}

// Random payload: incompressible, represents post-compression data like
// already-JPEG frames or encrypted blobs.
std::vector<uint8_t> make_random(size_t bytes) {
  std::vector<uint8_t> v(bytes, 0);
  std::mt19937_64 rng(0xC0FFEE);
  for (size_t i = 0; i < bytes; ++i) {
    v[i] = static_cast<uint8_t>(rng() & 0xFF);
  }
  return v;
}

struct BenchResult {
  double seconds = 0.0;
  size_t file_bytes = 0;
  size_t logical_bytes = 0;
  bool ok = false;
};

// Write `iters` messages using per-message writer.write().
BenchResult run_single_write(
  const std::string& path, Compression comp, CompressionLevel level, size_t iters,
  const std::vector<uint8_t>& payload
) {
  McapWriterOptions opts;
  opts.compression = comp;
  opts.compression_preset = level;
  opts.chunk_size = 4 * 1024 * 1024;

  McapWriterWrapper w;
  if (!w.open(path, opts)) {
    return {};
  }
  uint16_t sid = w.register_schema("bench/Payload", "raw", "uint8[] data");
  uint16_t cid = w.register_channel("/bench/topic", "raw", sid);

  auto t0 = std::chrono::steady_clock::now();
  for (size_t i = 0; i < iters; ++i) {
    uint64_t ns = static_cast<uint64_t>(i) * 1000;
    w.write(cid, ns, ns, payload.data(), payload.size());
  }
  w.close();
  auto t1 = std::chrono::steady_clock::now();

  BenchResult r;
  r.seconds = seconds_between(t0, t1);
  r.logical_bytes = iters * payload.size();
  std::error_code ec;
  auto sz = fs::file_size(path, ec);
  r.file_bytes = ec ? 0 : static_cast<size_t>(sz);
  r.ok = !ec;
  return r;
}

// Write `iters` messages using writer.write_batch() in chunks of batch_size.
BenchResult run_batch_write(
  const std::string& path, Compression comp, CompressionLevel level, size_t iters,
  size_t batch_size, const std::vector<uint8_t>& payload
) {
  McapWriterOptions opts;
  opts.compression = comp;
  opts.compression_preset = level;
  opts.chunk_size = 4 * 1024 * 1024;

  McapWriterWrapper w;
  if (!w.open(path, opts)) {
    return {};
  }
  uint16_t sid = w.register_schema("bench/Payload", "raw", "uint8[] data");
  uint16_t cid = w.register_channel("/bench/topic", "raw", sid);

  std::vector<BatchItem> items(batch_size);

  auto t0 = std::chrono::steady_clock::now();
  size_t written = 0;
  while (written < iters) {
    size_t take = std::min(batch_size, iters - written);
    for (size_t i = 0; i < take; ++i) {
      uint64_t ns = static_cast<uint64_t>(written + i) * 1000;
      items[i].channel_id = cid;
      items[i].sequence = 0;
      items[i].log_time_ns = ns;
      items[i].publish_time_ns = ns;
      items[i].data = payload.data();
      items[i].data_size = payload.size();
    }
    w.write_batch(items.data(), take, nullptr);
    written += take;
  }
  w.close();
  auto t1 = std::chrono::steady_clock::now();

  BenchResult r;
  r.seconds = seconds_between(t0, t1);
  r.logical_bytes = iters * payload.size();
  std::error_code ec;
  auto sz = fs::file_size(path, ec);
  r.file_bytes = ec ? 0 : static_cast<size_t>(sz);
  r.ok = !ec;
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

const char* level_name(CompressionLevel l) {
  switch (l) {
    case CompressionLevel::Fastest:
      return "Fastest";
    case CompressionLevel::Fast:
      return "Fast   ";
    case CompressionLevel::Default:
      return "Default";
    case CompressionLevel::Slow:
      return "Slow   ";
    case CompressionLevel::Slowest:
      return "Slowest";
  }
  return "?";
}

void print_result(
  const char* mode, Compression comp, CompressionLevel level, const BenchResult& r, size_t iters,
  size_t payload_bytes
) {
  double mib_in = static_cast<double>(r.logical_bytes) / (1024.0 * 1024.0);
  double mib_out = static_cast<double>(r.file_bytes) / (1024.0 * 1024.0);
  double ratio = (r.file_bytes > 0) ? (static_cast<double>(r.logical_bytes) / r.file_bytes) : 0.0;
  double mbs = mib_in / (r.seconds > 0.0 ? r.seconds : 1e-9);
  std::printf(
    "  [%s] comp=%s level=%s iters=%zu payload=%zuB  "
    "%.3fs  throughput=%.1f MiB/s  in=%.1f MiB  out=%.1f MiB  ratio=%.2fx\n",
    mode,
    comp_name(comp),
    level_name(level),
    iters,
    payload_bytes,
    r.seconds,
    mbs,
    mib_in,
    mib_out,
    ratio
  );
}

}  // namespace

int main(int argc, char** argv) {
  size_t iters = 20000;
  size_t payload_bytes = 64 * 1024;
  size_t batch_size = 64;

  if (argc > 1) iters = static_cast<size_t>(std::strtoull(argv[1], nullptr, 10));
  if (argc > 2) payload_bytes = static_cast<size_t>(std::strtoull(argv[2], nullptr, 10));
  if (argc > 3) batch_size = static_cast<size_t>(std::strtoull(argv[3], nullptr, 10));
  if (iters == 0) iters = 1;
  if (payload_bytes == 0) payload_bytes = 1;
  if (batch_size == 0) batch_size = 1;

  std::printf(
    "--- axon_mcap/bench_compression ---\n"
    "iters=%zu payload=%zu bytes batch=%zu\n",
    iters,
    payload_bytes,
    batch_size
  );

  auto compressible = make_compressible(payload_bytes);
  auto random_payload = make_random(payload_bytes);

  const Compression algos[] = {Compression::None, Compression::Zstd, Compression::Lz4};
  const CompressionLevel levels[] = {
    CompressionLevel::Fastest, CompressionLevel::Default, CompressionLevel::Slowest
  };

  struct Dataset {
    const char* name;
    const std::vector<uint8_t>* data;
  };
  Dataset datasets[] = {
    {"compressible (sensor-like)", &compressible},
    {"random (incompressible)", &random_payload},
  };

  fs::path tmpdir = fs::temp_directory_path() / "axon_bench_mcap";
  fs::create_directories(tmpdir);

  for (const auto& ds : datasets) {
    std::printf("\n== Dataset: %s ==\n", ds.name);
    std::printf("-- write() per message --\n");
    for (auto c : algos) {
      // For None, level is ignored; run only once.
      const CompressionLevel* lv_begin = levels;
      const CompressionLevel* lv_end = levels + sizeof(levels) / sizeof(levels[0]);
      if (c == Compression::None) {
        lv_end = lv_begin + 1;
      }
      for (auto lv_it = lv_begin; lv_it != lv_end; ++lv_it) {
        fs::path out = tmpdir / ("single_" + std::string(comp_name(c)) + "_" +
                                 std::to_string(static_cast<int>(*lv_it)) + ".mcap");
        fs::remove(out);
        auto r = run_single_write(out.string(), c, *lv_it, iters, *ds.data);
        print_result("single", c, *lv_it, r, iters, payload_bytes);
        fs::remove(out);
      }
    }

    std::printf("-- write_batch() size=%zu --\n", batch_size);
    for (auto c : algos) {
      const CompressionLevel* lv_begin = levels;
      const CompressionLevel* lv_end = levels + sizeof(levels) / sizeof(levels[0]);
      if (c == Compression::None) {
        lv_end = lv_begin + 1;
      }
      for (auto lv_it = lv_begin; lv_it != lv_end; ++lv_it) {
        fs::path out = tmpdir / ("batch_" + std::string(comp_name(c)) + "_" +
                                 std::to_string(static_cast<int>(*lv_it)) + ".mcap");
        fs::remove(out);
        auto r = run_batch_write(out.string(), c, *lv_it, iters, batch_size, *ds.data);
        print_result("batch ", c, *lv_it, r, iters, payload_bytes);
        fs::remove(out);
      }
    }
  }

  fs::remove_all(tmpdir);
  return 0;
}
