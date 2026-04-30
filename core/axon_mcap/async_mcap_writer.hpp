// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_MCAP_ASYNC_WRITER_HPP
#define AXON_MCAP_ASYNC_WRITER_HPP

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "mcap_writer_wrapper.hpp"

namespace axon {
namespace mcap_wrapper {

/**
 * Configuration for AsyncMcapWriter.
 */
struct AsyncWriterConfig {
  /// Maximum number of queued messages before back-pressure kicks in.
  /// The worker thread drains this queue; producers either block (when
  /// `drop_oldest_on_full == false`) or drop the oldest item.
  size_t queue_capacity = 1024;

  /// Maximum number of messages coalesced into a single write_batch() call.
  /// Larger values amortize the mutex / syscall overhead but increase worst
  /// case tail latency for individual messages.
  size_t batch_size = 64;

  /// When true, a full queue causes the OLDEST enqueued message to be
  /// dropped (O(1) policy for lossy real-time recording). When false,
  /// producers block until space is available (loss-free, but stalls
  /// the subscription thread).
  bool drop_oldest_on_full = false;
};

/**
 * Runtime statistics for AsyncMcapWriter.
 */
struct AsyncWriterStats {
  uint64_t enqueued = 0;         // total messages successfully enqueued
  uint64_t dequeued = 0;         // total messages successfully written by worker
  uint64_t dropped_full = 0;     // messages dropped because queue was full (drop policy)
  uint64_t batches_written = 0;  // number of write_batch() calls by worker
  uint64_t write_failures = 0;   // write_batch() calls that reported failure
  uint64_t current_depth = 0;    // current queue depth (snapshot)
  uint64_t peak_depth = 0;       // peak queue depth observed
};

/**
 * Thread-backed async MCAP writer.
 *
 * Why: the recorder hot path calls write() from subscription worker
 * threads. With compression enabled (Zstd/LZ4) those threads spend a
 * non-trivial fraction of their wall time inside the compressor. This
 * class moves that CPU work (and the disk syscall) to a dedicated
 * background thread by:
 *
 *   1. Copying the serialized payload into an internal buffer owned by the
 *      queue node (one malloc per enqueue — the buffer pool in
 *      core/axon_memory can further amortize this if used by callers).
 *   2. Enqueueing the node onto a mutex/condvar-protected queue.
 *   3. A single worker thread drains the queue in batches and forwards the
 *      messages to the underlying McapWriterWrapper::write_batch().
 *
 * Design notes:
 *   - The internal queue is intentionally simple (std::deque + mutex +
 *     condvar). A single consumer means lock contention is minimal; the
 *     producer side sees O(1) push + cond signal.
 *   - Batching is done on the consumer side, so the hot producer path
 *     never walks more than its own message.
 *   - `drop_oldest_on_full` enables a lossy real-time mode suited for
 *     high bandwidth topics (point clouds / images) where dropping a
 *     frame is preferable to stalling the middleware executor.
 *
 * Lifetime: call open()/start() before write(), close()/stop() before
 * destruction (destructor calls stop()). Thread-safe: write() can be
 * called from any number of producer threads.
 */
class AsyncMcapWriter {
public:
  AsyncMcapWriter();
  ~AsyncMcapWriter();

  AsyncMcapWriter(const AsyncMcapWriter&) = delete;
  AsyncMcapWriter& operator=(const AsyncMcapWriter&) = delete;
  AsyncMcapWriter(AsyncMcapWriter&&) = delete;
  AsyncMcapWriter& operator=(AsyncMcapWriter&&) = delete;

  /**
   * Open the underlying MCAP file and start the worker thread.
   *
   * @return true on success; false if the file could not be opened (see
   *         get_last_error()) or if already open.
   */
  bool open(
    const std::string& path, const McapWriterOptions& options = McapWriterOptions{},
    const AsyncWriterConfig& async_config = AsyncWriterConfig{}
  );

  /**
   * Flush any pending messages and close the file.
   *
   * Blocks until the worker thread has drained its queue. Safe to call
   * multiple times.
   */
  void close();

  bool is_open() const;

  /**
   * Register schema (forwarded to underlying writer under its mutex).
   * Must be called while the async writer is open.
   */
  uint16_t register_schema(
    const std::string& name, const std::string& encoding, const std::string& data
  );

  /**
   * Register channel (forwarded to underlying writer under its mutex).
   */
  uint16_t register_channel(
    const std::string& topic, const std::string& message_encoding, uint16_t schema_id
  );

  /**
   * Enqueue a message for asynchronous write.
   *
   * Copies `data` into an internal buffer and returns immediately. If the
   * queue is full, behavior depends on AsyncWriterConfig::drop_oldest_on_full:
   *   - true  : drops the oldest queued message; always succeeds (returns true).
   *   - false : blocks the caller until space is available or close() is called.
   *
   * @return true if the message was enqueued (or caller unblocked via new
   *         space); false if the writer was closed mid-wait or not open.
   */
  bool write(
    uint16_t channel_id, uint64_t log_time_ns, uint64_t publish_time_ns, const void* data,
    size_t data_size
  );

  bool write(
    uint16_t channel_id, uint32_t sequence, uint64_t log_time_ns, uint64_t publish_time_ns,
    const void* data, size_t data_size
  );

  /**
   * Block until the queue is drained. Useful for test synchronization and
   * orderly shutdown barriers.
   */
  void wait_until_empty();

  AsyncWriterStats get_async_stats() const;

  /// Forwarded statistics from the underlying writer.
  McapWriterWrapper::Statistics get_statistics() const;

  std::string get_last_error() const;

  std::string get_path() const;

private:
  struct QueuedMsg {
    uint16_t channel_id = 0;
    uint32_t sequence = 0;
    uint64_t log_time_ns = 0;
    uint64_t publish_time_ns = 0;
    std::vector<uint8_t> data;  // owned copy
  };

  void worker_loop();
  bool enqueue_(QueuedMsg&& msg);

  McapWriterWrapper writer_;
  AsyncWriterConfig async_config_;

  // Queue state.
  mutable std::mutex q_mu_;
  std::condition_variable not_full_;
  std::condition_variable not_empty_;
  std::deque<QueuedMsg> q_;

  // Stats (guarded by q_mu_ except where noted).
  uint64_t enqueued_ = 0;
  uint64_t dequeued_ = 0;
  uint64_t dropped_full_ = 0;
  uint64_t batches_written_ = 0;
  uint64_t write_failures_ = 0;
  uint64_t peak_depth_ = 0;
  size_t in_flight_ = 0;

  // Lifecycle.
  std::thread worker_;
  std::atomic<bool> running_{false};
  std::atomic<bool> open_{false};
  // Signals the worker to stop after draining the current queue.
  std::atomic<bool> stop_requested_{false};
};

}  // namespace mcap_wrapper
}  // namespace axon

#endif  // AXON_MCAP_ASYNC_WRITER_HPP
