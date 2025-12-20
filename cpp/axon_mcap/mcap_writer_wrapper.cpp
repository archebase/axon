// Define MCAP_IMPLEMENTATION in exactly one .cpp file
#define MCAP_IMPLEMENTATION
#include "mcap_writer_wrapper.hpp"

#include <mcap/writer.hpp>

#include <chrono>
#include <iostream>
#include <mutex>
#include <shared_mutex>

namespace axon {
namespace mcap_wrapper {

// Convert our compression enum to MCAP's
static mcap::Compression to_mcap_compression(Compression compression) {
  switch (compression) {
    case Compression::Zstd:
      return mcap::Compression::Zstd;
    case Compression::Lz4:
      return mcap::Compression::Lz4;
    case Compression::None:
    default:
      return mcap::Compression::None;
  }
}

/**
 * Implementation class (PIMPL pattern)
 *
 * This hides the MCAP implementation details from the header.
 */
class McapWriterWrapper::Impl {
public:
  Impl() = default;

  ~Impl() {
    close();
  }

  bool open(const std::string& path, const McapWriterOptions& options) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (is_open_) {
      last_error_ = "Writer already open";
      return false;
    }

    path_ = path;
    options_ = options;

    // Configure MCAP writer options
    mcap::McapWriterOptions mcap_opts(options.profile);
    mcap_opts.compression = to_mcap_compression(options.compression);
    mcap_opts.chunkSize = options.chunk_size;

    // Set compression level if supported
    if (options.compression == Compression::Zstd) {
      mcap_opts.compressionLevel = mcap::CompressionLevel(options.compression_level);
    }

    // Open the file
    auto status = writer_.open(path, mcap_opts);
    if (!status.ok()) {
      last_error_ = "Failed to open MCAP file: " + status.message;
      return false;
    }

    is_open_ = true;
    
    // Reset statistics
    messages_written_.store(0, std::memory_order_relaxed);
    bytes_written_.store(0, std::memory_order_relaxed);
    schemas_registered_.store(0, std::memory_order_relaxed);
    channels_registered_.store(0, std::memory_order_relaxed);
    
    return true;
  }

  void close() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!is_open_) {
      return;
    }

    writer_.close();
    is_open_ = false;
  }

  bool is_open() const {
    return is_open_.load(std::memory_order_acquire);
  }

  uint16_t register_schema(
    const std::string& name, const std::string& encoding, const std::string& data
  ) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!is_open_) {
      last_error_ = "Writer not open";
      return 0;
    }

    mcap::Schema schema(name, encoding, data);
    writer_.addSchema(schema);

    schemas_registered_.fetch_add(1, std::memory_order_relaxed);
    return schema.id;
  }

  uint16_t register_channel(
    const std::string& topic, const std::string& message_encoding, uint16_t schema_id,
    const std::unordered_map<std::string, std::string>& metadata
  ) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!is_open_) {
      last_error_ = "Writer not open";
      return 0;
    }

    mcap::Channel channel(topic, message_encoding, schema_id);

    // Add metadata
    for (const auto& [key, value] : metadata) {
      channel.metadata[key] = value;
    }

    writer_.addChannel(channel);

    channels_registered_.fetch_add(1, std::memory_order_relaxed);
    return channel.id;
  }

  bool write(
    uint16_t channel_id, uint64_t log_time_ns, uint64_t publish_time_ns, const void* data,
    size_t data_size
  ) {
    return write(channel_id, 0, log_time_ns, publish_time_ns, data, data_size);
  }

  bool write(
    uint16_t channel_id, uint32_t sequence, uint64_t log_time_ns, uint64_t publish_time_ns,
    const void* data, size_t data_size
  ) {
    // Use shared lock for reading is_open_, exclusive lock for writing
    // Actually, mcap::McapWriter is not thread-safe, so we need exclusive lock
    std::lock_guard<std::mutex> lock(write_mutex_);

    if (!is_open_.load(std::memory_order_acquire)) {
      // #region agent log - H2
      std::cerr << "[DEBUG-WRITER-H2] Write failed: writer not open" << std::endl;
      // #endregion
      return false;
    }

    mcap::Message msg;
    msg.channelId = channel_id;
    msg.sequence = sequence;
    msg.logTime = log_time_ns;
    msg.publishTime = publish_time_ns;
    msg.data = reinterpret_cast<const std::byte*>(data);
    msg.dataSize = data_size;

    auto status = writer_.write(msg);
    if (!status.ok()) {
      last_error_ = "Failed to write message: " + status.message;
      // #region agent log - H2
      std::cerr << "[DEBUG-WRITER-H2] Write failed with status: " << status.message << std::endl;
      // #endregion
      return false;
    }

    messages_written_.fetch_add(1, std::memory_order_relaxed);
    bytes_written_.fetch_add(data_size, std::memory_order_relaxed);
    return true;
  }

  void flush() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!is_open_) {
      return;
    }

    // Force a chunk boundary
    // Note: mcap::McapWriter doesn't have a public flush() method,
    // but closing chunks happens automatically based on chunk_size
  }

  std::string get_last_error() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_error_;
  }

  McapWriterWrapper::Statistics get_statistics() const {
    return McapWriterWrapper::Statistics{
      messages_written_.load(std::memory_order_relaxed),
      bytes_written_.load(std::memory_order_relaxed),
      schemas_registered_.load(std::memory_order_relaxed),
      channels_registered_.load(std::memory_order_relaxed)
    };
  }

  std::string get_path() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return path_;
  }

private:
  mcap::McapWriter writer_;
  std::string path_;
  McapWriterOptions options_;

  mutable std::mutex mutex_;       // Protects metadata operations
  mutable std::mutex write_mutex_; // Protects write operations (separate for performance)

  std::atomic<bool> is_open_{false};
  std::string last_error_;

  // Statistics (atomic for lock-free reads)
  std::atomic<uint64_t> messages_written_{0};
  std::atomic<uint64_t> bytes_written_{0};
  std::atomic<uint64_t> schemas_registered_{0};
  std::atomic<uint64_t> channels_registered_{0};
};

// =============================================================================
// McapWriterWrapper public implementation
// =============================================================================

McapWriterWrapper::McapWriterWrapper()
    : impl_(std::make_unique<Impl>()) {}

McapWriterWrapper::~McapWriterWrapper() = default;

bool McapWriterWrapper::open(const std::string& path, const McapWriterOptions& options) {
  return impl_->open(path, options);
}

void McapWriterWrapper::close() {
  impl_->close();
}

bool McapWriterWrapper::is_open() const {
  return impl_->is_open();
}

uint16_t McapWriterWrapper::register_schema(
  const std::string& name, const std::string& encoding, const std::string& data
) {
  return impl_->register_schema(name, encoding, data);
}

uint16_t McapWriterWrapper::register_channel(
  const std::string& topic, const std::string& message_encoding, uint16_t schema_id,
  const std::unordered_map<std::string, std::string>& metadata
) {
  return impl_->register_channel(topic, message_encoding, schema_id, metadata);
}

bool McapWriterWrapper::write(
  uint16_t channel_id, uint64_t log_time_ns, uint64_t publish_time_ns, const void* data,
  size_t data_size
) {
  return impl_->write(channel_id, log_time_ns, publish_time_ns, data, data_size);
}

bool McapWriterWrapper::write(
  uint16_t channel_id, uint32_t sequence, uint64_t log_time_ns, uint64_t publish_time_ns,
  const void* data, size_t data_size
) {
  return impl_->write(channel_id, sequence, log_time_ns, publish_time_ns, data, data_size);
}

void McapWriterWrapper::flush() {
  impl_->flush();
}

std::string McapWriterWrapper::get_last_error() const {
  return impl_->get_last_error();
}

McapWriterWrapper::Statistics McapWriterWrapper::get_statistics() const {
  return impl_->get_statistics();
}

std::string McapWriterWrapper::get_path() const {
  return impl_->get_path();
}

}  // namespace mcap_wrapper
}  // namespace axon
