// Define MCAP_IMPLEMENTATION in exactly one .cpp file
// This must be before including ANY mcap headers to get the implementation
#define MCAP_IMPLEMENTATION
#include "mcap_writer_wrapper.hpp"

#include <mcap/reader.hpp>  // Also need reader implementation for mcap_validator
#include <mcap/writer.hpp>

#include <chrono>
#include <filesystem>
#include <mutex>
#include <shared_mutex>
#include <string>

// Logging infrastructure (optional - only if axon_logging is linked)
#ifdef AXON_HAS_LOGGING
#define AXON_LOG_COMPONENT "axon_mcap"
#include <axon_log_macros.hpp>
#else
// No-op macros when logging is not available
#define AXON_LOG_DEBUG(msg) \
  do { \
  } while (0)
#define AXON_LOG_INFO(msg) \
  do { \
  } while (0)
#define AXON_LOG_WARN(msg) \
  do { \
  } while (0)
#define AXON_LOG_ERROR(msg) \
  do { \
  } while (0)
#define AXON_LOG_FATAL(msg) \
  do { \
  } while (0)
namespace axon {
namespace logging {
template<typename T>
inline std::string kv(const char* key, const T& value) {
  (void)key;
  (void)value;
  return "";
}
}  // namespace logging
}  // namespace axon
#endif

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
      AXON_LOG_WARN("Attempted to open already-open writer");
      return false;
    }

    AXON_LOG_DEBUG("Opening MCAP file" << axon::logging::kv("path", path));

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

    // Create parent directories if they don't exist
    std::filesystem::path file_path(path);
    if (file_path.has_parent_path()) {
      std::error_code ec;
      std::filesystem::create_directories(file_path.parent_path(), ec);
      if (ec) {
        last_error_ = "Failed to create directory: " + ec.message();
        AXON_LOG_ERROR("Failed to create directory" << axon::logging::kv("error", ec.message()));
        return false;
      }
    }

    // Open the file
    auto status = writer_.open(path, mcap_opts);
    if (!status.ok()) {
      last_error_ = "Failed to open MCAP file: " + status.message;
      AXON_LOG_ERROR(
        "Failed to open MCAP file" << axon::logging::kv("path", path)
                                   << axon::logging::kv("error", status.message)
      );
      return false;
    }

    is_open_ = true;

    // Reset statistics
    messages_written_.store(0, std::memory_order_relaxed);
    bytes_written_.store(0, std::memory_order_relaxed);
    schemas_registered_.store(0, std::memory_order_relaxed);
    channels_registered_.store(0, std::memory_order_relaxed);

    AXON_LOG_INFO("MCAP file opened" << axon::logging::kv("path", path));
    return true;
  }

  void close() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!is_open_) {
      return;
    }

    AXON_LOG_DEBUG("Closing MCAP file" << axon::logging::kv("path", path_));
    writer_.close();
    is_open_ = false;
    AXON_LOG_INFO(
      "MCAP file closed" << axon::logging::kv("messages_written", messages_written_.load())
                         << axon::logging::kv("bytes_written", bytes_written_.load())
    );
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
      AXON_LOG_WARN("Cannot register schema - writer not open");
      return 0;
    }

    mcap::Schema schema(name, encoding, data);
    writer_.addSchema(schema);

    schemas_registered_.fetch_add(1, std::memory_order_relaxed);
    AXON_LOG_DEBUG(
      "Schema registered" << axon::logging::kv("name", name)
                          << axon::logging::kv("schema_id", schema.id)
    );
    return schema.id;
  }

  uint16_t register_channel(
    const std::string& topic, const std::string& message_encoding, uint16_t schema_id,
    const std::unordered_map<std::string, std::string>& metadata
  ) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!is_open_) {
      last_error_ = "Writer not open";
      AXON_LOG_WARN("Cannot register channel - writer not open");
      return 0;
    }

    mcap::Channel channel(topic, message_encoding, schema_id);

    // Add metadata
    for (const auto& [key, value] : metadata) {
      channel.metadata[key] = value;
    }

    writer_.addChannel(channel);

    channels_registered_.fetch_add(1, std::memory_order_relaxed);
    AXON_LOG_DEBUG(
      "Channel registered" << axon::logging::kv("topic", topic)
                           << axon::logging::kv("channel_id", channel.id)
    );
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

      // Rate-limit error logging to avoid flooding logs on repeated failures
      // (e.g., disk full scenario)
      write_errors_.fetch_add(1, std::memory_order_relaxed);
      auto now = std::chrono::steady_clock::now();
      auto last = last_write_error_log_.load(std::memory_order_relaxed);
      if (now - last > std::chrono::seconds(1)) {
        // Atomically update the last log time
        if (last_write_error_log_.compare_exchange_weak(last, now)) {
          auto error_count = write_errors_.exchange(0, std::memory_order_relaxed);
          AXON_LOG_ERROR(
            "Failed to write message" << axon::logging::kv("error", status.message)
                                      << axon::logging::kv("error_count", error_count)
          );
        }
      }
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

  bool write_metadata(
    const std::string& name, const std::unordered_map<std::string, std::string>& metadata
  ) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!is_open_) {
      last_error_ = "Writer not open";
      AXON_LOG_WARN("Cannot write metadata - writer not open");
      return false;
    }

    mcap::Metadata mcap_metadata;
    mcap_metadata.name = name;
    for (const auto& [key, value] : metadata) {
      mcap_metadata.metadata[key] = value;
    }

    auto status = writer_.write(mcap_metadata);
    if (!status.ok()) {
      last_error_ = "Failed to write metadata: " + status.message;
      AXON_LOG_ERROR("Failed to write metadata" << axon::logging::kv("error", status.message));
      return false;
    }

    AXON_LOG_DEBUG("Metadata written" << axon::logging::kv("name", name));
    return true;
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
      channels_registered_.load(std::memory_order_relaxed)};
  }

  std::string get_path() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return path_;
  }

private:
  mcap::McapWriter writer_;
  std::string path_;
  McapWriterOptions options_;

  mutable std::mutex mutex_;        // Protects metadata operations
  mutable std::mutex write_mutex_;  // Protects write operations (separate for performance)

  std::atomic<bool> is_open_{false};
  std::string last_error_;

  // Statistics (atomic for lock-free reads)
  std::atomic<uint64_t> messages_written_{0};
  std::atomic<uint64_t> bytes_written_{0};
  std::atomic<uint64_t> schemas_registered_{0};
  std::atomic<uint64_t> channels_registered_{0};

  // Rate limiting for error logging (avoid flooding logs on repeated failures)
  std::atomic<uint64_t> write_errors_{0};
  std::atomic<std::chrono::steady_clock::time_point> last_write_error_log_{
    std::chrono::steady_clock::time_point{}};
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

bool McapWriterWrapper::write_metadata(
  const std::string& name, const std::unordered_map<std::string, std::string>& metadata
) {
  return impl_->write_metadata(name, metadata);
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
