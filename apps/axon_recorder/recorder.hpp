#ifndef AXON_RECORDER_HPP
#define AXON_RECORDER_HPP

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "http_server.hpp"
#include "plugin_loader.hpp"
#include "recording_session.hpp"
#include "state_machine.hpp"
#include "task_config.hpp"
#include "worker_thread_pool.hpp"

namespace axon {
namespace recorder {

/**
 * Message data structure for SPSC queue
 * Uses unique_ptr for zero-copy ownership transfer
 */
struct QueuedMessage {
  std::string topic_name;
  std::string message_type;
  uint64_t timestamp_ns;
  std::vector<uint8_t> data;

  QueuedMessage() = default;

  QueuedMessage(
    const char* topic, const uint8_t* msg_data, size_t msg_size, const char* type, uint64_t ts
  )
      : topic_name(topic)
      , message_type(type)
      , timestamp_ns(ts)
      , data(msg_data, msg_data + msg_size) {}

  // Move-only for efficiency
  QueuedMessage(const QueuedMessage&) = delete;
  QueuedMessage& operator=(const QueuedMessage&) = delete;
  QueuedMessage(QueuedMessage&&) = default;
  QueuedMessage& operator=(QueuedMessage&&) = default;
};

/**
 * Subscription configuration
 */
struct SubscriptionConfig {
  std::string topic_name;
  std::string message_type;

  // Batch writing configuration
  size_t batch_size = 1;        // Number of messages to batch before writing (1 = immediate)
  int flush_interval_ms = 100;  // Maximum time to wait before flushing (ms)
};

/**
 * Dataset configuration
 */
struct DatasetConfig {
  std::string path;
  std::string mode = "append";                                           // "create" or "append"
  std::string stats_file_path = "/data/recordings/recorder_stats.json";  // Stats output file
};

/**
 * Recording configuration
 */
struct RecordingConfig {
  double max_disk_usage_gb = 100.0;
};

/**
 * S3 upload configuration for edge uploader
 */
struct S3Config {
  std::string endpoint_url;          // S3-compatible endpoint (empty for AWS S3)
  std::string bucket;                // Bucket name (required when upload enabled)
  std::string region = "us-east-1";  // AWS region
  bool use_ssl = true;
  bool verify_ssl = true;
};

/**
 * Retry configuration for uploads
 */
struct RetryConfig {
  int max_retries = 5;
  int initial_delay_ms = 1000;
  int max_delay_ms = 300000;
  double exponential_base = 2.0;
  bool jitter = true;
};

/**
 * Edge upload configuration
 */
struct UploadConfig {
  bool enabled = false;
  S3Config s3;
  RetryConfig retry;
  int num_workers = 2;
  std::string state_db_path = "/var/lib/axon/uploader_state.db";
  bool delete_after_upload = true;
  std::string failed_uploads_dir = "/data/failed_uploads/";
  double warn_pending_gb = 8.0;
  double alert_pending_gb = 20.0;
};

/**
 * Logging configuration
 */
struct LoggingConfig {
  // Console sink
  bool console_enabled = true;
  bool console_colors = true;
  std::string console_level = "info";  // debug, info, warn, error, fatal

  // File sink
  bool file_enabled = false;
  std::string file_level = "debug";
  std::string file_directory = "/var/log/axon";
  std::string file_pattern = "axon_%Y%m%d_%H%M%S.log";
  std::string file_format = "json";  // json or text
  size_t rotation_size_mb = 100;
  size_t max_files = 10;
  bool rotate_at_midnight = true;
};

/**
 * Recorder configuration (unified for both axon_recorder and full recording service)
 */
struct RecorderConfig {
  // Basic recorder settings
  std::string output_file = "output.mcap";
  std::string plugin_path;
  std::vector<SubscriptionConfig> subscriptions;

  // Queue configuration
  size_t queue_capacity = 1024;
  size_t num_worker_threads = 1;

  // MCAP options
  std::string profile = "ros2";
  std::string compression = "zstd";
  int compression_level = 3;

  // Extended configuration for full recording service
  DatasetConfig dataset;
  RecordingConfig recording;
  LoggingConfig logging;
  UploadConfig upload;
};

/**
 * Main recorder class
 *
 * Manages plugin loading, message subscriptions, SPSC queues, state machine,
 * and recording sessions.
 *
 * Architecture:
 * 1. Load ROS plugin (e.g., ros2_plugin.so)
 * 2. Subscribe to topics via plugin's subscribe() function
 * 3. Plugin callbacks push messages to MPSC queue
 * 4. Worker thread pops messages and writes to MCAP via RecordingSession
 * 5. StateManager manages recording state (IDLE -> READY -> RECORDING -> PAUSED)
 * 6. RecordingSession manages MCAP file lifecycle for each recording task
 */
class AxonRecorder {
public:
  AxonRecorder();
  ~AxonRecorder();

  // Non-copyable, non-movable
  AxonRecorder(const AxonRecorder&) = delete;
  AxonRecorder& operator=(const AxonRecorder&) = delete;

  /**
   * Initialize recorder with configuration
   *
   * @param config Recorder configuration
   * @return true on success, false on failure
   */
  bool initialize(const RecorderConfig& config);

  /**
   * Start recording
   *
   * Loads plugin, subscribes to topics, starts worker thread.
   *
   * @return true on success, false on failure
   */
  bool start();

  /**
   * Stop recording
   *
   * Stops plugin, unsubscribes, flushes queue, closes MCAP.
   */
  void stop();

  /**
   * Check if recorder is running
   */
  bool is_running() const;

  /**
   * Get current recorder state
   */
  RecorderState get_state() const;

  /**
   * Get current recorder state as string
   */
  std::string get_state_string() const;

  /**
   * Set task configuration for metadata injection
   * Must be called before start() to enable metadata
   */
  void set_task_config(const TaskConfig& config);

  /**
   * Get task configuration (if set)
   * @return Pointer to task config, or nullptr if not set
   */
  const TaskConfig* get_task_config() const;

  /**
   * Get the current recording session (for testing/monitoring)
   */
  RecordingSession* get_recording_session();

  /**
   * Get last error message
   */
  std::string get_last_error() const;

  /**
   * Start HTTP RPC server
   *
   * @param host Host address to bind to (default: "0.0.0.0")
   * @param port Port number to listen on (default: 8080)
   * @return true on success, false on failure
   */
  bool start_http_server(const std::string& host = "0.0.0.0", uint16_t port = 8080);

  /**
   * Stop HTTP RPC server
   */
  void stop_http_server();

  /**
   * Check if HTTP server is running
   */
  bool is_http_server_running() const;

  /**
   * Get statistics
   */
  struct Statistics {
    uint64_t messages_received;
    uint64_t messages_written;
    uint64_t messages_dropped;
    uint64_t bytes_written;
  };
  Statistics get_statistics() const;

  /**
   * Message callback from plugin (public for C callback wrapper)
   * Called from plugin thread, pushes to queue
   */
  void on_message(
    const char* topic_name, const uint8_t* message_data, size_t message_size,
    const char* message_type, uint64_t timestamp
  );

private:
  /**
   * Message handler callback for WorkerThreadPool
   * Called by per-topic worker threads to process messages
   */
  bool message_handler(
    const std::string& topic, int64_t timestamp_ns, const uint8_t* data, size_t data_size,
    uint32_t sequence
  );

  /**
   * Handle state transition callbacks
   */
  void on_state_transition(RecorderState from, RecorderState to);

  /**
   * Register schemas and channels for subscriptions
   */
  bool register_topics();

  /**
   * Setup subscriptions via plugin
   */
  bool setup_subscriptions();

  /**
   * Get subscription configuration for a topic
   */
  const SubscriptionConfig* get_subscription_config(const std::string& topic_name) const;

  /**
   * Convert ABI status to string
   */
  static const char* status_to_string(AxonStatus status);

  /**
   * Helper to set error message (with mutex lock)
   */
  void set_error_helper(const std::string& error);

  RecorderConfig config_;
  PluginLoader plugin_loader_;

  // State machine for recording lifecycle management
  StateManager state_manager_;

  // Recording session manages MCAP file lifecycle
  std::unique_ptr<RecordingSession> recording_session_;

  // Task configuration for metadata injection
  std::optional<TaskConfig> task_config_;

  // Worker thread pool manages per-topic workers
  // Use unique_ptr with placement new for late initialization with custom config
  std::unique_ptr<WorkerThreadPool> worker_pool_;

  std::atomic<bool> running_;

  // HTTP RPC server
  std::unique_ptr<HttpServer> http_server_;

  // Error handling
  mutable std::mutex error_mutex_;
  std::string last_error_;
};

}  // namespace recorder
}  // namespace axon

#endif  // AXON_RECORDER_HPP
