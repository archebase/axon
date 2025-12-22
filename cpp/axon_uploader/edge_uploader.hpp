#ifndef AXON_EDGE_UPLOADER_HPP
#define AXON_EDGE_UPLOADER_HPP

#include "retry_handler.hpp"
#include "s3_client.hpp"
#include "upload_queue.hpp"
#include "upload_state_manager.hpp"

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace axon {
namespace uploader {

/**
 * Configuration for the Edge Uploader
 */
struct UploaderConfig {
  // S3 configuration
  S3Config s3;

  // Retry configuration
  RetryConfig retry;

  // Worker configuration
  int num_workers = 2;

  // State persistence
  std::string state_db_path = "/var/lib/axon/uploader_state.db";

  // Cleanup policy
  bool delete_after_upload = true;                         // Delete device copy after successful upload
  std::string failed_uploads_dir = "/data/failed_uploads/"; // Move permanently failed files here

  // Backpressure thresholds (bytes)
  uint64_t warn_pending_bytes = 8ULL * 1024 * 1024 * 1024;   // 8GB - emit warning
  uint64_t alert_pending_bytes = 20ULL * 1024 * 1024 * 1024; // 20GB - alert operator
};

/**
 * Uploader statistics
 */
struct UploaderStats {
  std::atomic<uint64_t> files_pending{0};
  std::atomic<uint64_t> files_uploading{0};
  std::atomic<uint64_t> files_completed{0};
  std::atomic<uint64_t> files_failed{0};
  std::atomic<uint64_t> bytes_uploaded{0};
  std::atomic<uint64_t> current_upload_bytes_per_sec{0};
};

/**
 * Health status of the uploader
 */
struct HealthStatus {
  bool healthy;
  std::string message;
  uint64_t pending_count;
  uint64_t pending_bytes;
  uint64_t failed_count;
  std::chrono::system_clock::time_point last_successful_upload;
};

/**
 * Callback for upload events
 */
using UploadCallback = std::function<void(const std::string& task_id, bool success, const std::string& error)>;

/**
 * Edge Uploader - Main orchestrator for uploading MCAP files to S3
 *
 * Features:
 * - Worker threads for concurrent uploads
 * - Crash recovery via SQLite state persistence
 * - Automatic retry with exponential backoff
 * - Upload order: MCAP first, JSON last (JSON signals completion)
 * - Backpressure alerts when queue grows too large
 *
 * Usage:
 *   EdgeUploader uploader(config);
 *   uploader.start();  // Performs crash recovery + starts workers
 *
 *   // In recording finalize:
 *   uploader.enqueue(mcap_path, json_path, task_id, factory, device, checksum);
 *
 *   // On shutdown:
 *   uploader.stop();
 */
class EdgeUploader {
public:
  /**
   * Create Edge Uploader with configuration
   *
   * @param config Uploader configuration
   */
  explicit EdgeUploader(const UploaderConfig& config);
  ~EdgeUploader();

  // Non-copyable, non-movable
  EdgeUploader(const EdgeUploader&) = delete;
  EdgeUploader& operator=(const EdgeUploader&) = delete;
  EdgeUploader(EdgeUploader&&) = delete;
  EdgeUploader& operator=(EdgeUploader&&) = delete;

  /**
   * Start the uploader
   *
   * Performs crash recovery:
   * 1. Queries State DB for incomplete uploads from previous run
   * 2. Re-queues them for upload
   * 3. Starts worker threads
   */
  void start();

  /**
   * Stop the uploader
   *
   * Signals workers to stop and waits for them to finish.
   * Does NOT wait for pending uploads to complete.
   */
  void stop();

  /**
   * Check if uploader is running
   */
  bool isRunning() const;

  /**
   * Enqueue a recording for upload
   *
   * Called by recorder after finalization.
   * Thread-safe.
   *
   * @param mcap_path Path to MCAP file
   * @param json_path Path to sidecar JSON file
   * @param task_id Task identifier
   * @param factory_id Factory identifier
   * @param device_id Device identifier
   * @param checksum_sha256 Pre-computed SHA-256 checksum
   */
  void enqueue(
      const std::string& mcap_path, const std::string& json_path, const std::string& task_id,
      const std::string& factory_id, const std::string& device_id, const std::string& checksum_sha256
  );

  /**
   * Get current statistics
   */
  const UploaderStats& stats() const;

  /**
   * Get health status
   */
  HealthStatus getHealthStatus() const;

  /**
   * Set callback for upload events
   */
  void setCallback(UploadCallback callback);

  /**
   * Get configuration
   */
  const UploaderConfig& config() const;

private:
  // Worker thread function
  void workerLoop(int worker_id);

  // Process a single upload item
  void processItem(const UploadItem& item);

  // Upload a single file (MCAP or JSON)
  bool uploadSingleFile(
      const std::string& local_path, const std::string& s3_key,
      const std::string& checksum, const std::string& task_id
  );

  // Construct S3 key from components
  std::string constructS3Key(
      const std::string& factory_id, const std::string& device_id,
      const std::string& task_id, const std::string& extension
  );

  // Get current date string for S3 key
  static std::string currentDateString();

  // Handle successful upload
  void onUploadSuccess(const UploadItem& item);

  // Handle failed upload
  void onUploadFailure(const UploadItem& item, const std::string& error, bool retryable);

  // Cleanup local files after successful upload
  void cleanupLocalFiles(const std::string& mcap_path, const std::string& json_path);

  // Move failed files to failed_uploads directory
  void moveToFailedDir(const std::string& mcap_path, const std::string& json_path);

  // Check and emit backpressure warnings
  void checkBackpressure();

  UploaderConfig config_;
  std::unique_ptr<UploadQueue> queue_;
  std::unique_ptr<S3Client> s3_client_;
  std::unique_ptr<UploadStateManager> state_manager_;
  std::unique_ptr<RetryHandler> retry_handler_;

  std::vector<std::thread> workers_;
  std::atomic<bool> running_{false};

  UploaderStats stats_;
  std::chrono::system_clock::time_point last_successful_upload_;
  mutable std::mutex stats_mutex_;

  UploadCallback callback_;
  std::mutex callback_mutex_;
};

}  // namespace uploader
}  // namespace axon

#endif  // AXON_EDGE_UPLOADER_HPP

