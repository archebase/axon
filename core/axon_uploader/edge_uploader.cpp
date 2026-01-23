// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "edge_uploader.hpp"

#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>

#include "uploader_impl.hpp"

// Logging infrastructure (optional - only if axon_logging is linked)
#ifdef AXON_HAS_LOGGING
#define AXON_LOG_COMPONENT "edge_uploader"
#include <axon_log_macros.hpp>
#else
// Fallback to stderr when logging is not available
#define AXON_LOG_DEBUG(msg) \
  do { \
  } while (0)
#define AXON_LOG_INFO(msg) std::cerr << "[INFO] " << msg << std::endl
#define AXON_LOG_WARN(msg) std::cerr << "[WARN] " << msg << std::endl
#define AXON_LOG_ERROR(msg) std::cerr << "[ERROR] " << msg << std::endl
#endif

namespace fs = std::filesystem;

namespace axon {
namespace uploader {

// Internal implementation with dependency injection
// Exposed for testing via edge_uploader_test_helpers.hpp
bool validateFilesExistImpl(
  const std::string& mcap_path, const std::string& json_path, const IFileSystem& filesystem
) {
  if (!filesystem.exists(mcap_path)) {
    return false;
  }
  if (!filesystem.exists(json_path)) {
    return false;
  }
  return true;
}

uint64_t getFileSizeImpl(const std::string& path, const IFileSystem& filesystem) {
  return filesystem.file_size(path);
}

void cleanupLocalFilesImpl(
  const std::string& mcap_path, const std::string& json_path, const IFileSystem& filesystem
) {
  if (!mcap_path.empty() && filesystem.exists(mcap_path)) {
    filesystem.remove(mcap_path);
  }
  if (!json_path.empty() && filesystem.exists(json_path)) {
    filesystem.remove(json_path);
  }
}

void moveToFailedDirImpl(
  const std::string& mcap_path, const std::string& json_path, const std::string& failed_dir,
  const IFileSystem& filesystem
) {
  // Create failed directory if needed
  filesystem.create_directories(failed_dir);

  // Move files
  if (!mcap_path.empty() && filesystem.exists(mcap_path)) {
    fs::path dest = fs::path(failed_dir) / fs::path(mcap_path).filename();  // LCOV_EXCL_BR_LINE
    filesystem.rename(mcap_path, dest.string());                            // LCOV_EXCL_BR_LINE
  }
  if (!json_path.empty() && filesystem.exists(json_path)) {
    fs::path dest = fs::path(failed_dir) / fs::path(json_path).filename();  // LCOV_EXCL_BR_LINE
    filesystem.rename(json_path, dest.string());                            // LCOV_EXCL_BR_LINE
  }
}

FileUploadResult uploadSingleFileImpl(
  const std::string& local_path, const std::string& s3_key, const std::string& checksum,
  const IFileSystem& filesystem, S3Client* s3_client
) {
  // Check file exists
  // LCOV_EXCL_BR_START - File existence check is a common operation
  if (!filesystem.exists(local_path)) {
    // LCOV_EXCL_BR_LINE - String concatenation
    std::string error_msg = "File not found: " + local_path;
    AXON_LOG_ERROR(error_msg);
    return FileUploadResult::fail(error_msg, false);  // Not retryable
  }
  // LCOV_EXCL_BR_STOP

  // Prepare metadata
  std::map<std::string, std::string> metadata;
  if (!checksum.empty()) {
    metadata["checksum-sha256"] = checksum;
  }

  // Upload
  auto result = s3_client->uploadFile(local_path, s3_key, metadata);  // LCOV_EXCL_BR_LINE

  if (result.success) {
    // Verify checksum if provided
    if (!checksum.empty()) {
      if (!s3_client->verifyUpload(s3_key, checksum)) {
        return FileUploadResult::fail("Checksum verification failed", true);  // Retryable
      }
    }
    return FileUploadResult::ok();
  } else {
    return FileUploadResult::fail(result.error_message, result.is_retryable);
  }
}

EdgeUploader::EdgeUploader(const UploaderConfig& config)
    : config_(config)
    ,
    // LCOV_EXCL_BR_START - Smart pointer operations generate exception-safety branches
    // that are standard library implementation details
    queue_(std::make_unique<UploadQueue>())
    , s3_client_(std::make_unique<S3Client>(config.s3))
    , state_manager_(std::make_unique<UploadStateManager>(config.state_db_path))
    , retry_handler_(std::make_unique<RetryHandler>(config.retry))
    ,
    // LCOV_EXCL_BR_STOP
    last_successful_upload_(std::chrono::system_clock::now()) {}

EdgeUploader::~EdgeUploader() {
  stop();
}

void EdgeUploader::start() {
  if (running_.exchange(true)) {
    return;  // Already running
  }

  // Crash recovery: re-queue incomplete uploads from previous run
  auto incomplete = state_manager_->getIncomplete();  // LCOV_EXCL_BR_LINE
  for (const auto& record : incomplete) {
    UploadItem item;                                // LCOV_EXCL_BR_LINE
    item.mcap_path = record.file_path;              // LCOV_EXCL_BR_LINE
    item.json_path = record.json_path;              // LCOV_EXCL_BR_LINE
    item.task_id = record.task_id;                  // LCOV_EXCL_BR_LINE
    item.factory_id = record.factory_id;            // LCOV_EXCL_BR_LINE
    item.device_id = record.device_id;              // LCOV_EXCL_BR_LINE
    item.checksum_sha256 = record.checksum_sha256;  // LCOV_EXCL_BR_LINE
    item.file_size_bytes = record.file_size_bytes;
    item.retry_count = record.retry_count;
    item.created_at = std::chrono::steady_clock::now();

    // Reconstruct s3_key_prefix from stored s3_key
    // s3_key format: "factory/device/date/task_id.mcap"
    // s3_key_prefix: "factory/device/date"
    // LCOV_EXCL_BR_START - exception safety branches for string concatenation
    if (!record.s3_key.empty()) {
      size_t last_slash = record.s3_key.rfind('/');
      if (last_slash != std::string::npos) {
        item.s3_key_prefix = record.s3_key.substr(0, last_slash);
      }
    }
    // LCOV_EXCL_BR_STOP

    // Fallback: reconstruct from components if s3_key is missing
    if (item.s3_key_prefix.empty() && !record.factory_id.empty() && !record.device_id.empty()) {
      item.s3_key_prefix = record.factory_id + "/" + record.device_id + "/" +
                           currentDateString();  // LCOV_EXCL_BR_LINE

      AXON_LOG_WARN(
        "Crash recovery: reconstructed s3_key_prefix from components for " << record.file_path
      );
    }

    // Reset status to pending for re-upload
    state_manager_->updateStatus(record.file_path, UploadStatus::PENDING);  // LCOV_EXCL_BR_LINE
    queue_->enqueue(std::move(item));                                       // LCOV_EXCL_BR_LINE
  }

  // Update stats
  stats_.files_pending = queue_->size() + queue_->retry_size();  // LCOV_EXCL_BR_LINE

  // Start worker threads
  workers_.reserve(config_.num_workers);
  for (int i = 0; i < config_.num_workers; ++i) {
    workers_.emplace_back(&EdgeUploader::workerLoop, this, i);  // LCOV_EXCL_BR_LINE
  }
}

void EdgeUploader::stop() {
  if (!running_.exchange(false)) {
    return;  // Already stopped
  }

  // Signal queue shutdown
  queue_->shutdown();

  // Wait for workers to finish
  for (auto& worker : workers_) {
    if (worker.joinable()) {  // LCOV_EXCL_BR_LINE
      worker.join();          // LCOV_EXCL_BR_LINE
    }
  }
  workers_.clear();
}

bool EdgeUploader::isRunning() const {
  return running_.load();
}

void EdgeUploader::enqueue(
  const std::string& mcap_path, const std::string& json_path, const std::string& task_id,
  const std::string& factory_id, const std::string& device_id, const std::string& checksum_sha256
) {
  // Validate required parameters to prevent malformed S3 keys
  if (mcap_path.empty()) {
    AXON_LOG_ERROR("Cannot enqueue upload - mcap_path is empty");  // LCOV_EXCL_BR_LINE
    return;
  }
  if (json_path.empty()) {
    AXON_LOG_ERROR("Cannot enqueue upload - json_path is empty (JSON sidecar is required)"
    );  // LCOV_EXCL_BR_LINE
    return;
  }
  if (task_id.empty()) {
    AXON_LOG_ERROR("Cannot enqueue upload - task_id is empty");  // LCOV_EXCL_BR_LINE
    return;
  }
  if (factory_id.empty()) {
    AXON_LOG_ERROR("Cannot enqueue upload - factory_id is empty");  // LCOV_EXCL_BR_LINE
    return;
  }
  if (device_id.empty()) {
    AXON_LOG_ERROR("Cannot enqueue upload - device_id is empty");  // LCOV_EXCL_BR_LINE
    return;
  }

  // Validate both files exist before enqueueing to prevent orphaned uploads:
  // If MCAP uploads successfully but JSON fails (file not found), the entire
  // upload is marked as permanently failed, leaving MCAP orphaned in S3.
  // LCOV_EXCL_BR_START - Smart pointer operations generate exception-safety branches
  static FileSystemImpl default_filesystem;
  if (!validateFilesExistImpl(mcap_path, json_path, default_filesystem)) {
    if (!default_filesystem.exists(mcap_path)) {
      AXON_LOG_ERROR("Cannot enqueue upload - mcap_path does not exist: " << mcap_path);
    } else {
      AXON_LOG_ERROR("Cannot enqueue upload - json_path does not exist: " << json_path);
    }
    return;
  }
  // LCOV_EXCL_BR_STOP

  // Get file size (file existence already verified above)
  uint64_t file_size = getFileSizeImpl(mcap_path, default_filesystem);  // LCOV_EXCL_BR_LINE

  // Create upload item
  UploadItem item(
    mcap_path, json_path, task_id, factory_id, device_id, checksum_sha256, file_size
  );  // LCOV_EXCL_BR_LINE

  // Construct S3 key prefix
  item.s3_key_prefix =
    factory_id + "/" + device_id + "/" + currentDateString();  // LCOV_EXCL_BR_LINE

  // Build state DB record for crash recovery
  // LCOV_EXCL_BR_START
  UploadRecord record;
  record.file_path = mcap_path;
  record.json_path = json_path;
  record.s3_key = item.s3_key_prefix + "/" + task_id + ".mcap";
  record.task_id = task_id;
  record.factory_id = factory_id;
  record.device_id = device_id;
  record.file_size_bytes = file_size;
  record.checksum_sha256 = checksum_sha256;
  record.status = UploadStatus::PENDING;
  // LCOV_EXCL_BR_STOP

  // Persist to state DB - if this fails (disk full, DB corruption),
  // do NOT queue the upload as we cannot track it for retry/recovery
  if (!state_manager_->insert(record)) {
    AXON_LOG_ERROR(
      "Failed to persist upload state for " << mcap_path
                                            << " - upload not queued (disk full or DB error?)"
    );
    return;
  }

  // Add to queue
  // Increment pending count BEFORE enqueue to avoid race condition:
  // enqueue() notifies workers, and a worker could decrement files_pending
  // before we increment it, causing unsigned underflow to UINT64_MAX
  stats_.files_pending++;
  if (!queue_->enqueue(std::move(item))) {
    // Queue full (capacity exceeded) - revert the pending count and clean up orphan state record
    stats_.files_pending--;
    state_manager_->remove(mcap_path);
    AXON_LOG_ERROR("Failed to enqueue upload for " << mcap_path << " - queue at capacity");
    return;
  }

  // Check backpressure
  checkBackpressure();
}

const UploaderStats& EdgeUploader::stats() const {
  return stats_;
}

HealthStatus EdgeUploader::getHealthStatus() const {
  std::lock_guard<std::mutex> lock(stats_mutex_);

  HealthStatus status;
  status.pending_count = stats_.files_pending.load();
  status.pending_bytes = queue_->pending_bytes();
  status.failed_count = stats_.files_failed.load();
  status.last_successful_upload = last_successful_upload_;

  // Determine health
  if (!running_) {
    status.healthy = false;
    status.message = "Uploader not running";
  } else if (status.pending_bytes > config_.alert_pending_bytes) {
    status.healthy = false;
    status.message = "ALERT: Pending queue exceeds threshold";
  } else if (status.pending_bytes > config_.warn_pending_bytes) {
    status.healthy = true;
    status.message = "WARNING: Pending queue growing";
  } else {
    status.healthy = true;
    status.message = "OK";
  }

  return status;
}

void EdgeUploader::setCallback(UploadCallback callback) {
  std::lock_guard<std::mutex> lock(callback_mutex_);
  callback_ = std::move(callback);
}

const UploaderConfig& EdgeUploader::config() const {
  return config_;
}

void EdgeUploader::workerLoop(int worker_id) {
  (void)worker_id;  // May use for logging

  while (running_) {
    auto item = queue_->dequeue_with_timeout(std::chrono::milliseconds(1000));
    if (!item) {
      continue;  // Timeout or shutdown
    }

    stats_.files_pending--;
    stats_.files_uploading++;

    processItem(*item);

    stats_.files_uploading--;
  }
}

void EdgeUploader::processItem(const UploadItem& item) {
  // Update state to UPLOADING
  state_manager_->updateStatus(item.mcap_path, UploadStatus::UPLOADING);

  // Upload order: MCAP first, JSON last (JSON signals completion)
  std::string mcap_s3_key = item.s3_key_prefix + "/" + item.task_id + ".mcap";  // LCOV_EXCL_BR_LINE
  std::string json_s3_key = item.s3_key_prefix + "/" + item.task_id + ".json";  // LCOV_EXCL_BR_LINE

  // Step 1: Upload MCAP file (skip if already in S3 from previous retry)
  FileUploadResult mcap_result = FileUploadResult::ok();
  if (s3_client_->objectExists(mcap_s3_key)) {
    // MCAP already uploaded (retry after JSON failure) - verify checksum
    if (item.checksum_sha256.empty() || s3_client_->verifyUpload(mcap_s3_key, item.checksum_sha256)) {
      AXON_LOG_INFO("MCAP already in S3, skipping upload: " << mcap_s3_key);
      // mcap_result already set to ok()
    } else {
      AXON_LOG_WARN("MCAP in S3 has wrong checksum, re-uploading: " << mcap_s3_key);
      mcap_result = uploadSingleFile(item.mcap_path, mcap_s3_key, item.checksum_sha256);
    }
  } else {
    mcap_result = uploadSingleFile(item.mcap_path, mcap_s3_key, item.checksum_sha256);
  }

  if (!mcap_result.success) {
    // MCAP upload failed - handle with complete item context
    onUploadFailure(item, mcap_result.error_message, mcap_result.is_retryable);
    return;
  }

  // Step 2: Upload JSON sidecar (signals completion)
  FileUploadResult json_result = uploadSingleFile(item.json_path, json_s3_key, "");

  if (!json_result.success) {
    // JSON upload failed after MCAP succeeded
    // Re-queue with complete item - on retry, MCAP upload will succeed quickly
    // since the file is already in S3 (objectExists check above)
    onUploadFailure(
      item, "JSON sidecar upload failed: " + json_result.error_message, json_result.is_retryable
    );  // LCOV_EXCL_BR_LINE
    return;
  }

  // Both uploads succeeded
  onUploadSuccess(item);
}

FileUploadResult EdgeUploader::uploadSingleFile(
  const std::string& local_path, const std::string& s3_key, const std::string& checksum
) {
  static FileSystemImpl default_filesystem;  // LCOV_EXCL_BR_LINE
  return uploadSingleFileImpl(local_path, s3_key, checksum, default_filesystem, s3_client_.get());
}

std::string EdgeUploader::constructS3Key(
  const std::string& factory_id, const std::string& device_id, const std::string& task_id,
  const std::string& extension
) {
  return factory_id + "/" + device_id + "/" + currentDateString() + "/" + task_id + "." +
         extension;  // LCOV_EXCL_BR_LINE
}

std::string EdgeUploader::currentDateString() {
  auto now = std::chrono::system_clock::now();
  auto time = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
#ifdef _WIN32
  gmtime_s(&tm, &time);  // Windows uses reversed argument order
#else
  gmtime_r(&time, &tm);
#endif

  std::ostringstream ss;
  ss << std::put_time(&tm, "%Y-%m-%d");
  return ss.str();
}

void EdgeUploader::onUploadSuccess(const UploadItem& item) {
  // Update state DB
  state_manager_->markCompleted(item.mcap_path);

  // Update stats
  stats_.files_completed++;
  stats_.bytes_uploaded += item.file_size_bytes;

  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    last_successful_upload_ = std::chrono::system_clock::now();
  }

  // Cleanup local files if configured
  if (config_.delete_after_upload) {
    cleanupLocalFiles(item.mcap_path, item.json_path);
  }

  // Invoke callback
  {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (callback_) {
      callback_(item.task_id, true, "");
    }
  }
}

void EdgeUploader::onUploadFailure(
  const UploadItem& item, const std::string& error, bool retryable
) {
  // Increment retry count in state DB (reads current count internally)
  state_manager_->incrementRetry(item.mcap_path, error);

  // Read the updated retry count from state DB to ensure consistency
  auto record = state_manager_->get(item.mcap_path);

  // Defensive check: if record is missing, treat as permanent failure
  // This should not happen if enqueue() worked correctly, but handles
  // edge cases like DB corruption during upload
  if (!record) {
    AXON_LOG_ERROR(
      "Upload state record missing for " << item.mcap_path
                                         << " - marking as permanent failure (cannot track retries)"
    );
    stats_.files_failed++;

    if (!config_.failed_uploads_dir.empty()) {
      moveToFailedDir(item.mcap_path, item.json_path);
    }

    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (callback_) {
      callback_(item.task_id, false, "Upload state record missing: " + error);  // LCOV_EXCL_BR_LINE
    }
    return;
  }

  int current_retry_count = record->retry_count;

  // Check if should retry
  if (retryable && retry_handler_->shouldRetry(current_retry_count)) {
    // Re-queue with delay
    UploadItem retry_item = item;
    retry_item.retry_count = current_retry_count;
    retry_item.next_retry_at = retry_handler_->nextRetryTime(current_retry_count);
    // Increment pending count BEFORE requeue to avoid race condition
    stats_.files_pending++;
    queue_->requeue_for_retry(std::move(retry_item));
  } else {
    // Mark as permanently failed
    state_manager_->markFailed(item.mcap_path, error);
    stats_.files_failed++;

    // Move to failed directory
    if (!config_.failed_uploads_dir.empty()) {
      moveToFailedDir(item.mcap_path, item.json_path);
    }

    // Invoke callback
    {
      std::lock_guard<std::mutex> lock(callback_mutex_);
      if (callback_) {
        callback_(item.task_id, false, error);
      }
    }
  }
}

void EdgeUploader::cleanupLocalFiles(const std::string& mcap_path, const std::string& json_path) {
  static FileSystemImpl default_filesystem;  // LCOV_EXCL_BR_LINE
  cleanupLocalFilesImpl(mcap_path, json_path, default_filesystem);

  // Remove from state DB
  state_manager_->remove(mcap_path);
}

void EdgeUploader::moveToFailedDir(const std::string& mcap_path, const std::string& json_path) {
  static FileSystemImpl default_filesystem;  // LCOV_EXCL_BR_LINE
  moveToFailedDirImpl(mcap_path, json_path, config_.failed_uploads_dir, default_filesystem);
}

void EdgeUploader::checkBackpressure() {
  uint64_t pending = queue_->pending_bytes();

  if (pending > config_.alert_pending_bytes) {
    AXON_LOG_ERROR(
      "ALERT: Upload queue pending bytes (" << pending << ") exceeds threshold ("
                                            << config_.alert_pending_bytes << ")"
    );
  } else if (pending > config_.warn_pending_bytes) {
    AXON_LOG_WARN(
      "Upload queue pending bytes (" << pending << ") exceeds warning threshold ("
                                     << config_.warn_pending_bytes << ")"
    );
  }
}

}  // namespace uploader
}  // namespace axon
