#include "edge_uploader.hpp"

#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

// Logging infrastructure (optional - only if axon_logging is linked)
#ifdef AXON_HAS_LOGGING
#define AXON_LOG_COMPONENT "edge_uploader"
#include <axon_log_macros.hpp>
#else
// Fallback to stderr when logging is not available
#define AXON_LOG_DEBUG(msg) do {} while(0)
#define AXON_LOG_INFO(msg) std::cerr << "[INFO] " << msg << std::endl
#define AXON_LOG_WARN(msg) std::cerr << "[WARN] " << msg << std::endl
#define AXON_LOG_ERROR(msg) std::cerr << "[ERROR] " << msg << std::endl
#endif

namespace fs = std::filesystem;

namespace axon {
namespace uploader {

EdgeUploader::EdgeUploader(const UploaderConfig& config)
    : config_(config),
      queue_(std::make_unique<UploadQueue>()),
      s3_client_(std::make_unique<S3Client>(config.s3)),
      state_manager_(std::make_unique<UploadStateManager>(config.state_db_path)),
      retry_handler_(std::make_unique<RetryHandler>(config.retry)),
      last_successful_upload_(std::chrono::system_clock::now()) {}

EdgeUploader::~EdgeUploader() { stop(); }

void EdgeUploader::start() {
  if (running_.exchange(true)) {
    return;  // Already running
  }

  // Crash recovery: re-queue incomplete uploads from previous run
  auto incomplete = state_manager_->getIncomplete();
  for (const auto& record : incomplete) {
    UploadItem item;
    item.mcap_path = record.file_path;
    item.json_path = record.json_path;
    item.task_id = record.task_id;
    item.factory_id = record.factory_id;
    item.device_id = record.device_id;
    item.checksum_sha256 = record.checksum_sha256;
    item.file_size_bytes = record.file_size_bytes;
    item.retry_count = record.retry_count;
    item.created_at = std::chrono::steady_clock::now();

    // Reset status to pending for re-upload
    state_manager_->updateStatus(record.file_path, UploadStatus::PENDING);
    queue_->enqueue(std::move(item));
  }

  // Update stats
  stats_.files_pending = queue_->size() + queue_->retry_size();

  // Start worker threads
  workers_.reserve(config_.num_workers);
  for (int i = 0; i < config_.num_workers; ++i) {
    workers_.emplace_back(&EdgeUploader::workerLoop, this, i);
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
    if (worker.joinable()) {
      worker.join();
    }
  }
  workers_.clear();
}

bool EdgeUploader::isRunning() const { return running_.load(); }

void EdgeUploader::enqueue(
    const std::string& mcap_path, const std::string& json_path, const std::string& task_id,
    const std::string& factory_id, const std::string& device_id, const std::string& checksum_sha256
) {
  // Validate required parameters to prevent malformed S3 keys
  if (mcap_path.empty()) {
    AXON_LOG_ERROR("Cannot enqueue upload - mcap_path is empty");
    return;
  }
  if (task_id.empty()) {
    AXON_LOG_ERROR("Cannot enqueue upload - task_id is empty");
    return;
  }
  if (factory_id.empty()) {
    AXON_LOG_ERROR("Cannot enqueue upload - factory_id is empty");
    return;
  }
  if (device_id.empty()) {
    AXON_LOG_ERROR("Cannot enqueue upload - device_id is empty");
    return;
  }

  // Get file size
  uint64_t file_size = 0;
  if (fs::exists(mcap_path)) {
    file_size = fs::file_size(mcap_path);
  }

  // Create upload item
  UploadItem item(mcap_path, json_path, task_id, factory_id, device_id, checksum_sha256, file_size);

  // Construct S3 key prefix
  item.s3_key_prefix = factory_id + "/" + device_id + "/" + currentDateString();

  // Persist to state DB first (for crash recovery)
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
  state_manager_->insert(record);

  // Add to queue
  queue_->enqueue(std::move(item));
  stats_.files_pending++;

  // Check backpressure
  checkBackpressure();
}

const UploaderStats& EdgeUploader::stats() const { return stats_; }

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

const UploaderConfig& EdgeUploader::config() const { return config_; }

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
  std::string mcap_s3_key = item.s3_key_prefix + "/" + item.task_id + ".mcap";
  std::string json_s3_key = item.s3_key_prefix + "/" + item.task_id + ".json";

  // Step 1: Upload MCAP file (skip if already in S3 from previous retry)
  bool mcap_success = false;
  if (s3_client_->objectExists(mcap_s3_key)) {
    // MCAP already uploaded (retry after JSON failure) - verify checksum
    if (item.checksum_sha256.empty() || 
        s3_client_->verifyUpload(mcap_s3_key, item.checksum_sha256)) {
      AXON_LOG_INFO("MCAP already in S3, skipping upload: " << mcap_s3_key);
      mcap_success = true;
    } else {
      AXON_LOG_WARN("MCAP in S3 has wrong checksum, re-uploading: " << mcap_s3_key);
      mcap_success = uploadSingleFile(item.mcap_path, mcap_s3_key, item.checksum_sha256, item.task_id);
    }
  } else {
    mcap_success = uploadSingleFile(item.mcap_path, mcap_s3_key, item.checksum_sha256, item.task_id);
  }

  if (!mcap_success) {
    // MCAP upload failed - don't upload JSON
    return;  // Error handling done in uploadSingleFile
  }

  // Step 2: Upload JSON sidecar (signals completion)
  bool json_success = uploadSingleFile(item.json_path, json_s3_key, "", item.task_id);

  if (!json_success) {
    // JSON upload failed after MCAP succeeded
    // Re-queue the entire upload - on retry, MCAP upload will succeed quickly
    // since the file is already in S3 (or we can add objectExists check)
    UploadItem retry_item = item;
    auto record = state_manager_->get(item.mcap_path);
    if (record) {
      retry_item.retry_count = record->retry_count;
    }
    onUploadFailure(retry_item, "JSON sidecar upload failed after MCAP succeeded", true);
    return;
  }

  // Both uploads succeeded
  onUploadSuccess(item);
}

bool EdgeUploader::uploadSingleFile(
    const std::string& local_path, const std::string& s3_key,
    const std::string& checksum, const std::string& task_id
) {
  // Check file exists
  if (!fs::exists(local_path)) {
    // File missing - mark as failed
    std::string error_msg = "File not found: " + local_path;
    AXON_LOG_ERROR(error_msg);
    state_manager_->markFailed(local_path, error_msg);
    stats_.files_failed++;
    return false;
  }

  // Prepare metadata
  std::map<std::string, std::string> metadata;
  metadata["task-id"] = task_id;
  if (!checksum.empty()) {
    metadata["checksum-sha256"] = checksum;
  }

  // Upload
  auto result = s3_client_->uploadFile(local_path, s3_key, metadata);

  if (result.success) {
    // Verify checksum if provided
    if (!checksum.empty()) {
      if (!s3_client_->verifyUpload(s3_key, checksum)) {
        // Checksum mismatch - treat as retryable error
        UploadItem retry_item;
        retry_item.mcap_path = local_path;
        retry_item.task_id = task_id;
        retry_item.checksum_sha256 = checksum;
        onUploadFailure(retry_item, "Checksum verification failed", true);
        return false;
      }
    }
    return true;
  } else {
    // Upload failed
    UploadItem retry_item;
    retry_item.mcap_path = local_path;
    retry_item.task_id = task_id;
    retry_item.checksum_sha256 = checksum;

    // Get current retry count from state DB
    auto record = state_manager_->get(local_path);
    if (record) {
      retry_item.retry_count = record->retry_count;
    }

    onUploadFailure(retry_item, result.error_message, result.is_retryable);
    return false;
  }
}

std::string EdgeUploader::constructS3Key(
    const std::string& factory_id, const std::string& device_id,
    const std::string& task_id, const std::string& extension
) {
  return factory_id + "/" + device_id + "/" + currentDateString() + "/" + task_id + "." + extension;
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

void EdgeUploader::onUploadFailure(const UploadItem& item, const std::string& error, bool retryable) {
  // Increment retry count in state DB
  state_manager_->incrementRetry(item.mcap_path, error);

  // Check if should retry
  if (retryable && retry_handler_->shouldRetry(item.retry_count)) {
    // Re-queue with delay
    UploadItem retry_item = item;
    retry_item.retry_count++;
    retry_item.next_retry_at = retry_handler_->nextRetryTime(retry_item.retry_count);
    queue_->requeue_for_retry(std::move(retry_item));
    stats_.files_pending++;
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
  std::error_code ec;
  if (!mcap_path.empty() && fs::exists(mcap_path)) {
    fs::remove(mcap_path, ec);
  }
  if (!json_path.empty() && fs::exists(json_path)) {
    fs::remove(json_path, ec);
  }

  // Remove from state DB
  state_manager_->remove(mcap_path);
}

void EdgeUploader::moveToFailedDir(const std::string& mcap_path, const std::string& json_path) {
  std::error_code ec;

  // Create failed directory if needed
  fs::create_directories(config_.failed_uploads_dir, ec);

  // Move files
  if (!mcap_path.empty() && fs::exists(mcap_path)) {
    fs::path dest = fs::path(config_.failed_uploads_dir) / fs::path(mcap_path).filename();
    fs::rename(mcap_path, dest, ec);
  }
  if (!json_path.empty() && fs::exists(json_path)) {
    fs::path dest = fs::path(config_.failed_uploads_dir) / fs::path(json_path).filename();
    fs::rename(json_path, dest, ec);
  }
}

void EdgeUploader::checkBackpressure() {
  uint64_t pending = queue_->pending_bytes();

  if (pending > config_.alert_pending_bytes) {
    AXON_LOG_ERROR("ALERT: Upload queue pending bytes (" << pending
                   << ") exceeds threshold (" << config_.alert_pending_bytes << ")");
  } else if (pending > config_.warn_pending_bytes) {
    AXON_LOG_WARN("Upload queue pending bytes (" << pending
                  << ") exceeds warning threshold (" << config_.warn_pending_bytes << ")");
  }
}

}  // namespace uploader
}  // namespace axon

