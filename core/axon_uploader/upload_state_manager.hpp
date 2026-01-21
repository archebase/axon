// Copyright (c) 2026 ArcheBase
// Axon is licensed under Mulan PSL v2.
// You can use this software according to the terms and conditions of the Mulan PSL v2.
// You may obtain a copy of Mulan PSL v2 at:
//          http://license.coscl.org.cn/MulanPSL2
// THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
// EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
// MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
// See the Mulan PSL v2 for more details.

#ifndef AXON_UPLOAD_STATE_MANAGER_HPP
#define AXON_UPLOAD_STATE_MANAGER_HPP

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace axon {
namespace uploader {

/**
 * Upload status enumeration
 */
enum class UploadStatus {
  PENDING,    // Queued for upload
  UPLOADING,  // Currently being uploaded
  COMPLETED,  // Successfully uploaded
  FAILED      // Permanently failed (exceeded retries)
};

/**
 * Convert UploadStatus to string for storage
 */
inline std::string uploadStatusToString(UploadStatus status) {
  switch (status) {
    case UploadStatus::PENDING:
      return "pending";
    case UploadStatus::UPLOADING:
      return "uploading";
    case UploadStatus::COMPLETED:
      return "completed";
    case UploadStatus::FAILED:
      return "failed";
    default:
      return "unknown";
  }
}

/**
 * Parse UploadStatus from string
 */
inline UploadStatus uploadStatusFromString(const std::string& str) {
  if (str == "pending") return UploadStatus::PENDING;
  if (str == "uploading") return UploadStatus::UPLOADING;
  if (str == "completed") return UploadStatus::COMPLETED;
  if (str == "failed") return UploadStatus::FAILED;
  return UploadStatus::PENDING;  // Default
}

/**
 * Record representing an upload's state in the database
 */
struct UploadRecord {
  std::string file_path;        // Primary key - local MCAP file path
  std::string json_path;        // Sidecar JSON path
  std::string s3_key;           // Target S3 key
  std::string task_id;          // Task identifier
  std::string factory_id;       // Factory identifier
  std::string device_id;        // Device identifier
  uint64_t file_size_bytes;     // File size
  std::string checksum_sha256;  // SHA-256 checksum
  UploadStatus status;          // Current status
  int retry_count;              // Number of retries so far
  std::string last_error;       // Last error message
  std::string created_at;       // ISO 8601 timestamp
  std::string updated_at;       // ISO 8601 timestamp
  std::string completed_at;     // ISO 8601 timestamp (empty if not completed)

  UploadRecord()
      : file_size_bytes(0)
      , status(UploadStatus::PENDING)
      , retry_count(0) {}
};

/**
 * SQLite-based upload state manager
 *
 * Provides persistent storage for upload state, enabling crash recovery.
 * Uses WAL mode for crash safety and better concurrent performance.
 *
 * Thread-safety: All methods are thread-safe (protected by mutex).
 */
class UploadStateManager {
public:
  /**
   * Create state manager with SQLite database at specified path
   *
   * Creates the database and tables if they don't exist.
   * Enables WAL mode for crash safety.
   *
   * @param db_path Path to SQLite database file
   * @throws std::runtime_error if database cannot be opened
   */
  explicit UploadStateManager(const std::string& db_path);
  ~UploadStateManager();

  // Non-copyable, non-movable
  UploadStateManager(const UploadStateManager&) = delete;
  UploadStateManager& operator=(const UploadStateManager&) = delete;
  UploadStateManager(UploadStateManager&&) = delete;
  UploadStateManager& operator=(UploadStateManager&&) = delete;

  /**
   * Insert a new upload record
   *
   * @param record Record to insert
   * @return true if inserted, false if record with same file_path exists
   */
  bool insert(const UploadRecord& record);

  /**
   * Update the status of an upload
   *
   * @param file_path Primary key
   * @param status New status
   * @param error Error message (optional, for FAILED status)
   * @return true if updated, false if record not found
   */
  bool updateStatus(
    const std::string& file_path, UploadStatus status, const std::string& error = ""
  );

  /**
   * Mark an upload as completed
   *
   * Sets status to COMPLETED and records completion timestamp.
   *
   * @param file_path Primary key
   * @return true if updated
   */
  bool markCompleted(const std::string& file_path);

  /**
   * Mark an upload as failed
   *
   * Sets status to FAILED with error message.
   *
   * @param file_path Primary key
   * @param error Error message
   * @return true if updated
   */
  bool markFailed(const std::string& file_path, const std::string& error);

  /**
   * Increment retry count for an upload
   *
   * @param file_path Primary key
   * @param error Error message from last attempt
   * @return true if updated
   */
  bool incrementRetry(const std::string& file_path, const std::string& error);

  /**
   * Remove an upload record
   *
   * @param file_path Primary key
   * @return true if removed, false if not found
   */
  bool remove(const std::string& file_path);

  /**
   * Get a specific upload record
   *
   * @param file_path Primary key
   * @return Record if found, nullopt otherwise
   */
  std::optional<UploadRecord> get(const std::string& file_path);

  /**
   * Get all pending uploads
   *
   * @return Vector of records with PENDING status
   */
  std::vector<UploadRecord> getPending();

  /**
   * Get all failed uploads
   *
   * @return Vector of records with FAILED status
   */
  std::vector<UploadRecord> getFailed();

  /**
   * Get all incomplete uploads (PENDING or UPLOADING)
   *
   * Used for crash recovery: uploads that were in progress when
   * the recorder crashed need to be retried.
   *
   * @return Vector of incomplete records
   */
  std::vector<UploadRecord> getIncomplete();

  /**
   * Count records by status
   *
   * @param status Status to count
   * @return Number of records with that status
   */
  size_t countByStatus(UploadStatus status);

  /**
   * Get total bytes of pending uploads
   *
   * @return Total file size of all pending/uploading records
   */
  uint64_t pendingBytes();

  /**
   * Delete records older than specified age
   *
   * Only deletes COMPLETED records.
   *
   * @param age Maximum age of records to keep
   * @return Number of records deleted
   */
  int deleteOlderThan(std::chrono::hours age);

  /**
   * Delete all completed records
   *
   * @return Number of records deleted
   */
  int deleteCompleted();

  /**
   * Get database path
   */
  const std::string& dbPath() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace uploader
}  // namespace axon

#endif  // AXON_UPLOAD_STATE_MANAGER_HPP
