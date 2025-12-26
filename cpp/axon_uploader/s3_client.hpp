#ifndef AXON_S3_CLIENT_HPP
#define AXON_S3_CLIENT_HPP

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>

namespace axon {
namespace uploader {

/**
 * S3 configuration options
 */
struct S3Config {
  std::string endpoint_url;    // e.g., "https://play.min.io" or "https://s3.amazonaws.com"
  std::string bucket;          // Bucket name
  std::string region = "us-east-1";
  bool use_ssl = true;
  bool verify_ssl = true;

  // Credentials (if not using environment variables)
  // If empty, will read from AWS_ACCESS_KEY_ID and AWS_SECRET_ACCESS_KEY
  std::string access_key;
  std::string secret_key;

  // Multipart upload settings (TransferManager automatically uses multipart for files > 5MB)
  uint64_t part_size = 64 * 1024 * 1024;  // 64MB per part (minimum 5MB, maximum 5GB for S3)
  int executor_thread_count = 4;          // Thread pool size for concurrent part uploads

  // Timeouts (in milliseconds)
  int connect_timeout_ms = 10000;
  int request_timeout_ms = 300000;  // 5 minutes for large files

  // AWS SDK internal retry configuration
  // Disabled by default (0) because EdgeUploader has its own retry logic with:
  //   - Crash recovery (persists state to SQLite)
  //   - Custom backoff strategies
  //   - Observability (stats, callbacks)
  // Set > 0 only if you want the SDK to handle immediate transient errors internally.
  int max_sdk_retries = 0;
};

/**
 * Result of an upload operation
 */
struct UploadResult {
  bool success;
  std::string etag;           // S3 ETag (for multipart, this is not a simple MD5)
  std::string version_id;     // Version ID (if versioning enabled)
  std::string error_message;  // Error message if failed
  std::string error_code;     // Error code for classification
  bool is_retryable;          // True for transient errors

  static UploadResult Success(const std::string& etag, const std::string& version_id = "") {
    return {true, etag, version_id, "", "", false};
  }

  static UploadResult Failure(
      const std::string& message, const std::string& code = "", bool retryable = false
  ) {
    return {false, "", "", message, code, retryable};
  }
};

/**
 * Progress callback type
 *
 * @param bytes_transferred Number of bytes uploaded so far
 * @param total_bytes Total file size
 */
using ProgressCallback = std::function<void(uint64_t bytes_transferred, uint64_t total_bytes)>;

/**
 * S3 client wrapper around AWS SDK for C++
 *
 * Provides a simplified interface for uploading files to S3-compatible storage.
 * Supports both AWS S3 and S3-compatible storage like MinIO.
 *
 * Features:
 * - Automatic multipart upload for large files (> 5MB) via TransferManager
 * - No file size limit (standard PutObject is limited to 5GB)
 * - Concurrent part uploads for improved throughput
 * - Credential auto-detection from environment variables
 * - Custom metadata support (for checksum verification)
 * - Progress callbacks
 * - S3-compatible endpoint support (MinIO, etc.)
 */
class S3Client {
public:
  /**
   * Create S3 client with configuration
   *
   * @param config S3 configuration (endpoint, bucket, credentials, etc.)
   */
  explicit S3Client(const S3Config& config);
  ~S3Client();

  // Non-copyable, non-movable
  S3Client(const S3Client&) = delete;
  S3Client& operator=(const S3Client&) = delete;
  S3Client(S3Client&&) = delete;
  S3Client& operator=(S3Client&&) = delete;

  /**
   * Upload a file to S3
   *
   * Uses TransferManager which automatically handles:
   * - Multipart upload for files > 5MB (configurable via part_size)
   * - Concurrent part uploads for improved throughput
   * - No file size limit (removes the 5GB PutObject restriction)
   *
   * @param local_path Path to local file
   * @param s3_key S3 object key (path within bucket)
   * @param metadata Custom metadata to attach (keys without x-amz-meta- prefix)
   * @param progress_cb Optional progress callback (polled every ~100ms during upload)
   * @return UploadResult with success/failure status
   */
  UploadResult uploadFile(
      const std::string& local_path, const std::string& s3_key,
      const std::map<std::string, std::string>& metadata = {}, ProgressCallback progress_cb = nullptr
  );

  /**
   * Check if an object exists in S3
   *
   * @param s3_key S3 object key
   * @return true if object exists
   */
  bool objectExists(const std::string& s3_key);

  /**
   * Get metadata for an S3 object
   *
   * Returns standard metadata plus custom user metadata.
   *
   * @param s3_key S3 object key
   * @return Map of metadata key-value pairs (empty if object doesn't exist)
   */
  std::map<std::string, std::string> getObjectMetadata(const std::string& s3_key);

  /**
   * Verify upload by checking stored checksum
   *
   * Compares the checksum stored in x-amz-meta-checksum-sha256 with expected value.
   *
   * @param s3_key S3 object key
   * @param expected_checksum Expected SHA-256 checksum
   * @return true if checksums match
   */
  bool verifyUpload(const std::string& s3_key, const std::string& expected_checksum);

  /**
   * Check if an error code is retryable
   *
   * @param error_code S3 error code
   * @return true if the error is transient and should be retried
   */
  static bool isRetryableError(const std::string& error_code);

  /**
   * Get the bucket name
   */
  const std::string& bucket() const;

  /**
   * Get the endpoint URL
   */
  const std::string& endpoint() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace uploader
}  // namespace axon

#endif  // AXON_S3_CLIENT_HPP

