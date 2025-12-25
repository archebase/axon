#include "s3_client.hpp"

#include <aws/core/Aws.h>
#include <aws/core/auth/AWSCredentials.h>
#include <aws/core/utils/DateTime.h>
#include <aws/core/utils/memory/stl/AWSStringStream.h>
#include <aws/core/utils/threading/Executor.h>
#include <aws/s3/S3Client.h>
#include <aws/s3/S3Errors.h>
#include <aws/s3/model/HeadObjectRequest.h>
#include <aws/transfer/TransferHandle.h>
#include <aws/transfer/TransferManager.h>

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <mutex>
#include <thread>

#include "retry_handler.hpp"

// Logging infrastructure (optional - only if axon_logging is linked)
#ifdef AXON_HAS_LOGGING
#define AXON_LOG_COMPONENT "s3_client"
#include <axon_log_macros.hpp>
#else
// Fallback to stderr when logging is not available
#define AXON_LOG_DEBUG(msg) \
  do {                      \
  } while (0)
#define AXON_LOG_INFO(msg) \
  do {                     \
  } while (0)
#define AXON_LOG_WARN(msg) std::cerr << "[WARN] " << msg << std::endl
#define AXON_LOG_ERROR(msg) std::cerr << "[ERROR] " << msg << std::endl
#endif

namespace axon {
namespace uploader {

// =============================================================================
// AWS SDK Lifecycle Management
// =============================================================================
// The AWS SDK requires InitAPI/ShutdownAPI to be called exactly once per process.
// We use a reference-counted singleton to manage this lifecycle.
// =============================================================================

class AwsSdkManager {
public:
  static AwsSdkManager& instance() {
    static AwsSdkManager instance;
    return instance;
  }

  void addRef() {
    std::lock_guard<std::mutex> lock(mutex_);
    // Initialize SDK if not already initialized (or if it was shut down)
    // The mutex ensures thread safety - no need for std::call_once
    if (!initialized_) {
      Aws::SDKOptions options;
      // Disable logging by default - can be enabled for debugging
      options.loggingOptions.logLevel = Aws::Utils::Logging::LogLevel::Off;
      Aws::InitAPI(options);
      options_ = options;
      initialized_ = true;
    }
    ++ref_count_;
  }

  void release() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (ref_count_ > 0) {
      --ref_count_;
      if (ref_count_ == 0 && initialized_) {
        Aws::ShutdownAPI(options_);
        initialized_ = false;
      }
    }
  }

private:
  AwsSdkManager() = default;
  ~AwsSdkManager() = default;

  std::mutex mutex_;
  bool initialized_ = false;
  int ref_count_ = 0;
  Aws::SDKOptions options_;
};

// =============================================================================
// S3Client Implementation
// =============================================================================

class S3Client::Impl {
public:
  S3Config config;
  std::shared_ptr<Aws::S3::S3Client> client;
  std::shared_ptr<Aws::Utils::Threading::PooledThreadExecutor> executor;
  std::shared_ptr<Aws::Transfer::TransferManager> transfer_manager;

  Impl() { AwsSdkManager::instance().addRef(); }

  ~Impl() {
    // IMPORTANT: All AWS SDK objects must be destroyed BEFORE calling release(),
    // which may invoke Aws::ShutdownAPI() when the reference count reaches zero.
    // Using SDK objects after ShutdownAPI() causes undefined behavior/crashes.
    //
    // Destruction order matters:
    // 1. TransferManager (uses S3Client and Executor)
    // 2. S3Client (uses SDK internals)
    // 3. Executor (manages threads)
    // 4. Release SDK reference (may shut down SDK)
    transfer_manager.reset();
    client.reset();
    executor.reset();
    AwsSdkManager::instance().release();
  }

  void initClient() {
    Aws::Client::ClientConfiguration client_config;

    // Set region
    client_config.region = config.region;

    // Set custom endpoint for S3-compatible storage (MinIO, etc.)
    // The endpoint should NOT include the bucket name
    if (!config.endpoint_url.empty()) {
      // Remove trailing slash if present
      std::string endpoint = config.endpoint_url;
      if (!endpoint.empty() && endpoint.back() == '/') {
        endpoint.pop_back();
      }
      client_config.endpointOverride = endpoint;
    }

    // Configure SSL
    client_config.verifySSL = config.verify_ssl;
    if (config.use_ssl) {
      client_config.scheme = Aws::Http::Scheme::HTTPS;
    } else {
      client_config.scheme = Aws::Http::Scheme::HTTP;
    }

    // Set timeouts
    client_config.connectTimeoutMs = config.connect_timeout_ms;
    client_config.requestTimeoutMs = config.request_timeout_ms;

    // Create credentials
    Aws::Auth::AWSCredentials credentials(config.access_key, config.secret_key);

    // Configure addressing style for S3 vs S3-compatible storage (MinIO, etc.)
    // The S3Client constructor's 4th parameter is 'useVirtualAddressing':
    // - true  = virtual-hosted style (bucket.s3.amazonaws.com/key) - default for AWS S3
    // - false = path style (s3.amazonaws.com/bucket/key) - required for MinIO
    //
    // For custom endpoints (MinIO), we need path-style, so useVirtualAddressing = false
    // For AWS S3 (no custom endpoint), we use virtual-hosted, so useVirtualAddressing = true
    bool use_virtual_addressing = config.endpoint_url.empty();

    // Create S3 client
    client = std::make_shared<Aws::S3::S3Client>(
        credentials,
        client_config,
        Aws::Client::AWSAuthV4Signer::PayloadSigningPolicy::Never,
        use_virtual_addressing  // false for MinIO (path style), true for AWS S3 (virtual hosted)
    );

    // Create thread pool executor for TransferManager
    executor = Aws::MakeShared<Aws::Utils::Threading::PooledThreadExecutor>(
        "S3Executor", config.executor_thread_count);

    // Configure TransferManager for multipart uploads
    Aws::Transfer::TransferManagerConfiguration transfer_config(executor.get());
    transfer_config.s3Client = client;

    // Enforce S3 part size constraints (5MB minimum, 5GB maximum)
    constexpr uint64_t MIN_PART_SIZE = 5 * 1024 * 1024;           // 5MB
    constexpr uint64_t MAX_PART_SIZE = 5ULL * 1024 * 1024 * 1024; // 5GB
    uint64_t validated_part_size = config.part_size;
    if (validated_part_size < MIN_PART_SIZE) {
      AXON_LOG_WARN("part_size " << config.part_size << " is below S3 minimum (5MB), using 5MB");
      validated_part_size = MIN_PART_SIZE;
    } else if (validated_part_size > MAX_PART_SIZE) {
      AXON_LOG_WARN("part_size " << config.part_size << " exceeds S3 maximum (5GB), using 5GB");
      validated_part_size = MAX_PART_SIZE;
    }
    transfer_config.bufferSize = validated_part_size;
    // TransferManager will automatically use multipart upload for files > 5MB

    transfer_manager = Aws::Transfer::TransferManager::Create(transfer_config);
  }
};

S3Client::S3Client(const S3Config& config) : impl_(std::make_unique<Impl>()) {
  impl_->config = config;

  // Load credentials from environment if not provided
  if (impl_->config.access_key.empty()) {
    if (const char* key = std::getenv("AWS_ACCESS_KEY_ID")) {
      impl_->config.access_key = key;
    }
  }
  if (impl_->config.secret_key.empty()) {
    if (const char* key = std::getenv("AWS_SECRET_ACCESS_KEY")) {
      impl_->config.secret_key = key;
    }
  }

  impl_->initClient();
}

S3Client::~S3Client() = default;

UploadResult S3Client::uploadFile(
    const std::string& local_path, const std::string& s3_key,
    const std::map<std::string, std::string>& metadata, ProgressCallback progress_cb
) {
  // Check file exists and get size
  std::ifstream file(local_path, std::ios::binary | std::ios::ate);
  if (!file) {
    return UploadResult::Failure("Cannot open local file: " + local_path, "FileNotFound", false);
  }
  uint64_t file_size = static_cast<uint64_t>(file.tellg());
  file.close();

  // Determine content type based on file extension
  std::string content_type = "application/octet-stream";
  if (s3_key.size() >= 5 && s3_key.substr(s3_key.size() - 5) == ".json") {
    content_type = "application/json";
  }

  // Convert metadata to AWS format
  Aws::Map<Aws::String, Aws::String> aws_metadata;
  for (const auto& [key, value] : metadata) {
    aws_metadata[key] = value;
  }

  // Create upload handle using TransferManager
  // TransferManager automatically uses multipart upload for files > 5MB
  // and handles retries, chunking, and concurrent part uploads
  auto upload_handle = impl_->transfer_manager->UploadFile(
      local_path.c_str(),
      impl_->config.bucket.c_str(),
      s3_key.c_str(),
      content_type.c_str(),
      aws_metadata
  );

  // Wait for upload to complete with optional progress polling
  // Note: TransferManager's progress callbacks are global (set on configuration),
  // not per-upload. For per-upload progress, we poll the handle's state.
  if (progress_cb) {
    // Poll progress while upload is in progress
    constexpr auto poll_interval = std::chrono::milliseconds(100);
    while (upload_handle->GetStatus() == Aws::Transfer::TransferStatus::IN_PROGRESS ||
           upload_handle->GetStatus() == Aws::Transfer::TransferStatus::NOT_STARTED) {
      uint64_t transferred = upload_handle->GetBytesTransferred();
      progress_cb(transferred, file_size);
      std::this_thread::sleep_for(poll_interval);
    }
    // Final progress update
    progress_cb(upload_handle->GetBytesTransferred(), file_size);
  } else {
    upload_handle->WaitUntilFinished();
  }

  // Check result
  auto status = upload_handle->GetStatus();

  if (status == Aws::Transfer::TransferStatus::COMPLETED) {
    // Get multipart upload ID as ETag equivalent for multipart uploads
    // For single-part uploads, this will be the actual ETag
    std::string etag;
    
    // For multipart uploads, the ETag is a composite; we need to fetch it from S3
    // TransferHandle doesn't expose the final ETag, so we retrieve it via HeadObject
    auto meta = getObjectMetadata(s3_key);
    auto it = meta.find("etag");
    if (it != meta.end()) {
      etag = it->second;
    }

    AXON_LOG_DEBUG("S3 upload succeeded: " << s3_key << " (size: " << file_size << " bytes)");
    return UploadResult::Success(etag, "");
  } else {
    // Handle failure
    auto error = upload_handle->GetLastError();
    std::string error_code = error.GetExceptionName();
    if (error_code.empty()) {
      // Map transfer status to error code
      switch (status) {
        case Aws::Transfer::TransferStatus::CANCELED:
          error_code = "TransferCanceled";
          break;
        case Aws::Transfer::TransferStatus::FAILED:
          error_code = "TransferFailed";
          break;
        case Aws::Transfer::TransferStatus::ABORTED:
          error_code = "TransferAborted";
          break;
        default:
          error_code = "TransferError";
          break;
      }
    }
    std::string error_msg = error.GetMessage();
    if (error_msg.empty()) {
      error_msg = "Transfer failed with status: " + std::to_string(static_cast<int>(status));
    }
    bool retryable = isRetryableError(error_code) || error.ShouldRetry();

    AXON_LOG_ERROR("S3 upload failed: " << s3_key << " - " << error_msg << " (code: " << error_code
                                        << ", retryable: " << (retryable ? "yes" : "no") << ")");
    return UploadResult::Failure(error_msg, error_code, retryable);
  }
}

bool S3Client::objectExists(const std::string& s3_key) {
  Aws::S3::Model::HeadObjectRequest request;
  request.SetBucket(impl_->config.bucket);
  request.SetKey(s3_key);

  auto outcome = impl_->client->HeadObject(request);
  return outcome.IsSuccess();
}

std::map<std::string, std::string> S3Client::getObjectMetadata(const std::string& s3_key) {
  Aws::S3::Model::HeadObjectRequest request;
  request.SetBucket(impl_->config.bucket);
  request.SetKey(s3_key);

  auto outcome = impl_->client->HeadObject(request);

  std::map<std::string, std::string> result;
  if (outcome.IsSuccess()) {
    const auto& head_result = outcome.GetResult();

    // Get ETag (remove quotes if present)
    std::string etag = head_result.GetETag();
    if (etag.size() >= 2 && etag.front() == '"' && etag.back() == '"') {
      etag = etag.substr(1, etag.size() - 2);
    }
    result["etag"] = etag;

    // Get size
    result["size"] = std::to_string(head_result.GetContentLength());

    // Get last modified
    result["last_modified"] = head_result.GetLastModified().ToGmtString(Aws::Utils::DateFormat::ISO_8601);

    // Get version ID
    result["version_id"] = head_result.GetVersionId();

    // Copy user metadata
    for (const auto& [key, value] : head_result.GetMetadata()) {
      result[key] = value;
    }
  }
  return result;
}

bool S3Client::verifyUpload(const std::string& s3_key, const std::string& expected_checksum) {
  auto metadata = getObjectMetadata(s3_key);

  // Look for checksum in user metadata
  // AWS SDK returns metadata keys in lowercase
  auto it = metadata.find("checksum-sha256");
  if (it == metadata.end()) {
    // Try with different case variations
    it = metadata.find("Checksum-Sha256");
  }
  if (it == metadata.end()) {
    it = metadata.find("x-amz-meta-checksum-sha256");
  }

  if (it == metadata.end()) {
    return false;  // Checksum not found in metadata
  }

  return it->second == expected_checksum;
}

bool S3Client::isRetryableError(const std::string& error_code) {
  // Delegate to RetryHandler for single source of truth on retryable errors
  return RetryHandler::isRetryableError(error_code);
}

const std::string& S3Client::bucket() const { return impl_->config.bucket; }

const std::string& S3Client::endpoint() const { return impl_->config.endpoint_url; }

}  // namespace uploader
}  // namespace axon
