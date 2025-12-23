#include "s3_client.hpp"

#include <aws/core/Aws.h>
#include <aws/core/auth/AWSCredentials.h>
#include <aws/core/utils/DateTime.h>
#include <aws/core/utils/memory/stl/AWSStringStream.h>
#include <aws/s3/S3Client.h>
#include <aws/s3/S3Errors.h>
#include <aws/s3/model/HeadObjectRequest.h>
#include <aws/s3/model/PutObjectRequest.h>

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <mutex>

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
    // Use call_once to ensure InitAPI is called exactly once, even if
    // multiple threads race to call addRef simultaneously
    std::call_once(init_flag_, [this]() {
      Aws::SDKOptions options;
      // Disable logging by default - can be enabled for debugging
      options.loggingOptions.logLevel = Aws::Utils::Logging::LogLevel::Off;
      Aws::InitAPI(options);
      options_ = options;
      initialized_ = true;
    });
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
  std::once_flag init_flag_;
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

  Impl() { AwsSdkManager::instance().addRef(); }

  ~Impl() { AwsSdkManager::instance().release(); }

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

    // Use path-style addressing for non-AWS endpoints (MinIO, etc.)
    // Path style: http://endpoint/bucket/key (required for MinIO)
    // Virtual hosted style: http://bucket.endpoint/key (default for AWS S3)
    bool use_path_style = !config.endpoint_url.empty();

    // Create S3 client
    // The constructor signature varies between SDK versions, using the most compatible form
    client = std::make_shared<Aws::S3::S3Client>(
        credentials,
        client_config,
        Aws::Client::AWSAuthV4Signer::PayloadSigningPolicy::Never,
        use_path_style  // forcePathStyle - enables path-style access for MinIO
    );
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

  // Open file for reading
  auto input_stream =
      Aws::MakeShared<Aws::FStream>("S3Upload", local_path.c_str(), std::ios_base::in | std::ios_base::binary);

  if (!input_stream->good()) {
    return UploadResult::Failure("Cannot open local file for reading: " + local_path, "FileNotFound", false);
  }

  // Create PutObject request
  Aws::S3::Model::PutObjectRequest request;
  request.SetBucket(impl_->config.bucket);
  request.SetKey(s3_key);
  request.SetBody(input_stream);
  request.SetContentLength(file_size);

  // Set content type based on file extension
  if (s3_key.size() >= 5 && s3_key.substr(s3_key.size() - 5) == ".mcap") {
    request.SetContentType("application/octet-stream");
  } else if (s3_key.size() >= 5 && s3_key.substr(s3_key.size() - 5) == ".json") {
    request.SetContentType("application/json");
  } else {
    request.SetContentType("application/octet-stream");
  }

  // Add custom metadata (these become x-amz-meta-* headers)
  for (const auto& [key, value] : metadata) {
    request.AddMetadata(key, value);
  }

  // Set up progress callback if provided
  if (progress_cb) {
    request.SetDataSentEventHandler(
        [progress_cb, file_size](const Aws::Http::HttpRequest*, long long bytes_sent) {
          progress_cb(static_cast<uint64_t>(bytes_sent), file_size);
        });
  }

  // Perform upload
  auto outcome = impl_->client->PutObject(request);

  if (outcome.IsSuccess()) {
    const auto& result = outcome.GetResult();
    std::string etag = result.GetETag();
    // Remove quotes from ETag if present
    if (etag.size() >= 2 && etag.front() == '"' && etag.back() == '"') {
      etag = etag.substr(1, etag.size() - 2);
    }
    std::string version_id = result.GetVersionId();

    AXON_LOG_DEBUG("S3 upload succeeded: " << s3_key);
    return UploadResult::Success(etag, version_id);
  } else {
    const auto& error = outcome.GetError();
    // Get error code as string - try exception name first, then error type
    std::string error_code = error.GetExceptionName();
    if (error_code.empty()) {
      // Use S3 error type for better categorization
      auto error_type = error.GetErrorType();
      if (error_type != Aws::S3::S3Errors::UNKNOWN) {
        error_code = "S3_" + std::to_string(static_cast<int>(error_type));
      } else {
        // Fallback to HTTP response code
        error_code = "HTTP_" + std::to_string(static_cast<int>(error.GetResponseCode()));
      }
    }
    std::string error_msg = error.GetMessage();
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
