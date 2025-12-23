#include "s3_client.hpp"

#include <miniocpp/client.h>

#include <cstdlib>
#include <fstream>
#include <iostream>

#include "retry_handler.hpp"

// Logging infrastructure (optional - only if axon_logging is linked)
#ifdef AXON_HAS_LOGGING
#define AXON_LOG_COMPONENT "s3_client"
#include <axon_log_macros.hpp>
#else
// Fallback to stderr when logging is not available
#define AXON_LOG_DEBUG(msg) do {} while(0)
#define AXON_LOG_INFO(msg) do {} while(0)
#define AXON_LOG_WARN(msg) std::cerr << "[WARN] " << msg << std::endl
#define AXON_LOG_ERROR(msg) std::cerr << "[ERROR] " << msg << std::endl
#endif

namespace axon {
namespace uploader {

class S3Client::Impl {
public:
  S3Config config;
  std::unique_ptr<minio::creds::StaticProvider> creds_provider;  // Must outlive client
  std::unique_ptr<minio::s3::Client> client;

  void initClient() {
    // Create base URL from endpoint
    minio::s3::BaseUrl base_url(config.endpoint_url);

    // Create credentials provider - must be stored as member to outlive client
    creds_provider = std::make_unique<minio::creds::StaticProvider>(
        config.access_key, config.secret_key);

    // Create client with pointer to credentials provider
    client = std::make_unique<minio::s3::Client>(base_url, creds_provider.get());

    // Configure SSL verification
    if (!config.verify_ssl) {
      client->IgnoreCertCheck();
    }
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
  // Check file exists
  std::ifstream file(local_path, std::ios::binary | std::ios::ate);
  if (!file) {
    return UploadResult::Failure("Cannot open local file: " + local_path, "FileNotFound", false);
  }
  uint64_t file_size = static_cast<uint64_t>(file.tellg());
  file.close();

  // Prepare upload arguments
  minio::s3::UploadObjectArgs args;
  args.bucket = impl_->config.bucket;
  args.object = s3_key;
  args.filename = local_path;

  // Add custom metadata (these become x-amz-meta-* headers)
  for (const auto& [key, value] : metadata) {
    args.user_metadata[key] = value;
  }

  // Set progress callback if provided
  if (progress_cb) {
    args.progressfunc = [progress_cb, file_size](const minio::http::ProgressFunctionArgs& args) {
      // args.upload_total_bytes is the total expected bytes
      // args.uploaded_bytes is the current progress
      progress_cb(
          static_cast<uint64_t>(args.uploaded_bytes), file_size
      );
      return true;  // Continue upload
    };
  }

  // Perform upload (minio-cpp automatically uses multipart for large files)
  minio::s3::UploadObjectResponse resp = impl_->client->UploadObject(args);

  if (resp) {
    AXON_LOG_DEBUG("S3 upload succeeded: " << s3_key);
    return UploadResult::Success(resp.etag, resp.version_id);
  } else {
    // Extract both error code (for retry logic) and message (for logging)
    auto error = resp.Error();
    std::string error_code = error.Code();
    std::string error_msg = error.String();
    bool retryable = isRetryableError(error_code);
    AXON_LOG_ERROR("S3 upload failed: " << s3_key << " - " << error_msg 
                   << " (code: " << error_code << ", retryable: " << (retryable ? "yes" : "no") << ")");
    return UploadResult::Failure(error_msg, error_code, retryable);
  }
}

bool S3Client::objectExists(const std::string& s3_key) {
  minio::s3::StatObjectArgs args;
  args.bucket = impl_->config.bucket;
  args.object = s3_key;

  minio::s3::StatObjectResponse resp = impl_->client->StatObject(args);
  return static_cast<bool>(resp);
}

std::map<std::string, std::string> S3Client::getObjectMetadata(const std::string& s3_key) {
  minio::s3::StatObjectArgs args;
  args.bucket = impl_->config.bucket;
  args.object = s3_key;

  minio::s3::StatObjectResponse resp = impl_->client->StatObject(args);

  std::map<std::string, std::string> result;
  if (resp) {
    result["etag"] = resp.etag;
    result["size"] = std::to_string(resp.size);
    result["last_modified"] = resp.last_modified.ToISO8601UTC();
    result["version_id"] = resp.version_id;

    // Copy user metadata
    for (const auto& [key, value] : resp.user_metadata) {
      result[key] = value;
    }
  }
  return result;
}

bool S3Client::verifyUpload(const std::string& s3_key, const std::string& expected_checksum) {
  auto metadata = getObjectMetadata(s3_key);

  // Look for checksum in user metadata
  auto it = metadata.find("checksum-sha256");
  if (it == metadata.end()) {
    // Try with x-amz-meta- prefix stripped (minio-cpp may return it either way)
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

