#ifndef AXON_S3_CLIENT_TEST_HELPERS_HPP
#define AXON_S3_CLIENT_TEST_HELPERS_HPP

// This header is for testing only - exposes internal implementations
// for dependency injection in tests

#include "s3_client.hpp"
#include "uploader_interfaces.hpp"

namespace axon {
namespace uploader {

// Forward declarations of internal implementations
// These are defined in s3_client.cpp

/**
 * Get file size for upload
 * @param local_path Path to local file
 * @param stream_factory File stream factory interface
 * @return File size in bytes, or 0 on failure
 */
uint64_t getFileSizeForUploadImpl(
    const std::string& local_path, IFileStreamFactory& stream_factory
);

}  // namespace uploader
}  // namespace axon

#endif  // AXON_S3_CLIENT_TEST_HELPERS_HPP

