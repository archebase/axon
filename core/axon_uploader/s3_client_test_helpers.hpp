// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_S3_CLIENT_TEST_HELPERS_HPP
#define AXON_S3_CLIENT_TEST_HELPERS_HPP

// This header is for testing only - exposes internal implementations
// for dependency injection in tests

#include <string>

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

/**
 * Convert AWS TransferStatus to error code string
 * Extracted for testability - tests can pass status codes directly
 * @param status Transfer status enum value as int
 * @return Error code string
 */
std::string transferStatusToErrorCode(int status);

}  // namespace uploader
}  // namespace axon

#endif  // AXON_S3_CLIENT_TEST_HELPERS_HPP
