// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_EDGE_UPLOADER_TEST_HELPERS_HPP
#define AXON_EDGE_UPLOADER_TEST_HELPERS_HPP

// This header is for testing only - exposes internal implementations
// for dependency injection in tests

#include "edge_uploader.hpp"
#include "uploader_interfaces.hpp"

namespace axon {
namespace uploader {

// Forward declarations of internal implementations
// These are defined in edge_uploader.cpp

/**
 * Validate that both MCAP and JSON files exist
 * @param mcap_path Path to MCAP file
 * @param json_path Path to JSON sidecar file
 * @param filesystem File system interface
 * @return true if both files exist, false otherwise
 */
bool validateFilesExistImpl(
  const std::string& mcap_path, const std::string& json_path, const IFileSystem& filesystem
);

/**
 * Get file size
 * @param path Path to file
 * @param filesystem File system interface
 * @return File size in bytes
 */
uint64_t getFileSizeImpl(const std::string& path, const IFileSystem& filesystem);

/**
 * Cleanup local files after successful upload
 * @param mcap_path Path to MCAP file
 * @param json_path Path to JSON sidecar file
 * @param filesystem File system interface
 */
void cleanupLocalFilesImpl(
  const std::string& mcap_path, const std::string& json_path, const IFileSystem& filesystem
);

/**
 * Move failed files to failed directory
 * @param mcap_path Path to MCAP file
 * @param json_path Path to JSON sidecar file
 * @param failed_dir Failed uploads directory
 * @param filesystem File system interface
 */
void moveToFailedDirImpl(
  const std::string& mcap_path, const std::string& json_path, const std::string& failed_dir,
  const IFileSystem& filesystem
);

/**
 * Upload a single file (MCAP or JSON)
 * @param local_path Path to local file
 * @param s3_key S3 object key
 * @param checksum SHA-256 checksum (empty for JSON)
 * @param filesystem File system interface
 * @param s3_client S3 client instance
 * @return Upload result
 */
FileUploadResult uploadSingleFileImpl(
  const std::string& local_path, const std::string& s3_key, const std::string& checksum,
  const IFileSystem& filesystem, S3Client* s3_client
);

}  // namespace uploader
}  // namespace axon

#endif  // AXON_EDGE_UPLOADER_TEST_HELPERS_HPP
