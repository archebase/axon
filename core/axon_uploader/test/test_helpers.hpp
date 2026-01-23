// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_UPLOADER_TEST_HELPERS_HPP
#define AXON_UPLOADER_TEST_HELPERS_HPP

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace axon {
namespace uploader {
namespace test {

namespace fs = std::filesystem;

/**
 * Create a temporary directory for testing
 */
inline std::string createTempDir(const std::string& prefix = "axon_test_") {
  std::string dir =
    "/tmp/" + prefix + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
  fs::create_directories(dir);
  return dir;
}

/**
 * Create a test MCAP file with minimal valid structure
 */
inline std::string createTestMcapFile(const std::string& path, size_t size = 1024) {
  std::ofstream file(path, std::ios::binary);
  if (!file) {
    return "";
  }

  // Write MCAP header magic
  static const char MAGIC[] = {'\x89', 'M', 'C', 'A', 'P', '0', '\r', '\n'};
  file.write(MAGIC, 8);

  // Write padding to reach desired size
  if (size > 16) {
    std::vector<char> padding(size - 16, 0);
    file.write(padding.data(), padding.size());
  }

  // Write MCAP footer magic
  file.write(MAGIC, 8);

  file.close();
  return path;
}

/**
 * Create a test JSON sidecar file
 */
inline std::string createTestJsonFile(
  const std::string& path, const std::string& task_id, const std::string& device_id = "test_device",
  const std::string& factory_id = "test_factory"
) {
  std::ofstream file(path);
  if (!file) {
    return "";
  }

  file << R"({
    "task_id": ")"
       << task_id << R"(",
    "device_id": ")"
       << device_id << R"(",
    "factory_id": ")"
       << factory_id << R"(",
    "checksum_sha256": "abc123",
    "file_size_bytes": 1024
  })";

  file.close();
  return path;
}

/**
 * Generate a test file of specified size
 */
inline std::string generateTestFile(const std::string& path, size_t size_bytes) {
  std::ofstream file(path, std::ios::binary);
  if (!file) {
    return "";
  }

  std::vector<char> data(size_bytes, 'x');
  file.write(data.data(), data.size());
  file.close();

  return path;
}

/**
 * Check if MinIO is available (basic check via environment variables)
 */
inline bool isMinIOAvailable() {
  return std::getenv("AWS_ACCESS_KEY_ID") != nullptr &&
         std::getenv("AWS_SECRET_ACCESS_KEY") != nullptr;
}

/**
 * Clean up temporary directory and files
 */
inline void cleanupTempDir(const std::string& dir) {
  std::error_code ec;
  fs::remove_all(dir, ec);
}

}  // namespace test
}  // namespace uploader
}  // namespace axon

#endif  // AXON_UPLOADER_TEST_HELPERS_HPP
