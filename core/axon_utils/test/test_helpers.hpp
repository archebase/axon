/**
 * @file test_helpers.hpp
 * @brief Common test utilities and fixtures for axon_utils tests
 */

#ifndef AXON_UTILS_TEST_HELPERS_HPP
#define AXON_UTILS_TEST_HELPERS_HPP

#include <chrono>
#include <filesystem>
#include <string>
#include <vector>

// Note: common_types has been moved to apps/axon_recorder
// These helper functions are no longer available in core/axon_utils
// They should be moved to apps/axon_recorder/test if needed

namespace axon {
namespace utils {
namespace test {

/**
 * RAII wrapper for temporary test directories.
 * Automatically creates a unique directory on construction and
 * removes it (with all contents) on destruction.
 */
class TempDirectory {
public:
  TempDirectory() {
    auto base = std::filesystem::temp_directory_path();
    auto timestamp = std::chrono::system_clock::now().time_since_epoch().count();
    path_ = base / ("axon_test_" + std::to_string(timestamp));
    std::filesystem::create_directories(path_);
  }

  ~TempDirectory() {
    if (!path_.empty() && std::filesystem::exists(path_)) {
      std::filesystem::remove_all(path_);
    }
  }

  // Non-copyable, movable
  TempDirectory(const TempDirectory&) = delete;
  TempDirectory& operator=(const TempDirectory&) = delete;
  TempDirectory(TempDirectory&& other) noexcept
      : path_(std::move(other.path_)) {
    other.path_.clear();
  }
  TempDirectory& operator=(TempDirectory&& other) noexcept {
    if (this != &other) {
      if (!path_.empty() && std::filesystem::exists(path_)) {
        std::filesystem::remove_all(path_);
      }
      path_ = std::move(other.path_);
      other.path_.clear();
    }
    return *this;
  }

  std::filesystem::path path() const {
    return path_;
  }
  std::string string() const {
    return path_.string();
  }

  /**
   * Create a subdirectory within the temp directory.
   */
  std::filesystem::path subdir(const std::string& name) const {
    auto sub = path_ / name;
    std::filesystem::create_directories(sub);
    return sub;
  }

  /**
   * Get path to a file within the temp directory.
   */
  std::filesystem::path file(const std::string& name) const {
    return path_ / name;
  }

private:
  std::filesystem::path path_;
};

// Note: TaskConfig and RecorderConfig helper functions have been moved to
// apps/axon_recorder/test/test_helpers.hpp along with common_types

/**
 * Generate a test message payload of specified size.
 * @param size Size in bytes
 * @return Vector of bytes filled with test pattern
 */
inline std::vector<uint8_t> make_test_message(size_t size) {
  std::vector<uint8_t> data(size);
  for (size_t i = 0; i < size; ++i) {
    data[i] = static_cast<uint8_t>(i & 0xFF);
  }
  return data;
}

/**
 * Get current time in nanoseconds since epoch.
 */
inline uint64_t now_ns() {
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                 std::chrono::system_clock::now().time_since_epoch()
  )
                                 .count());
}

}  // namespace test
}  // namespace utils
}  // namespace axon

#endif  // AXON_UTILS_TEST_HELPERS_HPP
