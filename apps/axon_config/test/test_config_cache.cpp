// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <vector>

#include "config_cache.hpp"

namespace fs = std::filesystem;

namespace axon {
namespace config {
namespace test {

class ConfigCacheTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Create a temporary test directory
    test_dir_ = fs::temp_directory_path() / "axon_config_test_XXXXXX";
    fs::create_directory(test_dir_);

    // Set config dir to test directory
    cache_.set_config_dir(test_dir_.string());

    // Clean up any existing global enabled marker from previous tests
    std::string marker_path = cache_.enabled_marker_path();
    fs::path marker_file(marker_path);
    if (fs::exists(marker_file)) {
      fs::remove(marker_file);
    }
  }

  void TearDown() override {
    // Clean up test directory
    if (fs::exists(test_dir_)) {
      fs::remove_all(test_dir_);
    }
    // Also clean up the global enabled marker
    std::string marker_path = cache_.enabled_marker_path();
    fs::path marker_file(marker_path);
    if (fs::exists(marker_file)) {
      fs::remove(marker_file);
    }
  }

  // Helper to create a test file
  void create_test_file(const std::string& rel_path, const std::string& content) {
    fs::path file_path = test_dir_ / rel_path;
    fs::create_directories(file_path.parent_path());

    std::ofstream file(file_path);
    file << content;
    file.close();
  }

  // Helper to check if file exists
  bool file_exists(const std::string& rel_path) {
    return fs::exists(test_dir_ / rel_path);
  }

  ConfigCache cache_;
  fs::path test_dir_;
};

// =============================================================================
// Test: guess_content_type
// =============================================================================

TEST(ContentTypeTest, DetectsJsonExtension) {
  EXPECT_EQ(guess_content_type("test.json"), "application/json");
  EXPECT_EQ(guess_content_type("config.json"), "application/json");
}

TEST(ContentTypeTest, DetectsYamlExtension) {
  EXPECT_EQ(guess_content_type("test.yaml"), "text/yaml");
  EXPECT_EQ(guess_content_type("test.yml"), "text/yaml");
}

TEST(ContentTypeTest, DetectsTxtExtension) {
  EXPECT_EQ(guess_content_type("test.txt"), "text/plain");
  EXPECT_EQ(guess_content_type("README.txt"), "text/plain");
}

TEST(ContentTypeTest, DetectsXmlExtension) {
  EXPECT_EQ(guess_content_type("test.xml"), "application/xml");
}

TEST(ContentTypeTest, DetectsUrdfExtension) {
  EXPECT_EQ(guess_content_type("robot.urdf"), "application/x-urdf");
}

TEST(ContentTypeTest, DetectsPdfExtension) {
  EXPECT_EQ(guess_content_type("doc.pdf"), "application/pdf");
}

TEST(ContentTypeTest, DefaultsToOctetStream) {
  EXPECT_EQ(guess_content_type("test.unknown"), "application/octet-stream");
  EXPECT_EQ(guess_content_type("test"), "application/octet-stream");
}

TEST(ContentTypeTest, IsCaseInsensitive) {
  EXPECT_EQ(guess_content_type("test.JSON"), "application/json");
  EXPECT_EQ(guess_content_type("test.TXT"), "text/plain");
  EXPECT_EQ(guess_content_type("test.YAML"), "text/yaml");
}

// =============================================================================
// Test: ConfigCache basic operations
// =============================================================================

TEST_F(ConfigCacheTest, SetConfigDir) {
  cache_.set_config_dir("/custom/path");
  // cache_path() now returns a fixed path outside config_dir
  // It should contain "cache.mcap"
  std::string cache_path = cache_.cache_path();
  EXPECT_TRUE(cache_path.find("cache.mcap") != std::string::npos);
}

TEST_F(ConfigCacheTest, DefaultConfigDir) {
  // cache_path() now returns a fixed path outside config_dir
  // It should contain "cache.mcap"
  std::string cache_path = cache_.cache_path();
  EXPECT_TRUE(cache_path.find("cache.mcap") != std::string::npos);
}

TEST_F(ConfigCacheTest, EnabledMarkerPath) {
  // enabled_marker_path() now returns a fixed path outside config_dir
  // It should contain ".enabled"
  std::string marker_path = cache_.enabled_marker_path();
  EXPECT_TRUE(marker_path.find(".enabled") != std::string::npos);
}

TEST_F(ConfigCacheTest, InitiallyDisabled) {
  EXPECT_FALSE(cache_.is_enabled());
}

TEST_F(ConfigCacheTest, EnableCreatesMarker) {
  EXPECT_TRUE(cache_.enable());
  EXPECT_TRUE(cache_.is_enabled());

  // Clean up
  EXPECT_TRUE(cache_.disable());
}

TEST_F(ConfigCacheTest, DisableRemovesMarker) {
  cache_.enable();
  EXPECT_TRUE(cache_.is_enabled());

  EXPECT_TRUE(cache_.disable());
  EXPECT_FALSE(cache_.is_enabled());

  // Marker file should not exist at the enabled_marker_path location
  std::string marker_path = cache_.enabled_marker_path();
  EXPECT_FALSE(fs::exists(fs::path(marker_path)));
}

TEST_F(ConfigCacheTest, DisableWhenAlreadyDisabled) {
  // When already disabled, disable() returns false (file doesn't exist)
  // But the state is still disabled (no exception thrown)
  EXPECT_FALSE(cache_.disable());
  EXPECT_FALSE(cache_.is_enabled());
}

TEST_F(ConfigCacheTest, ScanEmptyDirectory) {
  auto result = cache_.scan(false);
  EXPECT_EQ(result.status, ConfigCache::Status::Ok);
  EXPECT_EQ(result.file_count, 0);
  EXPECT_EQ(result.total_size, 0);
}

TEST_F(ConfigCacheTest, ScanWithFiles) {
  // Create test files
  create_test_file("robot/type.txt", "AGV-500");
  create_test_file("robot/sn.txt", "12345");
  create_test_file("sensors/camera.json", "{\"width\": 640}");

  auto result = cache_.scan(false);
  EXPECT_EQ(result.status, ConfigCache::Status::Ok);
  EXPECT_EQ(result.file_count, 3);
}

TEST_F(ConfigCacheTest, ScanCreatesCacheFile) {
  create_test_file("test.txt", "content");

  auto result = cache_.scan(false);
  EXPECT_EQ(result.status, ConfigCache::Status::Ok);

  // Cache file should exist at the cache_path location
  std::string cache_path_str = cache_.cache_path();
  EXPECT_TRUE(fs::exists(fs::path(cache_path_str)));
}

TEST_F(ConfigCacheTest, ScanWithoutDirectory) {
  // Use a non-existent directory
  cache_.set_config_dir("/nonexistent/path");

  auto result = cache_.scan(false);
  EXPECT_EQ(result.status, ConfigCache::Status::DirNotFound);
}

TEST_F(ConfigCacheTest, ClearRemovesDirectoryAndCache) {
  // Create some files
  create_test_file("test.txt", "content");
  cache_.scan(false);

  // Create enabled marker
  cache_.enable();

  // Clear
  auto status = cache_.clear();
  EXPECT_EQ(status, ConfigCache::Status::Ok);

  // Config directory should be removed
  EXPECT_FALSE(fs::exists(test_dir_));
}

// =============================================================================
// Test: Status information
// =============================================================================

TEST_F(ConfigCacheTest, StatusWhenDisabled) {
  auto info = cache_.get_status();
  EXPECT_FALSE(info.enabled);
  EXPECT_FALSE(info.cache_exists);
  EXPECT_EQ(info.file_count, 0);
  EXPECT_EQ(info.total_size, 0);
}

TEST_F(ConfigCacheTest, StatusWhenEnabled) {
  create_test_file("test.txt", "content");
  cache_.scan(false);
  cache_.enable();

  auto info = cache_.get_status();
  EXPECT_TRUE(info.enabled);
  EXPECT_TRUE(info.cache_exists);
  EXPECT_EQ(info.file_count, 1);
}

TEST_F(ConfigCacheTest, StatusWhenCached) {
  create_test_file("test.txt", "content");
  cache_.scan(false);

  auto info = cache_.get_status();
  EXPECT_TRUE(info.cache_exists);
  EXPECT_EQ(info.file_count, 1);
  EXPECT_GT(info.total_size, 0);
  EXPECT_GT(info.cache_mtime, 0);
}

// =============================================================================
// Test: Incremental scan
// =============================================================================

TEST_F(ConfigCacheTest, IncrementalScanDetectsNewFiles) {
  // Initial scan
  create_test_file("file1.txt", "content1");
  cache_.scan(false);

  // Add new file
  create_test_file("file2.txt", "content2");

  auto result = cache_.scan(true);
  EXPECT_EQ(result.status, ConfigCache::Status::Ok);

  // Should detect new file
  EXPECT_TRUE(
    result.message.find("added") != std::string::npos ||
    result.message.find("1 added") != std::string::npos
  );
}

TEST_F(ConfigCacheTest, IncrementalScanDetectsModifiedFiles) {
  // Initial scan
  create_test_file("file1.txt", "content1");
  cache_.scan(false);

  // Modify file
  create_test_file("file1.txt", "modified content");

  auto result = cache_.scan(true);
  EXPECT_EQ(result.status, ConfigCache::Status::Ok);

  // Should detect modified file
  EXPECT_TRUE(
    result.message.find("changed") != std::string::npos ||
    result.message.find("1 changed") != std::string::npos
  );
}

TEST_F(ConfigCacheTest, IncrementalScanDetectsDeletedFiles) {
  // Initial scan
  create_test_file("file1.txt", "content1");
  create_test_file("file2.txt", "content2");
  cache_.scan(false);

  // Delete a file
  fs::remove(test_dir_ / "file1.txt");

  auto result = cache_.scan(true);
  EXPECT_EQ(result.status, ConfigCache::Status::Ok);

  // Should detect deleted file
  EXPECT_TRUE(
    result.message.find("removed") != std::string::npos ||
    result.message.find("1 removed") != std::string::npos
  );
}

TEST_F(ConfigCacheTest, IncrementalScanNoChanges) {
  // Initial scan
  create_test_file("file1.txt", "content1");
  cache_.scan(false);

  // Incremental scan with no changes - since files_ is populated from first scan,
  // and no files were modified, it should report no changes
  auto result = cache_.scan(true);
  EXPECT_EQ(result.status, ConfigCache::Status::Ok);

  // Should report no changes (all files unchanged)
  EXPECT_TRUE(
    result.message.find("No changes") != std::string::npos ||
    result.message.find("0 changed") != std::string::npos
  );
}

// =============================================================================
// Test: Hidden files are skipped
// =============================================================================

TEST_F(ConfigCacheTest, ScanSkipsHiddenFiles) {
  // Create hidden file
  create_test_file(".hidden", "content");
  create_test_file("normal.txt", "content");

  auto result = cache_.scan(false);
  EXPECT_EQ(result.status, ConfigCache::Status::Ok);

  // Should only count non-hidden files
  EXPECT_EQ(result.file_count, 1);
}

TEST_F(ConfigCacheTest, ScanSkipsEnabledMarker) {
  // Create files including marker
  create_test_file("test.txt", "content");
  create_test_file(".enabled", "marker");

  auto result = cache_.scan(false);
  EXPECT_EQ(result.status, ConfigCache::Status::Ok);

  // Should skip .enabled file
  EXPECT_EQ(result.file_count, 1);
}

}  // namespace test
}  // namespace config
}  // namespace axon
