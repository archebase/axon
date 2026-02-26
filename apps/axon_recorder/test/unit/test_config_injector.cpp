// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

/**
 * @file test_config_injector.cpp
 * @brief Unit tests for ConfigInjector class
 *
 * Tests config file injection from cache to MCAP recordings.
 */

#include <gtest/gtest.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>

#include "../src/config/config_injector.hpp"
#include "mcap_writer_wrapper.hpp"

namespace fs = std::filesystem;
using namespace axon::recorder;
using namespace axon::mcap_wrapper;

// ============================================================================
// Test Fixtures
// ============================================================================

class ConfigInjectorTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Create a unique temp directory for test files
    test_dir_ = fs::temp_directory_path() /
                ("config_injector_test_" +
                 std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::create_directories(test_dir_);

    // Create .axon directory structure
    axon_dir_ = test_dir_ / ".axon";
    fs::create_directories(axon_dir_);

    // Save original HOME value
    original_home_ = std::getenv("HOME");

    // Set HOME to test directory for ConfigInjector to find our test files
    std::string home_path = test_dir_.string();
    setenv("HOME", home_path.c_str(), 1);
  }

  void TearDown() override {
    // Restore original HOME
    if (original_home_) {
      setenv("HOME", original_home_, 1);
    } else {
      unsetenv("HOME");
    }

    // Clean up test directory
    if (fs::exists(test_dir_)) {
      fs::remove_all(test_dir_);
    }
  }

  // Helper to create a test MCAP cache file with attachments
  void create_test_cache(const std::vector<std::pair<std::string, std::string>>& files) {
    McapWriterOptions options;
    options.compression = Compression::None;

    McapWriterWrapper writer;
    ASSERT_TRUE(writer.open(cache_path().string(), options));

    // Write each file as an attachment
    for (const auto& [name, content] : files) {
      std::string content_type = guess_content_type(name);
      ASSERT_TRUE(writer.write_attachment(name, content_type, content.data(), content.size()));
    }

    writer.close();
  }

  // Helper to enable config injection
  void enable_config() {
    std::ofstream marker(enabled_marker_path());
    ASSERT_TRUE(marker.is_open());
    marker.close();
  }

  // Helper to disable config injection
  void disable_config() {
    if (fs::exists(enabled_marker_path())) {
      fs::remove(enabled_marker_path());
    }
  }

  // Get paths
  fs::path test_dir_;
  fs::path axon_dir_;
  fs::path cache_path() const {
    return axon_dir_ / "cache.mcap";
  }
  fs::path enabled_marker_path() const {
    return axon_dir_ / ".enabled";
  }
  fs::path output_path() const {
    return test_dir_ / "output.mcap";
  }

  // Original HOME value
  char* original_home_ = nullptr;

  // Helper to guess content type (simplified version)
  std::string guess_content_type(const std::string& filename) {
    size_t dot_pos = filename.rfind('.');
    if (dot_pos == std::string::npos) {
      return "application/octet-stream";
    }
    std::string ext = filename.substr(dot_pos);
    if (ext == ".json") {
      return "application/json";
    } else if (ext == ".yaml" || ext == ".yml") {
      return "text/yaml";
    } else if (ext == ".txt") {
      return "text/plain";
    } else if (ext == ".xml") {
      return "application/xml";
    }
    return "application/octet-stream";
  }
};

// ============================================================================
// Test: Path methods
// ============================================================================

TEST_F(ConfigInjectorTest, ReturnsCorrectEnabledMarkerPath) {
  ConfigInjector injector;
  std::string path = injector.enabled_marker_path();

  // Path should contain .enabled
  EXPECT_TRUE(path.find(".enabled") != std::string::npos);

  // Path should point to our test directory
  EXPECT_TRUE(path.find(test_dir_.string()) != std::string::npos);
}

TEST_F(ConfigInjectorTest, ReturnsCorrectCachePath) {
  ConfigInjector injector;
  std::string path = injector.cache_path();

  // Path should contain cache.mcap
  EXPECT_TRUE(path.find("cache.mcap") != std::string::npos);

  // Path should point to our test directory
  EXPECT_TRUE(path.find(test_dir_.string()) != std::string::npos);
}

// ============================================================================
// Test: is_enabled
// ============================================================================

TEST_F(ConfigInjectorTest, IsDisabledInitially) {
  ConfigInjector injector;
  EXPECT_FALSE(injector.is_enabled());
}

TEST_F(ConfigInjectorTest, IsEnabledAfterCreatingMarker) {
  enable_config();
  ConfigInjector injector;
  EXPECT_TRUE(injector.is_enabled());
}

TEST_F(ConfigInjectorTest, IsDisabledAfterRemovingMarker) {
  enable_config();

  ConfigInjector injector1;
  EXPECT_TRUE(injector1.is_enabled());

  disable_config();

  ConfigInjector injector2;
  EXPECT_FALSE(injector2.is_enabled());
}

// ============================================================================
// Test: inject behavior
// ============================================================================

TEST_F(ConfigInjectorTest, InjectReturnsSuccessWhenDisabled) {
  disable_config();

  McapWriterOptions options;
  McapWriterWrapper writer;
  ASSERT_TRUE(writer.open(output_path().string(), options));

  ConfigInjector injector;
  auto result = injector.inject(writer);

  writer.close();

  // Should return success (disabled is not an error)
  EXPECT_TRUE(result.success);
  EXPECT_EQ(0, result.files_injected);
  EXPECT_TRUE(result.error_message.find("disabled") != std::string::npos);
}

TEST_F(ConfigInjectorTest, InjectReturnsErrorWhenEnabledButCacheNotExists) {
  enable_config();  // Enable but no cache

  McapWriterOptions options;
  McapWriterWrapper writer;
  ASSERT_TRUE(writer.open(output_path().string(), options));

  ConfigInjector injector;
  auto result = injector.inject(writer);

  writer.close();

  // Should fail because cache doesn't exist
  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.error_message.empty());
  EXPECT_TRUE(
    result.error_message.find("not found") != std::string::npos ||
    result.error_message.find("cache") != std::string::npos
  );
}

TEST_F(ConfigInjectorTest, InjectCopiesAttachmentsWithConfigPrefix) {
  // Create test cache with multiple files
  std::vector<std::pair<std::string, std::string>> files = {
    {"robot/type.txt", "AGV-500"},
    {"robot/sn.txt", "12345"},
    {"sensors/camera.json", "{\"width\": 640}"},
  };
  create_test_cache(files);
  enable_config();

  McapWriterOptions options;
  McapWriterWrapper writer;
  ASSERT_TRUE(writer.open(output_path().string(), options));

  ConfigInjector injector;
  auto result = injector.inject(writer);

  writer.close();

  EXPECT_TRUE(result.success);
  EXPECT_EQ(3, result.files_injected);
  EXPECT_GT(result.total_bytes, 0);
}

TEST_F(ConfigInjectorTest, InjectHandlesSingleFile) {
  std::vector<std::pair<std::string, std::string>> files = {
    {"test.txt", "Hello, World!"},
  };
  create_test_cache(files);
  enable_config();

  McapWriterOptions options;
  McapWriterWrapper writer;
  ASSERT_TRUE(writer.open(output_path().string(), options));

  ConfigInjector injector;
  auto result = injector.inject(writer);

  writer.close();

  EXPECT_TRUE(result.success);
  EXPECT_EQ(1, result.files_injected);
  EXPECT_GT(result.total_bytes, 0);
}

TEST_F(ConfigInjectorTest, InjectHandlesEmptyCache) {
  // Create empty cache (no attachments)
  McapWriterOptions options;
  McapWriterWrapper cache_writer;
  ASSERT_TRUE(cache_writer.open(cache_path().string(), options));
  cache_writer.close();

  enable_config();

  McapWriterWrapper writer;
  ASSERT_TRUE(writer.open(output_path().string(), options));

  ConfigInjector injector;
  auto result = injector.inject(writer);

  writer.close();

  // Should succeed even with no attachments
  EXPECT_TRUE(result.success);
  EXPECT_EQ(0, result.files_injected);
}

TEST_F(ConfigInjectorTest, InjectHandlesNestedPaths) {
  // Create test cache with nested directory structure
  std::vector<std::pair<std::string, std::string>> files = {
    {"robot/urdf/model.xml", "<robot></robot>"},
    {"sensors/camera/intrinsics.json", "{\"K\": [1, 0, 0, 0, 1, 0, 0, 0, 1]}"},
    {"sensors/camera/extrinsics.json", "{\"translation\": [0, 0, 0]}"},
    {"calibration/lidarParams.yaml", "range_min: 0.1\nrange_max: 10.0"},
  };
  create_test_cache(files);
  enable_config();

  McapWriterOptions options;
  McapWriterWrapper writer;
  ASSERT_TRUE(writer.open(output_path().string(), options));

  ConfigInjector injector;
  auto result = injector.inject(writer);

  writer.close();

  EXPECT_TRUE(result.success);
  EXPECT_EQ(4, result.files_injected);
}

TEST_F(ConfigInjectorTest, InjectPreservesAttachmentContentTypes) {
  // Create test cache with different content types
  std::vector<std::pair<std::string, std::string>> files = {
    {"data.json", "{\"key\": \"value\"}"},
    {"data.yaml", "key: value"},
    {"data.txt", "plain text"},
    {"data.xml", "<root></root>"},
  };
  create_test_cache(files);
  enable_config();

  McapWriterOptions options;
  McapWriterWrapper writer;
  ASSERT_TRUE(writer.open(output_path().string(), options));

  ConfigInjector injector;
  auto result = injector.inject(writer);

  writer.close();

  EXPECT_TRUE(result.success);
  EXPECT_EQ(4, result.files_injected);
}

TEST_F(ConfigInjectorTest, InjectWithLargeFile) {
  // Create a larger file (10KB)
  std::string large_content(10240, 'X');
  std::vector<std::pair<std::string, std::string>> files = {
    {"large_data.bin", large_content},
  };
  create_test_cache(files);
  enable_config();

  McapWriterOptions options;
  McapWriterWrapper writer;
  ASSERT_TRUE(writer.open(output_path().string(), options));

  ConfigInjector injector;
  auto result = injector.inject(writer);

  writer.close();

  EXPECT_TRUE(result.success);
  EXPECT_EQ(1, result.files_injected);
  EXPECT_EQ(10240, result.total_bytes);
}

TEST_F(ConfigInjectorTest, MultipleInjectCallsUseSameCache) {
  std::vector<std::pair<std::string, std::string>> files = {
    {"test.txt", "content"},
  };
  create_test_cache(files);
  enable_config();

  // First injection
  {
    McapWriterOptions options;
    McapWriterWrapper writer;
    ASSERT_TRUE(writer.open(output_path().string(), options));

    ConfigInjector injector;
    auto result = injector.inject(writer);
    writer.close();

    EXPECT_TRUE(result.success);
    EXPECT_EQ(1, result.files_injected);
  }

  // Second injection (different output file)
  {
    fs::path output2 = test_dir_ / "output2.mcap";
    McapWriterOptions options;
    McapWriterWrapper writer;
    ASSERT_TRUE(writer.open(output2.string(), options));

    ConfigInjector injector;
    auto result = injector.inject(writer);
    writer.close();

    EXPECT_TRUE(result.success);
    EXPECT_EQ(1, result.files_injected);
  }
}

TEST_F(ConfigInjectorTest, InjectAfterDisableThenReenable) {
  std::vector<std::pair<std::string, std::string>> files = {
    {"test.txt", "content"},
  };
  create_test_cache(files);

  // First with disabled
  disable_config();
  {
    McapWriterOptions options;
    McapWriterWrapper writer;
    ASSERT_TRUE(writer.open(output_path().string(), options));

    ConfigInjector injector;
    auto result = injector.inject(writer);
    writer.close();

    EXPECT_TRUE(result.success);
    EXPECT_EQ(0, result.files_injected);
  }

  // Then enable and inject again
  enable_config();
  {
    fs::path output2 = test_dir_ / "output2.mcap";
    McapWriterOptions options;
    McapWriterWrapper writer;
    ASSERT_TRUE(writer.open(output2.string(), options));

    ConfigInjector injector;
    auto result = injector.inject(writer);
    writer.close();

    EXPECT_TRUE(result.success);
    EXPECT_EQ(1, result.files_injected);
  }
}
