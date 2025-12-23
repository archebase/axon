/**
 * @file test_file_sink.cpp
 * @brief Unit tests for file sink creation, rotation, and configuration
 */

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

#include "axon_file_sink.hpp"
#include "axon_log_init.hpp"

namespace fs = std::filesystem;

using namespace axon::logging;

// ============================================================================
// File Sink Config Tests
// ============================================================================

class FileSinkConfigTest : public ::testing::Test {
protected:
  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(FileSinkConfigTest, DefaultValues) {
  FileSinkConfig config;
  
  EXPECT_EQ(config.directory, "/var/log/axon");
  EXPECT_EQ(config.file_pattern, "recorder_%Y%m%d_%H%M%S.log");
  EXPECT_EQ(config.rotation_size_mb, 100);
  EXPECT_TRUE(config.rotate_at_midnight);
  EXPECT_EQ(config.max_files, 10);
  EXPECT_TRUE(config.format_json);
}

TEST_F(FileSinkConfigTest, CustomValues) {
  FileSinkConfig config;
  config.directory = "/tmp/custom_logs";
  config.file_pattern = "myapp_%Y%m%d.log";
  config.rotation_size_mb = 50;
  config.rotate_at_midnight = false;
  config.max_files = 5;
  config.format_json = false;
  
  EXPECT_EQ(config.directory, "/tmp/custom_logs");
  EXPECT_EQ(config.file_pattern, "myapp_%Y%m%d.log");
  EXPECT_EQ(config.rotation_size_mb, 50);
  EXPECT_FALSE(config.rotate_at_midnight);
  EXPECT_EQ(config.max_files, 5);
  EXPECT_FALSE(config.format_json);
}

// ============================================================================
// File Sink Creation Tests
// ============================================================================

class FileSinkTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Create a unique temp directory for test files
    test_dir_ = fs::temp_directory_path() / ("file_sink_test_" +
        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::create_directories(test_dir_);
    
    // Ensure clean logging state
    if (is_logging_initialized()) {
      shutdown_logging();
    }
  }

  void TearDown() override {
    if (is_logging_initialized()) {
      shutdown_logging();
    }
    
    // Clean up test directory
    if (fs::exists(test_dir_)) {
      fs::remove_all(test_dir_);
    }
  }

  fs::path test_dir_;
};

TEST_F(FileSinkTest, CreateWithConfig) {
  FileSinkConfig config;
  config.directory = test_dir_.string();
  config.file_pattern = "test_%Y%m%d_%H%M%S.log";
  config.rotation_size_mb = 1;
  config.max_files = 3;
  config.format_json = true;
  
  auto sink = create_file_sink(config, severity_level::debug);
  ASSERT_NE(sink, nullptr);
}

TEST_F(FileSinkTest, CreateWithDebugLevel) {
  FileSinkConfig config;
  config.directory = test_dir_.string();
  
  auto sink = create_file_sink(config, severity_level::debug);
  ASSERT_NE(sink, nullptr);
}

TEST_F(FileSinkTest, CreateWithInfoLevel) {
  FileSinkConfig config;
  config.directory = test_dir_.string();
  
  auto sink = create_file_sink(config, severity_level::info);
  ASSERT_NE(sink, nullptr);
}

TEST_F(FileSinkTest, CreateWithWarnLevel) {
  FileSinkConfig config;
  config.directory = test_dir_.string();
  
  auto sink = create_file_sink(config, severity_level::warn);
  ASSERT_NE(sink, nullptr);
}

TEST_F(FileSinkTest, CreateTextFormat) {
  FileSinkConfig config;
  config.directory = test_dir_.string();
  config.format_json = false;
  
  auto sink = create_file_sink(config, severity_level::info);
  ASSERT_NE(sink, nullptr);
}

TEST_F(FileSinkTest, CreateJsonFormat) {
  FileSinkConfig config;
  config.directory = test_dir_.string();
  config.format_json = true;
  
  auto sink = create_file_sink(config, severity_level::info);
  ASSERT_NE(sink, nullptr);
}

TEST_F(FileSinkTest, AddSinkToCore) {
  FileSinkConfig config;
  config.directory = test_dir_.string();
  
  auto sink = create_file_sink(config, severity_level::info);
  ASSERT_NE(sink, nullptr);
  
  // Should be able to add to logging core
  EXPECT_NO_THROW(add_sink(sink));
  
  // Clean up
  remove_sink(sink);
}

TEST_F(FileSinkTest, SinkFlush) {
  FileSinkConfig config;
  config.directory = test_dir_.string();
  
  auto sink = create_file_sink(config, severity_level::info);
  ASSERT_NE(sink, nullptr);
  
  add_sink(sink);
  
  // Flush should not throw
  EXPECT_NO_THROW(sink->flush());
  
  remove_sink(sink);
}

TEST_F(FileSinkTest, SmallRotationSize) {
  FileSinkConfig config;
  config.directory = test_dir_.string();
  config.rotation_size_mb = 1;  // Small rotation for testing
  
  auto sink = create_file_sink(config, severity_level::debug);
  ASSERT_NE(sink, nullptr);
}

TEST_F(FileSinkTest, DisableMidnightRotation) {
  FileSinkConfig config;
  config.directory = test_dir_.string();
  config.rotate_at_midnight = false;
  
  auto sink = create_file_sink(config, severity_level::debug);
  ASSERT_NE(sink, nullptr);
}

TEST_F(FileSinkTest, SingleMaxFile) {
  FileSinkConfig config;
  config.directory = test_dir_.string();
  config.max_files = 1;
  
  auto sink = create_file_sink(config, severity_level::debug);
  ASSERT_NE(sink, nullptr);
}

TEST_F(FileSinkTest, LargeMaxFiles) {
  FileSinkConfig config;
  config.directory = test_dir_.string();
  config.max_files = 100;
  
  auto sink = create_file_sink(config, severity_level::debug);
  ASSERT_NE(sink, nullptr);
}

TEST_F(FileSinkTest, CustomFilePattern) {
  FileSinkConfig config;
  config.directory = test_dir_.string();
  config.file_pattern = "custom_app_%N.log";
  
  auto sink = create_file_sink(config, severity_level::debug);
  ASSERT_NE(sink, nullptr);
}

// ============================================================================
// Integration Tests
// ============================================================================

class FileSinkIntegrationTest : public ::testing::Test {
protected:
  void SetUp() override {
    test_dir_ = fs::temp_directory_path() / ("file_sink_int_" +
        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::create_directories(test_dir_);
    
    if (is_logging_initialized()) {
      shutdown_logging();
    }
  }

  void TearDown() override {
    if (is_logging_initialized()) {
      shutdown_logging();
    }
    
    if (fs::exists(test_dir_)) {
      fs::remove_all(test_dir_);
    }
  }

  fs::path test_dir_;
};

TEST_F(FileSinkIntegrationTest, InitLoggingWithFileSink) {
  LoggingConfig config;
  config.console_enabled = false;  // Disable console for cleaner test
  config.file_enabled = true;
  config.file_config.directory = test_dir_.string();
  config.file_config.format_json = true;
  config.file_level = severity_level::debug;
  
  init_logging(config);
  
  EXPECT_TRUE(is_logging_initialized());
  
  // Give async sink time to process
  flush_logging();
}

TEST_F(FileSinkIntegrationTest, ReconfigureWithDifferentSettings) {
  // Initial config with file disabled
  LoggingConfig config1;
  config1.console_enabled = true;
  config1.file_enabled = false;
  
  init_logging(config1);
  EXPECT_TRUE(is_logging_initialized());
  
  // Reconfigure with file enabled
  LoggingConfig config2;
  config2.console_enabled = true;
  config2.file_enabled = true;
  config2.file_config.directory = test_dir_.string();
  
  reconfigure_logging(config2);
  
  EXPECT_TRUE(is_logging_initialized());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

