// Copyright (c) 2026 ArcheBase
// Axon is licensed under Mulan PSL v2.
// You can use this software according to the terms and conditions of the Mulan PSL v2.
// You may obtain a copy of Mulan PSL v2 at:
//          http://license.coscl.org.cn/MulanPSL2
// THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
// EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
// MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
// See the Mulan PSL v2 for more details.

/**
 * @file test_file_sink.cpp
 * @brief Unit tests for file sink creation, rotation, and configuration
 */

#include <boost/log/attributes/constant.hpp>
#include <boost/log/attributes/scoped_attribute.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>

#include "axon_file_sink.hpp"
#include "axon_log_init.hpp"
#include "axon_log_macros.hpp"

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
    test_dir_ = fs::temp_directory_path() /
                ("file_sink_test_" +
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
// JSON Formatter Tests (exercise json_formatter and escape_json)
// ============================================================================

class FileSinkJsonFormatterTest : public ::testing::Test {
protected:
  void SetUp() override {
    test_dir_ = fs::temp_directory_path() /
                ("file_sink_json_" +
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

TEST_F(FileSinkJsonFormatterTest, JsonFormatAllSeverityLevels) {
  FileSinkConfig config;
  config.directory = test_dir_.string();
  config.format_json = true;
  config.file_pattern = "json_test_%N.log";

  auto sink = create_file_sink(config, severity_level::debug);
  ASSERT_NE(sink, nullptr);

  add_sink(sink);
  boost::log::add_common_attributes();

  // Log at all severity levels to exercise JSON formatter
  AXON_LOG_DEBUG("JSON debug message");
  AXON_LOG_INFO("JSON info message");
  AXON_LOG_WARN("JSON warn message");
  AXON_LOG_ERROR("JSON error message");
  AXON_LOG_FATAL("JSON fatal message");

  sink->flush();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  remove_sink(sink);
}

TEST_F(FileSinkJsonFormatterTest, JsonFormatWithContextAttributes) {
  FileSinkConfig config;
  config.directory = test_dir_.string();
  config.format_json = true;
  config.file_pattern = "json_context_%N.log";

  auto sink = create_file_sink(config, severity_level::debug);
  ASSERT_NE(sink, nullptr);

  add_sink(sink);
  boost::log::add_common_attributes();

  // Test with TaskID only
  {
    BOOST_LOG_SCOPED_THREAD_ATTR(
      "TaskID", boost::log::attributes::constant<std::string>("task_json_123")
    );
    AXON_LOG_INFO("Message with TaskID only in JSON format");
  }

  // Test with DeviceID only
  {
    BOOST_LOG_SCOPED_THREAD_ATTR(
      "DeviceID", boost::log::attributes::constant<std::string>("device_json_456")
    );
    AXON_LOG_INFO("Message with DeviceID only in JSON format");
  }

  // Test with both
  {
    BOOST_LOG_SCOPED_THREAD_ATTR(
      "TaskID", boost::log::attributes::constant<std::string>("task_both")
    );
    BOOST_LOG_SCOPED_THREAD_ATTR(
      "DeviceID", boost::log::attributes::constant<std::string>("device_both")
    );
    AXON_LOG_INFO("Message with both attributes in JSON format");
  }

  sink->flush();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  remove_sink(sink);
}

TEST_F(FileSinkJsonFormatterTest, JsonEscapeSpecialCharacters) {
  FileSinkConfig config;
  config.directory = test_dir_.string();
  config.format_json = true;
  config.file_pattern = "json_escape_%N.log";

  auto sink = create_file_sink(config, severity_level::debug);
  ASSERT_NE(sink, nullptr);

  add_sink(sink);
  boost::log::add_common_attributes();

  // Test JSON escaping of special characters
  // Quotes
  AXON_LOG_INFO("Message with \"quotes\" inside");

  // Backslash
  AXON_LOG_INFO("Message with \\backslash\\ inside");

  // Newline (embedded via string literal)
  AXON_LOG_INFO("Message with newline\ninside");

  // Tab
  AXON_LOG_INFO("Message with tab\tinside");

  // Carriage return
  AXON_LOG_INFO("Message with CR\rinside");

  // Backspace (using escape)
  std::string msg_with_backspace = "Message with backspace";
  msg_with_backspace += '\b';
  msg_with_backspace += "inside";
  AXON_LOG_INFO(msg_with_backspace);

  // Form feed
  std::string msg_with_ff = "Message with form feed";
  msg_with_ff += '\f';
  msg_with_ff += "inside";
  AXON_LOG_INFO(msg_with_ff);

  // Control characters (low ASCII, e.g., 0x01)
  std::string msg_with_ctrl = "Message with control char";
  msg_with_ctrl += '\x01';
  msg_with_ctrl += "inside";
  AXON_LOG_INFO(msg_with_ctrl);

  sink->flush();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  remove_sink(sink);
}

TEST_F(FileSinkJsonFormatterTest, JsonEscapeInContextAttributes) {
  FileSinkConfig config;
  config.directory = test_dir_.string();
  config.format_json = true;
  config.file_pattern = "json_ctx_escape_%N.log";

  auto sink = create_file_sink(config, severity_level::debug);
  ASSERT_NE(sink, nullptr);

  add_sink(sink);
  boost::log::add_common_attributes();

  // Test escaping in context attributes (TaskID and DeviceID)
  {
    BOOST_LOG_SCOPED_THREAD_ATTR(
      "TaskID", boost::log::attributes::constant<std::string>("task\"with\"quotes")
    );
    BOOST_LOG_SCOPED_THREAD_ATTR(
      "DeviceID", boost::log::attributes::constant<std::string>("device\\with\\backslash")
    );
    AXON_LOG_INFO("Message with escaped context attributes");
  }

  sink->flush();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  remove_sink(sink);
}

// ============================================================================
// Text Formatter Tests (exercise text_formatter)
// ============================================================================

class FileSinkTextFormatterTest : public ::testing::Test {
protected:
  void SetUp() override {
    test_dir_ = fs::temp_directory_path() /
                ("file_sink_text_" +
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

TEST_F(FileSinkTextFormatterTest, TextFormatAllSeverityLevels) {
  FileSinkConfig config;
  config.directory = test_dir_.string();
  config.format_json = false;  // Use text format
  config.file_pattern = "text_test_%N.log";

  auto sink = create_file_sink(config, severity_level::debug);
  ASSERT_NE(sink, nullptr);

  add_sink(sink);
  boost::log::add_common_attributes();

  AXON_LOG_DEBUG("Text debug message");
  AXON_LOG_INFO("Text info message");
  AXON_LOG_WARN("Text warn message");
  AXON_LOG_ERROR("Text error message");
  AXON_LOG_FATAL("Text fatal message");

  sink->flush();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  remove_sink(sink);
}

TEST_F(FileSinkTextFormatterTest, TextFormatWithContextAttributes) {
  FileSinkConfig config;
  config.directory = test_dir_.string();
  config.format_json = false;
  config.file_pattern = "text_context_%N.log";

  auto sink = create_file_sink(config, severity_level::debug);
  ASSERT_NE(sink, nullptr);

  add_sink(sink);
  boost::log::add_common_attributes();

  // Test with TaskID only
  {
    BOOST_LOG_SCOPED_THREAD_ATTR(
      "TaskID", boost::log::attributes::constant<std::string>("task_text_123")
    );
    AXON_LOG_INFO("Text message with TaskID only");
  }

  // Test with DeviceID only
  {
    BOOST_LOG_SCOPED_THREAD_ATTR(
      "DeviceID", boost::log::attributes::constant<std::string>("device_text_456")
    );
    AXON_LOG_INFO("Text message with DeviceID only");
  }

  // Test with both
  {
    BOOST_LOG_SCOPED_THREAD_ATTR(
      "TaskID", boost::log::attributes::constant<std::string>("task_text_both")
    );
    BOOST_LOG_SCOPED_THREAD_ATTR(
      "DeviceID", boost::log::attributes::constant<std::string>("device_text_both")
    );
    AXON_LOG_INFO("Text message with both attributes");
  }

  // Test with neither (no context)
  AXON_LOG_INFO("Text message without context attributes");

  sink->flush();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  remove_sink(sink);
}

// ============================================================================
// Directory Creation Tests (exercise fallback path)
// ============================================================================

class FileSinkDirectoryTest : public ::testing::Test {
protected:
  void SetUp() override {
    test_dir_ = fs::temp_directory_path() /
                ("file_sink_dir_" +
                 std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));

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

TEST_F(FileSinkDirectoryTest, CreateNonExistentDirectory) {
  // Directory doesn't exist yet
  ASSERT_FALSE(fs::exists(test_dir_));

  FileSinkConfig config;
  config.directory = test_dir_.string();
  config.file_pattern = "test_%N.log";

  // Should create the directory
  auto sink = create_file_sink(config, severity_level::info);
  ASSERT_NE(sink, nullptr);

  // Directory should now exist
  EXPECT_TRUE(fs::exists(test_dir_));
}

TEST_F(FileSinkDirectoryTest, CreateNestedNonExistentDirectory) {
  fs::path nested_dir = test_dir_ / "subdir1" / "subdir2" / "logs";
  ASSERT_FALSE(fs::exists(nested_dir));

  FileSinkConfig config;
  config.directory = nested_dir.string();
  config.file_pattern = "nested_%N.log";

  auto sink = create_file_sink(config, severity_level::info);
  ASSERT_NE(sink, nullptr);

  EXPECT_TRUE(fs::exists(nested_dir));
}

TEST_F(FileSinkDirectoryTest, FallbackToTmpOnUnwritableDirectory) {
  // Test the fallback path when directory creation fails
  // Use a path that cannot be created (e.g., under /proc or read-only)
  // Note: This test may not work on all systems, so we'll use a path that
  // should fail on most Unix systems
  FileSinkConfig config;
  config.directory = "/nonexistent_root_dir_12345/subdir/logs";
  config.file_pattern = "test_%N.log";

  // Should fall back to /tmp without crashing
  auto sink = create_file_sink(config, severity_level::info);
  ASSERT_NE(sink, nullptr);

  // Verify logs can be written (to /tmp fallback)
  add_sink(sink);
  boost::log::add_common_attributes();
  AXON_LOG_INFO("Message after fallback to /tmp");
  sink->flush();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  remove_sink(sink);
}

TEST_F(FileSinkTest, MidnightRotationEnabled) {
  FileSinkConfig config;
  config.directory = test_dir_.string();
  config.rotate_at_midnight = true;  // Explicitly test this path

  auto sink = create_file_sink(config, severity_level::debug);
  ASSERT_NE(sink, nullptr);

  // Verify sink works
  add_sink(sink);
  boost::log::add_common_attributes();
  AXON_LOG_INFO("Test message with midnight rotation enabled");
  sink->flush();
  remove_sink(sink);
}

// ============================================================================
// Integration Tests
// ============================================================================

class FileSinkIntegrationTest : public ::testing::Test {
protected:
  void SetUp() override {
    test_dir_ = fs::temp_directory_path() /
                ("file_sink_int_" +
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

TEST_F(FileSinkIntegrationTest, FullLoggingPipelineJson) {
  LoggingConfig config;
  config.console_enabled = false;
  config.file_enabled = true;
  config.file_config.directory = test_dir_.string();
  config.file_config.format_json = true;
  config.file_config.file_pattern = "pipeline_%N.log";
  config.file_level = severity_level::debug;

  init_logging(config);
  EXPECT_TRUE(is_logging_initialized());

  // Log messages at all levels with context
  BOOST_LOG_SCOPED_THREAD_ATTR(
    "TaskID", boost::log::attributes::constant<std::string>("pipeline_task")
  );
  BOOST_LOG_SCOPED_THREAD_ATTR(
    "DeviceID", boost::log::attributes::constant<std::string>("pipeline_device")
  );
  AXON_LOG_DEBUG("Pipeline debug");
  AXON_LOG_INFO("Pipeline info");
  AXON_LOG_WARN("Pipeline warn");
  AXON_LOG_ERROR("Pipeline error");

  flush_logging();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

TEST_F(FileSinkIntegrationTest, FullLoggingPipelineText) {
  LoggingConfig config;
  config.console_enabled = false;
  config.file_enabled = true;
  config.file_config.directory = test_dir_.string();
  config.file_config.format_json = false;  // Text format
  config.file_config.file_pattern = "pipeline_text_%N.log";
  config.file_level = severity_level::debug;

  init_logging(config);
  EXPECT_TRUE(is_logging_initialized());

  BOOST_LOG_SCOPED_THREAD_ATTR(
    "TaskID", boost::log::attributes::constant<std::string>("text_task")
  );
  BOOST_LOG_SCOPED_THREAD_ATTR(
    "DeviceID", boost::log::attributes::constant<std::string>("text_device")
  );
  AXON_LOG_DEBUG("Text pipeline debug");
  AXON_LOG_INFO("Text pipeline info");

  flush_logging();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
