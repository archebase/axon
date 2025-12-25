/**
 * @file test_log_init.cpp
 * @brief Unit tests for logging initialization and configuration
 */

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <sstream>
#include <string>
#include <thread>

#include "axon_log_init.hpp"
#include "axon_log_severity.hpp"
#include "axon_log_macros.hpp"

namespace fs = std::filesystem;

using namespace axon::logging;

// ============================================================================
// Severity Level Tests
// ============================================================================

class SeverityLevelTest : public ::testing::Test {
protected:
  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(SeverityLevelTest, ParseValidLevels) {
  auto debug = parse_severity_level("debug");
  ASSERT_TRUE(debug.has_value());
  EXPECT_EQ(*debug, severity_level::debug);

  auto info = parse_severity_level("info");
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(*info, severity_level::info);

  auto warn = parse_severity_level("warn");
  ASSERT_TRUE(warn.has_value());
  EXPECT_EQ(*warn, severity_level::warn);

  auto warning = parse_severity_level("warning");
  ASSERT_TRUE(warning.has_value());
  EXPECT_EQ(*warning, severity_level::warn);

  auto error = parse_severity_level("error");
  ASSERT_TRUE(error.has_value());
  EXPECT_EQ(*error, severity_level::error);

  auto fatal = parse_severity_level("fatal");
  ASSERT_TRUE(fatal.has_value());
  EXPECT_EQ(*fatal, severity_level::fatal);
}

TEST_F(SeverityLevelTest, ParseCaseInsensitive) {
  auto debug_upper = parse_severity_level("DEBUG");
  ASSERT_TRUE(debug_upper.has_value());
  EXPECT_EQ(*debug_upper, severity_level::debug);

  auto info_mixed = parse_severity_level("InFo");
  ASSERT_TRUE(info_mixed.has_value());
  EXPECT_EQ(*info_mixed, severity_level::info);

  auto warn_upper = parse_severity_level("WARN");
  ASSERT_TRUE(warn_upper.has_value());
  EXPECT_EQ(*warn_upper, severity_level::warn);
  
  // Additional case variations
  auto warning_mixed = parse_severity_level("WaRnInG");
  ASSERT_TRUE(warning_mixed.has_value());
  EXPECT_EQ(*warning_mixed, severity_level::warn);
  
  auto error_upper = parse_severity_level("ERROR");
  ASSERT_TRUE(error_upper.has_value());
  EXPECT_EQ(*error_upper, severity_level::error);
  
  auto fatal_mixed = parse_severity_level("FaTaL");
  ASSERT_TRUE(fatal_mixed.has_value());
  EXPECT_EQ(*fatal_mixed, severity_level::fatal);
}

TEST_F(SeverityLevelTest, ParseInvalidLevels) {
  auto invalid = parse_severity_level("invalid");
  EXPECT_FALSE(invalid.has_value());

  auto empty = parse_severity_level("");
  EXPECT_FALSE(empty.has_value());

  auto numeric = parse_severity_level("123");
  EXPECT_FALSE(numeric.has_value());
  
  // Whitespace
  auto whitespace = parse_severity_level("  debug  ");
  EXPECT_FALSE(whitespace.has_value());
  
  // Partial match
  auto partial = parse_severity_level("deb");
  EXPECT_FALSE(partial.has_value());
  
  // Extra characters
  auto extra = parse_severity_level("debugx");
  EXPECT_FALSE(extra.has_value());
}

TEST_F(SeverityLevelTest, SeverityLevelOutputStream) {
  std::ostringstream oss;
  
  oss << severity_level::debug;
  EXPECT_EQ(oss.str(), "DEBUG");
  
  oss.str("");
  oss << severity_level::info;
  EXPECT_EQ(oss.str(), "INFO");
  
  oss.str("");
  oss << severity_level::warn;
  EXPECT_EQ(oss.str(), "WARN");
  
  oss.str("");
  oss << severity_level::error;
  EXPECT_EQ(oss.str(), "ERROR");
  
  oss.str("");
  oss << severity_level::fatal;
  EXPECT_EQ(oss.str(), "FATAL");
}

TEST_F(SeverityLevelTest, SeverityLevelOutputStreamOutOfRange) {
  std::ostringstream oss;
  
  // Test out-of-range severity level (triggers the else branch in operator<<)
  // Cast an invalid integer value to severity_level
  severity_level invalid_level = static_cast<severity_level>(100);
  oss << invalid_level;
  
  // Should output the numeric value
  EXPECT_EQ(oss.str(), "100");
  
  // Test another out-of-range value
  oss.str("");
  invalid_level = static_cast<severity_level>(-1);
  oss << invalid_level;
  // -1 as unsigned size_t would be a very large number, triggering the else branch
  EXPECT_FALSE(oss.str().empty());
}

// ============================================================================
// Logging Config Tests
// ============================================================================

class LoggingConfigTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Clear any existing env vars
    unsetenv("AXON_LOG_LEVEL");
    unsetenv("AXON_LOG_CONSOLE_LEVEL");
    unsetenv("AXON_LOG_FILE_LEVEL");
    unsetenv("AXON_LOG_FILE_DIR");
    unsetenv("AXON_LOG_FORMAT");
    unsetenv("AXON_LOG_FILE_ENABLED");
    unsetenv("AXON_LOG_CONSOLE_ENABLED");
  }

  void TearDown() override {
    // Clean up env vars
    unsetenv("AXON_LOG_LEVEL");
    unsetenv("AXON_LOG_CONSOLE_LEVEL");
    unsetenv("AXON_LOG_FILE_LEVEL");
    unsetenv("AXON_LOG_FILE_DIR");
    unsetenv("AXON_LOG_FORMAT");
    unsetenv("AXON_LOG_FILE_ENABLED");
    unsetenv("AXON_LOG_CONSOLE_ENABLED");
    
    // Shutdown logging if initialized
    if (is_logging_initialized()) {
      shutdown_logging();
    }
  }
};

TEST_F(LoggingConfigTest, DefaultConfigValues) {
  LoggingConfig config;
  
  EXPECT_TRUE(config.console_enabled);
  EXPECT_TRUE(config.console_colors);
  EXPECT_EQ(config.console_level, severity_level::info);
  
  EXPECT_TRUE(config.file_enabled);
  EXPECT_EQ(config.file_level, severity_level::debug);
  EXPECT_EQ(config.file_config.directory, "/var/log/axon");
  EXPECT_TRUE(config.file_config.format_json);
}

TEST_F(LoggingConfigTest, ApplyEnvOverrides_GlobalLevel) {
  LoggingConfig config;
  setenv("AXON_LOG_LEVEL", "warn", 1);
  
  apply_env_overrides(config);
  
  EXPECT_EQ(config.console_level, severity_level::warn);
  EXPECT_EQ(config.file_level, severity_level::warn);
}

TEST_F(LoggingConfigTest, ApplyEnvOverrides_IndividualLevels) {
  LoggingConfig config;
  setenv("AXON_LOG_CONSOLE_LEVEL", "error", 1);
  setenv("AXON_LOG_FILE_LEVEL", "debug", 1);
  
  apply_env_overrides(config);
  
  EXPECT_EQ(config.console_level, severity_level::error);
  EXPECT_EQ(config.file_level, severity_level::debug);
}

TEST_F(LoggingConfigTest, ApplyEnvOverrides_FileDirectory) {
  LoggingConfig config;
  setenv("AXON_LOG_FILE_DIR", "/tmp/test_logs", 1);
  
  apply_env_overrides(config);
  
  EXPECT_EQ(config.file_config.directory, "/tmp/test_logs");
}

TEST_F(LoggingConfigTest, ApplyEnvOverrides_Format) {
  LoggingConfig config;
  config.file_config.format_json = true;
  
  setenv("AXON_LOG_FORMAT", "text", 1);
  apply_env_overrides(config);
  EXPECT_FALSE(config.file_config.format_json);
  
  setenv("AXON_LOG_FORMAT", "json", 1);
  apply_env_overrides(config);
  EXPECT_TRUE(config.file_config.format_json);
}

TEST_F(LoggingConfigTest, ApplyEnvOverrides_EnableDisable) {
  LoggingConfig config;
  
  setenv("AXON_LOG_FILE_ENABLED", "false", 1);
  setenv("AXON_LOG_CONSOLE_ENABLED", "false", 1);
  
  apply_env_overrides(config);
  
  EXPECT_FALSE(config.file_enabled);
  EXPECT_FALSE(config.console_enabled);
}

// ============================================================================
// Boolean Parsing Edge Cases (exercise parse_bool)
// ============================================================================

TEST_F(LoggingConfigTest, ApplyEnvOverrides_BooleanTrue_Variations) {
  LoggingConfig config;
  
  // Test "true" (case variations)
  config.console_enabled = false;
  setenv("AXON_LOG_CONSOLE_ENABLED", "true", 1);
  apply_env_overrides(config);
  EXPECT_TRUE(config.console_enabled);
  
  config.console_enabled = false;
  setenv("AXON_LOG_CONSOLE_ENABLED", "TRUE", 1);
  apply_env_overrides(config);
  EXPECT_TRUE(config.console_enabled);
  
  config.console_enabled = false;
  setenv("AXON_LOG_CONSOLE_ENABLED", "True", 1);
  apply_env_overrides(config);
  EXPECT_TRUE(config.console_enabled);
  
  // Test "1"
  config.console_enabled = false;
  setenv("AXON_LOG_CONSOLE_ENABLED", "1", 1);
  apply_env_overrides(config);
  EXPECT_TRUE(config.console_enabled);
  
  // Test "yes"
  config.console_enabled = false;
  setenv("AXON_LOG_CONSOLE_ENABLED", "yes", 1);
  apply_env_overrides(config);
  EXPECT_TRUE(config.console_enabled);
  
  config.console_enabled = false;
  setenv("AXON_LOG_CONSOLE_ENABLED", "YES", 1);
  apply_env_overrides(config);
  EXPECT_TRUE(config.console_enabled);
  
  // Test "on"
  config.console_enabled = false;
  setenv("AXON_LOG_CONSOLE_ENABLED", "on", 1);
  apply_env_overrides(config);
  EXPECT_TRUE(config.console_enabled);
  
  config.console_enabled = false;
  setenv("AXON_LOG_CONSOLE_ENABLED", "ON", 1);
  apply_env_overrides(config);
  EXPECT_TRUE(config.console_enabled);
}

TEST_F(LoggingConfigTest, ApplyEnvOverrides_BooleanFalse_Variations) {
  LoggingConfig config;
  
  // Test "false" (case variations)
  config.console_enabled = true;
  setenv("AXON_LOG_CONSOLE_ENABLED", "false", 1);
  apply_env_overrides(config);
  EXPECT_FALSE(config.console_enabled);
  
  config.console_enabled = true;
  setenv("AXON_LOG_CONSOLE_ENABLED", "FALSE", 1);
  apply_env_overrides(config);
  EXPECT_FALSE(config.console_enabled);
  
  // Test "0"
  config.console_enabled = true;
  setenv("AXON_LOG_CONSOLE_ENABLED", "0", 1);
  apply_env_overrides(config);
  EXPECT_FALSE(config.console_enabled);
  
  // Test "no"
  config.console_enabled = true;
  setenv("AXON_LOG_CONSOLE_ENABLED", "no", 1);
  apply_env_overrides(config);
  EXPECT_FALSE(config.console_enabled);
  
  config.console_enabled = true;
  setenv("AXON_LOG_CONSOLE_ENABLED", "NO", 1);
  apply_env_overrides(config);
  EXPECT_FALSE(config.console_enabled);
  
  // Test "off"
  config.console_enabled = true;
  setenv("AXON_LOG_CONSOLE_ENABLED", "off", 1);
  apply_env_overrides(config);
  EXPECT_FALSE(config.console_enabled);
  
  config.console_enabled = true;
  setenv("AXON_LOG_CONSOLE_ENABLED", "OFF", 1);
  apply_env_overrides(config);
  EXPECT_FALSE(config.console_enabled);
}

TEST_F(LoggingConfigTest, ApplyEnvOverrides_BooleanInvalid_UsesDefault) {
  LoggingConfig config;
  
  // Invalid boolean should use default value
  config.console_enabled = true;
  setenv("AXON_LOG_CONSOLE_ENABLED", "invalid_bool", 1);
  apply_env_overrides(config);
  // Should keep the default (true) when value is invalid
  EXPECT_TRUE(config.console_enabled);
  
  config.console_enabled = false;
  setenv("AXON_LOG_CONSOLE_ENABLED", "maybe", 1);
  apply_env_overrides(config);
  // Should keep the default (false) when value is invalid
  EXPECT_FALSE(config.console_enabled);
}

TEST_F(LoggingConfigTest, ApplyEnvOverrides_InvalidLevel_KeepsDefault) {
  LoggingConfig config;
  config.console_level = severity_level::info;
  config.file_level = severity_level::debug;
  
  // Invalid level string should not change the config
  setenv("AXON_LOG_LEVEL", "invalid_level", 1);
  apply_env_overrides(config);
  
  // Should keep original values
  EXPECT_EQ(config.console_level, severity_level::info);
  EXPECT_EQ(config.file_level, severity_level::debug);
}

TEST_F(LoggingConfigTest, ApplyEnvOverrides_EmptyEnvVar) {
  LoggingConfig config;
  config.console_level = severity_level::info;
  
  // Empty env var should not change config
  setenv("AXON_LOG_LEVEL", "", 1);
  apply_env_overrides(config);
  
  EXPECT_EQ(config.console_level, severity_level::info);
}

// ============================================================================
// Logging Initialization Tests
// ============================================================================

class LoggingInitTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Ensure clean state
    if (is_logging_initialized()) {
      shutdown_logging();
    }
  }

  void TearDown() override {
    if (is_logging_initialized()) {
      shutdown_logging();
    }
  }
};

TEST_F(LoggingInitTest, InitDefault) {
  EXPECT_FALSE(is_logging_initialized());
  
  init_logging_default();
  
  EXPECT_TRUE(is_logging_initialized());
}

TEST_F(LoggingInitTest, InitWithConfig) {
  LoggingConfig config;
  config.console_enabled = true;
  config.file_enabled = false;  // Disable file to avoid filesystem access
  config.console_level = severity_level::debug;
  
  EXPECT_FALSE(is_logging_initialized());
  
  init_logging(config);
  
  EXPECT_TRUE(is_logging_initialized());
}

TEST_F(LoggingInitTest, Shutdown) {
  init_logging_default();
  EXPECT_TRUE(is_logging_initialized());
  
  shutdown_logging();
  
  EXPECT_FALSE(is_logging_initialized());
}

TEST_F(LoggingInitTest, Reconfigure) {
  LoggingConfig config1;
  config1.console_enabled = true;
  config1.file_enabled = false;
  config1.console_level = severity_level::info;
  
  init_logging(config1);
  EXPECT_TRUE(is_logging_initialized());
  
  LoggingConfig config2;
  config2.console_enabled = true;
  config2.file_enabled = false;
  config2.console_level = severity_level::debug;
  
  reconfigure_logging(config2);
  
  EXPECT_TRUE(is_logging_initialized());
}

TEST_F(LoggingInitTest, FlushLogging) {
  LoggingConfig config;
  config.console_enabled = true;
  config.file_enabled = false;
  
  init_logging(config);
  
  // Should not throw
  EXPECT_NO_THROW(flush_logging());
}

TEST_F(LoggingInitTest, MultipleInitShutdownCycles) {
  for (int i = 0; i < 3; ++i) {
    EXPECT_FALSE(is_logging_initialized());
    
    init_logging_default();
    EXPECT_TRUE(is_logging_initialized());
    
    shutdown_logging();
    EXPECT_FALSE(is_logging_initialized());
  }
}

TEST_F(LoggingInitTest, DoubleInitIsIdempotent) {
  LoggingConfig config;
  config.console_enabled = true;
  config.file_enabled = false;
  
  EXPECT_FALSE(is_logging_initialized());
  
  init_logging(config);
  EXPECT_TRUE(is_logging_initialized());
  
  // Second init should be a no-op (already initialized)
  init_logging(config);
  EXPECT_TRUE(is_logging_initialized());
  
  // Should only need one shutdown
  shutdown_logging();
  EXPECT_FALSE(is_logging_initialized());
}

TEST_F(LoggingInitTest, ShutdownWhenNotInitialized) {
  EXPECT_FALSE(is_logging_initialized());
  
  // Shutdown when not initialized should be safe (no-op)
  EXPECT_NO_THROW(shutdown_logging());
  
  EXPECT_FALSE(is_logging_initialized());
}

TEST_F(LoggingInitTest, FlushWhenNotInitialized) {
  EXPECT_FALSE(is_logging_initialized());
  
  // Flush when not initialized should be safe
  EXPECT_NO_THROW(flush_logging());
}

TEST_F(LoggingInitTest, GetLogger) {
  init_logging_default();
  
  // get_logger should return a valid reference
  auto& logger = get_logger();
  
  // Should be able to use the logger (basic sanity check)
  BOOST_LOG_SEV(logger, severity_level::info) << "Test message from GetLogger test";
  
  flush_logging();
}

// ============================================================================
// Add/Remove Sink Tests
// ============================================================================

class SinkManagementTest : public ::testing::Test {
protected:
  void SetUp() override {
    if (is_logging_initialized()) {
      shutdown_logging();
    }
  }

  void TearDown() override {
    if (is_logging_initialized()) {
      shutdown_logging();
    }
  }
};

TEST_F(SinkManagementTest, AddAndRemoveConsoleSink) {
  auto sink = create_console_sink(severity_level::debug, true);
  ASSERT_NE(sink, nullptr);
  
  // Add sink
  EXPECT_NO_THROW(add_sink(sink));
  
  // Remove sink
  EXPECT_NO_THROW(remove_sink(sink));
}

TEST_F(SinkManagementTest, RemoveNonExistentSink) {
  auto sink = create_console_sink(severity_level::debug, true);
  ASSERT_NE(sink, nullptr);
  
  // Remove a sink that was never added - should not crash
  EXPECT_NO_THROW(remove_sink(sink));
}

TEST_F(SinkManagementTest, AddMultipleSinks) {
  auto sink1 = create_console_sink(severity_level::debug, true);
  auto sink2 = create_console_sink(severity_level::info, false);
  
  ASSERT_NE(sink1, nullptr);
  ASSERT_NE(sink2, nullptr);
  
  EXPECT_NO_THROW(add_sink(sink1));
  EXPECT_NO_THROW(add_sink(sink2));
  
  // Clean up
  EXPECT_NO_THROW(remove_sink(sink1));
  EXPECT_NO_THROW(remove_sink(sink2));
}

// ============================================================================
// Full Integration with File Sink
// ============================================================================

class LoggingFullIntegrationTest : public ::testing::Test {
protected:
  void SetUp() override {
    test_dir_ = fs::temp_directory_path() / ("log_init_int_" +
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

TEST_F(LoggingFullIntegrationTest, InitWithBothSinks) {
  LoggingConfig config;
  config.console_enabled = true;
  config.console_level = severity_level::debug;
  config.file_enabled = true;
  config.file_level = severity_level::debug;
  config.file_config.directory = test_dir_.string();
  config.file_config.format_json = true;
  
  init_logging(config);
  EXPECT_TRUE(is_logging_initialized());
  
  // Log messages through both sinks
  AXON_LOG_DEBUG("Integration test debug");
  AXON_LOG_INFO("Integration test info");
  AXON_LOG_WARN("Integration test warn");
  AXON_LOG_ERROR("Integration test error");
  
  flush_logging();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

TEST_F(LoggingFullIntegrationTest, InitConsoleOnlyThenAddFile) {
  // Start with console only
  LoggingConfig config1;
  config1.console_enabled = true;
  config1.file_enabled = false;
  
  init_logging(config1);
  EXPECT_TRUE(is_logging_initialized());
  
  AXON_LOG_INFO("Console only message");
  
  // Create and add file sink separately
  FileSinkConfig file_config;
  file_config.directory = test_dir_.string();
  file_config.format_json = true;
  
  auto file_sink = create_file_sink(file_config, severity_level::debug);
  ASSERT_NE(file_sink, nullptr);
  
  add_sink(file_sink);
  
  AXON_LOG_INFO("Message to both console and file");
  
  flush_logging();
  
  remove_sink(file_sink);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

