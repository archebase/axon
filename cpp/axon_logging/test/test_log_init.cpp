/**
 * @file test_log_init.cpp
 * @brief Unit tests for logging initialization and configuration
 */

#include <gtest/gtest.h>

#include <cstdlib>
#include <string>

#include "axon_log_init.hpp"
#include "axon_log_severity.hpp"

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
}

TEST_F(SeverityLevelTest, ParseInvalidLevels) {
  auto invalid = parse_severity_level("invalid");
  EXPECT_FALSE(invalid.has_value());

  auto empty = parse_severity_level("");
  EXPECT_FALSE(empty.has_value());

  auto numeric = parse_severity_level("123");
  EXPECT_FALSE(numeric.has_value());
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

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

