/**
 * @file test_console_sink.cpp
 * @brief Unit tests for console sink creation and configuration
 */

#include <gtest/gtest.h>

#include <boost/log/core.hpp>

#include "axon_console_sink.hpp"
#include "axon_log_init.hpp"

using namespace axon::logging;

// ============================================================================
// Console Sink Tests
// ============================================================================

class ConsoleSinkTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Ensure clean logging state
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

TEST_F(ConsoleSinkTest, CreateWithDefaultParams) {
  auto sink = create_console_sink();
  ASSERT_NE(sink, nullptr);
}

TEST_F(ConsoleSinkTest, CreateWithDebugLevel) {
  auto sink = create_console_sink(severity_level::debug, true);
  ASSERT_NE(sink, nullptr);
}

TEST_F(ConsoleSinkTest, CreateWithInfoLevel) {
  auto sink = create_console_sink(severity_level::info, true);
  ASSERT_NE(sink, nullptr);
}

TEST_F(ConsoleSinkTest, CreateWithWarnLevel) {
  auto sink = create_console_sink(severity_level::warn, true);
  ASSERT_NE(sink, nullptr);
}

TEST_F(ConsoleSinkTest, CreateWithErrorLevel) {
  auto sink = create_console_sink(severity_level::error, true);
  ASSERT_NE(sink, nullptr);
}

TEST_F(ConsoleSinkTest, CreateWithFatalLevel) {
  auto sink = create_console_sink(severity_level::fatal, true);
  ASSERT_NE(sink, nullptr);
}

TEST_F(ConsoleSinkTest, CreateWithoutColors) {
  auto sink = create_console_sink(severity_level::info, false);
  ASSERT_NE(sink, nullptr);
}

TEST_F(ConsoleSinkTest, AddSinkToCore) {
  auto sink = create_console_sink(severity_level::info, true);
  ASSERT_NE(sink, nullptr);
  
  // Should be able to add to logging core
  EXPECT_NO_THROW(add_sink(sink));
  
  // Clean up
  remove_sink(sink);
}

TEST_F(ConsoleSinkTest, MultipleSinks) {
  auto sink1 = create_console_sink(severity_level::debug, true);
  auto sink2 = create_console_sink(severity_level::error, false);
  
  ASSERT_NE(sink1, nullptr);
  ASSERT_NE(sink2, nullptr);
  
  // Both should be addable
  EXPECT_NO_THROW(add_sink(sink1));
  EXPECT_NO_THROW(add_sink(sink2));
  
  // Clean up
  remove_sink(sink1);
  remove_sink(sink2);
}

TEST_F(ConsoleSinkTest, SinkFlush) {
  auto sink = create_console_sink(severity_level::info, true);
  ASSERT_NE(sink, nullptr);
  
  add_sink(sink);
  
  // Flush should not throw
  EXPECT_NO_THROW(sink->flush());
  
  remove_sink(sink);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

