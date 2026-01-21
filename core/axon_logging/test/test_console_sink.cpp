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
 * @file test_console_sink.cpp
 * @brief Unit tests for console sink creation and configuration
 */

#include <boost/log/attributes/constant.hpp>
#include <boost/log/attributes/scoped_attribute.hpp>
#include <boost/log/core.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <gtest/gtest.h>

#include <chrono>
#include <sstream>
#include <thread>

#include "axon_console_sink.hpp"
#include "axon_log_init.hpp"
#include "axon_log_macros.hpp"

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

// ============================================================================
// Color Function Tests (exercise get_color and get_reset_color)
// ============================================================================

class ConsoleSinkColoredFormatterTest : public ::testing::Test {
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

TEST_F(ConsoleSinkColoredFormatterTest, ColoredSinkAllSeverityLevels) {
  // Create a colored console sink with debug level to capture all severities
  auto sink = create_console_sink(severity_level::debug, true);
  ASSERT_NE(sink, nullptr);

  add_sink(sink);
  boost::log::add_common_attributes();

  // Log at all severity levels to exercise all color code paths
  AXON_LOG_DEBUG("Test debug message");
  AXON_LOG_INFO("Test info message");
  AXON_LOG_WARN("Test warn message");
  AXON_LOG_ERROR("Test error message");
  AXON_LOG_FATAL("Test fatal message");

  // Flush to ensure messages are processed
  sink->flush();

  // Give async sink time to process
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  remove_sink(sink);
}

TEST_F(ConsoleSinkColoredFormatterTest, NonColoredSinkAllSeverityLevels) {
  // Create a non-colored console sink with debug level
  auto sink = create_console_sink(severity_level::debug, false);
  ASSERT_NE(sink, nullptr);

  add_sink(sink);
  boost::log::add_common_attributes();

  // Log at all severity levels to exercise non-colored formatter
  AXON_LOG_DEBUG("Test debug message no color");
  AXON_LOG_INFO("Test info message no color");
  AXON_LOG_WARN("Test warn message no color");
  AXON_LOG_ERROR("Test error message no color");
  AXON_LOG_FATAL("Test fatal message no color");

  sink->flush();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  remove_sink(sink);
}

// ============================================================================
// Context Attribute Tests (exercise TaskID and DeviceID extraction)
// ============================================================================

class ConsoleSinkContextTest : public ::testing::Test {
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

TEST_F(ConsoleSinkContextTest, ColoredSinkWithTaskIDOnly) {
  auto sink = create_console_sink(severity_level::debug, true);
  ASSERT_NE(sink, nullptr);

  add_sink(sink);
  boost::log::add_common_attributes();

  // Add only TaskID attribute
  BOOST_LOG_SCOPED_THREAD_ATTR("TaskID", boost::log::attributes::constant<std::string>("task_123"));

  AXON_LOG_INFO("Message with TaskID only");

  sink->flush();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  remove_sink(sink);
}

TEST_F(ConsoleSinkContextTest, ColoredSinkWithDeviceIDOnly) {
  auto sink = create_console_sink(severity_level::debug, true);
  ASSERT_NE(sink, nullptr);

  add_sink(sink);
  boost::log::add_common_attributes();

  // Add only DeviceID attribute
  BOOST_LOG_SCOPED_THREAD_ATTR(
    "DeviceID", boost::log::attributes::constant<std::string>("robot_001")
  );

  AXON_LOG_INFO("Message with DeviceID only");

  sink->flush();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  remove_sink(sink);
}

TEST_F(ConsoleSinkContextTest, ColoredSinkWithBothContextAttributes) {
  auto sink = create_console_sink(severity_level::debug, true);
  ASSERT_NE(sink, nullptr);

  add_sink(sink);
  boost::log::add_common_attributes();

  // Use separate attribute declarations to avoid macro collision
  BOOST_LOG_SCOPED_THREAD_ATTR("TaskID", boost::log::attributes::constant<std::string>("task_456"));
  BOOST_LOG_SCOPED_THREAD_ATTR(
    "DeviceID", boost::log::attributes::constant<std::string>("device_789")
  );

  AXON_LOG_INFO("Message with both TaskID and DeviceID");
  AXON_LOG_WARN("Warning with context");
  AXON_LOG_ERROR("Error with context");

  sink->flush();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  remove_sink(sink);
}

TEST_F(ConsoleSinkContextTest, NonColoredSinkWithTaskIDOnly) {
  auto sink = create_console_sink(severity_level::debug, false);
  ASSERT_NE(sink, nullptr);

  add_sink(sink);
  boost::log::add_common_attributes();

  BOOST_LOG_SCOPED_THREAD_ATTR("TaskID", boost::log::attributes::constant<std::string>("task_abc"));

  AXON_LOG_INFO("Non-colored message with TaskID only");

  sink->flush();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  remove_sink(sink);
}

TEST_F(ConsoleSinkContextTest, NonColoredSinkWithDeviceIDOnly) {
  auto sink = create_console_sink(severity_level::debug, false);
  ASSERT_NE(sink, nullptr);

  add_sink(sink);
  boost::log::add_common_attributes();

  BOOST_LOG_SCOPED_THREAD_ATTR(
    "DeviceID", boost::log::attributes::constant<std::string>("device_xyz")
  );

  AXON_LOG_INFO("Non-colored message with DeviceID only");

  sink->flush();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  remove_sink(sink);
}

TEST_F(ConsoleSinkContextTest, NonColoredSinkWithBothContextAttributes) {
  auto sink = create_console_sink(severity_level::debug, false);
  ASSERT_NE(sink, nullptr);

  add_sink(sink);
  boost::log::add_common_attributes();

  BOOST_LOG_SCOPED_THREAD_ATTR(
    "TaskID", boost::log::attributes::constant<std::string>("task_full")
  );
  BOOST_LOG_SCOPED_THREAD_ATTR(
    "DeviceID", boost::log::attributes::constant<std::string>("device_full")
  );

  AXON_LOG_INFO("Non-colored message with both attributes");

  sink->flush();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  remove_sink(sink);
}

TEST_F(ConsoleSinkContextTest, NoContextAttributes) {
  auto sink = create_console_sink(severity_level::debug, true);
  ASSERT_NE(sink, nullptr);

  add_sink(sink);
  boost::log::add_common_attributes();

  // No context attributes - exercises the "neither tid nor did" path
  AXON_LOG_INFO("Message without any context attributes");

  sink->flush();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  remove_sink(sink);
}

// ============================================================================
// Timestamp Path Tests
// ============================================================================

TEST_F(ConsoleSinkContextTest, TimestampExtraction) {
  auto sink = create_console_sink(severity_level::debug, true);
  ASSERT_NE(sink, nullptr);

  add_sink(sink);
  // Add common attributes which includes TimeStamp
  boost::log::add_common_attributes();

  AXON_LOG_INFO("Message to test timestamp extraction");

  sink->flush();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  remove_sink(sink);
}

// ============================================================================
// Missing Coverage Tests
// ============================================================================

TEST_F(ConsoleSinkColoredFormatterTest, NonColoredFormatterExecutes) {
  // Create non-colored sink and verify the formatter lambda actually executes
  auto sink = create_console_sink(severity_level::debug, false);
  ASSERT_NE(sink, nullptr);

  add_sink(sink);
  boost::log::add_common_attributes();

  // Log messages to trigger the non-colored formatter path
  // This exercises lines 79-107 in axon_console_sink.cpp
  AXON_LOG_DEBUG("Non-colored debug message");
  AXON_LOG_INFO("Non-colored info message");
  AXON_LOG_WARN("Non-colored warn message");
  AXON_LOG_ERROR("Non-colored error message");
  AXON_LOG_FATAL("Non-colored fatal message");

  // Test with context attributes to exercise all branches
  {
    BOOST_LOG_SCOPED_THREAD_ATTR(
      "TaskID", boost::log::attributes::constant<std::string>("task_nocolor")
    );
    AXON_LOG_INFO("Non-colored with TaskID");
  }

  {
    BOOST_LOG_SCOPED_THREAD_ATTR(
      "DeviceID", boost::log::attributes::constant<std::string>("device_nocolor")
    );
    AXON_LOG_INFO("Non-colored with DeviceID");
  }

  {
    BOOST_LOG_SCOPED_THREAD_ATTR(
      "TaskID", boost::log::attributes::constant<std::string>("task_both")
    );
    BOOST_LOG_SCOPED_THREAD_ATTR(
      "DeviceID", boost::log::attributes::constant<std::string>("device_both")
    );
    AXON_LOG_INFO("Non-colored with both attributes");
  }

  // Test without timestamp (edge case)
  sink->flush();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  remove_sink(sink);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
