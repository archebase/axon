/**
 * @file test_ros_sink.cpp
 * @brief Unit tests for axon_ros_sink.cpp
 *
 * Tests the Boost.Log to ROS logging bridge.
 */

#include <gtest/gtest.h>

#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/attributes/constant.hpp>

#include "logging/axon_ros_sink.hpp"
#include "mock_ros_interface_logging.hpp"

using namespace axon::logging;
using namespace axon::recorder::testing;

// ============================================================================
// Test Fixture
// ============================================================================

class RosSinkTest : public ::testing::Test {
protected:
  MockRosInterfaceLogging mock_;
  boost::shared_ptr<ros_sink_t> sink_;

  void SetUp() override {
    // Add common attributes (needed for Boost.Log)
    boost::log::add_common_attributes();
    mock_.clear_logs();
  }

  void TearDown() override {
    // Remove sink from core if it was added
    if (sink_) {
      boost::log::core::get()->remove_sink(sink_);
      sink_.reset();
    }
  }

  void add_sink(severity_level min_level = severity_level::debug) {
    sink_ = create_ros_sink(&mock_, min_level);
    boost::log::core::get()->add_sink(sink_);
  }

  // Helper to log with a specific severity using raw Boost.Log
  void log_message(severity_level level, const std::string& message) {
    boost::log::sources::severity_logger<severity_level> slg;
    BOOST_LOG_SEV(slg, level) << message;
  }
};

// ============================================================================
// Creation Tests
// ============================================================================

TEST_F(RosSinkTest, CreateRosSink_ReturnsNonNull) {
  auto sink = create_ros_sink(&mock_, severity_level::info);
  EXPECT_NE(sink, nullptr);
}

TEST_F(RosSinkTest, CreateRosSink_WithDifferentLevels) {
  auto sink_debug = create_ros_sink(&mock_, severity_level::debug);
  auto sink_info = create_ros_sink(&mock_, severity_level::info);
  auto sink_warn = create_ros_sink(&mock_, severity_level::warn);
  auto sink_error = create_ros_sink(&mock_, severity_level::error);

  EXPECT_NE(sink_debug, nullptr);
  EXPECT_NE(sink_info, nullptr);
  EXPECT_NE(sink_warn, nullptr);
  EXPECT_NE(sink_error, nullptr);
}

// ============================================================================
// Log Routing Tests
// ============================================================================

TEST_F(RosSinkTest, InfoLog_GoesToInfoMethod) {
  add_sink(severity_level::debug);
  log_message(severity_level::info, "Test info message");

  // Give the synchronous sink time to process
  EXPECT_GE(mock_.info_logs.size(), 1u);
  if (!mock_.info_logs.empty()) {
    EXPECT_NE(mock_.info_logs.back().find("Test info message"), std::string::npos);
  }
}

TEST_F(RosSinkTest, WarnLog_GoesToWarnMethod) {
  add_sink(severity_level::debug);
  log_message(severity_level::warn, "Test warning message");

  EXPECT_GE(mock_.warn_logs.size(), 1u);
  if (!mock_.warn_logs.empty()) {
    EXPECT_NE(mock_.warn_logs.back().find("Test warning message"), std::string::npos);
  }
}

TEST_F(RosSinkTest, ErrorLog_GoesToErrorMethod) {
  add_sink(severity_level::debug);
  log_message(severity_level::error, "Test error message");

  EXPECT_GE(mock_.error_logs.size(), 1u);
  if (!mock_.error_logs.empty()) {
    EXPECT_NE(mock_.error_logs.back().find("Test error message"), std::string::npos);
  }
}

TEST_F(RosSinkTest, DebugLog_GoesToDebugMethod) {
  add_sink(severity_level::debug);
  log_message(severity_level::debug, "Test debug message");

  EXPECT_GE(mock_.debug_logs.size(), 1u);
  if (!mock_.debug_logs.empty()) {
    EXPECT_NE(mock_.debug_logs.back().find("Test debug message"), std::string::npos);
  }
}

TEST_F(RosSinkTest, FatalLog_GoesToErrorMethod) {
  add_sink(severity_level::debug);
  log_message(severity_level::fatal, "Test fatal message");

  // Fatal logs should go to error in ROS
  EXPECT_GE(mock_.error_logs.size(), 1u);
  if (!mock_.error_logs.empty()) {
    EXPECT_NE(mock_.error_logs.back().find("Test fatal message"), std::string::npos);
  }
}

// ============================================================================
// Filter Tests
// ============================================================================

TEST_F(RosSinkTest, Filter_InfoLevel_BlocksDebug) {
  add_sink(severity_level::info);  // Min level = info

  log_message(severity_level::debug, "Debug message should be blocked");
  log_message(severity_level::info, "Info message should pass");

  EXPECT_EQ(mock_.debug_logs.size(), 0u);  // Debug blocked
  EXPECT_GE(mock_.info_logs.size(), 1u);   // Info passes
}

TEST_F(RosSinkTest, Filter_WarnLevel_BlocksInfoAndDebug) {
  add_sink(severity_level::warn);  // Min level = warn

  log_message(severity_level::debug, "Debug blocked");
  log_message(severity_level::info, "Info blocked");
  log_message(severity_level::warn, "Warn passes");

  EXPECT_EQ(mock_.debug_logs.size(), 0u);
  EXPECT_EQ(mock_.info_logs.size(), 0u);
  EXPECT_GE(mock_.warn_logs.size(), 1u);
}

TEST_F(RosSinkTest, Filter_ErrorLevel_OnlyErrorAndFatal) {
  add_sink(severity_level::error);  // Min level = error

  log_message(severity_level::debug, "Debug blocked");
  log_message(severity_level::info, "Info blocked");
  log_message(severity_level::warn, "Warn blocked");
  log_message(severity_level::error, "Error passes");
  log_message(severity_level::fatal, "Fatal passes");

  EXPECT_EQ(mock_.debug_logs.size(), 0u);
  EXPECT_EQ(mock_.info_logs.size(), 0u);
  EXPECT_EQ(mock_.warn_logs.size(), 0u);
  EXPECT_GE(mock_.error_logs.size(), 2u);  // error + fatal both go to error
}

TEST_F(RosSinkTest, Filter_DebugLevel_AllowsAll) {
  add_sink(severity_level::debug);  // Min level = debug

  log_message(severity_level::debug, "Debug");
  log_message(severity_level::info, "Info");
  log_message(severity_level::warn, "Warn");
  log_message(severity_level::error, "Error");

  EXPECT_GE(mock_.debug_logs.size(), 1u);
  EXPECT_GE(mock_.info_logs.size(), 1u);
  EXPECT_GE(mock_.warn_logs.size(), 1u);
  EXPECT_GE(mock_.error_logs.size(), 1u);
}

// ============================================================================
// Multiple Messages Tests
// ============================================================================

TEST_F(RosSinkTest, MultipleMessages_AllCaptured) {
  add_sink(severity_level::debug);

  for (int i = 0; i < 5; ++i) {
    log_message(severity_level::info, "Message " + std::to_string(i));
  }

  EXPECT_GE(mock_.info_logs.size(), 5u);
}

// ============================================================================
// Context Attribute Tests
// ============================================================================

TEST_F(RosSinkTest, ContextAttributes_IncludedInOutput) {
  add_sink(severity_level::debug);

  // Create logger with context attributes
  boost::log::sources::severity_logger<severity_level> slg;
  slg.add_attribute("TaskID", boost::log::attributes::constant<std::string>("task-123"));
  slg.add_attribute("DeviceID", boost::log::attributes::constant<std::string>("device-456"));

  BOOST_LOG_SEV(slg, severity_level::info) << "Context test";

  EXPECT_GE(mock_.info_logs.size(), 1u);
  if (!mock_.info_logs.empty()) {
    const auto& msg = mock_.info_logs.back();
    // The formatter includes context attributes
    EXPECT_NE(msg.find("task_id=task-123"), std::string::npos);
    EXPECT_NE(msg.find("device_id=device-456"), std::string::npos);
  }
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST_F(RosSinkTest, EmptyMessage_Handled) {
  add_sink(severity_level::debug);
  log_message(severity_level::info, "");

  // Should not crash, message is logged (even if empty)
  EXPECT_GE(mock_.total_logs(), 1u);
}

TEST_F(RosSinkTest, LongMessage_Handled) {
  add_sink(severity_level::debug);
  std::string long_msg(10000, 'x');
  log_message(severity_level::info, long_msg);

  EXPECT_GE(mock_.info_logs.size(), 1u);
}

TEST_F(RosSinkTest, SpecialCharacters_Handled) {
  add_sink(severity_level::debug);
  log_message(severity_level::info, "Special chars: \t\n\r\\\"'");

  EXPECT_GE(mock_.info_logs.size(), 1u);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

