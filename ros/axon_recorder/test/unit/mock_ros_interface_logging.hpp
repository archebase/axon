/**
 * @file mock_ros_interface_logging.hpp
 * @brief Minimal mock for RosInterface logging methods
 *
 * This is a lightweight mock that only implements the logging methods needed
 * to test axon_ros_sink.cpp. Other RosInterface methods are stubbed.
 */

#ifndef MOCK_ROS_INTERFACE_LOGGING_HPP
#define MOCK_ROS_INTERFACE_LOGGING_HPP

#include "ros_interface.hpp"

#include <string>
#include <vector>

namespace axon {
namespace recorder {
namespace testing {

/**
 * Minimal mock RosInterface for testing log routing.
 * Captures all logged messages for verification.
 */
class MockRosInterfaceLogging : public RosInterface {
public:
  // Captured log messages by level
  std::vector<std::string> info_logs;
  std::vector<std::string> warn_logs;
  std::vector<std::string> error_logs;
  std::vector<std::string> debug_logs;

  // =========================================================================
  // Logging methods (the ones we're actually testing)
  // =========================================================================

  void log_info(const std::string& msg) const override {
    const_cast<MockRosInterfaceLogging*>(this)->info_logs.push_back(msg);
  }

  void log_warn(const std::string& msg) const override {
    const_cast<MockRosInterfaceLogging*>(this)->warn_logs.push_back(msg);
  }

  void log_error(const std::string& msg) const override {
    const_cast<MockRosInterfaceLogging*>(this)->error_logs.push_back(msg);
  }

  void log_debug(const std::string& msg) const override {
    const_cast<MockRosInterfaceLogging*>(this)->debug_logs.push_back(msg);
  }

  // =========================================================================
  // Stub methods (not used by ros_sink)
  // =========================================================================

  bool init(int, char**, const std::string&) override {
    return true;
  }

  void shutdown() override {}

  bool ok() const override {
    return true;
  }

  void* get_node_handle() override {
    return nullptr;
  }

  void* subscribe(
    const std::string&, const std::string&,
    std::function<void(const void*)>
  ) override {
    return nullptr;
  }

  void* subscribe_zero_copy(
    const std::string&, const std::string&,
    std::function<void(SerializedMessageData&&)>,
    const SubscriptionConfig&
  ) override {
    return nullptr;
  }

  void unsubscribe(void*) override {}

  void* advertise_service(
    const std::string&, const std::string&,
    std::function<bool(const void*, void*)>
  ) override {
    return nullptr;
  }

  void spin_once() override {}
  void spin() override {}

  int64_t now_nsec() const override {
    return 0;
  }

  std::string get_message_definition(const std::string&) const override {
    return "";
  }

  // =========================================================================
  // Test helpers
  // =========================================================================

  void clear_logs() {
    info_logs.clear();
    warn_logs.clear();
    error_logs.clear();
    debug_logs.clear();
  }

  size_t total_logs() const {
    return info_logs.size() + warn_logs.size() +
           error_logs.size() + debug_logs.size();
  }
};

}  // namespace testing
}  // namespace recorder
}  // namespace axon

#endif  // MOCK_ROS_INTERFACE_LOGGING_HPP

