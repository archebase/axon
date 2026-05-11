// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_AGENT_ROBOT_ADAPTER_HPP
#define AXON_AGENT_ROBOT_ADAPTER_HPP

#include <cstdint>
#include <string>
#include <utility>

namespace axon {
namespace agent {

constexpr std::int32_t kRobotAdapterAbiVersion = 1;
constexpr const char* kRobotAdapterDescriptorSymbol = "axon_agent_get_adapter_descriptor";
constexpr const char* kRobotAdapterCreateSymbol = "axon_agent_create_adapter";
constexpr const char* kRobotAdapterDestroySymbol = "axon_agent_destroy_adapter";

struct RobotAdapterDescriptor {
  std::int32_t abi_version = kRobotAdapterAbiVersion;
  const char* adapter_id = "";
  const char* robot_model = "";
  const char* adapter_version = "";
  const char* vendor = "";
};

struct RobotAdapterContext {
  std::string profile_id;
  std::string adapter_id;
  std::string robot_model;
  std::string profile_root;
  std::string adapter_yaml;
  std::string state_dir;
};

struct RobotAdapterResult {
  bool success = false;
  std::string message;

  static RobotAdapterResult ok(std::string result_message = "ok") {
    return {true, std::move(result_message)};
  }

  static RobotAdapterResult fail(std::string result_message) {
    return {false, std::move(result_message)};
  }
};

struct RobotAdapterStatus {
  bool reachable = false;
  std::string state = "unknown";
  std::string message;
  std::string log_path;
};

class RobotAdapter {
public:
  virtual ~RobotAdapter() = default;

  // Called before axon-agent starts the configured robot_startup process.
  virtual RobotAdapterResult start(const RobotAdapterContext& context) = 0;

  // Called before axon-agent gracefully stops the configured robot_startup process.
  virtual RobotAdapterResult stop(const RobotAdapterContext& context, int timeout_ms) = 0;

  // Called when axon-agent force-stops robot_startup.
  virtual RobotAdapterResult force_stop(const RobotAdapterContext& context) = 0;

  // Called by axon-agent status/report paths for adapter-specific status.
  virtual RobotAdapterStatus status(const RobotAdapterContext& context) = 0;
};

using GetRobotAdapterDescriptorFn = const RobotAdapterDescriptor* (*)();
using CreateRobotAdapterFn = RobotAdapter* (*)();
using DestroyRobotAdapterFn = void (*)(RobotAdapter*);

}  // namespace agent
}  // namespace axon

#endif  // AXON_AGENT_ROBOT_ADAPTER_HPP
