// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include <axon/agent/robot_adapter.hpp>

#include <filesystem>
#include <string>

namespace {

constexpr const char* kStartupScript = "/home/nav01/start_teleop.sh";

bool is_executable(const std::filesystem::path& path) {
  namespace fs = std::filesystem;
  if (!fs::exists(path) || !fs::is_regular_file(path)) {
    return false;
  }

  const auto permissions = fs::status(path).permissions();
  return (permissions & fs::perms::owner_exec) != fs::perms::none
    || (permissions & fs::perms::group_exec) != fs::perms::none
    || (permissions & fs::perms::others_exec) != fs::perms::none;
}

class Nav01TeleopAdapter final : public axon::agent::RobotAdapter {
public:
  axon::agent::RobotAdapterResult start(const axon::agent::RobotAdapterContext& context) override {
    (void)context;
    if (!is_executable(kStartupScript)) {
      return axon::agent::RobotAdapterResult::fail(std::string("startup script is not executable: ") + kStartupScript);
    }
    return axon::agent::RobotAdapterResult::ok("nav01 teleop startup script is ready");
  }

  axon::agent::RobotAdapterResult stop(const axon::agent::RobotAdapterContext& context, int timeout_ms) override {
    (void)context;
    (void)timeout_ms;
    return axon::agent::RobotAdapterResult::ok("nav01 teleop stop delegated to ProcessManager");
  }

  axon::agent::RobotAdapterResult force_stop(const axon::agent::RobotAdapterContext& context) override {
    (void)context;
    return axon::agent::RobotAdapterResult::ok("nav01 teleop force stop delegated to ProcessManager");
  }

  axon::agent::RobotAdapterStatus status(const axon::agent::RobotAdapterContext& context) override {
    (void)context;
    axon::agent::RobotAdapterStatus status;
    status.reachable = is_executable(kStartupScript);
    status.state = status.reachable ? "ready" : "missing_startup_script";
    status.message = status.reachable
      ? std::string("startup script is executable: ") + kStartupScript
      : std::string("startup script is not executable: ") + kStartupScript;
    return status;
  }
};

}  // namespace

extern "C" const axon::agent::RobotAdapterDescriptor* axon_agent_get_adapter_descriptor() {
  static const axon::agent::RobotAdapterDescriptor descriptor{
    axon::agent::kRobotAdapterAbiVersion,
    "nav01_teleop",
    "nav01",
    "0.1.0",
    "ArcheBase",
  };
  return &descriptor;
}

extern "C" axon::agent::RobotAdapter* axon_agent_create_adapter() {
  return new Nav01TeleopAdapter();
}

extern "C" void axon_agent_destroy_adapter(axon::agent::RobotAdapter* adapter) {
  delete adapter;
}
