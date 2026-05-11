// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include <axon/agent/robot_adapter.hpp>

#include <atomic>
#include <string>

namespace {

class DemoRobotAdapter final : public axon::agent::RobotAdapter {
public:
  axon::agent::RobotAdapterResult start(const axon::agent::RobotAdapterContext& context) override {
    running_.store(true);
    return axon::agent::RobotAdapterResult::ok("demo adapter prepared profile " + context.profile_id);
  }

  axon::agent::RobotAdapterResult stop(const axon::agent::RobotAdapterContext& context, int timeout_ms) override {
    (void)timeout_ms;
    running_.store(false);
    return axon::agent::RobotAdapterResult::ok("demo adapter stopped profile " + context.profile_id);
  }

  axon::agent::RobotAdapterResult force_stop(const axon::agent::RobotAdapterContext& context) override {
    running_.store(false);
    return axon::agent::RobotAdapterResult::ok("demo adapter force stopped profile " + context.profile_id);
  }

  axon::agent::RobotAdapterStatus status(const axon::agent::RobotAdapterContext& context) override {
    axon::agent::RobotAdapterStatus status;
    status.reachable = true;
    status.state = running_.load() ? "prepared" : "idle";
    status.message = "demo adapter for " + context.robot_model;
    return status;
  }

private:
  std::atomic<bool> running_{false};
};

}  // namespace

extern "C" const axon::agent::RobotAdapterDescriptor* axon_agent_get_adapter_descriptor() {
  static const axon::agent::RobotAdapterDescriptor descriptor{
    axon::agent::kRobotAdapterAbiVersion,
    "demo_robot",
    "demo",
    "0.1.0",
    "ArcheBase",
  };
  return &descriptor;
}

extern "C" axon::agent::RobotAdapter* axon_agent_create_adapter() {
  return new DemoRobotAdapter();
}

extern "C" void axon_agent_destroy_adapter(axon::agent::RobotAdapter* adapter) {
  delete adapter;
}
