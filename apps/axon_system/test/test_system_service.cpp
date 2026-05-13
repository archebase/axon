// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include <chrono>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unistd.h>

#include "system_service.hpp"

namespace {

std::filesystem::path make_temp_dir(const std::string& name) {
  const auto base = std::filesystem::temp_directory_path() /
                    (name + "_" + std::to_string(getpid()) + "_" +
                     std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::create_directories(base);
  return base;
}

void require(bool condition, const std::string& message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

}  // namespace

int main() {
  const auto root = make_temp_dir("axon_system_service");
  try {
    axon::system::SystemService service(root);
    std::string error;
    require(service.initialize(&error), "initialize failed: " + error);

    auto state = service.get_state();
    require(state.success, "state response failed");
    require(state.message == "ok", "state message mismatch");
    require(state.data["service"]["name"].get<std::string>() == "axon-system", "name mismatch");
    require(state.data["service"]["state"].get<std::string>() == "running", "state mismatch");
    require(state.data["service"]["pid"].get<int>() == static_cast<int>(getpid()), "pid mismatch");
    require(
      state.data["service"]["state_dir"].get<std::string>() == root.lexically_normal().string(),
      "state_dir mismatch"
    );
    require(
      state.data["service"]["shutdown_requested"].get<bool>() == false,
      "shutdown flag should start false"
    );
    require(state.data["service"].contains("sample_cadence_ms"), "sample cadence missing");
    require(
      state.data["service"]["sample_cadence_ms"].contains("processes"), "process cadence missing"
    );
    require(state.data.contains("resources"), "resources missing");
    require(state.data["resources"].contains("cpu"), "cpu metrics missing");
    require(state.data["resources"].contains("memory"), "memory metrics missing");
    require(state.data["resources"].contains("disk"), "disk metrics missing");
    require(state.data["resources"].contains("network"), "network metrics missing");
    require(state.data.contains("processes"), "processes placeholder missing");
    require(state.data["processes"].contains("recorder"), "recorder process missing");
    require(state.data["processes"].contains("transfer"), "transfer process missing");
    require(state.data.contains("alerts"), "alerts placeholder missing");

    auto metrics = service.get_metrics();
    require(metrics.success, "metrics response failed");
    require(metrics.data.contains("sample_cadence_ms"), "metrics cadence missing");
    require(
      metrics.data["sample_cadence_ms"]["unit"].get<std::string>() == "milliseconds", "cadence unit"
    );
    require(metrics.data.contains("processes"), "metrics processes missing");

    auto processes = service.get_processes();
    require(processes.success, "processes response failed");
    require(processes.data.contains("recorder"), "processes recorder missing");

    auto health = service.get_health();
    require(health.success, "health response should be successful");
    require(health.data["state"].get<std::string>() == "running", "health state mismatch");

    auto shutdown = service.request_shutdown();
    require(shutdown.success, "shutdown response failed");
    require(service.shutdown_requested(), "shutdown flag not set");
    require(
      shutdown.data["service"]["state"].get<std::string>() == "stopping", "shutdown state mismatch"
    );

    service.mark_stopped();
    state = service.get_state();
    require(state.data["service"]["state"].get<std::string>() == "stopped", "stopped mismatch");

    std::filesystem::remove_all(root);
    return 0;
  } catch (const std::exception& ex) {
    std::cerr << ex.what() << std::endl;
    std::filesystem::remove_all(root);
    return 1;
  }
}
