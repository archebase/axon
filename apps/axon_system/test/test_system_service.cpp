// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include <chrono>
#include <filesystem>
#include <iostream>
#include <optional>
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
    require(state.data["alerts"].contains("rules"), "alert rules missing");
    require(state.data["service"]["sample_cadence_ms"].contains("alerts"), "alert cadence missing");

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

    auto alerts = service.get_alerts();
    require(alerts.success, "alerts response failed");
    require(alerts.data.contains("firing_count"), "alerts firing count missing");

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

    axon::system::SystemServiceOptions bad_alert_options;
    bad_alert_options.state_dir = root / "bad_alert_state";
    bad_alert_options.resource_options.proc_root = root / "missing_proc";
    bad_alert_options.resource_options.disk_paths = {
      {"state_dir", bad_alert_options.state_dir},
    };
    bad_alert_options.process_options.proc_root = bad_alert_options.resource_options.proc_root;
    bad_alert_options.process_options.targets = {
      {"recorder", "axon-recorder", {}, {}, {}, std::nullopt},
    };
    bad_alert_options.alert_options.state_dir = bad_alert_options.state_dir;
    bad_alert_options.alert_options.sinks = {{"log", {}}};
    bad_alert_options.alert_options.rules = {
      {"bad_memory_available", "warning", "memory.available", "eq", 1.0, {}, "", "", 0},
    };

    axon::system::SystemService bad_alert_service(bad_alert_options);
    error.clear();
    require(
      bad_alert_service.initialize(&error), "bad alert initialize failed unexpectedly: " + error
    );
    alerts = bad_alert_service.get_alerts();
    require(alerts.success, "bad alert response failed");
    require(
      alerts.data["evaluation_available"].get<bool>() == false,
      "alert evaluation failure should be isolated"
    );
    require(
      !alerts.data["evaluation_error"].get<std::string>().empty(), "alert evaluation error missing"
    );
    health = bad_alert_service.get_health();
    require(health.success, "health should survive alert evaluation failure");
    bad_alert_service.request_shutdown();
    bad_alert_service.mark_stopped();

    std::filesystem::remove_all(root);
    return 0;
  } catch (const std::exception& ex) {
    std::cerr << ex.what() << std::endl;
    std::filesystem::remove_all(root);
    return 1;
  }
}
