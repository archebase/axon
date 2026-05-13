// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unistd.h>

#include "system_config.hpp"

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

void write_file(const std::filesystem::path& path, const std::string& content) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("failed to write " + path.string());
  }
  output << content;
}

}  // namespace

int main() {
  const auto root = make_temp_dir("axon_system_config");
  try {
    const auto config_path = root / "system.yaml";
    write_file(
      config_path,
      "server:\n"
      "  host: 0.0.0.0\n"
      "  port: 18091\n"
      "state_dir: " +
        (root / "state").string() +
        "\n"
        "sampling:\n"
        "  resources_ms: 1500\n"
        "  processes_ms: 2500\n"
        "  alerts_ms: 3500\n"
        "  disk_ms: 7000\n"
        "disk_paths:\n"
        "  - id: state_dir\n"
        "    path: " +
        (root / "state").string() +
        "\n"
        "  - id: data\n"
        "    path: " +
        (root / "data").string() +
        "\n"
        "network:\n"
        "  interfaces:\n"
        "    - eth0\n"
        "    - wlan0\n"
        "monitored_processes:\n"
        "  - id: recorder\n"
        "    executable: axon-recorder\n"
        "    pid_file_candidates:\n"
        "      - " +
        (root / "agent" / "*_recorder.pid").string() +
        "\n"
        "    rpc:\n"
        "      type: http\n"
        "      url: http://127.0.0.1:8080/rpc/state\n"
        "      timeout_ms: 250\n"
        "  - id: transfer\n"
        "    executable: axon-transfer\n"
        "alerts:\n"
        "  evaluate_interval_ms: 4500\n"
        "  sinks:\n"
        "    - type: file\n"
        "      path: " +
        (root / "alerts.jsonl").string() +
        "\n"
        "  rules:\n"
        "    - id: recorder_unavailable\n"
        "      process_id: recorder\n"
        "      status: unavailable\n"
        "      for_sec: 10\n"
        "      severity: critical\n"
    );

    auto config = axon::system::default_system_config();
    std::string error;
    require(axon::system::load_system_config(config_path, &config, &error), error);
    require(config.host == "0.0.0.0", "host mismatch");
    require(config.port == 18091, "port mismatch");
    require(config.state_dir == root / "state", "state_dir mismatch");
    require(config.resource_options.resource_sample_cadence_ms == 1500, "resource cadence");
    require(config.resource_options.disk_sample_cadence_ms == 7000, "disk cadence");
    require(config.resource_options.disk_paths.size() == 2, "disk path count");
    require(config.resource_options.disk_paths.at(1).id == "data", "disk path id");
    require(config.resource_options.network_interfaces.size() == 2, "network interface count");
    require(config.resource_options.network_interfaces.at(1) == "wlan0", "network interface");
    require(config.process_options.process_sample_cadence_ms == 2500, "process cadence");
    require(config.process_options.targets.size() == 2, "process target count");
    require(config.process_options.targets.at(0).id == "recorder", "process target id");
    require(config.process_options.targets.at(0).rpc.has_value(), "process target rpc");
    require(config.process_options.targets.at(0).rpc->timeout_ms == 250, "process rpc timeout");
    require(config.alert_options.evaluate_interval_ms == 4500, "alert cadence");
    require(config.alert_options.sinks.size() == 1, "alert sink count");
    require(config.alert_options.rules.size() == 1, "alert rule count");
    require(config.alert_options.rules.at(0).severity == "critical", "alert severity");

    write_file(config_path, "server:\n  port: 70000\n");
    config = axon::system::default_system_config();
    require(!axon::system::load_system_config(config_path, &config, &error), "bad port accepted");
    require(!error.empty(), "bad port error missing");

    std::filesystem::remove_all(root);
    return 0;
  } catch (const std::exception& ex) {
    std::cerr << ex.what() << std::endl;
    std::filesystem::remove_all(root);
    return 1;
  }
}
