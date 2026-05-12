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

#include "agent_service.hpp"

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
  const auto root = make_temp_dir("axon_agent_service");
  try {
    const auto profile_root = root / "profiles";
    const auto profile_dir = profile_root / "demo_robot";
    const auto state_dir = root / "state";

    write_file(
      profile_dir / "adapter.yaml",
      R"YAML(adapter_id: demo_robot
robot_model: demo
abi_version: 1
entry_symbol: axon_agent_create_adapter
auto_start: false
managed_processes:
  robot_startup:
    executable: /bin/sleep
    args:
      - "30"
    working_directory: runtime
    pid_file: custom/robot.pid
    metadata_file: custom/robot.json
    stdout_log: custom/robot.stdout.log
    stderr_log: custom/robot.stderr.log
    stop_timeout_sec: 7
    fingerprint: robot-fingerprint
    health_check:
      type: none
      timeout_ms: 123
)YAML"
    );
    write_file(
      profile_dir / "recorder.yaml",
      R"YAML(process:
  executable: axon-recorder
  args:
    - --config
    - recorder.yaml
  working_directory: .
  pid_file: custom/recorder.pid
  metadata_file: custom/recorder.json
  stdout_log: custom/recorder.stdout.log
  stderr_log: custom/recorder.stderr.log
  stop_timeout_sec: 11
  fingerprint: recorder-fingerprint
  health_check:
    type: process
    timeout_ms: 250
)YAML"
    );
    write_file(
      profile_dir / "transfer.yaml",
      R"YAML(process:
  executable: axon-transfer
  args:
    - --config
    - transfer.yaml
)YAML"
    );

    axon::agent::AgentService service(profile_root, state_dir);
    std::string error;
    require(service.initialize(&error), "initialize failed: " + error);

    auto response = service.select_profile({{"profile_id", "demo_robot"}});
    require(response.success, "select profile failed: " + response.message);

    const auto& processes = response.data["processes"];
    const auto& recorder = processes["recorder"];
    require(
      recorder["pid_file"].get<std::string>() == (state_dir / "custom/recorder.pid").string(),
      "recorder pid_file mismatch"
    );
    require(
      recorder["metadata_file"].get<std::string>() == (state_dir / "custom/recorder.json").string(),
      "recorder metadata_file mismatch"
    );
    require(
      recorder["stdout_log"].get<std::string>() ==
        (state_dir / "custom/recorder.stdout.log").string(),
      "recorder stdout_log mismatch"
    );
    require(recorder["stop_timeout_sec"].get<int>() == 11, "recorder stop_timeout_sec mismatch");
    require(
      recorder["fingerprint"].get<std::string>() == "recorder-fingerprint",
      "recorder fingerprint mismatch"
    );
    require(
      recorder["health_check"]["type"].get<std::string>() == "process",
      "recorder health type mismatch"
    );
    require(
      recorder["health_check"]["timeout_ms"].get<int>() == 250, "recorder health timeout mismatch"
    );

    const auto& robot = processes["robot_startup"];
    require(
      robot["working_directory"].get<std::string>() == (profile_dir / "runtime").string(),
      "robot cwd mismatch"
    );
    require(
      robot["pid_file"].get<std::string>() == (state_dir / "custom/robot.pid").string(),
      "robot pid_file mismatch"
    );
    require(robot["stop_timeout_sec"].get<int>() == 7, "robot stop_timeout_sec mismatch");
    require(
      robot["health_check"]["enabled"].get<bool>() == false, "robot health should be disabled"
    );
    require(
      robot["health"]["status"].get<std::string>() == "disabled", "robot health status mismatch"
    );

    std::filesystem::remove_all(root);
    return 0;
  } catch (const std::exception& ex) {
    std::cerr << ex.what() << std::endl;
    std::filesystem::remove_all(root);
    return 1;
  }
}
