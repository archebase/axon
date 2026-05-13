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

#include "action_registry.hpp"

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

bool diagnostics_contain(const nlohmann::json& diagnostics, const std::string& needle) {
  for (const auto& diagnostic : diagnostics) {
    if (diagnostic.value("message", std::string()).find(needle) != std::string::npos) {
      return true;
    }
  }
  return false;
}

}  // namespace

int main() {
  const auto root = make_temp_dir("axon_agent_action_registry");
  try {
    const auto manifest_dir = root / "actions.d";
    const auto command_dir = root / "opt/axon/actions";
    std::filesystem::create_directories(command_dir);

    const auto action_command = command_dir / "restart_sensor";
    write_file(action_command, "#!/bin/sh\nexit 0\n");

    write_file(
      manifest_dir / "01_restart_sensor.yaml",
      "id: restart_sensor\n"
      "title: Restart sensor\n"
      "description: Restart the robot sensor bridge\n"
      "command: " +
        action_command.string() +
        "\n"
        "args_schema:\n"
        "  type: object\n"
        "  required:\n"
        "    - sensor_id\n"
        "  properties:\n"
        "    sensor_id:\n"
        "      type: string\n"
        "timeout_sec: 120\n"
        "run_as: root\n"
        "allowed_roles:\n"
        "  - operator\n"
        "  - admin\n"
        "requires_approval: true\n"
    );

    write_file(
      manifest_dir / "02_duplicate.yaml",
      "id: restart_sensor\n"
      "title: Duplicate\n"
      "description: Duplicate action id\n"
      "command: " +
        action_command.string() +
        "\n"
        "args_schema:\n"
        "  type: object\n"
        "timeout_sec: 30\n"
        "run_as: root\n"
        "allowed_roles:\n"
        "  - admin\n"
        "requires_approval: false\n"
    );

    write_file(
      manifest_dir / "03_missing_title.yaml",
      "id: missing_title\n"
      "description: Missing title field\n"
      "command: " +
        action_command.string() +
        "\n"
        "args_schema:\n"
        "  type: object\n"
        "timeout_sec: 30\n"
        "run_as: root\n"
        "allowed_roles:\n"
        "  - admin\n"
        "requires_approval: false\n"
    );

    write_file(
      manifest_dir / "04_invalid_command.yaml",
      "id: invalid_command\n"
      "title: Invalid command\n"
      "description: Command leaves the allowlisted directory\n"
      "command: " +
        (root / "usr/bin/restart_sensor").string() +
        "\n"
        "args_schema:\n"
        "  type: object\n"
        "timeout_sec: 30\n"
        "run_as: root\n"
        "allowed_roles:\n"
        "  - admin\n"
        "requires_approval: false\n"
    );

    write_file(manifest_dir / "ignored.txt", "id: ignored\n");

    axon::agent::ActionRegistry registry(manifest_dir, command_dir);
    std::string error;
    require(registry.load(&error), "load failed: " + error);

    const auto catalog = registry.to_json();
    require(catalog["loaded_count"].get<std::size_t>() == 1, "expected one valid action");
    require(catalog["invalid_count"].get<std::size_t>() == 3, "expected three invalid manifests");

    const auto& action = catalog["actions"][0];
    require(action["id"].get<std::string>() == "restart_sensor", "action id mismatch");
    require(action["title"].get<std::string>() == "Restart sensor", "action title mismatch");
    require(action["command"].get<std::string>() == action_command.string(), "command mismatch");
    require(action["timeout_sec"].get<int>() == 120, "timeout_sec mismatch");
    require(action["run_as"].get<std::string>() == "root", "run_as mismatch");
    require(action["requires_approval"].get<bool>(), "requires_approval mismatch");
    require(action["allowed_roles"].size() == 2, "allowed_roles size mismatch");
    require(
      action["args_schema"]["properties"]["sensor_id"]["type"].get<std::string>() == "string",
      "args_schema was not preserved"
    );

    const auto& diagnostics = catalog["diagnostics"];
    require(
      diagnostics_contain(diagnostics, "duplicate action id"), "missing duplicate diagnostic"
    );
    require(diagnostics_contain(diagnostics, "missing field title"), "missing field diagnostic");
    require(
      diagnostics_contain(diagnostics, "approved action directory"),
      "missing command path diagnostic"
    );

    std::filesystem::remove_all(root);
    return 0;
  } catch (const std::exception& ex) {
    std::cerr << ex.what() << std::endl;
    std::filesystem::remove_all(root);
    return 1;
  }
}
