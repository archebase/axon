// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include <sys/stat.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unistd.h>

#include "action_executor.hpp"

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

void write_executable(const std::filesystem::path& path, const std::string& content) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("failed to write " + path.string());
  }
  output << content;
  output.close();
  chmod(path.c_str(), 0755);
}

axon::agent::ActionDefinition make_action(const std::filesystem::path& command) {
  axon::agent::ActionDefinition action;
  action.id = "restart_sensor";
  action.title = "Restart sensor";
  action.description = "Restart a sensor process";
  action.command = command;
  action.args_schema = {
    {"type", "object"},
    {"required", nlohmann::json::array({"sensor_id"})},
    {"additionalProperties", false},
    {"properties",
     {
       {"sensor_id", {{"type", "string"}}},
       {"attempt", {{"type", "integer"}}},
     }},
  };
  action.timeout_sec = 2;
  action.run_as = "current";
  action.allowed_roles = {"operator"};
  return action;
}

}  // namespace

int main() {
  const auto root = make_temp_dir("axon_agent_action_executor");
  try {
    const auto command = root / "actions" / "restart_sensor";
    write_executable(
      command,
      "#!/bin/sh\n"
      "echo action=$AXON_ACTION_ID\n"
      "cat\n"
      "printf '\\n'\n"
      "echo token=super-secret\n"
    );

    axon::agent::ActionExecutionOptions options;
    options.max_output_bytes = 128;
    axon::agent::ActionExecutor executor(options);
    auto action = make_action(command);

    auto result = executor.execute(action, {{"sensor_id", "front_lidar"}, {"attempt", 2}});
    require(result.status == "succeeded", "action should succeed: " + result.to_json().dump());
    require(result.exit_code == 0, "exit code");
    require(result.stdout_text.find("front_lidar") != std::string::npos, "args passed on stdin");
    require(result.stdout_text.find("super-secret") == std::string::npos, "secret not redacted");
    require(result.stdout_text.find("[REDACTED]") != std::string::npos, "redaction marker missing");

    result = executor.execute(action, {{"attempt", 2}});
    require(result.status == "rejected", "missing required arg rejected");
    require(result.error_summary.find("sensor_id") != std::string::npos, "missing arg diagnostic");

    result = executor.execute(action, {{"sensor_id", 42}});
    require(result.status == "rejected", "invalid arg type rejected");
    require(result.error_summary.find("expected string") != std::string::npos, "type diagnostic");

    result = executor.execute(action, {{"sensor_id", "front_lidar"}, {"extra", true}});
    require(result.status == "rejected", "unknown arg rejected");
    require(result.error_summary.find("extra") != std::string::npos, "unknown arg diagnostic");

    const auto failing_command = root / "actions" / "failing";
    write_executable(failing_command, "#!/bin/sh\necho failed >&2\nexit 7\n");
    action = make_action(failing_command);
    result = executor.execute(action, {{"sensor_id", "front_lidar"}});
    require(result.status == "failed", "non-zero exit failed");
    require(result.exit_code == 7, "non-zero exit code");

    const auto timeout_command = root / "actions" / "timeout";
    write_executable(timeout_command, "#!/bin/sh\nsleep 5\n");
    action = make_action(timeout_command);
    action.timeout_sec = 1;
    result = executor.execute(action, {{"sensor_id", "front_lidar"}});
    require(result.status == "timed_out", "timeout status");
    require(result.timed_out, "timeout flag");

    const auto noisy_command = root / "actions" / "noisy";
    write_executable(
      noisy_command,
      "#!/bin/sh\n"
      "i=0\n"
      "while [ \"$i\" -lt 300 ]; do printf xxxxxxxxxxxxxxxx; i=$((i + 1)); done\n"
    );
    action = make_action(noisy_command);
    result = executor.execute(action, {{"sensor_id", "front_lidar"}});
    require(result.status == "succeeded", "noisy action should succeed");
    require(result.stdout_truncated, "stdout should be truncated");
    require(result.stdout_text.size() == options.max_output_bytes, "stdout truncation size");

    std::filesystem::remove_all(root);
    return 0;
  } catch (const std::exception& ex) {
    std::cerr << ex.what() << std::endl;
    std::filesystem::remove_all(root);
    return 1;
  }
}
