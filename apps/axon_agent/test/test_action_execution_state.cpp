// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include <sys/stat.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <thread>
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

void write_executable(const std::filesystem::path& path, const std::string& content) {
  write_file(path, content);
  chmod(path.c_str(), 0755);
}

nlohmann::json find_record(const nlohmann::json& executions, const std::string& request_id) {
  for (const auto& record : executions["records"]) {
    if (record.value("request_id", std::string()) == request_id) {
      return record;
    }
  }
  return nullptr;
}

int read_counter(const std::filesystem::path& path) {
  if (!std::filesystem::exists(path)) {
    return 0;
  }
  std::ifstream input(path);
  int value = 0;
  input >> value;
  return value;
}

std::string action_manifest(
  const std::string& id, const std::string& title, const std::filesystem::path& command
) {
  return "id: " + id +
         "\n"
         "title: " +
         title +
         "\n"
         "description: Test action\n"
         "command: " +
         command.string() +
         "\n"
         "args_schema:\n"
         "  type: object\n"
         "  required:\n"
         "    - sensor_id\n"
         "  additionalProperties: false\n"
         "  properties:\n"
         "    sensor_id:\n"
         "      type: string\n"
         "    password:\n"
         "      type: string\n"
         "timeout_sec: 2\n"
         "run_as: current\n"
         "allowed_roles:\n"
         "  - operator\n"
         "requires_approval: false\n";
}

}  // namespace

int main() {
  const auto root = make_temp_dir("axon_agent_action_execution_state");
  try {
    const auto profile_root = root / "profiles";
    const auto state_dir = root / "state";
    const auto manifest_dir = root / "actions.d";
    const auto command_dir = root / "opt" / "axon" / "actions";
    const auto counter_path = root / "counter.txt";
    std::filesystem::create_directories(profile_root);

    const auto success_command = command_dir / "success";
    write_executable(
      success_command,
      "#!/bin/sh\n"
      "count_file='" +
        counter_path.string() +
        "'\n"
        "count=0\n"
        "if [ -f \"$count_file\" ]; then count=$(cat \"$count_file\"); fi\n"
        "count=$((count + 1))\n"
        "echo \"$count\" > \"$count_file\"\n"
        "cat >/dev/null\n"
        "echo completed\n"
    );
    const auto fail_command = command_dir / "fail";
    write_executable(fail_command, "#!/bin/sh\ncat >/dev/null\necho failed >&2\nexit 7\n");
    const auto slow_command = command_dir / "slow";
    write_executable(slow_command, "#!/bin/sh\ncat >/dev/null\nsleep 1\necho slow-complete\n");

    write_file(
      manifest_dir / "success.yaml", action_manifest("restart_sensor", "Restart", success_command)
    );
    write_file(manifest_dir / "fail.yaml", action_manifest("fail_sensor", "Fail", fail_command));
    write_file(manifest_dir / "slow.yaml", action_manifest("slow_sensor", "Slow", slow_command));

    write_file(
      state_dir / "action_executions.json",
      R"JSON({
  "records": [
    {
      "request_id": "stale-running",
      "action_id": "restart_sensor",
      "status": "running",
      "queued_at": "2026-01-01T00:00:00Z",
      "started_at": "2026-01-01T00:00:01Z",
      "args": {"sensor_id": "front"}
    }
  ]
})JSON"
    );

    axon::agent::AgentService service(profile_root, state_dir, manifest_dir, command_dir);
    std::string error;
    require(service.initialize(&error), "initialize failed: " + error);

    auto executions = service.list_action_executions();
    require(executions.success, "list executions failed");
    auto stale = find_record(executions.data, "stale-running");
    require(!stale.is_null(), "stale record missing");
    require(stale["status"].get<std::string>() == "failed", "stale record not recovered");
    require(
      stale["error_summary"].get<std::string>().find("agent restart") != std::string::npos,
      "stale recovery diagnostic missing"
    );

    auto response = service.execute_action(
      {{"request_id", "req-success"},
       {"action_id", "restart_sensor"},
       {"args", {{"sensor_id", "front_lidar"}, {"password", "cleartext-secret"}}}}
    );
    require(response.success, "success action failed: " + response.message);
    require(response.data["status"].get<std::string>() == "completed", "completed status");
    require(
      response.data["args"]["password"].get<std::string>() == "[REDACTED]",
      "sensitive args should be redacted in response"
    );
    require(read_counter(counter_path) == 1, "success action did not execute once");

    response = service.execute_action(
      {{"request_id", "req-success"},
       {"action_id", "restart_sensor"},
       {"args", {{"sensor_id", "front_lidar"}, {"password", "cleartext-secret"}}}}
    );
    require(response.success, "duplicate request should return existing state");
    require(response.data["idempotent"].get<bool>(), "duplicate idempotent flag missing");
    require(response.data["status"].get<std::string>() == "completed", "duplicate status");
    require(read_counter(counter_path) == 1, "duplicate request executed action twice");

    response = service.execute_action(
      {{"request_id", "req-expired"},
       {"action_id", "restart_sensor"},
       {"expires_at", "2000-01-01T00:00:00Z"},
       {"args", {{"sensor_id", "front_lidar"}}}}
    );
    require(!response.success, "expired request should not succeed");
    require(response.data["status"].get<std::string>() == "expired", "expired status");
    require(read_counter(counter_path) == 1, "expired request executed action");

    response = service.execute_action(
      {{"request_id", "req-failed"},
       {"action_id", "fail_sensor"},
       {"args", {{"sensor_id", "front_lidar"}}}}
    );
    require(!response.success, "failed action should not succeed");
    require(response.data["status"].get<std::string>() == "failed", "failed status");
    require(response.data["exit_code"].get<int>() == 7, "failed exit code persisted");

    axon::agent::RpcResponse slow_response;
    std::thread slow_thread([&service, &slow_response]() {
      slow_response = service.execute_action(
        {{"request_id", "req-running"},
         {"action_id", "slow_sensor"},
         {"args", {{"sensor_id", "front_lidar"}}}}
      );
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    response = service.execute_action(
      {{"request_id", "req-running"},
       {"action_id", "slow_sensor"},
       {"args", {{"sensor_id", "front_lidar"}}}}
    );
    require(response.success, "running duplicate should return existing state");
    require(response.data["idempotent"].get<bool>(), "running duplicate idempotent flag missing");
    require(response.data["status"].get<std::string>() == "running", "running duplicate status");
    slow_thread.join();
    require(slow_response.success, "slow action should complete");

    axon::agent::AgentService restarted(profile_root, state_dir, manifest_dir, command_dir);
    require(restarted.initialize(&error), "restart initialize failed: " + error);
    executions = restarted.list_action_executions();
    auto completed = find_record(executions.data, "req-success");
    auto failed = find_record(executions.data, "req-failed");
    auto expired = find_record(executions.data, "req-expired");
    auto running_duplicate = find_record(executions.data, "req-running");
    require(completed["status"].get<std::string>() == "completed", "completed not persisted");
    require(failed["status"].get<std::string>() == "failed", "failed not persisted");
    require(expired["status"].get<std::string>() == "expired", "expired not persisted");
    require(
      running_duplicate["duplicate_count"].get<int>() == 1, "running duplicate count not preserved"
    );
    require(executions.data["count"].get<std::size_t>() == 5, "diagnostic count mismatch");

    std::ifstream persisted_state(state_dir / "action_executions.json");
    const std::string persisted_content(
      (std::istreambuf_iterator<char>(persisted_state)), std::istreambuf_iterator<char>()
    );
    require(
      persisted_content.find("cleartext-secret") == std::string::npos,
      "sensitive args should be redacted on disk"
    );

    std::filesystem::remove_all(root);
    return 0;
  } catch (const std::exception& ex) {
    std::cerr << ex.what() << std::endl;
    std::filesystem::remove_all(root);
    return 1;
  }
}
