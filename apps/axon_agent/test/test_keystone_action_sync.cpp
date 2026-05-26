// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include <sys/stat.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <vector>

#include "agent_service.hpp"
#include "keystone_action_sync.hpp"

namespace {

struct TransportCall {
  std::string method;
  std::string path;
  nlohmann::json body = nullptr;
};

class FakeTransport : public axon::agent::KeystoneActionTransport {
public:
  using Handler = std::function<axon::agent::KeystoneHttpResponse(
    const std::string&, const std::string&, const nlohmann::json&
  )>;

  axon::agent::KeystoneHttpResponse get(
    const std::string& path, std::chrono::milliseconds
  ) override {
    calls.push_back({"GET", path, nullptr});
    return handler("GET", path, nullptr);
  }

  axon::agent::KeystoneHttpResponse put_json(
    const std::string& path, const nlohmann::json& body, std::chrono::milliseconds
  ) override {
    calls.push_back({"PUT", path, body});
    return handler("PUT", path, body);
  }

  axon::agent::KeystoneHttpResponse post_json(
    const std::string& path, const nlohmann::json& body, std::chrono::milliseconds
  ) override {
    calls.push_back({"POST", path, body});
    return handler("POST", path, body);
  }

  Handler handler = [](const std::string&, const std::string&, const nlohmann::json&) {
    return axon::agent::KeystoneHttpResponse{true, 200, nlohmann::json::object(), ""};
  };
  std::vector<TransportCall> calls;
};

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

int read_counter(const std::filesystem::path& path) {
  if (!std::filesystem::exists(path)) {
    return 0;
  }
  std::ifstream input(path);
  int value = 0;
  input >> value;
  return value;
}

std::string action_manifest(const std::string& id, const std::filesystem::path& command) {
  return "id: " + id +
         "\n"
         "title: Restart sensor\n"
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
         "timeout_sec: 2\n"
         "run_as: current\n"
         "allowed_roles:\n"
         "  - operator\n"
         "requires_approval: false\n";
}

}  // namespace

int main() {
  const auto root = make_temp_dir("axon_agent_keystone_action_sync");
  try {
    const auto profile_root = root / "profiles";
    const auto state_dir = root / "state";
    const auto manifest_dir = root / "actions.d";
    const auto command_dir = root / "opt" / "axon" / "actions";
    const auto counter_path = root / "counter.txt";
    std::filesystem::create_directories(profile_root);

    write_file(
      profile_root / "demo_robot" / "adapter.yaml",
      R"YAML(adapter_id: demo_robot
robot_model: demo
abi_version: 1
auto_start: false
)YAML"
    );
    const auto command = command_dir / "restart_sensor";
    write_executable(
      command,
      "#!/bin/sh\n"
      "count_file='" +
        counter_path.string() +
        "'\n"
        "count=0\n"
        "if [ -f \"$count_file\" ]; then count=$(cat \"$count_file\"); fi\n"
        "count=$((count + 1))\n"
        "echo \"$count\" > \"$count_file\"\n"
        "cat >/dev/null\n"
        "echo restarted\n"
    );
    write_file(manifest_dir / "restart.yaml", action_manifest("restart_sensor", command));

    axon::agent::AgentService service(profile_root, state_dir, manifest_dir, command_dir);
    std::string error;
    require(service.initialize(&error), "initialize failed: " + error);

    axon::agent::KeystoneActionSyncConfig config;
    config.enabled = true;
    config.base_url = "http://keystone.local";
    config.robot_id = "robot-A";
    config.min_backoff = std::chrono::milliseconds(100);
    config.max_backoff = std::chrono::milliseconds(200);

    auto catalog_transport = std::make_unique<FakeTransport>();
    auto* catalog_fake = catalog_transport.get();
    axon::agent::KeystoneActionSync catalog_sync(service, config, std::move(catalog_transport));
    require(catalog_sync.sync_catalog_once(&error), "catalog sync failed: " + error);
    require(catalog_fake->calls.size() == 1, "expected one catalog sync call");
    const auto& catalog_call = catalog_fake->calls.front();
    require(catalog_call.method == "PUT", "catalog sync should use PUT");
    require(
      catalog_call.path == "/api/v1/robots/robot-A/action-catalog",
      "catalog sync path mismatch: " + catalog_call.path
    );
    require(catalog_call.body["robot_id"].get<std::string>() == "robot-A", "robot_id mismatch");
    require(catalog_call.body["actions"].size() == 1, "catalog action count mismatch");
    const auto& action = catalog_call.body["actions"][0];
    require(action["id"].get<std::string>() == "restart_sensor", "catalog action id mismatch");
    require(!action.contains("command"), "catalog must not expose local command paths");
    require(!action.contains("source_path"), "catalog must not expose manifest paths");

    auto poll_transport = std::make_unique<FakeTransport>();
    auto* poll_fake = poll_transport.get();
    std::vector<std::string> statuses;
    poll_fake->handler =
      [&statuses](const std::string& method, const std::string& path, const nlohmann::json& body) {
        if (method == "GET" && path == "/api/v1/robots/robot-A/action-requests/pending?limit=10") {
          return axon::agent::KeystoneHttpResponse{
            true, 200, nlohmann::json{{"requests", {{{"request_id", "req-1"}}}}}, ""
          };
        }
        if (method == "GET" && path == "/api/v1/action-requests/req-1") {
          return axon::agent::KeystoneHttpResponse{
            true,
            200,
            nlohmann::json{
              {"request",
               {{"request_id", "req-1"},
                {"action_id", "restart_sensor"},
                {"status", "pending"},
                {"args", {{"sensor_id", "front_lidar"}}}}}
            },
            ""
          };
        }
        if (method == "POST" && path == "/api/v1/action-requests/req-1/status") {
          statuses.push_back(body["status"].get<std::string>());
          return axon::agent::KeystoneHttpResponse{true, 200, nlohmann::json::object(), ""};
        }
        return axon::agent::KeystoneHttpResponse{false, 404, nullptr, "unexpected request"};
      };
    axon::agent::KeystoneActionSync poll_sync(service, config, std::move(poll_transport));
    require(poll_sync.poll_once(&error), "poll failed: " + error);
    require(statuses.size() == 3, "expected queued/running/final status reports");
    require(statuses[0] == "queued", "queued status missing");
    require(statuses[1] == "running", "running status missing");
    require(statuses[2] == "succeeded", "succeeded status missing");
    require(read_counter(counter_path) == 1, "polled action did not execute once");
    require(poll_fake->calls.size() == 5, "pending/detail/status call count mismatch");
    require(
      poll_fake->calls[1].path == "/api/v1/action-requests/req-1", "request detail fetch missing"
    );

    auto cancel_transport = std::make_unique<FakeTransport>();
    auto* cancel_fake = cancel_transport.get();
    std::vector<std::string> cancel_statuses;
    cancel_fake->handler =
      [&cancel_statuses](
        const std::string& method, const std::string& path, const nlohmann::json& body
      ) {
        if (method == "GET" && path == "/api/v1/robots/robot-A/action-requests/pending?limit=10") {
          return axon::agent::KeystoneHttpResponse{
            true, 200, nlohmann::json{{"requests", {{{"request_id", "req-cancel"}}}}}, ""
          };
        }
        if (method == "GET" && path == "/api/v1/action-requests/req-cancel") {
          return axon::agent::KeystoneHttpResponse{
            true,
            200,
            nlohmann::json{
              {"request",
               {{"request_id", "req-cancel"},
                {"action_id", "restart_sensor"},
                {"status", "cancelled"},
                {"args", {{"sensor_id", "front_lidar"}}}}}
            },
            ""
          };
        }
        if (method == "POST" && path == "/api/v1/action-requests/req-cancel/status") {
          cancel_statuses.push_back(body["status"].get<std::string>());
          return axon::agent::KeystoneHttpResponse{true, 200, nlohmann::json::object(), ""};
        }
        return axon::agent::KeystoneHttpResponse{false, 404, nullptr, "unexpected request"};
      };
    axon::agent::KeystoneActionSync cancel_sync(service, config, std::move(cancel_transport));
    require(cancel_sync.poll_once(&error), "cancel poll failed: " + error);
    require(cancel_statuses.size() == 1, "cancelled request should report one status");
    require(cancel_statuses[0] == "cancelled", "cancelled status mismatch");
    require(read_counter(counter_path) == 1, "cancelled request should not execute");

    auto outage_transport = std::make_unique<FakeTransport>();
    outage_transport->handler =
      [](const std::string& method, const std::string& path, const nlohmann::json&) {
        if (method == "GET" && path == "/api/v1/robots/robot-A/action-requests/pending?limit=10") {
          return axon::agent::KeystoneHttpResponse{false, 503, nullptr, "unavailable"};
        }
        return axon::agent::KeystoneHttpResponse{false, 404, nullptr, "unexpected request"};
      };
    axon::agent::KeystoneActionSync outage_sync(service, config, std::move(outage_transport));
    require(!outage_sync.poll_once(&error), "outage poll should fail");
    auto status = outage_sync.status_to_json();
    require(status["consecutive_failures"].get<std::uint64_t>() == 1, "failure count mismatch");
    require(status["current_backoff_ms"].get<int>() == 100, "initial backoff mismatch");
    require(!outage_sync.poll_once(&error), "second outage poll should fail");
    status = outage_sync.status_to_json();
    require(status["current_backoff_ms"].get<int>() == 200, "bounded backoff mismatch");

    std::filesystem::remove_all(root);
    return 0;
  } catch (const std::exception& ex) {
    std::cerr << ex.what() << std::endl;
    std::filesystem::remove_all(root);
    return 1;
  }
}
