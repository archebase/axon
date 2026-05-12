// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include <chrono>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>

#include "process_manager.hpp"

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
  const auto root = make_temp_dir("axon_agent_process_manager");
  auto manager = std::make_unique<axon::agent::ProcessManager>(root / "state");
  bool started = false;
  try {
    axon::agent::ManagedProcessConfig config;
    config.process_id = "worker";
    config.executable = "/bin/sh";
    config.args = {"-c", "echo health-ready; echo health-error >&2; sleep 30"};
    config.working_directory = root;
    config.stop_timeout_sec = 1;
    config.health_check.type = "process";

    std::string error;
    require(manager->start(config, &error), "start failed: " + error);
    started = true;

    nlohmann::json stdout_log;
    for (int i = 0; i < 20; ++i) {
      require(
        manager->read_log("worker", "stdout", 4096, &stdout_log, &error),
        "read stdout failed: " + error
      );
      if (stdout_log.value("content", std::string()).find("health-ready") != std::string::npos) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    require(
      stdout_log.value("content", std::string()).find("health-ready") != std::string::npos,
      "stdout log did not contain expected content"
    );

    nlohmann::json stderr_log;
    require(
      manager->read_log("worker", "stderr", 4096, &stderr_log, &error),
      "read stderr failed: " + error
    );
    require(
      stderr_log.value("content", std::string()).find("health-error") != std::string::npos,
      "stderr log did not contain expected content"
    );

    const auto running_state = manager->state_to_json();
    require(running_state["worker"]["running"].get<bool>(), "worker should be running");
    require(
      running_state["worker"]["health"]["healthy"].get<bool>(),
      "worker process health should be healthy"
    );
    require(
      running_state["worker"]["health"]["status"].get<std::string>() == "healthy",
      "unexpected healthy status"
    );

    require(manager->stop("worker", false, &error), "stop failed: " + error);
    started = false;
    const auto stopped_state = manager->state_to_json();
    require(!stopped_state["worker"]["running"].get<bool>(), "worker should be stopped");
    require(
      !stopped_state["worker"]["health"]["healthy"].get<bool>(),
      "stopped worker should not be healthy"
    );

    std::filesystem::remove_all(root);
    return 0;
  } catch (const std::exception& ex) {
    std::cerr << ex.what() << std::endl;
    if (started) {
      std::string cleanup_error;
      (void)manager->stop("worker", true, &cleanup_error);
    }
    std::filesystem::remove_all(root);
    return 1;
  }
}
