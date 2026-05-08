// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "agent_service.hpp"

#include <yaml-cpp/yaml.h>

#include <exception>
#include <stdexcept>
#include <utility>

namespace axon {
namespace agent {

AgentService::AgentService(std::filesystem::path profile_root, std::filesystem::path state_dir)
  : profiles_(std::move(profile_root)), processes_(state_dir), state_dir_(std::move(state_dir)) {}

bool AgentService::initialize(std::string* error) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!profiles_.scan(error)) {
    if (error != nullptr) {
      last_error_ = *error;
    }
    return false;
  }
  return true;
}

RpcResponse AgentService::get_state() {
  std::lock_guard<std::mutex> lock(mutex_);
  RpcResponse response;
  response.message = "ok";
  response.data = {
    {"active_profile", profiles_.active_profile_to_json()},
    {"adapter", adapter_loader_.status_to_json()},
    {"processes", processes_.state_to_json()},
    {"last_error", last_error_},
  };
  return response;
}

RpcResponse AgentService::get_report() {
  std::lock_guard<std::mutex> lock(mutex_);
  RpcResponse response;
  response.message = "ok";
  response.data = {
    {"profiles", profiles_.profiles_to_json()},
    {"active_profile", profiles_.active_profile_to_json()},
    {"adapter", adapter_loader_.status_to_json()},
    {"processes", processes_.state_to_json()},
  };
  return response;
}

RpcResponse AgentService::list_profiles() {
  std::lock_guard<std::mutex> lock(mutex_);
  RpcResponse response;
  response.message = "ok";
  response.data = {{"profiles", profiles_.profiles_to_json()}, {"active_profile", profiles_.active_profile_to_json()}};
  return response;
}

RpcResponse AgentService::select_profile(const nlohmann::json& params) {
  std::lock_guard<std::mutex> lock(mutex_);
  RpcResponse response;
  try {
    const auto profile_id = require_string(params, "profile_id");
    std::string error;
    if (!profiles_.select_profile(profile_id, &error)) {
      last_error_ = error;
      return {false, error, nullptr};
    }

    const auto* profile = profiles_.active_profile();
    if (profile == nullptr) {
      return {false, "active profile is missing after selection", nullptr};
    }

    discover_active_processes(*profile);
    if (profile->auto_start && !adapter_loader_.load(*profile, &error)) {
      last_error_ = error;
      return {false, error, nullptr};
    }

    response.message = "profile selected";
    response.data = {{"active_profile", profiles_.active_profile_to_json()}, {"adapter", adapter_loader_.status_to_json()}};
    return response;
  } catch (const std::exception& ex) {
    last_error_ = ex.what();
    return {false, ex.what(), nullptr};
  }
}

RpcResponse AgentService::start_process(const nlohmann::json& params) {
  std::lock_guard<std::mutex> lock(mutex_);
  try {
    const auto process_id = require_string(params, "process_id");
    const auto* profile = profiles_.active_profile();
    if (profile == nullptr) {
      return {false, "no active profile selected", nullptr};
    }

    std::string error;
    if (process_id == "robot_startup" && !adapter_loader_.is_loaded()) {
      if (!adapter_loader_.load(*profile, &error)) {
        last_error_ = error;
        return {false, error, nullptr};
      }
    }

    const auto config = build_process_config(*profile, process_id);
    if (!processes_.start(config, &error)) {
      last_error_ = error;
      return {false, error, nullptr};
    }

    return {true, "process started", {{"process_id", process_id}, {"processes", processes_.state_to_json()}}};
  } catch (const std::exception& ex) {
    last_error_ = ex.what();
    return {false, ex.what(), nullptr};
  }
}

RpcResponse AgentService::stop_process(const nlohmann::json& params, bool force) {
  std::lock_guard<std::mutex> lock(mutex_);
  try {
    const auto process_id = require_string(params, "process_id");
    std::string error;
    if (!processes_.stop(process_id, force, &error)) {
      last_error_ = error;
      return {false, error, nullptr};
    }
    return {true, force ? "process force stopped" : "process stopped", processes_.state_to_json()};
  } catch (const std::exception& ex) {
    last_error_ = ex.what();
    return {false, ex.what(), nullptr};
  }
}

ManagedProcessConfig AgentService::build_process_config(const RobotProfile& profile, const std::string& process_id) {
  if (process_id == "recorder") {
    return load_yaml_process_config(profile, process_id, profile.recorder_yaml);
  }
  if (process_id == "transfer") {
    return load_yaml_process_config(profile, process_id, profile.transfer_yaml);
  }
  if (process_id == "robot_startup") {
    return load_adapter_process_config(profile, process_id);
  }
  throw std::runtime_error("unknown process_id: " + process_id);
}

ManagedProcessConfig AgentService::load_yaml_process_config(
  const RobotProfile& profile, const std::string& process_id, const std::filesystem::path& yaml_path
) {
  ManagedProcessConfig config;
  config.process_id = process_id;
  config.pid_file = state_dir_ / (profile.profile_id + "_" + process_id + ".pid");
  config.working_directory = profile.root_dir;

  if (!std::filesystem::exists(yaml_path)) {
    throw std::runtime_error("missing yaml for process " + process_id + ": " + yaml_path.string());
  }

  const auto yaml = YAML::LoadFile(yaml_path.string());
  const auto process = yaml["process"] ? yaml["process"] : yaml;
  config.executable =
    process["executable"] ? process["executable"].as<std::string>() : (process_id == "recorder" ? "axon-recorder" : "axon-transfer");

  if (process["args"]) {
    for (const auto& arg : process["args"]) {
      config.args.push_back(arg.as<std::string>());
    }
  } else {
    config.args.push_back("--config");
    config.args.push_back(yaml_path.string());
  }

  if (process["working_directory"]) {
    config.working_directory = profile.root_dir / process["working_directory"].as<std::string>();
  }
  if (process["env"]) {
    for (const auto& item : process["env"]) {
      config.env[item.first.as<std::string>()] = item.second.as<std::string>();
    }
  }
  return config;
}

ManagedProcessConfig AgentService::load_adapter_process_config(
  const RobotProfile& profile, const std::string& process_id
) {
  ManagedProcessConfig config;
  config.process_id = process_id;
  config.pid_file = state_dir_ / (profile.profile_id + "_" + process_id + ".pid");
  config.working_directory = profile.root_dir;

  const auto yaml = YAML::LoadFile(profile.adapter_yaml.string());
  const auto process = yaml["managed_processes"] && yaml["managed_processes"][process_id]
    ? yaml["managed_processes"][process_id]
    : YAML::Node();
  if (!process) {
    throw std::runtime_error("adapter.yaml does not declare managed_processes." + process_id);
  }

  config.executable = process["executable"].as<std::string>();
  if (process["args"]) {
    for (const auto& arg : process["args"]) {
      config.args.push_back(arg.as<std::string>());
    }
  }
  if (process["working_directory"]) {
    config.working_directory = profile.root_dir / process["working_directory"].as<std::string>();
  }
  if (process["env"]) {
    for (const auto& item : process["env"]) {
      config.env[item.first.as<std::string>()] = item.second.as<std::string>();
    }
  }
  return config;
}

void AgentService::discover_active_processes(const RobotProfile& profile) {
  for (const auto& process_id : {"robot_startup", "recorder", "transfer"}) {
    try {
      processes_.discover(build_process_config(profile, process_id));
    } catch (const std::exception&) {
      // Optional process configs should not block profile selection.
    }
  }
}

std::string AgentService::require_string(const nlohmann::json& params, const std::string& key) {
  if (!params.contains(key) || !params[key].is_string()) {
    throw std::runtime_error("missing string parameter: " + key);
  }
  return params[key].get<std::string>();
}

}  // namespace agent
}  // namespace axon
