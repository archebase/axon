// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_AGENT_AGENT_SERVICE_HPP
#define AXON_AGENT_AGENT_SERVICE_HPP

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <filesystem>
#include <mutex>
#include <string>

#include "adapter_loader.hpp"
#include "process_manager.hpp"
#include "profile_manager.hpp"
#include "rpc_response.hpp"

namespace axon {
namespace agent {

class AgentService {
public:
  AgentService(std::filesystem::path profile_root, std::filesystem::path state_dir);

  bool initialize(std::string* error);
  RpcResponse get_state();
  RpcResponse get_report();
  RpcResponse list_profiles();
  RpcResponse select_profile(const nlohmann::json& params);
  RpcResponse start_process(const nlohmann::json& params);
  RpcResponse stop_process(const nlohmann::json& params, bool force);
  RpcResponse read_process_log(const nlohmann::json& params);

private:
  RpcResponse auto_start_robot_process(const RobotProfile& profile, const std::string& trigger);
  RpcResponse start_robot_process(const RobotProfile& profile);
  RpcResponse stop_robot_process(const RobotProfile& profile, bool force);
  ManagedProcessConfig build_process_config(
    const RobotProfile& profile, const std::string& process_id
  );
  ManagedProcessConfig load_yaml_process_config(
    const RobotProfile& profile, const std::string& process_id,
    const std::filesystem::path& yaml_path
  );
  ManagedProcessConfig load_adapter_process_config(
    const RobotProfile& profile, const std::string& process_id
  );
  void apply_process_yaml(
    const RobotProfile& profile, const YAML::Node& process,
    const std::filesystem::path& config_path, ManagedProcessConfig* config
  );
  RobotAdapterContext build_adapter_context(const RobotProfile& profile) const;
  void discover_active_processes(const RobotProfile& profile);
  bool restore_active_profile(std::string* error);
  bool persist_active_profile(const RobotProfile& profile, std::string* error) const;
  std::filesystem::path active_profile_state_file() const;
  static std::string require_string(const nlohmann::json& params, const std::string& key);

  std::mutex mutex_;
  std::filesystem::path profile_root_;
  ProfileManager profiles_;
  AdapterLoader adapter_loader_;
  ProcessManager processes_;
  std::filesystem::path state_dir_;
  std::chrono::steady_clock::time_point started_at_ = std::chrono::steady_clock::now();
  std::string started_at_iso_;
  std::string last_error_;
};

}  // namespace agent
}  // namespace axon

#endif  // AXON_AGENT_AGENT_SERVICE_HPP
