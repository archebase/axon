// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_AGENT_AGENT_SERVICE_HPP
#define AXON_AGENT_AGENT_SERVICE_HPP

#include "adapter_loader.hpp"
#include "process_manager.hpp"
#include "profile_manager.hpp"
#include "rpc_response.hpp"

#include <filesystem>
#include <mutex>
#include <string>

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

private:
  ManagedProcessConfig build_process_config(const RobotProfile& profile, const std::string& process_id);
  ManagedProcessConfig load_yaml_process_config(
    const RobotProfile& profile, const std::string& process_id, const std::filesystem::path& yaml_path
  );
  ManagedProcessConfig load_adapter_process_config(const RobotProfile& profile, const std::string& process_id);
  void discover_active_processes(const RobotProfile& profile);
  static std::string require_string(const nlohmann::json& params, const std::string& key);

  std::mutex mutex_;
  ProfileManager profiles_;
  AdapterLoader adapter_loader_;
  ProcessManager processes_;
  std::filesystem::path state_dir_;
  std::string last_error_;
};

}  // namespace agent
}  // namespace axon

#endif  // AXON_AGENT_AGENT_SERVICE_HPP
