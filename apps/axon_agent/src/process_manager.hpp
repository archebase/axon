// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_AGENT_PROCESS_MANAGER_HPP
#define AXON_AGENT_PROCESS_MANAGER_HPP

#include <nlohmann/json.hpp>

#include <sys/types.h>

#include <filesystem>
#include <map>
#include <string>
#include <vector>

namespace axon {
namespace agent {

struct ManagedProcessConfig {
  std::string process_id;
  std::string executable;
  std::vector<std::string> args;
  std::filesystem::path working_directory;
  std::filesystem::path pid_file;
  std::map<std::string, std::string> env;
  int stop_timeout_sec = 5;
};

struct ManagedProcessState {
  std::string process_id;
  pid_t pid = -1;
  pid_t process_group = -1;
  std::string state = "stopped";
  std::string last_error;
};

class ProcessManager {
public:
  explicit ProcessManager(std::filesystem::path state_dir);

  bool start(const ManagedProcessConfig& config, std::string* error);
  bool stop(const std::string& process_id, bool force, std::string* error);
  void discover(const ManagedProcessConfig& config);
  nlohmann::json state_to_json() const;

private:
  static bool is_running(pid_t pid);
  static bool read_pid_file(const std::filesystem::path& path, pid_t* pid);
  static void write_pid_file(const std::filesystem::path& path, pid_t pid);
  static std::vector<char*> build_exec_array(
    const std::string& executable, const std::vector<std::string>& args, std::vector<std::string>* storage
  );
  static std::vector<char*> build_env_array(
    const std::map<std::string, std::string>& env, std::vector<std::string>* storage
  );

  std::filesystem::path default_pid_file(const std::string& process_id) const;

  std::filesystem::path state_dir_;
  std::map<std::string, ManagedProcessState> states_;
};

}  // namespace agent
}  // namespace axon

#endif  // AXON_AGENT_PROCESS_MANAGER_HPP
