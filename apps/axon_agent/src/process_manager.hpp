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
  std::filesystem::path metadata_file;
  std::filesystem::path stdout_log;
  std::filesystem::path stderr_log;
  std::map<std::string, std::string> env;
  int stop_timeout_sec = 5;
  std::string fingerprint;
};

struct ManagedProcessState {
  std::string process_id;
  pid_t pid = -1;
  pid_t process_group = -1;
  std::string state = "stopped";
  std::string last_error;
  std::string executable;
  std::vector<std::string> args;
  std::filesystem::path working_directory;
  std::filesystem::path pid_file;
  std::filesystem::path metadata_file;
  std::filesystem::path stdout_log;
  std::filesystem::path stderr_log;
  std::string fingerprint;
  std::string started_at;
  std::string stopped_at;
  int exit_code = -1;
  int term_signal = -1;
  int stop_timeout_sec = 5;
  bool discovered = false;
  bool force_stopped = false;
  std::string proc_cmdline;
};

class ProcessManager {
public:
  explicit ProcessManager(std::filesystem::path state_dir);

  bool start(const ManagedProcessConfig& config, std::string* error);
  bool stop(const std::string& process_id, bool force, std::string* error);
  void discover(const ManagedProcessConfig& config);
  nlohmann::json state_to_json() const;
  bool read_log(
    const std::string& process_id, const std::string& stream, std::size_t tail_bytes, nlohmann::json* output,
    std::string* error
  ) const;

private:
  ManagedProcessConfig normalize_config(const ManagedProcessConfig& config) const;
  ManagedProcessState state_from_config(const ManagedProcessConfig& config) const;
  void apply_metadata(ManagedProcessState* state) const;
  void update_exit_state(ManagedProcessState* state, int wait_status, bool force);
  bool write_metadata_file(const ManagedProcessState& state, std::string* error) const;
  static bool is_running(pid_t pid);
  static bool read_pid_file(const std::filesystem::path& path, pid_t* pid);
  static void write_pid_file(const std::filesystem::path& path, pid_t pid);
  static std::string now_iso8601();
  static std::string build_fingerprint(const ManagedProcessConfig& config);
  static std::string join_cmdline(const std::vector<std::string>& argv);
  static std::vector<std::string> expected_argv(const ManagedProcessConfig& config);
  static bool read_proc_cmdline(pid_t pid, std::vector<std::string>* argv);
  static bool proc_cmdline_matches(const ManagedProcessConfig& config, const std::vector<std::string>& argv);
  static void redirect_to_file(const std::filesystem::path& path, int fd);
  static std::vector<char*> build_exec_array(
    const std::string& executable, const std::vector<std::string>& args, std::vector<std::string>* storage
  );
  static std::vector<char*> build_env_array(
    const std::map<std::string, std::string>& env, std::vector<std::string>* storage
  );

  std::filesystem::path default_pid_file(const std::string& process_id) const;
  std::filesystem::path default_metadata_file(const std::string& process_id) const;
  std::filesystem::path default_log_file(const ManagedProcessConfig& config, const std::string& stream) const;

  std::filesystem::path state_dir_;
  std::map<std::string, ManagedProcessState> states_;
};

}  // namespace agent
}  // namespace axon

#endif  // AXON_AGENT_PROCESS_MANAGER_HPP
