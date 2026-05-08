// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "process_manager.hpp"

#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstring>
#include <fstream>
#include <thread>

extern char** environ;

namespace axon {
namespace agent {

ProcessManager::ProcessManager(std::filesystem::path state_dir) : state_dir_(std::move(state_dir)) {
  std::filesystem::create_directories(state_dir_);
}

bool ProcessManager::start(const ManagedProcessConfig& config, std::string* error) {
  if (config.executable.empty()) {
    if (error != nullptr) {
      *error = "process executable is empty: " + config.process_id;
    }
    return false;
  }

  auto state = states_[config.process_id];
  if (state.pid > 0 && is_running(state.pid)) {
    if (error != nullptr) {
      *error = "process is already running: " + config.process_id;
    }
    return false;
  }

  std::vector<std::string> argv_storage;
  auto argv = build_exec_array(config.executable, config.args, &argv_storage);
  std::vector<std::string> env_storage;
  auto envp = build_env_array(config.env, &env_storage);

  pid_t pid = fork();
  if (pid < 0) {
    if (error != nullptr) {
      *error = std::strerror(errno);
    }
    return false;
  }

  if (pid == 0) {
    setsid();
    if (!config.working_directory.empty()) {
      (void)chdir(config.working_directory.c_str());
    }

    if (envp.empty()) {
      execvp(config.executable.c_str(), argv.data());
    } else {
      execvpe(config.executable.c_str(), argv.data(), envp.data());
    }
    _exit(127);
  }

  state.process_id = config.process_id;
  state.pid = pid;
  state.process_group = pid;
  state.state = "running";
  state.last_error.clear();
  states_[config.process_id] = state;

  const auto pid_file = config.pid_file.empty() ? default_pid_file(config.process_id) : config.pid_file;
  write_pid_file(pid_file, pid);
  return true;
}

bool ProcessManager::stop(const std::string& process_id, bool force, std::string* error) {
  auto it = states_.find(process_id);
  if (it == states_.end() || it->second.pid <= 0 || !is_running(it->second.pid)) {
    if (error != nullptr) {
      *error = "process is not running: " + process_id;
    }
    return false;
  }

  const pid_t process_group = it->second.process_group > 0 ? it->second.process_group : it->second.pid;
  if (kill(-process_group, force ? SIGKILL : SIGTERM) != 0 && errno != ESRCH) {
    if (error != nullptr) {
      *error = std::strerror(errno);
    }
    return false;
  }

  if (!force) {
    for (int i = 0; i < 50; ++i) {
      if (!is_running(it->second.pid)) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (is_running(it->second.pid)) {
      (void)kill(-process_group, SIGKILL);
    }
  }

  (void)waitpid(it->second.pid, nullptr, WNOHANG);
  it->second.pid = -1;
  it->second.process_group = -1;
  it->second.state = "stopped";
  return true;
}

void ProcessManager::discover(const ManagedProcessConfig& config) {
  const auto pid_file = config.pid_file.empty() ? default_pid_file(config.process_id) : config.pid_file;
  pid_t pid = -1;
  ManagedProcessState state;
  state.process_id = config.process_id;
  if (read_pid_file(pid_file, &pid) && is_running(pid)) {
    state.pid = pid;
    state.process_group = pid;
    state.state = "running";
  } else {
    state.state = "stopped";
  }
  states_[config.process_id] = state;
}

nlohmann::json ProcessManager::state_to_json() const {
  nlohmann::json processes = nlohmann::json::object();
  for (const auto& pair : states_) {
    processes[pair.first] = {
      {"process_id", pair.second.process_id},
      {"pid", pair.second.pid},
      {"process_group", pair.second.process_group},
      {"state", pair.second.state},
      {"last_error", pair.second.last_error},
    };
  }
  return processes;
}

bool ProcessManager::is_running(pid_t pid) {
  return pid > 0 && (kill(pid, 0) == 0 || errno == EPERM);
}

bool ProcessManager::read_pid_file(const std::filesystem::path& path, pid_t* pid) {
  std::ifstream input(path);
  if (!input) {
    return false;
  }
  input >> *pid;
  return input.good() || input.eof();
}

void ProcessManager::write_pid_file(const std::filesystem::path& path, pid_t pid) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream output(path);
  output << pid << '\n';
}

std::vector<char*> ProcessManager::build_exec_array(
  const std::string& executable, const std::vector<std::string>& args, std::vector<std::string>* storage
) {
  storage->clear();
  storage->push_back(executable);
  storage->insert(storage->end(), args.begin(), args.end());

  std::vector<char*> result;
  for (auto& item : *storage) {
    result.push_back(item.data());
  }
  result.push_back(nullptr);
  return result;
}

std::vector<char*> ProcessManager::build_env_array(
  const std::map<std::string, std::string>& env, std::vector<std::string>* storage
) {
  storage->clear();
  for (const auto& pair : env) {
    storage->push_back(pair.first + "=" + pair.second);
  }

  std::vector<char*> result;
  for (auto& item : *storage) {
    result.push_back(item.data());
  }
  if (!result.empty()) {
    result.push_back(nullptr);
  }
  return result;
}

std::filesystem::path ProcessManager::default_pid_file(const std::string& process_id) const {
  return state_dir_ / (process_id + ".pid");
}

}  // namespace agent
}  // namespace axon
