// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "process_manager.hpp"

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <ctime>
#include <cstring>
#include <fstream>
#include <sstream>
#include <thread>

extern char** environ;

namespace axon {
namespace agent {

ProcessManager::ProcessManager(std::filesystem::path state_dir) : state_dir_(std::move(state_dir)) {
  std::filesystem::create_directories(state_dir_);
}

bool ProcessManager::start(const ManagedProcessConfig& config, std::string* error) {
  const auto runtime_config = normalize_config(config);
  if (runtime_config.executable.empty()) {
    if (error != nullptr) {
      *error = "process executable is empty: " + runtime_config.process_id;
    }
    return false;
  }

  auto state = states_[runtime_config.process_id];
  if (state.pid > 0 && is_running(state.pid)) {
    if (error != nullptr) {
      *error = "process is already running: " + runtime_config.process_id;
    }
    return false;
  }

  std::filesystem::create_directories(runtime_config.stdout_log.parent_path());
  std::filesystem::create_directories(runtime_config.stderr_log.parent_path());
  std::filesystem::create_directories(runtime_config.pid_file.parent_path());
  std::filesystem::create_directories(runtime_config.metadata_file.parent_path());

  std::vector<std::string> argv_storage;
  auto argv = build_exec_array(runtime_config.executable, runtime_config.args, &argv_storage);
  std::vector<std::string> env_storage;
  auto envp = build_env_array(runtime_config.env, &env_storage);

  pid_t pid = fork();
  if (pid < 0) {
    if (error != nullptr) {
      *error = std::strerror(errno);
    }
    return false;
  }

  if (pid == 0) {
    setsid();
    redirect_to_file(runtime_config.stdout_log, STDOUT_FILENO);
    redirect_to_file(runtime_config.stderr_log, STDERR_FILENO);
    if (!runtime_config.working_directory.empty()) {
      (void)chdir(runtime_config.working_directory.c_str());
    }

    if (envp.empty()) {
      execvp(runtime_config.executable.c_str(), argv.data());
    } else {
      execvpe(runtime_config.executable.c_str(), argv.data(), envp.data());
    }
    _exit(127);
  }

  state = state_from_config(runtime_config);
  state.pid = pid;
  state.process_group = pid;
  state.state = "running";
  state.last_error.clear();
  state.started_at = now_iso8601();
  state.stopped_at.clear();
  state.exit_code = -1;
  state.term_signal = -1;
  state.discovered = false;
  state.force_stopped = false;

  write_pid_file(runtime_config.pid_file, pid);
  if (!write_metadata_file(state, error)) {
    states_[runtime_config.process_id] = state;
    return false;
  }
  states_[runtime_config.process_id] = state;
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

  const int timeout_ms = force ? 1000 : std::max(1, it->second.stop_timeout_sec) * 1000;
  const int poll_interval_ms = 100;
  bool exited = false;
  int wait_status = 0;
  for (int elapsed_ms = 0; elapsed_ms <= timeout_ms; elapsed_ms += poll_interval_ms) {
    const pid_t waited = waitpid(it->second.pid, &wait_status, WNOHANG);
    if (waited == it->second.pid) {
      exited = true;
      break;
    }
    if (!is_running(it->second.pid)) {
      exited = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(poll_interval_ms));
  }

  if (!exited && !force && is_running(it->second.pid)) {
    (void)kill(-process_group, SIGKILL);
    for (int elapsed_ms = 0; elapsed_ms <= 1000; elapsed_ms += poll_interval_ms) {
      const pid_t waited = waitpid(it->second.pid, &wait_status, WNOHANG);
      if (waited == it->second.pid) {
        exited = true;
        break;
      }
      if (!is_running(it->second.pid)) {
        exited = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(poll_interval_ms));
    }
  }

  if (!exited) {
    if (error != nullptr) {
      *error = "process did not exit after signal: " + process_id;
    }
    it->second.last_error = error == nullptr ? "process did not exit after signal" : *error;
    it->second.state = "failed";
    (void)write_metadata_file(it->second, nullptr);
    return false;
  }

  update_exit_state(&it->second, wait_status, force);
  (void)write_metadata_file(it->second, nullptr);
  return true;
}

void ProcessManager::discover(const ManagedProcessConfig& config) {
  const auto runtime_config = normalize_config(config);
  pid_t pid = -1;
  ManagedProcessState state = state_from_config(runtime_config);
  const bool metadata_matches = apply_metadata(&state);

  std::vector<std::string> proc_argv;
  const bool has_pid = read_pid_file(runtime_config.pid_file, &pid);
  const bool pid_running = has_pid && is_running(pid);
  const bool cmdline_read = pid_running && read_proc_cmdline(pid, &proc_argv);
  const bool cmdline_matches = cmdline_read && proc_cmdline_matches(runtime_config, proc_argv);

  if (pid_running && cmdline_matches && metadata_matches) {
    state.pid = pid;
    state.process_group = pid;
    state.state = "running";
    state.discovered = true;
    state.proc_cmdline = join_cmdline(proc_argv);
  } else {
    state.state = "stopped";
    state.pid = -1;
    state.process_group = -1;
    if (pid_running) {
      if (!cmdline_matches) {
        state.last_error = "pid file points to an unrelated process";
      } else if (state.last_error.empty()) {
        state.last_error = "metadata fingerprint mismatch";
      }
      state.proc_cmdline = join_cmdline(proc_argv);
    }
  }
  states_[runtime_config.process_id] = state;
}

void ProcessManager::forget(const std::string& process_id) {
  states_.erase(process_id);
}

bool ProcessManager::is_process_running(const std::string& process_id) const {
  const auto it = states_.find(process_id);
  return it != states_.end() && it->second.pid > 0 && is_running(it->second.pid);
}

std::vector<std::string> ProcessManager::running_processes() const {
  std::vector<std::string> result;
  for (const auto& pair : states_) {
    if (pair.second.pid > 0 && is_running(pair.second.pid)) {
      result.push_back(pair.first);
    }
  }
  return result;
}

nlohmann::json ProcessManager::state_to_json() const {
  nlohmann::json processes = nlohmann::json::object();
  for (const auto& pair : states_) {
    processes[pair.first] = {
      {"process_id", pair.second.process_id},
      {"pid", pair.second.pid},
      {"process_group", pair.second.process_group},
      {"state", pair.second.state},
      {"running", pair.second.pid > 0 && is_running(pair.second.pid)},
      {"last_error", pair.second.last_error},
      {"executable", pair.second.executable},
      {"args", pair.second.args},
      {"working_directory", pair.second.working_directory.string()},
      {"pid_file", pair.second.pid_file.string()},
      {"metadata_file", pair.second.metadata_file.string()},
      {"stdout_log", pair.second.stdout_log.string()},
      {"stderr_log", pair.second.stderr_log.string()},
      {"fingerprint", pair.second.fingerprint},
      {"started_at", pair.second.started_at},
      {"stopped_at", pair.second.stopped_at},
      {"exit_code", pair.second.exit_code},
      {"term_signal", pair.second.term_signal},
      {"stop_timeout_sec", pair.second.stop_timeout_sec},
      {"discovered", pair.second.discovered},
      {"force_stopped", pair.second.force_stopped},
      {"proc_cmdline", pair.second.proc_cmdline},
    };
  }
  return processes;
}

bool ProcessManager::read_log(
  const std::string& process_id, const std::string& stream, std::size_t tail_bytes, nlohmann::json* output,
  std::string* error
) const {
  const auto it = states_.find(process_id);
  if (it == states_.end()) {
    if (error != nullptr) {
      *error = "process state not found: " + process_id;
    }
    return false;
  }

  std::filesystem::path log_path;
  if (stream == "stdout") {
    log_path = it->second.stdout_log;
  } else if (stream == "stderr") {
    log_path = it->second.stderr_log;
  } else {
    if (error != nullptr) {
      *error = "unknown log stream: " + stream;
    }
    return false;
  }

  if (!std::filesystem::exists(log_path)) {
    if (output != nullptr) {
      *output = {
        {"process_id", process_id},
        {"stream", stream},
        {"log_path", log_path.string()},
        {"exists", false},
        {"size", 0},
        {"tail_bytes", 0},
        {"content", ""},
      };
    }
    return true;
  }

  std::ifstream input(log_path, std::ios::binary);
  if (!input) {
    if (error != nullptr) {
      *error = "log file not found: " + log_path.string();
    }
    return false;
  }

  input.seekg(0, std::ios::end);
  const auto size = static_cast<std::size_t>(std::max<std::streamoff>(0, input.tellg()));
  const auto bytes = tail_bytes == 0 ? size : std::min(size, tail_bytes);
  input.seekg(static_cast<std::streamoff>(size - bytes), std::ios::beg);

  std::string content(bytes, '\0');
  if (bytes > 0) {
    input.read(content.data(), static_cast<std::streamsize>(bytes));
    content.resize(static_cast<std::size_t>(input.gcount()));
  }

  if (output != nullptr) {
    *output = {
      {"process_id", process_id},
      {"stream", stream},
      {"log_path", log_path.string()},
      {"exists", true},
      {"size", size},
      {"tail_bytes", bytes},
      {"content", content},
    };
  }
  return true;
}

ManagedProcessConfig ProcessManager::normalize_config(const ManagedProcessConfig& config) const {
  ManagedProcessConfig normalized = config;
  if (normalized.pid_file.empty()) {
    normalized.pid_file = default_pid_file(normalized.process_id);
  }
  if (normalized.metadata_file.empty()) {
    normalized.metadata_file = default_metadata_file(normalized.process_id);
  }
  if (normalized.stdout_log.empty()) {
    normalized.stdout_log = default_log_file(normalized, "stdout");
  }
  if (normalized.stderr_log.empty()) {
    normalized.stderr_log = default_log_file(normalized, "stderr");
  }
  if (normalized.fingerprint.empty()) {
    normalized.fingerprint = build_fingerprint(normalized);
  }
  if (normalized.stop_timeout_sec <= 0) {
    normalized.stop_timeout_sec = 5;
  }
  return normalized;
}

ManagedProcessState ProcessManager::state_from_config(const ManagedProcessConfig& config) const {
  ManagedProcessState state;
  state.process_id = config.process_id;
  state.executable = config.executable;
  state.args = config.args;
  state.working_directory = config.working_directory;
  state.pid_file = config.pid_file;
  state.metadata_file = config.metadata_file;
  state.stdout_log = config.stdout_log;
  state.stderr_log = config.stderr_log;
  state.fingerprint = config.fingerprint;
  state.stop_timeout_sec = config.stop_timeout_sec;
  return state;
}

bool ProcessManager::apply_metadata(ManagedProcessState* state) const {
  if (state == nullptr || state->metadata_file.empty()) {
    return true;
  }
  std::ifstream input(state->metadata_file);
  if (!input) {
    return true;
  }
  try {
    const auto metadata = nlohmann::json::parse(input);
    const auto metadata_fingerprint = metadata.value("fingerprint", std::string());
    if (!metadata_fingerprint.empty() && metadata_fingerprint != state->fingerprint) {
      state->last_error = "metadata fingerprint mismatch";
      return false;
    }
    state->started_at = metadata.value("started_at", state->started_at);
    state->stopped_at = metadata.value("stopped_at", state->stopped_at);
    state->exit_code = metadata.value("exit_code", state->exit_code);
    state->term_signal = metadata.value("term_signal", state->term_signal);
    state->last_error = metadata.value("last_error", state->last_error);
    state->force_stopped = metadata.value("force_stopped", state->force_stopped);
    return true;
  } catch (const std::exception& ex) {
    state->last_error = std::string("failed to parse metadata: ") + ex.what();
    return false;
  }
}

void ProcessManager::update_exit_state(ManagedProcessState* state, int wait_status, bool force) {
  if (state == nullptr) {
    return;
  }
  state->pid = -1;
  state->process_group = -1;
  state->state = "stopped";
  state->stopped_at = now_iso8601();
  state->force_stopped = force;
  state->last_error.clear();
  if (WIFEXITED(wait_status)) {
    state->exit_code = WEXITSTATUS(wait_status);
    state->term_signal = -1;
  } else if (WIFSIGNALED(wait_status)) {
    state->exit_code = -1;
    state->term_signal = WTERMSIG(wait_status);
  }
}

bool ProcessManager::write_metadata_file(const ManagedProcessState& state, std::string* error) const {
  if (state.metadata_file.empty()) {
    return true;
  }
  std::filesystem::create_directories(state.metadata_file.parent_path());
  std::ofstream output(state.metadata_file);
  if (!output) {
    if (error != nullptr) {
      *error = "failed to write metadata: " + state.metadata_file.string();
    }
    return false;
  }
  const nlohmann::json metadata = {
    {"process_id", state.process_id},
    {"pid", state.pid},
    {"process_group", state.process_group},
    {"state", state.state},
    {"executable", state.executable},
    {"args", state.args},
    {"working_directory", state.working_directory.string()},
    {"pid_file", state.pid_file.string()},
    {"metadata_file", state.metadata_file.string()},
    {"stdout_log", state.stdout_log.string()},
    {"stderr_log", state.stderr_log.string()},
    {"fingerprint", state.fingerprint},
    {"started_at", state.started_at},
    {"stopped_at", state.stopped_at},
    {"exit_code", state.exit_code},
    {"term_signal", state.term_signal},
    {"stop_timeout_sec", state.stop_timeout_sec},
    {"last_error", state.last_error},
    {"force_stopped", state.force_stopped},
  };
  output << metadata.dump(2) << '\n';
  return true;
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

std::string ProcessManager::now_iso8601() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t time = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  gmtime_r(&time, &tm);

  char buffer[32];
  std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &tm);
  return buffer;
}

std::string ProcessManager::build_fingerprint(const ManagedProcessConfig& config) {
  std::ostringstream stream;
  stream << "cwd=" << config.working_directory.string() << '\n';
  stream << "argv=" << config.executable;
  for (const auto& arg : config.args) {
    stream << '\0' << arg;
  }
  return stream.str();
}

std::string ProcessManager::join_cmdline(const std::vector<std::string>& argv) {
  std::ostringstream stream;
  for (std::size_t i = 0; i < argv.size(); ++i) {
    if (i > 0) {
      stream << ' ';
    }
    stream << argv[i];
  }
  return stream.str();
}

std::vector<std::string> ProcessManager::expected_argv(const ManagedProcessConfig& config) {
  std::vector<std::string> argv;
  argv.push_back(config.executable);
  argv.insert(argv.end(), config.args.begin(), config.args.end());
  return argv;
}

bool ProcessManager::read_proc_cmdline(pid_t pid, std::vector<std::string>* argv) {
  if (argv == nullptr || pid <= 0) {
    return false;
  }
  std::ifstream input("/proc/" + std::to_string(pid) + "/cmdline", std::ios::binary);
  if (!input) {
    return false;
  }
  std::string content((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
  argv->clear();
  std::string item;
  for (const char ch : content) {
    if (ch == '\0') {
      if (!item.empty()) {
        argv->push_back(item);
        item.clear();
      }
    } else {
      item.push_back(ch);
    }
  }
  if (!item.empty()) {
    argv->push_back(item);
  }
  return !argv->empty();
}

bool ProcessManager::proc_cmdline_matches(const ManagedProcessConfig& config, const std::vector<std::string>& argv) {
  const auto expected = expected_argv(config);
  if (argv == expected) {
    return true;
  }

  for (const auto& expected_item : expected) {
    const auto found = std::find(argv.begin(), argv.end(), expected_item);
    if (found == argv.end()) {
      return false;
    }
  }
  return true;
}

void ProcessManager::redirect_to_file(const std::filesystem::path& path, int fd) {
  if (path.empty()) {
    return;
  }
  const int log_fd = open(path.c_str(), O_CREAT | O_WRONLY | O_APPEND, 0644);
  if (log_fd < 0) {
    return;
  }
  (void)dup2(log_fd, fd);
  if (log_fd != fd) {
    (void)close(log_fd);
  }
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

std::filesystem::path ProcessManager::default_metadata_file(const std::string& process_id) const {
  return state_dir_ / (process_id + ".json");
}

std::filesystem::path ProcessManager::default_log_file(
  const ManagedProcessConfig& config, const std::string& stream
) const {
  const auto pid_file = config.pid_file.empty() ? default_pid_file(config.process_id) : config.pid_file;
  return state_dir_ / "logs" / (pid_file.stem().string() + "." + stream + ".log");
}

}  // namespace agent
}  // namespace axon
