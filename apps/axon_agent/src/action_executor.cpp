// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "action_executor.hpp"

#include <sys/wait.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <poll.h>
#include <pwd.h>
#include <signal.h>
#include <sstream>
#include <thread>
#include <unistd.h>
#include <unordered_set>
#include <vector>

namespace axon {
namespace agent {

namespace {

constexpr int kPollIntervalMs = 50;

void set_nonblocking(int fd) {
  const int flags = fcntl(fd, F_GETFL, 0);
  if (flags >= 0) {
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
  }
}

void close_fd(int* fd) {
  if (fd != nullptr && *fd >= 0) {
    close(*fd);
    *fd = -1;
  }
}

std::string lower_copy(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return value;
}

bool contains_sensitive_marker(const std::string& line) {
  const auto lowered = lower_copy(line);
  const std::array<std::string, 7> markers = {
    "token", "password", "secret", "api_key", "private_key", "certificate", " key="
  };
  return std::any_of(markers.begin(), markers.end(), [&lowered](const std::string& marker) {
    return lowered.find(marker) != std::string::npos;
  });
}

std::string wait_status_summary(int status) {
  if (WIFEXITED(status)) {
    return "exit code " + std::to_string(WEXITSTATUS(status));
  }
  if (WIFSIGNALED(status)) {
    return "terminated by signal " + std::to_string(WTERMSIG(status));
  }
  return "process ended";
}

}  // namespace

ActionExecutor::ActionExecutor(ActionExecutionOptions options)
    : options_(options) {
  if (options_.max_output_bytes == 0) {
    options_.max_output_bytes = 64 * 1024;
  }
}

nlohmann::json ActionExecutionResult::to_json() const {
  return {
    {"action_id", action_id},
    {"status", status},
    {"exit_code", exit_code},
    {"timed_out", timed_out},
    {"duration_ms", duration_ms},
    {"stdout", stdout_text},
    {"stderr", stderr_text},
    {"stdout_truncated", stdout_truncated},
    {"stderr_truncated", stderr_truncated},
    {"error_summary", error_summary},
  };
}

bool ActionExecutor::validate_args(
  const ActionDefinition& action, const nlohmann::json& args, std::string* error
) const {
  if (!args.is_object()) {
    if (error != nullptr) {
      *error = "action args must be a JSON object";
    }
    return false;
  }

  const auto& schema = action.args_schema;
  if (!schema.is_object()) {
    if (error != nullptr) {
      *error = "action args_schema must be a JSON object";
    }
    return false;
  }

  if (schema.contains("required") && schema["required"].is_array()) {
    for (const auto& required : schema["required"]) {
      if (!required.is_string()) {
        continue;
      }
      const auto key = required.get<std::string>();
      if (!args.contains(key)) {
        if (error != nullptr) {
          *error = "missing required action arg: " + key;
        }
        return false;
      }
    }
  }

  std::unordered_set<std::string> known_properties;
  const auto properties_it = schema.find("properties");
  if (properties_it != schema.end() && properties_it->is_object()) {
    for (const auto& property : properties_it->items()) {
      known_properties.insert(property.key());
      if (!args.contains(property.key()) || !property.value().is_object() ||
          !property.value().contains("type")) {
        continue;
      }
      const auto expected_type = property.value()["type"].get<std::string>();
      if (!json_type_matches(args.at(property.key()), expected_type)) {
        if (error != nullptr) {
          *error = "invalid type for action arg " + property.key() + ": expected " + expected_type;
        }
        return false;
      }
    }
  }

  const bool allow_additional =
    !schema.contains("additionalProperties") || schema["additionalProperties"].get<bool>();
  if (!allow_additional) {
    for (const auto& arg : args.items()) {
      if (known_properties.find(arg.key()) == known_properties.end()) {
        if (error != nullptr) {
          *error = "unknown action arg: " + arg.key();
        }
        return false;
      }
    }
  }

  return true;
}

ActionExecutionResult ActionExecutor::execute(
  const ActionDefinition& action, const nlohmann::json& args
) const {
  ActionExecutionResult result;
  result.action_id = action.id;

  std::string error;
  if (!validate_args(action, args, &error)) {
    result.status = "rejected";
    result.error_summary = error;
    return result;
  }
  if (!run_as_allowed(action.run_as, &error)) {
    result.status = "rejected";
    result.error_summary = error;
    return result;
  }
  if (!std::filesystem::exists(action.command)) {
    result.status = "rejected";
    result.error_summary = "action command does not exist";
    return result;
  }

  int stdin_pipe[2] = {-1, -1};
  int stdout_pipe[2] = {-1, -1};
  int stderr_pipe[2] = {-1, -1};
  if (pipe(stdin_pipe) != 0 || pipe(stdout_pipe) != 0 || pipe(stderr_pipe) != 0) {
    result.status = "failed";
    result.error_summary = "failed to create action process pipes";
    close_fd(&stdin_pipe[0]);
    close_fd(&stdin_pipe[1]);
    close_fd(&stdout_pipe[0]);
    close_fd(&stdout_pipe[1]);
    close_fd(&stderr_pipe[0]);
    close_fd(&stderr_pipe[1]);
    return result;
  }

  const auto started_at = std::chrono::steady_clock::now();
  const pid_t pid = fork();
  if (pid == 0) {
    setsid();
    dup2(stdin_pipe[0], STDIN_FILENO);
    dup2(stdout_pipe[1], STDOUT_FILENO);
    dup2(stderr_pipe[1], STDERR_FILENO);
    close_fd(&stdin_pipe[0]);
    close_fd(&stdin_pipe[1]);
    close_fd(&stdout_pipe[0]);
    close_fd(&stdout_pipe[1]);
    close_fd(&stderr_pipe[0]);
    close_fd(&stderr_pipe[1]);

    std::string action_env = "AXON_ACTION_ID=" + action.id;
    std::vector<char*> argv = {const_cast<char*>(action.command.c_str()), nullptr};
    std::vector<char*> envp = {
      const_cast<char*>("PATH=/usr/sbin:/usr/bin:/sbin:/bin"),
      const_cast<char*>("LANG=C.UTF-8"),
      action_env.data(),
      nullptr,
    };
    execve(action.command.c_str(), argv.data(), envp.data());
    _exit(127);
  }

  close_fd(&stdin_pipe[0]);
  close_fd(&stdout_pipe[1]);
  close_fd(&stderr_pipe[1]);

  if (pid < 0) {
    result.status = "failed";
    result.error_summary = "failed to fork action process";
    close_fd(&stdin_pipe[1]);
    close_fd(&stdout_pipe[0]);
    close_fd(&stderr_pipe[0]);
    return result;
  }

  const auto args_json = args.dump();
  std::size_t written = 0;
  while (written < args_json.size()) {
    const ssize_t n = write(stdin_pipe[1], args_json.data() + written, args_json.size() - written);
    if (n > 0) {
      written += static_cast<std::size_t>(n);
      continue;
    }
    if (n < 0 && errno == EINTR) {
      continue;
    }
    break;
  }
  close_fd(&stdin_pipe[1]);

  set_nonblocking(stdout_pipe[0]);
  set_nonblocking(stderr_pipe[0]);

  CapturedOutput stdout_capture;
  CapturedOutput stderr_capture;
  bool stdout_open = true;
  bool stderr_open = true;
  bool child_exited = false;
  int wait_status = 0;

  const auto deadline = started_at + std::chrono::seconds(std::max(1, action.timeout_sec));
  while (stdout_open || stderr_open || !child_exited) {
    if (!child_exited) {
      const pid_t wait_result = waitpid(pid, &wait_status, WNOHANG);
      if (wait_result == pid) {
        child_exited = true;
      } else if (wait_result < 0 && errno == ECHILD) {
        child_exited = true;
      }
    }

    const auto now = std::chrono::steady_clock::now();
    if (!child_exited && now >= deadline) {
      result.timed_out = true;
      kill(-pid, SIGTERM);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      if (waitpid(pid, &wait_status, WNOHANG) != pid) {
        kill(-pid, SIGKILL);
        waitpid(pid, &wait_status, 0);
      }
      child_exited = true;
    }

    std::array<pollfd, 2> fds = {
      pollfd{stdout_pipe[0], static_cast<short>(stdout_open ? POLLIN : 0), 0},
      pollfd{stderr_pipe[0], static_cast<short>(stderr_open ? POLLIN : 0), 0},
    };
    poll(fds.data(), fds.size(), kPollIntervalMs);

    const auto drain = [&](int* fd, bool* open, CapturedOutput* capture) {
      std::array<char, 4096> buffer{};
      while (*open) {
        const ssize_t n = read(*fd, buffer.data(), buffer.size());
        if (n > 0) {
          *capture = append_output(std::move(*capture), buffer.data(), static_cast<std::size_t>(n));
          continue;
        }
        if (n == 0) {
          *open = false;
          close_fd(fd);
        }
        break;
      }
    };

    if (stdout_open) {
      drain(&stdout_pipe[0], &stdout_open, &stdout_capture);
    }
    if (stderr_open) {
      drain(&stderr_pipe[0], &stderr_open, &stderr_capture);
    }
    if (child_exited && !stdout_open && !stderr_open) {
      break;
    }
  }

  close_fd(&stdout_pipe[0]);
  close_fd(&stderr_pipe[0]);

  result.duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::steady_clock::now() - started_at
  )
                         .count();
  result.stdout_text = redact_sensitive_output(stdout_capture.text);
  result.stderr_text = redact_sensitive_output(stderr_capture.text);
  result.stdout_truncated = stdout_capture.truncated;
  result.stderr_truncated = stderr_capture.truncated;

  if (result.timed_out) {
    result.status = "timed_out";
    result.error_summary = "action timed out after " + std::to_string(action.timeout_sec) + "s";
    return result;
  }

  if (WIFEXITED(wait_status)) {
    result.exit_code = WEXITSTATUS(wait_status);
  }
  result.status = result.exit_code == 0 ? "succeeded" : "failed";
  if (result.status == "failed") {
    result.error_summary = wait_status_summary(wait_status);
  }
  return result;
}

bool ActionExecutor::json_type_matches(const nlohmann::json& value, const std::string& expected) {
  if (expected == "string") return value.is_string();
  if (expected == "integer") return value.is_number_integer();
  if (expected == "number") return value.is_number();
  if (expected == "boolean") return value.is_boolean();
  if (expected == "object") return value.is_object();
  if (expected == "array") return value.is_array();
  if (expected == "null") return value.is_null();
  return true;
}

std::string ActionExecutor::redact_sensitive_output(const std::string& output) {
  std::istringstream input(output);
  std::ostringstream redacted;
  std::string line;
  bool first = true;
  while (std::getline(input, line)) {
    if (!first) {
      redacted << '\n';
    }
    first = false;
    redacted << (contains_sensitive_marker(line) ? "[REDACTED]" : line);
  }
  if (!output.empty() && output.back() == '\n') {
    redacted << '\n';
  }
  return redacted.str();
}

std::string ActionExecutor::current_user_name() {
  const auto uid = geteuid();
  if (const auto* pwd = getpwuid(uid); pwd != nullptr && pwd->pw_name != nullptr) {
    return pwd->pw_name;
  }
  return "";
}

bool ActionExecutor::run_as_allowed(const std::string& run_as, std::string* error) {
  if (run_as.empty() || run_as == "current" || run_as == current_user_name()) {
    return true;
  }
  if (geteuid() == 0 && run_as == "root") {
    return true;
  }
  if (error != nullptr) {
    *error = "action run_as is not supported by current agent user: " + run_as;
  }
  return false;
}

ActionExecutor::CapturedOutput ActionExecutor::append_output(
  CapturedOutput output, const char* data, std::size_t size
) const {
  if (output.text.size() < options_.max_output_bytes) {
    const auto remaining = options_.max_output_bytes - output.text.size();
    output.text.append(data, std::min(remaining, size));
  }
  if (size > 0 && output.text.size() >= options_.max_output_bytes) {
    output.truncated = true;
  }
  return output;
}

}  // namespace agent
}  // namespace axon
