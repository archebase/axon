// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_AGENT_ACTION_EXECUTOR_HPP
#define AXON_AGENT_ACTION_EXECUTOR_HPP

#include <nlohmann/json.hpp>

#include <cstddef>
#include <string>

#include "action_registry.hpp"

namespace axon {
namespace agent {

struct ActionExecutionOptions {
  std::size_t max_output_bytes = 64 * 1024;
};

struct ActionExecutionResult {
  std::string action_id;
  std::string status;
  int exit_code = -1;
  bool timed_out = false;
  bool stdout_truncated = false;
  bool stderr_truncated = false;
  std::string stdout_text;
  std::string stderr_text;
  std::string error_summary;
  long duration_ms = 0;

  nlohmann::json to_json() const;
};

class ActionExecutor {
public:
  explicit ActionExecutor(ActionExecutionOptions options = {});

  ActionExecutionResult execute(const ActionDefinition& action, const nlohmann::json& args) const;
  bool validate_args(
    const ActionDefinition& action, const nlohmann::json& args, std::string* error
  ) const;

private:
  struct CapturedOutput {
    std::string text;
    bool truncated = false;
  };

  static bool json_type_matches(const nlohmann::json& value, const std::string& expected);
  static std::string redact_sensitive_output(const std::string& output);
  static std::string current_user_name();
  static bool run_as_allowed(const std::string& run_as, std::string* error);
  CapturedOutput append_output(CapturedOutput output, const char* data, std::size_t size) const;

  ActionExecutionOptions options_;
};

}  // namespace agent
}  // namespace axon

#endif  // AXON_AGENT_ACTION_EXECUTOR_HPP
