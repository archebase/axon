// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_AGENT_ACTION_EXECUTION_HPP
#define AXON_AGENT_ACTION_EXECUTION_HPP

#include <nlohmann/json.hpp>

#include <filesystem>
#include <map>
#include <optional>
#include <string>

#include "action_registry.hpp"

namespace axon {
namespace agent {

struct ActionExecutionRecord {
  std::string request_id;
  std::string action_id;
  std::string status = "queued";
  std::string queued_at;
  std::string started_at;
  std::string finished_at;
  std::string expires_at;
  std::optional<int> exit_code;
  std::string error_summary;
  nlohmann::json args = nlohmann::json::object();
  std::string stdout_text;
  std::string stderr_text;
  bool stdout_truncated = false;
  bool stderr_truncated = false;
  int duplicate_count = 0;
};

class ActionExecutionStore {
public:
  explicit ActionExecutionStore(std::filesystem::path state_file);

  bool load_and_recover(std::string* error);
  std::optional<ActionExecutionRecord> get(const std::string& request_id) const;
  bool upsert(const ActionExecutionRecord& record, std::string* error);
  nlohmann::json to_json() const;
  static nlohmann::json record_to_json(const ActionExecutionRecord& record);

private:
  bool persist(std::string* error) const;
  static ActionExecutionRecord record_from_json(const nlohmann::json& json);
  static bool is_in_flight(const ActionExecutionRecord& record);

  std::filesystem::path state_file_;
  std::map<std::string, ActionExecutionRecord> records_;
};

std::string action_execution_now_iso8601();
bool action_execution_is_expired(const std::string& expires_at, std::string* error);

}  // namespace agent
}  // namespace axon

#endif  // AXON_AGENT_ACTION_EXECUTION_HPP
