// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "action_execution.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace axon {
namespace agent {

namespace {

std::string path_string(const std::filesystem::path& path) {
  return path.lexically_normal().string();
}

bool parse_iso8601_utc(const std::string& value, std::chrono::system_clock::time_point* output) {
  if (value.empty() || output == nullptr) {
    return false;
  }

  auto normalized = value;
  const auto z_pos = normalized.find('Z');
  if (z_pos == std::string::npos) {
    return false;
  }
  const auto dot_pos = normalized.find('.');
  if (dot_pos != std::string::npos && dot_pos < z_pos) {
    normalized.erase(dot_pos, z_pos - dot_pos);
  }

  std::tm tm{};
  std::istringstream input(normalized);
  input >> std::get_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
  if (input.fail()) {
    return false;
  }

#ifdef _WIN32
  const std::time_t time = _mkgmtime(&tm);
#else
  const std::time_t time = timegm(&tm);
#endif
  if (time < 0) {
    return false;
  }
  *output = std::chrono::system_clock::from_time_t(time);
  return true;
}

std::string json_string_or_empty(const nlohmann::json& json, const char* key) {
  return json.contains(key) && json[key].is_string() ? json[key].get<std::string>() : "";
}

bool json_bool_or_false(const nlohmann::json& json, const char* key) {
  return json.contains(key) && json[key].is_boolean() ? json[key].get<bool>() : false;
}

std::optional<int> json_optional_int(const nlohmann::json& json, const char* key) {
  if (!json.contains(key) || json[key].is_null() || !json[key].is_number_integer()) {
    return std::nullopt;
  }
  return json[key].get<int>();
}

std::string lower_copy(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return value;
}

bool is_sensitive_key(const std::string& key) {
  const auto lowered = lower_copy(key);
  const std::array<std::string, 9> markers = {
    "token",
    "password",
    "passwd",
    "secret",
    "api_key",
    "apikey",
    "private_key",
    "certificate",
    "cert"
  };
  return std::any_of(
           markers.begin(),
           markers.end(),
           [&lowered](const std::string& marker) {
             return lowered.find(marker) != std::string::npos;
           }
         ) ||
         lowered.find("key") != std::string::npos;
}

nlohmann::json redact_sensitive_json(const nlohmann::json& value) {
  if (value.is_object()) {
    nlohmann::json redacted = nlohmann::json::object();
    for (const auto& item : value.items()) {
      redacted[item.key()] = is_sensitive_key(item.key()) ? nlohmann::json("[REDACTED]")
                                                          : redact_sensitive_json(item.value());
    }
    return redacted;
  }
  if (value.is_array()) {
    nlohmann::json redacted = nlohmann::json::array();
    for (const auto& item : value) {
      redacted.push_back(redact_sensitive_json(item));
    }
    return redacted;
  }
  return value;
}

}  // namespace

std::string action_execution_now_iso8601() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t time = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  gmtime_r(&time, &tm);

  char buffer[32];
  std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &tm);
  return buffer;
}

bool action_execution_is_expired(const std::string& expires_at, std::string* error) {
  if (expires_at.empty()) {
    return false;
  }

  std::chrono::system_clock::time_point expires_at_time;
  if (!parse_iso8601_utc(expires_at, &expires_at_time)) {
    if (error != nullptr) {
      *error = "expires_at must be an ISO-8601 UTC timestamp";
    }
    return false;
  }

  return std::chrono::system_clock::now() >= expires_at_time;
}

ActionExecutionStore::ActionExecutionStore(std::filesystem::path state_file)
    : state_file_(std::move(state_file)) {}

bool ActionExecutionStore::load_and_recover(std::string* error) {
  records_.clear();
  if (!std::filesystem::exists(state_file_)) {
    return true;
  }

  try {
    std::ifstream input(state_file_);
    if (!input) {
      if (error != nullptr) {
        *error = "failed to read action execution state: " + path_string(state_file_);
      }
      return false;
    }

    const auto root = nlohmann::json::parse(input);
    const auto records = root.contains("records") ? root["records"] : nlohmann::json::array();
    if (!records.is_array()) {
      throw std::runtime_error("records must be an array");
    }

    const auto recovered_at = action_execution_now_iso8601();
    bool recovered = false;
    for (const auto& item : records) {
      auto record = record_from_json(item);
      if (record.request_id.empty()) {
        continue;
      }
      if (is_in_flight(record)) {
        record.status = "failed";
        record.finished_at = recovered_at;
        record.error_summary = "action execution interrupted by agent restart";
        recovered = true;
      }
      records_[record.request_id] = std::move(record);
    }

    return recovered ? persist(error) : true;
  } catch (const std::exception& ex) {
    if (error != nullptr) {
      *error = std::string("failed to load action execution state: ") + ex.what();
    }
    return false;
  }
}

std::optional<ActionExecutionRecord> ActionExecutionStore::get(
  const std::string& request_id
) const {
  const auto it = records_.find(request_id);
  if (it == records_.end()) {
    return std::nullopt;
  }
  return it->second;
}

bool ActionExecutionStore::upsert(const ActionExecutionRecord& record, std::string* error) {
  records_[record.request_id] = record;
  return persist(error);
}

nlohmann::json ActionExecutionStore::to_json() const {
  nlohmann::json records = nlohmann::json::array();
  nlohmann::json counts_by_status = nlohmann::json::object();
  for (const auto& pair : records_) {
    records.push_back(record_to_json(pair.second));
    const auto count = counts_by_status.value(pair.second.status, 0);
    counts_by_status[pair.second.status] = count + 1;
  }

  return {
    {"state_file", path_string(state_file_)},
    {"count", records_.size()},
    {"counts_by_status", counts_by_status},
    {"records", records},
  };
}

bool ActionExecutionStore::persist(std::string* error) const {
  try {
    std::filesystem::create_directories(state_file_.parent_path());
    const auto tmp = state_file_.string() + ".tmp";
    {
      std::ofstream output(tmp);
      if (!output) {
        if (error != nullptr) {
          *error = "failed to write action execution state: " + tmp;
        }
        return false;
      }
      output << nlohmann::json{{"records", to_json()["records"]}}.dump(2) << '\n';
    }
    std::filesystem::rename(tmp, state_file_);
    return true;
  } catch (const std::exception& ex) {
    if (error != nullptr) {
      *error = std::string("failed to persist action execution state: ") + ex.what();
    }
    return false;
  }
}

ActionExecutionRecord ActionExecutionStore::record_from_json(const nlohmann::json& json) {
  ActionExecutionRecord record;
  record.request_id = json_string_or_empty(json, "request_id");
  record.action_id = json_string_or_empty(json, "action_id");
  record.status = json_string_or_empty(json, "status");
  if (record.status.empty()) {
    record.status = "queued";
  }
  record.queued_at = json_string_or_empty(json, "queued_at");
  record.started_at = json_string_or_empty(json, "started_at");
  record.finished_at = json_string_or_empty(json, "finished_at");
  record.expires_at = json_string_or_empty(json, "expires_at");
  record.exit_code = json_optional_int(json, "exit_code");
  record.error_summary = json_string_or_empty(json, "error_summary");
  record.args =
    json.contains("args") && json["args"].is_object() ? json["args"] : nlohmann::json::object();
  record.stdout_text = json_string_or_empty(json, "stdout");
  record.stderr_text = json_string_or_empty(json, "stderr");
  record.stdout_truncated = json_bool_or_false(json, "stdout_truncated");
  record.stderr_truncated = json_bool_or_false(json, "stderr_truncated");
  record.duplicate_count =
    json.contains("duplicate_count") && json["duplicate_count"].is_number_integer()
      ? json["duplicate_count"].get<int>()
      : 0;
  return record;
}

nlohmann::json ActionExecutionStore::record_to_json(const ActionExecutionRecord& record) {
  return {
    {"request_id", record.request_id},
    {"action_id", record.action_id},
    {"status", record.status},
    {"queued_at", record.queued_at},
    {"started_at",
     record.started_at.empty() ? nlohmann::json(nullptr) : nlohmann::json(record.started_at)},
    {"finished_at",
     record.finished_at.empty() ? nlohmann::json(nullptr) : nlohmann::json(record.finished_at)},
    {"expires_at",
     record.expires_at.empty() ? nlohmann::json(nullptr) : nlohmann::json(record.expires_at)},
    {"exit_code",
     record.exit_code.has_value() ? nlohmann::json(record.exit_code.value())
                                  : nlohmann::json(nullptr)},
    {"error_summary", record.error_summary},
    {"args", redact_sensitive_json(record.args)},
    {"stdout", record.stdout_text},
    {"stderr", record.stderr_text},
    {"stdout_truncated", record.stdout_truncated},
    {"stderr_truncated", record.stderr_truncated},
    {"duplicate_count", record.duplicate_count},
  };
}

bool ActionExecutionStore::is_in_flight(const ActionExecutionRecord& record) {
  return record.status == "queued" || record.status == "running";
}

}  // namespace agent
}  // namespace axon
