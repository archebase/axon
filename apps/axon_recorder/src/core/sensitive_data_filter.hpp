// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_RECORDER_SENSITIVE_DATA_FILTER_HPP
#define AXON_RECORDER_SENSITIVE_DATA_FILTER_HPP

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <string>

namespace axon {
namespace recorder {

inline std::string lowercase_copy(const std::string& value) {
  std::string out = value;
  std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return out;
}

inline bool is_sensitive_key(const std::string& key) {
  const std::string lower = lowercase_copy(key);
  return lower.find("token") != std::string::npos || lower.find("secret") != std::string::npos ||
         lower.find("password") != std::string::npos ||
         lower.find("credential") != std::string::npos ||
         lower.find("authorization") != std::string::npos ||
         lower.find("access_key") != std::string::npos ||
         lower.find("access-key") != std::string::npos ||
         lower.find("callback_url") != std::string::npos;
}

inline nlohmann::json redact_sensitive_json(const nlohmann::json& input) {
  if (input.is_object()) {
    nlohmann::json out = nlohmann::json::object();
    for (const auto& [key, value] : input.items()) {
      out[key] =
        is_sensitive_key(key) ? nlohmann::json("[REDACTED]") : redact_sensitive_json(value);
    }
    return out;
  }

  if (input.is_array()) {
    nlohmann::json out = nlohmann::json::array();
    for (const auto& value : input) {
      out.push_back(redact_sensitive_json(value));
    }
    return out;
  }

  return input;
}

}  // namespace recorder
}  // namespace axon

#endif  // AXON_RECORDER_SENSITIVE_DATA_FILTER_HPP
