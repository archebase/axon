// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_RECORDER_INCIDENT_DEBUG_BUNDLE_HPP
#define AXON_RECORDER_INCIDENT_DEBUG_BUNDLE_HPP

#include <nlohmann/json.hpp>

#include <cstdint>
#include <string>

#include "recorder.hpp"

namespace axon {
namespace recorder {

struct IncidentDebugBundleRequest {
  IncidentBundleConfig config;
  std::string mcap_path;
  uint64_t mcap_file_size = 0;
  std::string checksum_sha256;
  std::string sidecar_path;
  bool sidecar_enabled = true;
  bool sidecar_generated = false;
  const TaskConfig* task_config = nullptr;
  const RecorderConfig* recorder_config = nullptr;
  nlohmann::json diagnostic_snapshot = nlohmann::json::object();
};

struct IncidentDebugBundleResult {
  bool enabled = false;
  bool success = true;
  bool created = false;
  std::string bundle_path;
  std::string manifest_path;
  std::string error_message;
};

class IncidentDebugBundleWriter {
public:
  IncidentDebugBundleResult create(const IncidentDebugBundleRequest& request) const;

private:
  nlohmann::json build_manifest(
    const IncidentDebugBundleRequest& request, const std::string& bundle_name
  ) const;
  nlohmann::json safe_task_config_json(const TaskConfig* config) const;
  nlohmann::json safe_recorder_config_json(const RecorderConfig* config) const;
};

}  // namespace recorder
}  // namespace axon

#endif  // AXON_RECORDER_INCIDENT_DEBUG_BUNDLE_HPP
