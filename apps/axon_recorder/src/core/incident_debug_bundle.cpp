// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "incident_debug_bundle.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <system_error>
#include <unistd.h>

#include "sensitive_data_filter.hpp"
#include "version.hpp"

#ifndef AXON_RECORDER_GIT_SHA
#define AXON_RECORDER_GIT_SHA "unknown"
#endif

namespace axon {
namespace recorder {

namespace fs = std::filesystem;

namespace {

int64_t now_epoch_ms() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
           std::chrono::system_clock::now().time_since_epoch()
  )
    .count();
}

std::string hostname() {
  char buffer[256];
  if (gethostname(buffer, sizeof(buffer)) != 0) {
    return "";
  }
  buffer[sizeof(buffer) - 1] = '\0';
  return std::string(buffer);
}

}  // namespace

IncidentDebugBundleResult IncidentDebugBundleWriter::create(
  const IncidentDebugBundleRequest& request
) const {
  IncidentDebugBundleResult result;
  result.enabled = request.config.enabled;
  if (!request.config.enabled) {
    return result;
  }

  if (request.mcap_path.empty()) {
    result.success = false;
    result.error_message = "MCAP path is empty";
    return result;
  }

  fs::path mcap_path(request.mcap_path);
  std::error_code ec;
  if (!fs::exists(mcap_path, ec) || ec) {
    result.success = false;
    result.error_message = "MCAP file does not exist: " + request.mcap_path;
    return result;
  }

  fs::path parent =
    request.config.directory.empty() ? mcap_path.parent_path() : fs::path(request.config.directory);
  if (parent.empty()) {
    parent = ".";
  }

  const std::string bundle_name =
    mcap_path.stem().string() + ".incident_debug_bundle." + std::to_string(now_epoch_ms());
  fs::path tmp_dir = parent / (bundle_name + ".tmp");
  fs::path final_dir = parent / bundle_name;

  try {
    fs::create_directories(parent);
    fs::create_directory(tmp_dir);

    fs::copy_file(mcap_path, tmp_dir / "recording.mcap", fs::copy_options::none);

    if (request.sidecar_generated && !request.sidecar_path.empty()) {
      fs::path sidecar_path(request.sidecar_path);
      if (fs::exists(sidecar_path)) {
        fs::copy_file(sidecar_path, tmp_dir / "sidecar.json", fs::copy_options::none);
      }
    }

    nlohmann::json manifest = build_manifest(request, bundle_name);
    manifest = redact_sensitive_json(manifest);

    const fs::path manifest_path = tmp_dir / "manifest.json";
    std::ofstream manifest_stream(manifest_path);
    if (!manifest_stream) {
      throw std::runtime_error("failed to open manifest for writing");
    }
    manifest_stream << manifest.dump(2);
    manifest_stream.close();
    if (!manifest_stream) {
      throw std::runtime_error("failed to flush manifest");
    }

    fs::rename(tmp_dir, final_dir);

    result.success = true;
    result.created = true;
    result.bundle_path = final_dir.string();
    result.manifest_path = (final_dir / "manifest.json").string();
    return result;
  } catch (const std::exception& e) {
    fs::remove_all(tmp_dir, ec);
    result.success = false;
    result.created = false;
    result.error_message = e.what();
    return result;
  }
}

nlohmann::json IncidentDebugBundleWriter::build_manifest(
  const IncidentDebugBundleRequest& request, const std::string& bundle_name
) const {
  nlohmann::json manifest;
  manifest["schema_version"] = "1.0";
  manifest["bundle_name"] = bundle_name;
  manifest["created_at_ms"] = now_epoch_ms();

  manifest["version"]["recorder_version"] = AXON_RECORDER_VERSION;
  manifest["version"]["git_sha"] = AXON_RECORDER_GIT_SHA;

  manifest["runtime"]["hostname"] = hostname();
  manifest["runtime"]["pid"] = static_cast<int>(getpid());
  std::error_code ec;
  manifest["runtime"]["cwd"] = fs::current_path(ec).string();

  manifest["artifacts"]["mcap"]["path"] = "recording.mcap";
  manifest["artifacts"]["mcap"]["original_path"] = request.mcap_path;
  manifest["artifacts"]["mcap"]["file_size_bytes"] = request.mcap_file_size;
  if (!request.checksum_sha256.empty()) {
    manifest["artifacts"]["mcap"]["checksum_sha256"] = request.checksum_sha256;
  }
  manifest["artifacts"]["sidecar"]["enabled"] = request.sidecar_enabled;
  manifest["artifacts"]["sidecar"]["generated"] = request.sidecar_generated;
  if (request.sidecar_generated && !request.sidecar_path.empty()) {
    manifest["artifacts"]["sidecar"]["path"] = "sidecar.json";
    manifest["artifacts"]["sidecar"]["original_path"] = request.sidecar_path;
  }

  manifest["config"]["task_config"] = safe_task_config_json(request.task_config);
  manifest["config"]["recorder_config"] = safe_recorder_config_json(request.recorder_config);
  manifest["diagnostics"] = request.diagnostic_snapshot;

  return manifest;
}

nlohmann::json IncidentDebugBundleWriter::safe_task_config_json(const TaskConfig* config) const {
  if (config == nullptr) {
    return nullptr;
  }

  nlohmann::json j;
  j["task_id"] = config->task_id;
  j["device_id"] = config->device_id;
  j["data_collector_id"] = config->data_collector_id;
  j["order_id"] = config->order_id;
  j["operator_name"] = config->operator_name;
  j["scene"] = config->scene;
  j["subscene"] = config->subscene;
  j["skills"] = config->skills;
  j["factory"] = config->factory;
  j["topics"] = config->topics;
  j["started_at"] = config->started_at;
  return j;
}

nlohmann::json IncidentDebugBundleWriter::safe_recorder_config_json(
  const RecorderConfig* config
) const {
  if (config == nullptr) {
    return nullptr;
  }

  nlohmann::json j;
  j["output_file"] = config->output_file;
  j["output_file_is_explicit"] = config->output_file_is_explicit;
  j["plugin_path"] = config->plugin_path;
  j["plugin_paths_ordered"] = config->plugin_paths_ordered;
  j["auxiliary_plugin_paths"] = config->auxiliary_plugin_paths;
  j["queue_capacity"] = config->queue_capacity;
  j["num_worker_threads"] = config->num_worker_threads;

  j["dataset"]["path"] = config->dataset.path;
  j["dataset"]["mode"] = config->dataset.mode;
  j["dataset"]["stats_file_path"] = config->dataset.stats_file_path;

  j["recording"]["profile"] = config->recording.profile;
  j["recording"]["compression"] = config->recording.compression;
  j["recording"]["compression_level"] = config->recording.compression_level;
  j["recording"]["sidecar_json_enabled"] = config->recording.sidecar_json_enabled;
  j["recording"]["subtract_pause_duration_from_timestamps"] =
    config->recording.subtract_pause_duration_from_timestamps;
  j["recording"]["enforce_monotonic_timestamps_per_topic"] =
    config->recording.enforce_monotonic_timestamps_per_topic;

  j["rpc"]["mode"] = config->rpc.mode == RpcMode::WS_CLIENT ? "ws_client" : "http_server";
  j["rpc"]["ws_client"]["url"] = config->rpc.ws_client.url;
  j["rpc"]["ws_client"]["time_gap_check_enabled"] = config->rpc.ws_client.time_gap_check_enabled;
  j["rpc"]["ws_client"]["time_gap_warning_threshold_ms"] =
    config->rpc.ws_client.time_gap_warning_threshold_ms;
  j["rpc"]["ws_client"]["time_gap_critical_threshold_ms"] =
    config->rpc.ws_client.time_gap_critical_threshold_ms;
  j["rpc"]["ws_client"]["time_gap_stale_after_ms"] = config->rpc.ws_client.time_gap_stale_after_ms;

  j["incident_bundle"]["enabled"] = config->incident_bundle.enabled;
  j["incident_bundle"]["directory"] = config->incident_bundle.directory;
  return redact_sensitive_json(j);
}

}  // namespace recorder
}  // namespace axon
