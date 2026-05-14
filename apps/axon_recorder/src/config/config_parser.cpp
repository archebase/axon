// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "config_parser.hpp"

#include <nlohmann/json.hpp>

#include <fstream>
#include <sstream>

#include "../core/recorder.hpp"

// Logging infrastructure
#define AXON_LOG_COMPONENT "config_parser"
#include <axon_log_init.hpp>
#include <axon_log_macros.hpp>

using axon::logging::kv;

namespace axon {
namespace recorder {

// ============================================================================
// YAML to JSON conversion helper
// ============================================================================

static nlohmann::json yaml_node_to_json(const YAML::Node& node) {
  switch (node.Type()) {
    case YAML::NodeType::Null:
      return nullptr;
    case YAML::NodeType::Scalar: {
      // Try integer, then double, then bool, then string
      try {
        return node.as<int64_t>();
      } catch (...) {
      }
      try {
        return node.as<double>();
      } catch (...) {
      }
      try {
        return node.as<bool>();
      } catch (...) {
      }
      return node.as<std::string>();
    }
    case YAML::NodeType::Sequence: {
      nlohmann::json arr = nlohmann::json::array();
      for (const auto& item : node) {
        arr.push_back(yaml_node_to_json(item));
      }
      return arr;
    }
    case YAML::NodeType::Map: {
      nlohmann::json obj = nlohmann::json::object();
      for (const auto& kv : node) {
        obj[kv.first.as<std::string>()] = yaml_node_to_json(kv.second);
      }
      return obj;
    }
    default:
      return nullptr;
  }
}

// ============================================================================
// ConfigParser Implementation
// ============================================================================

bool ConfigParser::load_from_file(const std::string& path, RecorderConfig& config) {
  std::ifstream file(path);
  if (!file.good()) {
    last_error_ = "Config file not found or not readable: " + path;
    return false;
  }

  try {
    YAML::Node yaml = YAML::LoadFile(path);
    return load_from_string(YAML::Dump(yaml), config);
  } catch (const YAML::Exception& e) {
    last_error_ = "Failed to parse YAML file: " + std::string(e.what());
    return false;
  }
}

bool ConfigParser::load_from_string(const std::string& yaml_content, RecorderConfig& config) {
  try {
    YAML::Node node = YAML::Load(yaml_content);

    // Hybrid recording: ordered plugin list (exclusive with plugin.path + auxiliary_paths)
    if (node["plugins"] && node["plugins"].IsSequence()) {
      config.plugin_paths_ordered.clear();
      for (const auto& item : node["plugins"]) {
        std::string p;
        if (item.IsMap() && item["path"]) {
          p = item["path"].as<std::string>();
        } else if (item.IsScalar()) {
          p = item.as<std::string>();
        }
        if (!p.empty()) {
          config.plugin_paths_ordered.push_back(p);
        }
      }
    }

    // Parse plugin configuration (optional, for backwards compatibility)
    if (node["plugin"]) {
      if (config.plugin_paths_ordered.empty() && node["plugin"]["path"]) {
        config.plugin_path = node["plugin"]["path"].as<std::string>();
      }
      if (config.plugin_paths_ordered.empty() && node["plugin"]["auxiliary_paths"] &&
          node["plugin"]["auxiliary_paths"].IsSequence()) {
        for (const auto& ap : node["plugin"]["auxiliary_paths"]) {
          config.auxiliary_plugin_paths.push_back(ap.as<std::string>());
        }
      }
    }

    // Serialize the entire YAML document as JSON for the plugin's init() call.
    // The plugin is responsible for extracting the section it needs (e.g. "udp").
    config.plugin_config = yaml_node_to_json(node).dump();

    // Auto-populate subscriptions from UDP streams if no subscriptions are explicitly configured.
    // This ensures per-topic workers are created and the message callback is registered.
    if (config.subscriptions.empty() && node["udp"] && node["udp"]["streams"]) {
      for (const auto& stream : node["udp"]["streams"]) {
        bool enabled = true;
        if (stream["enabled"]) {
          enabled = stream["enabled"].as<bool>();
        }
        if (!enabled) {
          continue;
        }
        std::string topic;
        if (stream["topic"]) {
          topic = stream["topic"].as<std::string>();
        }
        if (topic.empty()) {
          std::string name = stream["name"] ? stream["name"].as<std::string>() : "(unnamed)";
          AXON_LOG_WARN("Skipping UDP stream with empty topic: " << kv("name", name));
          continue;
        }
        SubscriptionConfig sub;
        sub.topic_name = topic;
        sub.message_type = "axon_udp/json";
        sub.batch_size = 1;
        sub.flush_interval_ms = 100;
        config.subscriptions.push_back(sub);
      }
    }

    // Parse dataset config
    if (node["dataset"]) {
      parse_dataset(node["dataset"], config.dataset);
      // Copy dataset.output_file to config.output_file for backward compatibility
      if (node["dataset"]["output_file"]) {
        config.output_file = node["dataset"]["output_file"].as<std::string>();
        config.output_file_is_explicit = true;
      }
      // Map dataset.queue_size to queue_capacity (per-topic SPSC queue depth).
      // This field was documented in default_config_ros1.yaml but previously
      // silently ignored because parse_dataset() only touches DatasetConfig.
      if (node["dataset"]["queue_size"]) {
        config.queue_capacity = node["dataset"]["queue_size"].as<size_t>();
      }
    }

    // Parse subscriptions
    if (node["subscriptions"]) {
      parse_subscriptions(node["subscriptions"], config.subscriptions);
    }

    // Parse recording config
    if (node["recording"]) {
      parse_recording(node["recording"], config.recording);
    }

    // Parse logging config
    if (node["logging"]) {
      parse_logging(node["logging"], config.logging);
    }

    // Parse upload config
    if (node["upload"]) {
      parse_upload(node["upload"], config.upload);
    }

    if (node["metadata"]) {
      const auto& metadata = node["metadata"];
      if (metadata["sidecar"]) {
        const auto& sidecar = metadata["sidecar"];
        if (sidecar.IsMap() && sidecar["enabled"]) {
          config.recording.sidecar_json_enabled = sidecar["enabled"].as<bool>();
        } else if (sidecar.IsScalar()) {
          config.recording.sidecar_json_enabled = sidecar.as<bool>();
        }
      }
      if (metadata["sidecar_enabled"]) {
        config.recording.sidecar_json_enabled = metadata["sidecar_enabled"].as<bool>();
      }
      if (metadata["sidecar_generation_mode"]) {
        const std::string mode = metadata["sidecar_generation_mode"].as<std::string>();
        config.recording.sidecar_json_enabled =
          mode != "disabled" && mode != "off" && mode != "none";
      }
      if (metadata["incident_bundle"]) {
        parse_incident_bundle(metadata["incident_bundle"], config.incident_bundle);
      }
      if (metadata["incident_debug_bundle"]) {
        parse_incident_bundle(metadata["incident_debug_bundle"], config.incident_bundle);
      }
    }

    if (node["incident_bundle"]) {
      parse_incident_bundle(node["incident_bundle"], config.incident_bundle);
    }
    if (node["incident_debug_bundle"]) {
      parse_incident_bundle(node["incident_debug_bundle"], config.incident_bundle);
    }

    // Parse HTTP server config
    if (node["http_server"]) {
      parse_http_server(node["http_server"], config.http_server);
    }

    // Parse RPC mode config (new unified config)
    if (node["rpc"]) {
      parse_rpc(node["rpc"], config.rpc);
    }

    return true;
  } catch (const YAML::Exception& e) {
    last_error_ = "Failed to parse YAML content: " + std::string(e.what());
    return false;
  }
}

bool ConfigParser::save_to_file(const std::string& path, const RecorderConfig& config) {
  try {
    YAML::Node node;

    // Dataset
    node["dataset"]["path"] = config.dataset.path;
    node["dataset"]["mode"] = config.dataset.mode;
    node["dataset"]["stats_file_path"] = config.dataset.stats_file_path;

    // Subscriptions
    node["subscriptions"] = YAML::Node(YAML::NodeType::Sequence);
    for (const auto& subscription : config.subscriptions) {
      YAML::Node subscription_node;
      subscription_node["name"] = subscription.topic_name;
      subscription_node["message_type"] = subscription.message_type;
      subscription_node["batch_size"] = subscription.batch_size;
      subscription_node["flush_interval_ms"] = subscription.flush_interval_ms;
      node["subscriptions"].push_back(subscription_node);
    }

    // Recording
    node["recording"]["max_disk_usage_gb"] = config.recording.max_disk_usage_gb;
    node["recording"]["disk_usage"]["enabled"] = config.recording.disk_usage.enabled;
    node["recording"]["disk_usage"]["warn_usage_gb"] = config.recording.disk_usage.warn_usage_gb;
    node["recording"]["disk_usage"]["hard_limit_gb"] = config.recording.disk_usage.hard_limit_gb;
    node["recording"]["disk_usage"]["max_task_size_gb"] =
      config.recording.disk_usage.max_task_size_gb;
    node["recording"]["disk_usage"]["cleanup_enabled"] =
      config.recording.disk_usage.cleanup_enabled;
    node["recording"]["disk_usage"]["cleanup_target_gb"] =
      config.recording.disk_usage.cleanup_target_gb;
    node["recording"]["disk_usage"]["cleanup_min_age_sec"] =
      config.recording.disk_usage.cleanup_min_age_sec;
    node["recording"]["disk_usage"]["cleanup_upload_backlog"] =
      config.recording.disk_usage.cleanup_upload_backlog;
    node["recording"]["sidecar"]["enabled"] = config.recording.sidecar_json_enabled;
    node["metadata"]["incident_bundle"]["enabled"] = config.incident_bundle.enabled;
    node["metadata"]["incident_bundle"]["directory"] = config.incident_bundle.directory;

    std::ofstream file(path);
    file << node;
    return true;
  } catch (const std::exception& e) {
    AXON_LOG_ERROR("Failed to save config" << ::axon::logging::kv("error", e.what()));
    return false;
  }
}

bool ConfigParser::parse_dataset(const YAML::Node& node, DatasetConfig& dataset) {
  if (node["path"]) {
    dataset.path = node["path"].as<std::string>();
  }
  if (node["mode"]) {
    dataset.mode = node["mode"].as<std::string>();
  }
  if (node["stats_file_path"]) {
    dataset.stats_file_path = node["stats_file_path"].as<std::string>();
  }
  return true;
}

bool ConfigParser::parse_subscriptions(
  const YAML::Node& node, std::vector<SubscriptionConfig>& subscriptions
) {
  if (!node.IsSequence()) {
    last_error_ = "Subscriptions must be a sequence";
    return false;
  }

  subscriptions.clear();
  for (const auto& subscription_node : node) {
    SubscriptionConfig subscription;
    if (subscription_node["name"]) {
      subscription.topic_name = subscription_node["name"].as<std::string>();
    }
    if (subscription_node["message_type"]) {
      subscription.message_type = subscription_node["message_type"].as<std::string>();
    }
    if (subscription_node["batch_size"]) {
      subscription.batch_size = subscription_node["batch_size"].as<size_t>();
    }
    if (subscription_node["flush_interval_ms"]) {
      subscription.flush_interval_ms = subscription_node["flush_interval_ms"].as<int>();
    }
    // Parse depth_compression section (can be boolean or object)
    if (subscription_node["depth_compression"]) {
      const auto& dc_node = subscription_node["depth_compression"];
      // Check if it's a boolean (simple form: depth_compression: true)
      if (dc_node.IsScalar()) {
        subscription.depth_compression.enabled = dc_node.as<bool>();
      } else if (dc_node.IsMap()) {
        // Object form: depth_compression: { enabled: true, level: fast }
        if (dc_node["enabled"]) {
          subscription.depth_compression.enabled = dc_node["enabled"].as<bool>();
        }
        if (dc_node["level"]) {
          subscription.depth_compression.level = dc_node["level"].as<std::string>();
        }
      }
    }
    subscriptions.push_back(subscription);
  }
  return true;
}

bool ConfigParser::parse_recording(const YAML::Node& node, RecordingConfig& recording) {
  bool hard_limit_set = false;
  bool warn_limit_set = false;

  if (node["max_disk_usage_gb"]) {
    recording.max_disk_usage_gb = node["max_disk_usage_gb"].as<double>();
    recording.disk_usage.hard_limit_gb = recording.max_disk_usage_gb;
    hard_limit_set = true;
  }
  if (node["warn_disk_usage_gb"]) {
    recording.disk_usage.warn_usage_gb = node["warn_disk_usage_gb"].as<double>();
    warn_limit_set = true;
  }
  if (node["hard_disk_usage_gb"]) {
    recording.disk_usage.hard_limit_gb = node["hard_disk_usage_gb"].as<double>();
    recording.max_disk_usage_gb = recording.disk_usage.hard_limit_gb;
    hard_limit_set = true;
  }
  if (node["max_task_size_gb"]) {
    recording.disk_usage.max_task_size_gb = node["max_task_size_gb"].as<double>();
  }

  auto parse_disk_usage_node = [&](const YAML::Node& disk_node) {
    if (!disk_node || !disk_node.IsMap()) {
      return;
    }
    if (disk_node["enabled"]) {
      recording.disk_usage.enabled = disk_node["enabled"].as<bool>();
    }
    if (disk_node["warn_usage_gb"]) {
      recording.disk_usage.warn_usage_gb = disk_node["warn_usage_gb"].as<double>();
      warn_limit_set = true;
    }
    if (disk_node["warn_gb"]) {
      recording.disk_usage.warn_usage_gb = disk_node["warn_gb"].as<double>();
      warn_limit_set = true;
    }
    if (disk_node["hard_limit_gb"]) {
      recording.disk_usage.hard_limit_gb = disk_node["hard_limit_gb"].as<double>();
      recording.max_disk_usage_gb = recording.disk_usage.hard_limit_gb;
      hard_limit_set = true;
    }
    if (disk_node["hard_gb"]) {
      recording.disk_usage.hard_limit_gb = disk_node["hard_gb"].as<double>();
      recording.max_disk_usage_gb = recording.disk_usage.hard_limit_gb;
      hard_limit_set = true;
    }
    if (disk_node["max_task_size_gb"]) {
      recording.disk_usage.max_task_size_gb = disk_node["max_task_size_gb"].as<double>();
    }
    if (disk_node["cleanup_enabled"]) {
      recording.disk_usage.cleanup_enabled = disk_node["cleanup_enabled"].as<bool>();
    }
    if (disk_node["cleanup_target_gb"]) {
      recording.disk_usage.cleanup_target_gb = disk_node["cleanup_target_gb"].as<double>();
    }
    if (disk_node["cleanup_min_age_sec"]) {
      recording.disk_usage.cleanup_min_age_sec = disk_node["cleanup_min_age_sec"].as<int>();
    }
    if (disk_node["cleanup_upload_backlog"]) {
      recording.disk_usage.cleanup_upload_backlog = disk_node["cleanup_upload_backlog"].as<bool>();
    }
  };

  if (node["disk_usage"]) {
    parse_disk_usage_node(node["disk_usage"]);
  }
  if (node["disk_limits"]) {
    parse_disk_usage_node(node["disk_limits"]);
  }

  if (hard_limit_set && !warn_limit_set && recording.disk_usage.hard_limit_gb > 0.0 &&
      recording.disk_usage.warn_usage_gb > recording.disk_usage.hard_limit_gb) {
    recording.disk_usage.warn_usage_gb = recording.disk_usage.hard_limit_gb * 0.8;
  }
  if (node["compression"]) {
    recording.compression = node["compression"].as<std::string>();
  }
  if (node["compression_level"]) {
    recording.compression_level = node["compression_level"].as<int>();
  }
  if (node["profile"]) {
    recording.profile = node["profile"].as<std::string>();
  }

  // Parse schema search paths for message definition resolution
  if (node["schema_search_paths"]) {
    const auto& paths_node = node["schema_search_paths"];
    if (paths_node.IsSequence()) {
      for (const auto& path_node : paths_node) {
        recording.schema_search_paths.push_back(path_node.as<std::string>());
      }
    }
  }

  if (node["subtract_pause_duration_from_timestamps"]) {
    recording.subtract_pause_duration_from_timestamps =
      node["subtract_pause_duration_from_timestamps"].as<bool>();
  }
  if (node["enforce_monotonic_timestamps_per_topic"]) {
    recording.enforce_monotonic_timestamps_per_topic =
      node["enforce_monotonic_timestamps_per_topic"].as<bool>();
  }
  if (node["sidecar_enabled"]) {
    recording.sidecar_json_enabled = node["sidecar_enabled"].as<bool>();
  }
  if (node["sidecar_json_enabled"]) {
    recording.sidecar_json_enabled = node["sidecar_json_enabled"].as<bool>();
  }
  if (node["sidecar_generation_mode"]) {
    const std::string mode = node["sidecar_generation_mode"].as<std::string>();
    recording.sidecar_json_enabled = mode != "disabled" && mode != "off" && mode != "none";
  }
  if (node["sidecar"]) {
    const auto& sidecar = node["sidecar"];
    if (sidecar.IsMap() && sidecar["enabled"]) {
      recording.sidecar_json_enabled = sidecar["enabled"].as<bool>();
    } else if (sidecar.IsScalar()) {
      recording.sidecar_json_enabled = sidecar.as<bool>();
    }
  }

  return true;
}

bool ConfigParser::parse_logging(const YAML::Node& node, LoggingConfig& logging) {
  // Parse console section
  if (node["console"]) {
    const auto& console = node["console"];
    if (console["enabled"]) {
      logging.console_enabled = console["enabled"].as<bool>();
    }
    if (console["colors"]) {
      logging.console_colors = console["colors"].as<bool>();
    }
    if (console["level"]) {
      logging.console_level = console["level"].as<std::string>();
    }
  }

  // Parse file section
  if (node["file"]) {
    const auto& file = node["file"];
    if (file["enabled"]) {
      logging.file_enabled = file["enabled"].as<bool>();
    }
    if (file["level"]) {
      logging.file_level = file["level"].as<std::string>();
    }
    if (file["directory"]) {
      logging.file_directory = file["directory"].as<std::string>();
    }
    if (file["pattern"]) {
      logging.file_pattern = file["pattern"].as<std::string>();
    }
    if (file["format"]) {
      logging.file_format = file["format"].as<std::string>();
    }
    if (file["rotation_size_mb"]) {
      logging.rotation_size_mb = file["rotation_size_mb"].as<size_t>();
    }
    if (file["max_files"]) {
      logging.max_files = file["max_files"].as<size_t>();
    }
    if (file["rotate_at_midnight"]) {
      logging.rotate_at_midnight = file["rotate_at_midnight"].as<bool>();
    }
  }

  return true;
}

bool ConfigParser::parse_upload(const YAML::Node& node, UploadConfig& upload) {
  if (node["enabled"]) {
    upload.enabled = node["enabled"].as<bool>();
  }

  // Parse S3 section
  if (node["s3"]) {
    const auto& s3 = node["s3"];
    if (s3["endpoint_url"]) {
      upload.s3.endpoint_url = s3["endpoint_url"].as<std::string>();
    }
    if (s3["bucket"]) {
      upload.s3.bucket = s3["bucket"].as<std::string>();
    }
    if (s3["region"]) {
      upload.s3.region = s3["region"].as<std::string>();
    }
    if (s3["use_ssl"]) {
      upload.s3.use_ssl = s3["use_ssl"].as<bool>();
    }
    if (s3["verify_ssl"]) {
      upload.s3.verify_ssl = s3["verify_ssl"].as<bool>();
    }
  }

  // Parse retry section
  if (node["retry"]) {
    const auto& retry = node["retry"];
    if (retry["max_retries"]) {
      upload.retry.max_retries = retry["max_retries"].as<int>();
    }
    if (retry["initial_delay_ms"]) {
      upload.retry.initial_delay_ms = retry["initial_delay_ms"].as<int>();
    }
    if (retry["max_delay_ms"]) {
      upload.retry.max_delay_ms = retry["max_delay_ms"].as<int>();
    }
    if (retry["exponential_base"]) {
      upload.retry.exponential_base = retry["exponential_base"].as<double>();
    }
    if (retry["jitter"]) {
      upload.retry.jitter = retry["jitter"].as<bool>();
    }
  }

  // Parse other upload settings
  if (node["num_workers"]) {
    upload.num_workers = node["num_workers"].as<int>();
  }
  if (node["state_db_path"]) {
    upload.state_db_path = node["state_db_path"].as<std::string>();
  }
  if (node["delete_after_upload"]) {
    upload.delete_after_upload = node["delete_after_upload"].as<bool>();
  }
  if (node["failed_uploads_dir"]) {
    upload.failed_uploads_dir = node["failed_uploads_dir"].as<std::string>();
  }
  if (node["warn_pending_gb"]) {
    upload.warn_pending_gb = node["warn_pending_gb"].as<double>();
  }
  if (node["alert_pending_gb"]) {
    upload.alert_pending_gb = node["alert_pending_gb"].as<double>();
  }

  return true;
}

bool ConfigParser::parse_incident_bundle(
  const YAML::Node& node, IncidentBundleConfig& incident_bundle
) {
  if (node["enabled"]) {
    incident_bundle.enabled = node["enabled"].as<bool>();
  }
  if (node["directory"]) {
    incident_bundle.directory = node["directory"].as<std::string>();
  }
  if (node["path"]) {
    incident_bundle.directory = node["path"].as<std::string>();
  }

  return true;
}

bool ConfigParser::parse_http_server(const YAML::Node& node, HttpServerConfig& http_server) {
  if (node["host"]) {
    http_server.host = node["host"].as<std::string>();
  }
  if (node["port"]) {
    http_server.port = node["port"].as<uint16_t>();
  }
  if (node["auth_token"]) {
    http_server.auth_token = node["auth_token"].as<std::string>();
  }

  return true;
}

bool ConfigParser::parse_rpc(const YAML::Node& node, RpcModeConfig& rpc) {
  // Parse mode (default: http_server)
  if (node["mode"]) {
    std::string mode_str = node["mode"].as<std::string>();
    if (mode_str == "http_server") {
      rpc.mode = RpcMode::HTTP_SERVER;
    } else if (mode_str == "ws_client") {
      rpc.mode = RpcMode::WS_CLIENT;
    } else {
      last_error_ = "Invalid rpc.mode: " + mode_str + ". Must be 'http_server' or 'ws_client'";
      return false;
    }
  }

  // Parse ws_client section (only used when mode == ws_client)
  if (node["ws_client"]) {
    const auto& ws = node["ws_client"];
    if (ws["url"]) {
      rpc.ws_client.url = ws["url"].as<std::string>();
    }
    if (ws["auth_token"]) {
      rpc.ws_client.auth_token = ws["auth_token"].as<std::string>();
    }
    if (ws["reconnect_initial_delay_ms"]) {
      rpc.ws_client.reconnect_initial_delay_ms = ws["reconnect_initial_delay_ms"].as<int>();
    }
    if (ws["reconnect_backoff_multiplier"]) {
      rpc.ws_client.reconnect_backoff_multiplier = ws["reconnect_backoff_multiplier"].as<double>();
    }
    if (ws["reconnect_max_delay_ms"]) {
      rpc.ws_client.reconnect_max_delay_ms = ws["reconnect_max_delay_ms"].as<int>();
    }
    if (ws["reconnect_jitter_factor"]) {
      rpc.ws_client.reconnect_jitter_factor = ws["reconnect_jitter_factor"].as<double>();
    }
    if (ws["ping_interval_ms"]) {
      rpc.ws_client.ping_interval_ms = ws["ping_interval_ms"].as<int>();
    }
    if (ws["time_gap_check_enabled"]) {
      rpc.ws_client.time_gap_check_enabled = ws["time_gap_check_enabled"].as<bool>();
    }
    if (ws["time_gap_warning_threshold_ms"]) {
      rpc.ws_client.time_gap_warning_threshold_ms =
        ws["time_gap_warning_threshold_ms"].as<int64_t>();
    }
    if (ws["time_gap_critical_threshold_ms"]) {
      rpc.ws_client.time_gap_critical_threshold_ms =
        ws["time_gap_critical_threshold_ms"].as<int64_t>();
    }
    if (ws["time_gap_stale_after_ms"]) {
      rpc.ws_client.time_gap_stale_after_ms = ws["time_gap_stale_after_ms"].as<int64_t>();
    }
  }

  return true;
}

bool ConfigParser::validate(const RecorderConfig& config, std::string& error_msg) {
  // Validate upload config first if enabled (to get specific error messages)
  if (config.upload.enabled) {
    if (!validate_upload_config(config.upload, error_msg)) {
      return false;
    }
  }

  // Then validate basic config
  if (config.dataset.path.empty()) {
    error_msg = "Dataset path is empty";
    return false;
  }

  if (config.subscriptions.empty()) {
    error_msg = "No subscriptions configured";
    return false;
  }

  for (const auto& subscription : config.subscriptions) {
    if (subscription.topic_name.empty()) {
      error_msg = "Subscription topic_name is empty";
      return false;
    }
    if (subscription.message_type.empty()) {
      error_msg = "Subscription message_type is empty";
      return false;
    }
    if (subscription.batch_size == 0) {
      error_msg = "Subscription batch_size must be > 0";
      return false;
    }
  }

  if (config.dataset.mode != "create" && config.dataset.mode != "append") {
    error_msg = "Dataset mode must be 'create' or 'append'";
    return false;
  }

  const auto& disk = config.recording.disk_usage;
  if (disk.enabled) {
    if (disk.warn_usage_gb < 0.0 || disk.hard_limit_gb < 0.0 || disk.max_task_size_gb < 0.0 ||
        disk.cleanup_target_gb < 0.0) {
      error_msg = "Disk usage thresholds must be >= 0";
      return false;
    }

    if (disk.hard_limit_gb > 0.0 && disk.warn_usage_gb > disk.hard_limit_gb) {
      error_msg = "Disk usage warn_usage_gb must be <= hard_limit_gb";
      return false;
    }

    if (disk.cleanup_min_age_sec < 0) {
      error_msg = "Disk cleanup_min_age_sec must be >= 0";
      return false;
    }

    if (disk.cleanup_enabled && disk.hard_limit_gb > 0.0 &&
        disk.cleanup_target_gb >= disk.hard_limit_gb) {
      error_msg = "Disk cleanup_target_gb must be < hard_limit_gb when cleanup is enabled";
      return false;
    }
  }

  return true;
}

bool ConfigParser::validate_upload_config(const UploadConfig& upload, std::string& error_msg) {
  if (!upload.enabled) {
    return true;  // Nothing to validate if disabled
  }

  // Validate S3 bucket (required)
  if (upload.s3.bucket.empty()) {
    error_msg = "Upload enabled but s3.bucket is not configured";
    return false;
  }

  // Validate endpoint_url format if provided
  if (!upload.s3.endpoint_url.empty()) {
    // Basic URL validation - should start with http:// or https://
    if (upload.s3.endpoint_url.find("http://") != 0 &&
        upload.s3.endpoint_url.find("https://") != 0) {
      error_msg = "Invalid s3.endpoint_url - must start with http:// or https://";
      return false;
    }
  }

  // Validate num_workers
  if (upload.num_workers < 1 || upload.num_workers > 16) {
    error_msg = "Invalid num_workers - must be between 1 and 16";
    return false;
  }

  // Validate retry configuration
  if (upload.retry.max_retries < 0 || upload.retry.max_retries > 100) {
    error_msg = "Invalid retry.max_retries - must be between 0 and 100";
    return false;
  }

  if (upload.retry.initial_delay_ms < 0) {
    error_msg = "Invalid retry.initial_delay_ms - must be >= 0";
    return false;
  }

  if (upload.retry.max_delay_ms < upload.retry.initial_delay_ms) {
    error_msg = "Invalid retry.max_delay_ms - must be >= initial_delay_ms";
    return false;
  }

  // Validate backpressure thresholds
  if (upload.warn_pending_gb < 0 || upload.alert_pending_gb < 0) {
    error_msg = "Invalid backpressure thresholds - must be >= 0";
    return false;
  }

  if (upload.alert_pending_gb < upload.warn_pending_gb) {
    error_msg = "Invalid backpressure thresholds - alert_pending_gb must be >= warn_pending_gb";
    return false;
  }

  // Validate state_db_path is not empty
  if (upload.state_db_path.empty()) {
    error_msg = "Upload enabled but state_db_path is empty";
    return false;
  }

  return true;
}

void convert_logging_config(
  const LoggingConfig& yaml_config, ::axon::logging::LoggingConfig& log_config
) {
  // Console settings
  log_config.console_enabled = yaml_config.console_enabled;
  log_config.console_colors = yaml_config.console_colors;

  // Parse console level
  if (auto level = ::axon::logging::parse_severity_level(yaml_config.console_level)) {
    log_config.console_level = *level;
  }

  // File settings
  log_config.file_enabled = yaml_config.file_enabled;

  // Parse file level
  if (auto level = ::axon::logging::parse_severity_level(yaml_config.file_level)) {
    log_config.file_level = *level;
  }

  // File sink config
  log_config.file_config.directory = yaml_config.file_directory;
  log_config.file_config.file_pattern = yaml_config.file_pattern;
  log_config.file_config.format_json = (yaml_config.file_format == "json");
  log_config.file_config.rotation_size_mb = yaml_config.rotation_size_mb;
  log_config.file_config.max_files = yaml_config.max_files;
  log_config.file_config.rotate_at_midnight = yaml_config.rotate_at_midnight;
}

}  // namespace recorder
}  // namespace axon
