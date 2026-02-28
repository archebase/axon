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

    // Parse plugin configuration (optional, for backwards compatibility)
    if (node["plugin"]) {
      if (node["plugin"]["path"]) {
        config.plugin_path = node["plugin"]["path"].as<std::string>();
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

    // Parse HTTP server config
    if (node["http_server"]) {
      parse_http_server(node["http_server"], config.http_server);
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
  if (node["max_disk_usage_gb"]) {
    recording.max_disk_usage_gb = node["max_disk_usage_gb"].as<double>();
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
