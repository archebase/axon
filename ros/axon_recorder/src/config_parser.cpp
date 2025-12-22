#include "config_parser.hpp"

#include <fstream>
#include <sstream>

// Logging infrastructure
#define AXON_LOG_COMPONENT "config_parser"
#include <axon_log_macros.hpp>
#include <axon_log_init.hpp>

namespace axon {
namespace core {

RecorderConfig RecorderConfig::from_yaml(const std::string& yaml_path) {
  ConfigParser parser;
  RecorderConfig config;
  if (!parser.load_from_file(yaml_path, config)) {
    AXON_LOG_ERROR("Failed to load config from file" << ::axon::logging::kv("path", yaml_path));
  }
  return config;
}

RecorderConfig RecorderConfig::from_yaml_string(const std::string& yaml_content) {
  ConfigParser parser;
  RecorderConfig config;
  if (!parser.load_from_string(yaml_content, config)) {
    AXON_LOG_ERROR("Failed to load config from string");
  }
  return config;
}

bool RecorderConfig::validate() const {
  if (dataset.path.empty()) {
    return false;
  }

  if (topics.empty()) {
    return false;
  }

  for (const auto& topic : topics) {
    if (topic.name.empty() || topic.message_type.empty()) {
      return false;
    }
    if (topic.batch_size == 0) {
      return false;
    }
  }

  if (dataset.mode != "create" && dataset.mode != "append") {
    return false;
  }

  return true;
}

std::string RecorderConfig::to_string() const {
  std::ostringstream oss;
  oss << "Dataset: " << dataset.path << " (mode: " << dataset.mode << ")\n";
  oss << "Topics:\n";
  for (const auto& topic : topics) {
    oss << "  - " << topic.name << " (" << topic.message_type
        << ", batch_size: " << topic.batch_size << ", flush_interval: " << topic.flush_interval_ms
        << "ms)\n";
  }
  oss << "Recording: max_disk_usage=" << recording.max_disk_usage_gb << "GB\n";
  return oss.str();
}

bool ConfigParser::load_from_file(const std::string& path, RecorderConfig& config) {
  try {
    YAML::Node node = YAML::LoadFile(path);
    return load_from_string(YAML::Dump(node), config);
  } catch (const YAML::Exception& e) {
    AXON_LOG_ERROR("YAML parsing error" << ::axon::logging::kv("error", e.what()));
    return false;
  }
}

bool ConfigParser::load_from_string(const std::string& yaml_content, RecorderConfig& config) {
  try {
    YAML::Node node = YAML::Load(yaml_content);

    // Parse dataset config
    if (node["dataset"]) {
      parse_dataset(node["dataset"], config.dataset);
    }

    // Parse topics
    if (node["topics"]) {
      parse_topics(node["topics"], config.topics);
    }

    // Parse recording config
    if (node["recording"]) {
      parse_recording(node["recording"], config.recording);
    }

    // Parse logging config (optional - uses defaults if not present)
    if (node["logging"]) {
      parse_logging(node["logging"], config.logging);
    }

    // Parse upload config (optional - disabled by default)
    if (node["upload"]) {
      parse_upload(node["upload"], config.upload);
    }

    return config.validate();
  } catch (const YAML::Exception& e) {
    AXON_LOG_ERROR("YAML parsing error" << ::axon::logging::kv("error", e.what()));
    return false;
  }
}

bool ConfigParser::save_to_file(const std::string& path, const RecorderConfig& config) {
  try {
    YAML::Node node;

    // Dataset
    node["dataset"]["path"] = config.dataset.path;
    node["dataset"]["mode"] = config.dataset.mode;

    // Topics
    node["topics"] = YAML::Node(YAML::NodeType::Sequence);
    for (const auto& topic : config.topics) {
      YAML::Node topic_node;
      topic_node["name"] = topic.name;
      topic_node["message_type"] = topic.message_type;
      topic_node["batch_size"] = topic.batch_size;
      topic_node["flush_interval_ms"] = topic.flush_interval_ms;
      node["topics"].push_back(topic_node);
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
  return true;
}

bool ConfigParser::parse_topics(const YAML::Node& node, std::vector<TopicConfig>& topics) {
  if (!node.IsSequence()) {
    return false;
  }

  topics.clear();
  for (const auto& topic_node : node) {
    TopicConfig topic;
    if (topic_node["name"]) {
      topic.name = topic_node["name"].as<std::string>();
    }
    if (topic_node["message_type"]) {
      topic.message_type = topic_node["message_type"].as<std::string>();
    }
    if (topic_node["batch_size"]) {
      topic.batch_size = topic_node["batch_size"].as<size_t>();
    }
    if (topic_node["flush_interval_ms"]) {
      topic.flush_interval_ms = topic_node["flush_interval_ms"].as<int>();
    }
    topics.push_back(topic);
  }
  return true;
}

bool ConfigParser::parse_recording(const YAML::Node& node, RecordingConfig& recording) {
  if (node["max_disk_usage_gb"]) {
    recording.max_disk_usage_gb = node["max_disk_usage_gb"].as<double>();
  }
  return true;
}

bool ConfigParser::parse_logging(const YAML::Node& node, LoggingConfigYaml& logging) {
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

bool ConfigParser::parse_upload(const YAML::Node& node, UploadConfigYaml& upload) {
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

bool ConfigParser::validate(const RecorderConfig& config, std::string& error_msg) {
  if (!config.validate()) {
    error_msg = "Configuration validation failed";
    return false;
  }
  return true;
}

void convert_logging_config(const LoggingConfigYaml& yaml_config, 
                             ::axon::logging::LoggingConfig& log_config) {
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

}  // namespace core
}  // namespace axon

