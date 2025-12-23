#ifndef AXON_RECORDER_CONFIG_PARSER_HPP
#define AXON_RECORDER_CONFIG_PARSER_HPP

#include <yaml-cpp/yaml.h>

#include <memory>
#include <string>
#include <vector>

// Forward declaration in global namespace
namespace axon { namespace logging { struct LoggingConfig; } }

namespace axon {
namespace core {

struct TopicConfig {
  std::string name;
  std::string message_type;
  size_t batch_size = 100;
  int flush_interval_ms = 1000;
};

struct DatasetConfig {
  std::string path;
  std::string mode = "append";  // "create" or "append"
};

struct RecordingConfig {
  double max_disk_usage_gb = 100.0;
};

/**
 * S3 upload configuration for edge uploader
 */
struct S3ConfigYaml {
  std::string endpoint_url;             // S3-compatible endpoint (empty for AWS S3)
  std::string bucket;                   // Bucket name (required when upload enabled)
  std::string region = "us-east-1";     // AWS region
  bool use_ssl = true;
  bool verify_ssl = true;
};

/**
 * Retry configuration for uploads
 */
struct RetryConfigYaml {
  int max_retries = 5;
  int initial_delay_ms = 1000;
  int max_delay_ms = 300000;
  double exponential_base = 2.0;
  bool jitter = true;
};

/**
 * Edge upload configuration
 */
struct UploadConfigYaml {
  bool enabled = false;
  S3ConfigYaml s3;
  RetryConfigYaml retry;
  int num_workers = 2;
  std::string state_db_path = "/var/lib/axon/uploader_state.db";
  bool delete_after_upload = true;
  std::string failed_uploads_dir = "/data/failed_uploads/";
  double warn_pending_gb = 8.0;
  double alert_pending_gb = 20.0;
};

/**
 * Logging configuration parsed from YAML.
 * This mirrors axon::logging::LoggingConfig structure.
 */
struct LoggingConfigYaml {
  // Console sink
  bool console_enabled = true;
  bool console_colors = true;
  std::string console_level = "info";  // debug, info, warn, error, fatal
  
  // File sink  
  bool file_enabled = false;
  std::string file_level = "debug";
  std::string file_directory = "/var/log/axon";
  std::string file_pattern = "axon_%Y%m%d_%H%M%S.log";
  std::string file_format = "json";  // json or text
  size_t rotation_size_mb = 100;
  size_t max_files = 10;
  bool rotate_at_midnight = true;
};

/**
 * Convert LoggingConfigYaml to axon::logging::LoggingConfig.
 * This bridges the YAML parser output to the logging library input.
 */
void convert_logging_config(const LoggingConfigYaml& yaml_config, 
                             ::axon::logging::LoggingConfig& log_config);

struct RecorderConfig {
  DatasetConfig dataset;
  std::vector<TopicConfig> topics;
  RecordingConfig recording;
  LoggingConfigYaml logging;       // Logging configuration
  UploadConfigYaml upload;         // Edge upload configuration

  static RecorderConfig from_yaml(const std::string& yaml_path);
  static RecorderConfig from_yaml_string(const std::string& yaml_content);

  bool validate() const;
  std::string to_string() const;
};

class ConfigParser {
public:
  ConfigParser() = default;

  /**
   * Load configuration from YAML file
   */
  bool load_from_file(const std::string& path, RecorderConfig& config);

  /**
   * Load configuration from YAML string
   */
  bool load_from_string(const std::string& yaml_content, RecorderConfig& config);

  /**
   * Save configuration to YAML file
   */
  bool save_to_file(const std::string& path, const RecorderConfig& config);

  /**
   * Validate configuration
   */
  static bool validate(const RecorderConfig& config, std::string& error_msg);
  
  /**
   * Validate upload configuration when enabled
   */
  static bool validate_upload_config(const UploadConfigYaml& upload, std::string& error_msg);

private:
  bool parse_dataset(const YAML::Node& node, DatasetConfig& dataset);
  bool parse_topics(const YAML::Node& node, std::vector<TopicConfig>& topics);
  bool parse_recording(const YAML::Node& node, RecordingConfig& recording);
  bool parse_logging(const YAML::Node& node, LoggingConfigYaml& logging);
  bool parse_upload(const YAML::Node& node, UploadConfigYaml& upload);
};

}  // namespace core
}  // namespace axon

#endif  // AXON_RECORDER_CONFIG_PARSER_HPP

