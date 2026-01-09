#ifndef AXON_RECORDER_CONFIG_PARSER_HPP
#define AXON_RECORDER_CONFIG_PARSER_HPP

#include <yaml-cpp/yaml.h>

#include <string>

#include "common_types.hpp"

// Forward declaration
namespace axon {
namespace logging {
struct LoggingConfig;
}
}  // namespace axon

namespace axon {
namespace recorder {

/**
 * Configuration parser for YAML files.
 * Loads recorder configuration from YAML format.
 * Uses types from axon::utils but implementation is in apps/axon_recorder.
 */
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
  static bool validate_upload_config(const UploadConfig& upload, std::string& error_msg);

private:
  bool parse_dataset(const YAML::Node& node, DatasetConfig& dataset);
  bool parse_topics(const YAML::Node& node, std::vector<TopicConfig>& topics);
  bool parse_recording(const YAML::Node& node, RecordingConfig& recording);
  bool parse_logging(const YAML::Node& node, LoggingConfig& logging);
  bool parse_upload(const YAML::Node& node, UploadConfig& upload);
};

/**
 * Helper function to load RecorderConfig from YAML file.
 * This provides a convenient interface similar to RecorderConfig::from_yaml().
 */
RecorderConfig from_yaml(const std::string& yaml_path);

/**
 * Helper function to load RecorderConfig from YAML string.
 * This provides a convenient interface similar to RecorderConfig::from_yaml_string().
 */
RecorderConfig from_yaml_string(const std::string& yaml_content);

/**
 * Convert LoggingConfig from axon::recorder to axon::logging::LoggingConfig.
 * This bridges the YAML parser output to the logging library input.
 */
void convert_logging_config(
  const LoggingConfig& recorder_config, ::axon::logging::LoggingConfig& log_config
);

}  // namespace recorder
}  // namespace axon

#endif  // AXON_RECORDER_CONFIG_PARSER_HPP
