#ifndef AXON_UTILS_CONFIG_PARSER_HPP
#define AXON_UTILS_CONFIG_PARSER_HPP

#include <yaml-cpp/yaml.h>

#include <string>

#include "common_types.hpp"

namespace axon {
namespace utils {

/**
 * Configuration parser for YAML files.
 * Loads recorder configuration from YAML format.
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

}  // namespace utils
}  // namespace axon

#endif  // AXON_UTILS_CONFIG_PARSER_HPP
