#ifndef AXON_RECORDER_CONFIG_PARSER_HPP
#define AXON_RECORDER_CONFIG_PARSER_HPP

#include <yaml-cpp/yaml.h>

#include <memory>
#include <string>
#include <vector>

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

struct RecorderConfig {
  DatasetConfig dataset;
  std::vector<TopicConfig> topics;
  RecordingConfig recording;

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

private:
  bool parse_dataset(const YAML::Node& node, DatasetConfig& dataset);
  bool parse_topics(const YAML::Node& node, std::vector<TopicConfig>& topics);
  bool parse_recording(const YAML::Node& node, RecordingConfig& recording);
};

}  // namespace core
}  // namespace axon

#endif  // AXON_RECORDER_CONFIG_PARSER_HPP

