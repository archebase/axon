// Copyright (c) 2026 ArcheBase
// Axon is licensed under Mulan PSL v2.
// You can use this software according to the terms and conditions of the Mulan PSL v2.
// You may obtain a copy of Mulan PSL v2 at:
//          http://license.coscl.org.cn/MulanPSL2
// THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
// EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
// MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
// See the Mulan PSL v2 for more details.

#ifndef AXON_RECORDER_CONFIG_PARSER_HPP
#define AXON_RECORDER_CONFIG_PARSER_HPP

#include <yaml-cpp/yaml.h>

#include <memory>
#include <string>
#include <vector>

// Forward declaration in global namespace
namespace axon {
namespace logging {
struct LoggingConfig;
}
}  // namespace axon

namespace axon {
namespace recorder {

// Forward declarations (all defined in recorder.hpp)
struct SubscriptionConfig;
struct DatasetConfig;
struct RecordingConfig;
struct S3Config;
struct RetryConfig;
struct UploadConfig;
struct LoggingConfig;
struct HttpServerConfig;
struct RecorderConfig;

/**
 * Convert LoggingConfig to axon::logging::LoggingConfig.
 * This bridges the YAML parser output to the logging library input.
 */
void convert_logging_config(
  const LoggingConfig& yaml_config, ::axon::logging::LoggingConfig& log_config
);

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

  /**
   * Get last error message
   */
  std::string get_last_error() const {
    return last_error_;
  }

private:
  bool parse_dataset(const YAML::Node& node, DatasetConfig& dataset);
  bool parse_subscriptions(const YAML::Node& node, std::vector<SubscriptionConfig>& subscriptions);
  bool parse_recording(const YAML::Node& node, RecordingConfig& recording);
  bool parse_logging(const YAML::Node& node, LoggingConfig& logging);
  bool parse_upload(const YAML::Node& node, UploadConfig& upload);
  bool parse_http_server(const YAML::Node& node, HttpServerConfig& http_server);

  // Last error message
  mutable std::string last_error_;
};

}  // namespace recorder
}  // namespace axon

#endif  // AXON_RECORDER_CONFIG_PARSER_HPP
