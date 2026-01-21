// Copyright (c) 2026 ArcheBase
// Axon is licensed under Mulan PSL v2.
// You can use this software according to the terms and conditions of the Mulan PSL v2.
// You may obtain a copy of Mulan PSL v2 at:
//          http://license.coscl.org.cn/MulanPSL2
// THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
// EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
// MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
// See the Mulan PSL v2 for more details.

#ifndef AXON_LOG_INIT_HPP
#define AXON_LOG_INIT_HPP

#include <boost/log/sinks/sink.hpp>

#include <optional>
#include <string>

#include "axon_console_sink.hpp"
#include "axon_file_sink.hpp"
#include "axon_log_severity.hpp"

namespace axon {
namespace logging {

/**
 * Logging configuration for axon applications.
 * This covers console and file sinks (core logging).
 * ROS sink is configured separately in middlewares/axon_recorder/.
 */
struct LoggingConfig {
  // Console sink
  bool console_enabled = true;
  bool console_colors = true;
  severity_level console_level = severity_level::info;

  // File sink
  bool file_enabled = true;
  FileSinkConfig file_config;  // Uses FileSinkConfig defaults
  severity_level file_level = severity_level::debug;
};

/**
 * Parse a string to severity_level.
 * Accepts: "debug", "info", "warn", "warning", "error", "fatal" (case-insensitive)
 *
 * @param level_str The string representation of the level
 * @return The parsed severity_level, or std::nullopt if invalid
 */
std::optional<severity_level> parse_severity_level(const std::string& level_str);

/**
 * Apply environment variable overrides to a LoggingConfig.
 *
 * Supported environment variables:
 *   AXON_LOG_LEVEL          - Global level (overrides both console and file)
 *   AXON_LOG_CONSOLE_LEVEL  - Console sink level
 *   AXON_LOG_FILE_LEVEL     - File sink level
 *   AXON_LOG_FILE_DIR       - Log file directory
 *   AXON_LOG_FORMAT         - File format ("json" or "text")
 *   AXON_LOG_FILE_ENABLED   - Enable file logging ("true" or "false")
 *   AXON_LOG_CONSOLE_ENABLED - Enable console logging ("true" or "false")
 *
 * @param config The configuration to modify (in-place)
 */
void apply_env_overrides(LoggingConfig& config);

/**
 * Initialize core logging (console and file sinks).
 * Should be called once at application startup.
 *
 * ROS sink should be added separately via add_sink() after ROS is initialized.
 *
 * @param config Logging configuration
 */
void init_logging(const LoggingConfig& config);

/**
 * Initialize with default configuration.
 * Console: INFO level, colors enabled
 * File: disabled (set file_enabled=true and provide directory)
 */
void init_logging_default();

/**
 * Shutdown logging infrastructure.
 * Stops async sink threads and flushes all pending records.
 * Should be called before application exit.
 */
void shutdown_logging();

/**
 * Add a custom sink to the logging core.
 * Use this to add ROS sink or other custom sinks.
 *
 * @param sink The sink to add
 */
void add_sink(boost::shared_ptr<boost::log::sinks::sink> sink);

/**
 * Remove a sink from the logging core.
 *
 * @param sink The sink to remove
 */
void remove_sink(boost::shared_ptr<boost::log::sinks::sink> sink);

/**
 * Flush all sinks.
 */
void flush_logging();

/**
 * Reconfigure logging with new settings.
 * This shuts down existing sinks and reinitializes with the new config.
 * Environment variable overrides are applied automatically.
 *
 * @param config New logging configuration
 */
void reconfigure_logging(const LoggingConfig& config);

/**
 * Check if logging is initialized.
 */
bool is_logging_initialized();

}  // namespace logging
}  // namespace axon

#endif  // AXON_LOG_INIT_HPP
