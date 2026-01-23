// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "axon_log_init.hpp"

#include <boost/log/core.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/make_shared.hpp>

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <mutex>
#include <vector>

#include "axon_log_macros.hpp"

namespace axon {
namespace logging {

namespace {
// Thread safety for sink management
std::mutex g_sinks_mutex;

// Store sink pointers for cleanup
std::vector<boost::shared_ptr<boost::log::sinks::sink>> g_sinks;
boost::shared_ptr<async_console_sink_t> g_console_sink;
boost::shared_ptr<async_file_sink_t> g_file_sink;

// Global logger instance (Meyers singleton)
logger_type& get_logger_impl() {
  static logger_type instance;
  return instance;
}

bool g_initialized = false;

// Helper to convert string to lowercase
std::string to_lower(const std::string& s) {
  std::string result = s;
  std::transform(result.begin(), result.end(), result.begin(), [](unsigned char c) {
    return std::tolower(c);
  });
  return result;
}

// Helper to get environment variable as string
std::optional<std::string> get_env(const char* name) {
  const char* value = std::getenv(name);
  if (value && value[0] != '\0') {
    return std::string(value);
  }
  return std::nullopt;
}

// Helper to parse boolean from string
bool parse_bool(const std::string& s, bool default_value) {
  std::string lower = to_lower(s);
  if (lower == "true" || lower == "1" || lower == "yes" || lower == "on") {
    return true;
  }
  if (lower == "false" || lower == "0" || lower == "no" || lower == "off") {
    return false;
  }
  return default_value;
}
}  // namespace

std::optional<severity_level> parse_severity_level(const std::string& level_str) {
  std::string lower = to_lower(level_str);

  if (lower == "debug") {
    return severity_level::debug;
  } else if (lower == "info") {
    return severity_level::info;
  } else if (lower == "warn" || lower == "warning") {
    return severity_level::warn;
  } else if (lower == "error") {
    return severity_level::error;
  } else if (lower == "fatal") {
    return severity_level::fatal;
  }

  return std::nullopt;
}

void apply_env_overrides(LoggingConfig& config) {
  // Global level override (applies to both console and file)
  if (auto level_str = get_env("AXON_LOG_LEVEL")) {
    if (auto level = parse_severity_level(*level_str)) {
      config.console_level = *level;
      config.file_level = *level;
    }
  }

  // Console-specific overrides
  if (auto level_str = get_env("AXON_LOG_CONSOLE_LEVEL")) {
    if (auto level = parse_severity_level(*level_str)) {
      config.console_level = *level;
    }
  }

  if (auto enabled_str = get_env("AXON_LOG_CONSOLE_ENABLED")) {
    config.console_enabled = parse_bool(*enabled_str, config.console_enabled);
  }

  // File-specific overrides
  if (auto level_str = get_env("AXON_LOG_FILE_LEVEL")) {
    if (auto level = parse_severity_level(*level_str)) {
      config.file_level = *level;
    }
  }

  if (auto enabled_str = get_env("AXON_LOG_FILE_ENABLED")) {
    config.file_enabled = parse_bool(*enabled_str, config.file_enabled);
  }

  if (auto dir = get_env("AXON_LOG_FILE_DIR")) {
    config.file_config.directory = *dir;
  }

  if (auto format = get_env("AXON_LOG_FORMAT")) {
    std::string lower = to_lower(*format);
    config.file_config.format_json = (lower == "json");
  }
}

logger_type& get_logger() {
  return get_logger_impl();
}

void init_logging(const LoggingConfig& config) {
  std::lock_guard<std::mutex> lock(g_sinks_mutex);

  if (g_initialized) {
    return;  // Already initialized
  }

  auto core = boost::log::core::get();

  // Add common attributes (TimeStamp, ThreadID, etc.)
  boost::log::add_common_attributes();

  // Console sink
  if (config.console_enabled) {
    g_console_sink = create_console_sink(config.console_level, config.console_colors);
    core->add_sink(g_console_sink);
    g_sinks.push_back(g_console_sink);
  }

  // File sink
  if (config.file_enabled) {
    g_file_sink = create_file_sink(config.file_config, config.file_level);
    core->add_sink(g_file_sink);
    g_sinks.push_back(g_file_sink);
  }

  g_initialized = true;
}

void init_logging_default() {
  LoggingConfig config;
  config.console_enabled = true;
  config.console_colors = true;
  config.console_level = severity_level::info;
  config.file_enabled = false;  // Disabled by default

  init_logging(config);
}

void shutdown_logging() {
  std::lock_guard<std::mutex> lock(g_sinks_mutex);

  if (!g_initialized) {
    return;
  }

  auto core = boost::log::core::get();

  // Stop async sinks first (this drains the queue)
  if (g_console_sink) {
    g_console_sink->stop();
    g_console_sink->flush();
  }
  if (g_file_sink) {
    g_file_sink->stop();
    g_file_sink->flush();
  }

  // Remove all sinks
  for (auto& sink : g_sinks) {
    core->remove_sink(sink);
  }
  g_sinks.clear();

  g_console_sink.reset();
  g_file_sink.reset();

  g_initialized = false;
}

void add_sink(boost::shared_ptr<boost::log::sinks::sink> sink) {
  std::lock_guard<std::mutex> lock(g_sinks_mutex);

  auto core = boost::log::core::get();
  core->add_sink(sink);
  g_sinks.push_back(sink);
}

void remove_sink(boost::shared_ptr<boost::log::sinks::sink> sink) {
  std::lock_guard<std::mutex> lock(g_sinks_mutex);

  auto core = boost::log::core::get();
  core->remove_sink(sink);

  // Remove from our tracking vector
  auto it = std::find(g_sinks.begin(), g_sinks.end(), sink);
  if (it != g_sinks.end()) {
    g_sinks.erase(it);
  }
}

void flush_logging() {
  std::lock_guard<std::mutex> lock(g_sinks_mutex);

  if (g_console_sink) {
    g_console_sink->flush();
  }
  if (g_file_sink) {
    g_file_sink->flush();
  }
}

void reconfigure_logging(const LoggingConfig& config) {
  // Apply environment variable overrides to a mutable copy
  LoggingConfig final_config = config;
  apply_env_overrides(final_config);

  // Shutdown existing logging
  shutdown_logging();

  // Reinitialize with new config
  // Note: init_logging checks g_initialized, which shutdown_logging sets to false
  init_logging(final_config);
}

bool is_logging_initialized() {
  std::lock_guard<std::mutex> lock(g_sinks_mutex);
  return g_initialized;
}

}  // namespace logging
}  // namespace axon
