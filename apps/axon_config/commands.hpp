// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_CONFIG_COMMANDS_HPP
#define AXON_CONFIG_COMMANDS_HPP

#include <chrono>
#include <string>

#include "config_cache.hpp"

namespace axon {
namespace config {

/**
 * Options for registering this device with Keystone.
 */
struct RegisterOptions {
  std::string factory;
  std::string robot_type;
  std::string keystone_url;
  std::string config_dir = "/etc/axon";
  long timeout_seconds = 10;
  bool fetch_configs = true;
};

/**
 * Options for refreshing rendered configs from a saved device registration.
 */
struct RefreshOptions {
  std::string keystone_url;
  std::string config_dir = "/etc/axon";
  long timeout_seconds = 10;
};

/**
 * Command handler for axon_config CLI
 */
class Commands {
public:
  Commands();
  ~Commands() = default;

  // Non-copyable
  Commands(const Commands&) = delete;
  Commands& operator=(const Commands&) = delete;

  /**
   * Set verbose mode
   */
  void set_verbose(bool verbose) {
    verbose_ = verbose;
  }

  /**
   * Execute init command
   */
  int init();

  /**
   * Execute scan command
   */
  int scan(bool incremental);

  /**
   * Execute enable command
   */
  int enable();

  /**
   * Execute disable command
   */
  int disable();

  /**
   * Execute clear command
   */
  int clear(bool force);

  /**
   * Execute status command
   */
  int status();
  int status(const std::string& registration_config_dir);

  /**
   * Execute register command
   */
  int register_device(const RegisterOptions& options);

  /**
   * Execute refresh command
   */
  int refresh_configs(const RefreshOptions& options);

  /**
   * Parse and execute command line
   */
  int execute(int argc, char* argv[]);

#ifdef AXON_CONFIG_TESTING
  /**
   * Get cache instance for testing
   */
  ConfigCache& cache() {
    return cache_;
  }

  static std::string build_register_payload_for_test(
    const std::string& factory, const std::string& robot_type
  );

  static std::string build_register_url_for_test(const std::string& keystone_url);

  static std::string build_config_url_for_test(
    const std::string& keystone_url, const std::string& factory, const std::string& robot_type,
    const std::string& filename
  );
#endif

private:
  ConfigCache cache_;
  bool verbose_;

  /**
   * Print usage message
   */
  void print_usage();

  /**
   * Print register command usage message
   */
  void print_register_usage();

  /**
   * Print refresh command usage message
   */
  void print_refresh_usage();

  /**
   * Parse status command options.
   */
  bool parse_status_args(int argc, char* argv[], std::string& config_dir, std::string& error);

  /**
   * Parse register command options.
   */
  bool parse_register_args(int argc, char* argv[], RegisterOptions& options, std::string& error);

  /**
   * Parse refresh command options.
   */
  bool parse_refresh_args(int argc, char* argv[], RefreshOptions& options, std::string& error);

  /**
   * Print tree structure of directory
   */
  void print_tree(const std::string& dir, const std::string& prefix = "");

  /**
   * Format size for human readable output
   */
  std::string format_size(uint64_t size);

  /**
   * Format timestamp for human readable output
   */
  std::string format_time(uint64_t timestamp);
};

}  // namespace config
}  // namespace axon

#endif  // AXON_CONFIG_COMMANDS_HPP
