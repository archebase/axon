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
#endif

private:
  ConfigCache cache_;
  bool verbose_;

  /**
   * Print usage message
   */
  void print_usage();

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
