// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_RECORDER_CONFIG_INJECTOR_HPP
#define AXON_RECORDER_CONFIG_INJECTOR_HPP

#include <cstdint>
#include <string>
#include <vector>

namespace axon {
namespace recorder {

/**
 * ConfigInjector handles reading the cached configuration files
 * and injecting them as MCAP attachments during recording.
 *
 * Based on the axon_config design:
 * - Checks for ~/.axon/.enabled marker file
 * - Reads attachments from ~/.axon/cache.mcap
 * - Copies to recording MCAP with "config/" prefix
 *
 * Thread Safety:
 * - This class is NOT thread-safe
 * - All methods should be called from a single thread
 */
class ConfigInjector {
public:
  /**
   * Result of config injection operation
   */
  struct InjectionResult {
    bool success = false;         // Overall success status
    uint64_t files_injected = 0;  // Number of config files injected
    uint64_t total_bytes = 0;     // Total size of injected data
    std::string error_message;    // Error message if failed
    std::string cache_path;       // Path to cache file
  };

  ConfigInjector();
  ~ConfigInjector();

  // Non-copyable, non-movable
  ConfigInjector(const ConfigInjector&) = delete;
  ConfigInjector& operator=(const ConfigInjector&) = delete;
  ConfigInjector(ConfigInjector&&) = delete;
  ConfigInjector& operator=(ConfigInjector&&) = delete;

  /**
   * Check if config injection is enabled
   *
   * @return true if ~/.axon/.enabled exists
   */
  bool is_enabled() const;

  /**
   * Get the path to the enabled marker file
   */
  std::string enabled_marker_path() const;

  /**
   * Get the path to the config cache file
   */
  std::string cache_path() const;

  /**
   * Inject config files from cache into MCAP writer
   *
   * This method:
   * 1. Checks if config injection is enabled
   * 2. Opens the cache MCAP file
   * 3. Reads all attachments from cache
   * 4. Writes them to the target MCAP with "config/" prefix
   *
   * @tparam WriterType MCAP writer type with write_attachment() method
   * @param writer Reference to MCAP writer (must be open)
   * @return InjectionResult with details about the operation
   */
  template<typename WriterType>
  InjectionResult inject(WriterType& writer);

private:
  /**
   * Cached attachment data
   */
  struct CachedAttachment {
    std::string name;
    std::string content_type;
    std::vector<uint8_t> data;
  };

  /**
   * Internal implementation of inject
   *
   * @tparam WriterType MCAP writer type
   * @param writer Reference to MCAP writer
   * @param result Output result struct
   * @return true on success, false on failure
   */
  template<typename WriterType>
  bool do_inject(WriterType& writer, InjectionResult& result);

  /**
   * Read all attachments from the cache MCAP file
   *
   * @param attachments Output vector of cached attachments
   * @param error_msg Output error message if failed
   * @return true on success, false on failure
   */
  bool read_cache_attachments(std::vector<CachedAttachment>& attachments, std::string& error_msg);
};

}  // namespace recorder
}  // namespace axon

#endif  // AXON_RECORDER_CONFIG_INJECTOR_HPP
