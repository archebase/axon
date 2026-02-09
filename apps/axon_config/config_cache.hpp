// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_CONFIG_CONFIG_CACHE_HPP
#define AXON_CONFIG_CONFIG_CACHE_HPP

#include <chrono>
#include <cstdint>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

namespace axon {
namespace config {

/**
 * Content type mapping for file extensions
 */
std::string guess_content_type(const std::string& filename);

/**
 * Cached file entry representing one attachment
 */
struct CachedFile {
  std::string name;           // Relative path (e.g., "robot/type.txt")
  std::string content_type;   // MIME type
  std::vector<uint8_t> data;  // File content
  uint64_t size;              // File size in bytes
  uint64_t mtime;             // Modification time (unix epoch)

  CachedFile()
      : size(0)
      , mtime(0) {}
};

/**
 * Cache manager for configuration files
 *
 * Handles scanning configuration directory and creating/loading MCAP cache.
 */
class ConfigCache {
public:
  // Default paths
  static constexpr const char* CACHE_FILENAME = "cache.mcap";
  static constexpr const char* ENABLED_MARKER = ".enabled";

  /**
   * Get default config directory path
   * Returns ~/.axon/config for user-writable location
   */
  static std::string default_config_dir();

  /**
   * Get cache file path (in parent of config directory)
   * Returns ~/.axon/cache.mcap
   */
  static std::string default_cache_path();

  /**
   * Result type for cache operations
   */
  enum class Status {
    Ok,
    DirNotFound,
    CacheNotFound,
    CacheCorrupted,
    PermissionDenied,
    WriteError,
    ReadError,
  };

  /**
   * Scan result containing statistics
   */
  struct ScanResult {
    Status status;
    size_t file_count;
    uint64_t total_size;
    std::string message;

    ScanResult()
        : status(Status::Ok)
        , file_count(0)
        , total_size(0) {}
  };

  ConfigCache();
  ~ConfigCache();

  // Non-copyable, non-movable
  ConfigCache(const ConfigCache&) = delete;
  ConfigCache& operator=(const ConfigCache&) = delete;
  ConfigCache(ConfigCache&&) = delete;
  ConfigCache& operator=(ConfigCache&&) = delete;

  /**
   * Set the config directory path
   */
  void set_config_dir(const std::string& path);

  /**
   * Get the full cache file path
   */
  std::string cache_path() const;

  /**
   * Get the enabled marker file path
   */
  std::string enabled_marker_path() const;

  /**
   * Check if config injection is enabled
   */
  bool is_enabled() const;

  /**
   * Scan config directory and create cache
   *
   * @param incremental If true, only update changed files
   * @return Scan result with statistics
   */
  ScanResult scan(bool incremental = false);

  /**
   * Load cache from MCAP file
   *
   * @return Status code
   */
  Status load();

  /**
   * Get list of cached files
   */
  const std::vector<CachedFile>& files() const {
    return files_;
  }

  /**
   * Check if cache is valid (has files loaded)
   */
  bool is_valid() const {
    return !files_.empty();
  }

  /**
   * Create enabled marker
   */
  bool enable();

  /**
   * Remove enabled marker
   */
  bool disable();

  /**
   * Clear config directory and cache
   */
  Status clear();

  /**
   * Get status information
   */
  struct StatusInfo {
    bool enabled;
    bool cache_exists;
    size_t file_count;
    uint64_t total_size;
    uint64_t cache_mtime;
  };

  StatusInfo get_status() const;

private:
  std::string config_dir_;
  std::vector<CachedFile> files_;

  /**
   * File metadata for tracking during scan
   */
  struct FileMeta {
    uint64_t mtime;
    uint64_t size;
  };

  /**
   * Recursively scan directory for files
   */
  Status scan_directory(
    const std::string& dir, const std::string& base_path,
    std::unordered_map<std::string, FileMeta>& file_map
  );

  /**
   * Write cache MCAP file
   */
  Status write_cache();

  /**
   * Read cache MCAP file
   */
  Status read_cache();

  /**
   * Compare mtime/size to determine changed files
   */
  struct FileChange {
    enum class Type { Unchanged, Added, Modified, Deleted };
    Type type;
    std::string path;
    uint64_t mtime;
    uint64_t size;
  };

  std::vector<FileChange> detect_changes(
    const std::unordered_map<std::string, FileMeta>& current_files
  );

  /**
   * Get modification time and size of a file
   */
  static bool get_file_stats(const std::string& path, uint64_t& mtime, uint64_t& size);
};

/**
 * Confirmation prompt helper
 */
class Confirmation {
public:
  /**
   * Prompt user for confirmation
   *
   * @param prompt Message to display
   * @param force If true, skip confirmation
   * @return true if confirmed, false otherwise
   */
  static bool prompt(const std::string& prompt, bool force = false);

  /**
   * Prompt with timeout
   */
  static bool prompt(const std::string& prompt, bool force, std::chrono::seconds timeout);
};

}  // namespace config
}  // namespace axon

#endif  // AXON_CONFIG_CONFIG_CACHE_HPP
