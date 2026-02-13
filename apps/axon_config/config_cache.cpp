// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "config_cache.hpp"

#include <mcap/mcap.hpp>
#include <mcap/reader.hpp>
#include <sys/stat.h>

#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <system_error>
#include <unistd.h>

namespace axon {
namespace config {

namespace fs = std::filesystem;

// =============================================================================
// Content type detection
// =============================================================================

std::string guess_content_type(const std::string& filename) {
  // Get file extension
  size_t dot_pos = filename.rfind('.');
  if (dot_pos == std::string::npos) {
    return "application/octet-stream";
  }

  std::string ext = filename.substr(dot_pos);

  // Convert to lowercase for case-insensitive comparison
  for (char& c : ext) {
    c = static_cast<char>(std::tolower(c));
  }

  static const std::unordered_map<std::string, std::string> content_types = {
    {".json", "application/json"},
    {".yaml", "text/yaml"},
    {".yml", "text/yaml"},
    {".txt", "text/plain"},
    {".xml", "application/xml"},
    {".urdf", "application/x-urdf"},
    {".pdf", "application/pdf"},
    {".png", "image/png"},
    {".jpg", "image/jpeg"},
    {".jpeg", "image/jpeg"},
    {".bin", "application/octet-stream"},
  };

  auto it = content_types.find(ext);
  if (it != content_types.end()) {
    return it->second;
  }

  return "application/octet-stream";
}

// =============================================================================
// ConfigCache implementation
// =============================================================================

std::string ConfigCache::default_config_dir() {
  // Try to get user's home directory from environment
  const char* home = std::getenv("HOME");
  if (home && home[0] != '\0') {
    return std::string(home) + "/.axon/config";
  }

  // Fallback to /tmp if HOME is not set
  return "/tmp/axon/config";
}

std::string ConfigCache::default_cache_path() {
  // Try to get user's home directory from environment
  const char* home = std::getenv("HOME");
  if (home && home[0] != '\0') {
    return std::string(home) + "/.axon/" + CACHE_FILENAME;
  }

  // Fallback to /tmp if HOME is not set
  return "/tmp/axon/" + std::string(CACHE_FILENAME);
}

ConfigCache::ConfigCache() = default;

ConfigCache::~ConfigCache() = default;

void ConfigCache::set_config_dir(const std::string& path) {
  config_dir_ = path;
}

std::string ConfigCache::cache_path() const {
  // Cache file is always in the parent directory of config
  return default_cache_path();
}

std::string ConfigCache::enabled_marker_path() const {
  // Enabled marker is in the parent directory alongside cache file
  const char* home = std::getenv("HOME");
  if (home && home[0] != '\0') {
    return std::string(home) + "/.axon/" + ENABLED_MARKER;
  }
  return "/tmp/axon/" + std::string(ENABLED_MARKER);
}

bool ConfigCache::is_enabled() const {
  return fs::exists(enabled_marker_path());
}

ConfigCache::ScanResult ConfigCache::scan(bool incremental) {
  ScanResult result;

  // Check if config directory exists
  if (!fs::exists(config_dir_)) {
    result.status = Status::DirNotFound;
    result.message = "Config directory not found. Run 'axon_config init' first.";
    return result;
  }

  std::unordered_map<std::string, ConfigCache::FileMeta> current_files;

  // Scan all files in config directory
  Status scan_status_val = scan_directory(config_dir_, "", current_files);
  if (scan_status_val != Status::Ok) {
    result.status = scan_status_val;
    result.message = "Failed to scan config directory: " + std::string(strerror(errno));
    return result;
  }

  result.file_count = current_files.size();
  result.total_size = 0;
  for (const auto& [path, meta] : current_files) {
    result.total_size += meta.size;
  }

  if (incremental) {
    // Detect changes
    auto changes = detect_changes(current_files);

    size_t added = 0, modified = 0, deleted = 0;
    for (const auto& change : changes) {
      switch (change.type) {
        case FileChange::Type::Added:
          added++;
          break;
        case FileChange::Type::Modified:
          modified++;
          break;
        case FileChange::Type::Deleted:
          deleted++;
          break;
        default:
          break;
      }
    }

    if (added == 0 && modified == 0 && deleted == 0) {
      result.message = "No changes detected.";
      return result;
    }

    // Build new files list keeping unchanged files
    std::vector<CachedFile> new_files;
    std::unordered_map<std::string, size_t> old_file_map;

    // Create index of old files
    for (size_t i = 0; i < files_.size(); ++i) {
      old_file_map[files_[i].name] = i;
    }

    // Add unchanged and modified files
    for (const auto& change : changes) {
      if (change.type == FileChange::Type::Deleted) {
        continue;  // Skip deleted files
      }

      if (change.type == FileChange::Type::Unchanged) {
        auto it = old_file_map.find(change.path);
        if (it != old_file_map.end()) {
          new_files.push_back(files_[it->second]);
        }
      } else {
        // Added or Modified - need to read file content
        std::string full_path = config_dir_ + "/" + change.path;

        // Get file stats
        uint64_t mtime, size;
        if (!get_file_stats(full_path, mtime, size)) {
          continue;
        }

        // Read file content
        std::vector<uint8_t> data(size);
        std::ifstream file(full_path, std::ios::binary);
        if (!file) {
          continue;
        }
        file.read(reinterpret_cast<char*>(data.data()), size);
        if (!file) {
          continue;
        }

        CachedFile cached;
        cached.name = change.path;
        cached.content_type = guess_content_type(change.path);
        cached.data = std::move(data);
        cached.size = size;
        cached.mtime = mtime;

        new_files.push_back(std::move(cached));
      }
    }

    files_ = std::move(new_files);

    std::ostringstream oss;
    oss << "Incremental scan: " << modified << " changed, " << added << " added, " << deleted
        << " removed";
    result.message = oss.str();

  } else {
    // Full scan - read all files
    files_.clear();
    files_.reserve(current_files.size());

    for (const auto& [rel_path, meta] : current_files) {
      std::string full_path = config_dir_ + "/" + rel_path;

      // Read file content
      std::vector<uint8_t> data(meta.size);
      std::ifstream file(full_path, std::ios::binary);
      if (!file) {
        result.status = Status::ReadError;
        result.message = "Failed to read file: " + rel_path;
        return result;
      }
      file.read(reinterpret_cast<char*>(data.data()), meta.size);
      if (!file) {
        result.status = Status::ReadError;
        result.message = "Failed to read file content: " + rel_path;
        return result;
      }

      CachedFile cached;
      cached.name = rel_path;
      cached.content_type = guess_content_type(rel_path);
      cached.data = std::move(data);
      cached.size = meta.size;
      cached.mtime = meta.mtime;

      files_.push_back(std::move(cached));
    }

    std::ostringstream oss;
    oss << "Cache written: " << files_.size() << " files, " << result.total_size << " bytes";
    result.message = oss.str();
  }

  // Write cache file
  Status write_status = write_cache();
  if (write_status != Status::Ok) {
    result.status = write_status;
    result.message = "Failed to write cache: " + std::string(strerror(errno));
    return result;
  }

  result.status = Status::Ok;
  return result;
}

ConfigCache::Status ConfigCache::scan_directory(
  const std::string& dir, const std::string& base_path,
  std::unordered_map<std::string, ConfigCache::FileMeta>& file_map
) {
  try {
    for (const auto& entry : fs::recursive_directory_iterator(dir)) {
      if (entry.is_regular_file()) {
        // Skip hidden files and marker file
        std::string filename = entry.path().filename();
        if (filename[0] == '.') {
          continue;
        }

        // Get relative path from config_dir
        std::string full_path = entry.path().string();
        std::string rel_path = full_path.substr(config_dir_.size() + 1);

        // Get modification time and size (use stat for portable epoch time)
        struct stat st{};
        if (stat(full_path.c_str(), &st) != 0) {
          continue;  // Skip files we can't stat
        }
        uint64_t mtime = static_cast<uint64_t>(st.st_mtime);
        uint64_t size = static_cast<uint64_t>(st.st_size);

        FileMeta meta;
        meta.mtime = mtime;
        meta.size = size;
        file_map[rel_path] = meta;
      }
    }
  } catch (const fs::filesystem_error&) {
    return Status::PermissionDenied;
  }

  return Status::Ok;
}

ConfigCache::Status ConfigCache::write_cache() {
  std::string cache_file = cache_path();

  // Ensure parent directory exists
  fs::path cache_path_obj(cache_file);
  if (cache_path_obj.has_parent_path()) {
    try {
      fs::create_directories(cache_path_obj.parent_path());
    } catch (const fs::filesystem_error&) {
      return Status::PermissionDenied;
    }
  }

  mcap::McapWriterOptions options("");
  options.compression = mcap::Compression::None;
  options.compressionLevel = mcap::CompressionLevel::Fastest;

  mcap::McapWriter writer;
  mcap::Status status = writer.open(cache_file, options);
  if (!status.ok()) {
    return Status::WriteError;
  }

  // Write each file as an attachment
  for (const auto& cached_file : files_) {
    mcap::Attachment attachment;
    attachment.name = cached_file.name;
    attachment.mediaType = cached_file.content_type;
    attachment.dataSize = cached_file.data.size();
    attachment.data = reinterpret_cast<const std::byte*>(cached_file.data.data());
    attachment.logTime = cached_file.mtime * 1000000000ULL;     // Convert to nanoseconds
    attachment.createTime = cached_file.mtime * 1000000000ULL;  // Convert to nanoseconds
    attachment.crc = 0;

    mcap::Status attach_status = writer.write(attachment);
    if (!attach_status.ok()) {
      writer.close();
      return Status::WriteError;
    }
  }

  writer.close();
  return Status::Ok;
}

ConfigCache::Status ConfigCache::read_cache() {
  std::string cache_file = cache_path();

  // Check if cache exists
  if (!fs::exists(cache_file)) {
    return Status::CacheNotFound;
  }

  // Open MCAP file
  mcap::McapReader reader;
  mcap::Status status = reader.open(cache_file);
  if (!status.ok()) {
    return Status::CacheCorrupted;
  }

  // Read summary to populate indexes
  status = reader.readSummary(mcap::ReadSummaryMethod::NoFallbackScan);
  if (!status.ok()) {
    reader.close();
    return Status::CacheCorrupted;
  }

  // Get attachment indexes
  files_.clear();
  const auto& indexes = reader.attachmentIndexes();

  // Note: For now, we only read metadata from indexes
  // Reading actual attachment data would require parsing each attachment record
  // which is complex. In this design, scan() always re-reads from source files.
  for (const auto& [name, index] : indexes) {
    CachedFile cached;
    cached.name = index.name;
    cached.content_type = index.mediaType;
    cached.size = index.dataSize;
    // Convert from nanoseconds to seconds
    cached.mtime = index.createTime / 1000000000ULL;
    // Data is not loaded - use scan() to load actual file contents
    cached.data.clear();

    files_.push_back(std::move(cached));
  }

  reader.close();
  return Status::Ok;
}

ConfigCache::Status ConfigCache::load() {
  Status status_val = read_cache();
  if (status_val == Status::CacheNotFound) {
    return status_val;
  }

  return status_val;
}

bool ConfigCache::enable() {
  try {
    // Ensure parent directory exists
    std::string marker_path = enabled_marker_path();
    fs::path marker_file(marker_path);
    if (marker_file.has_parent_path()) {
      fs::create_directories(marker_file.parent_path());
    }
    std::ofstream marker(marker_path);
    return marker.good();
  } catch (const fs::filesystem_error&) {
    return false;
  }
}

bool ConfigCache::disable() {
  try {
    return fs::remove(enabled_marker_path());
  } catch (const fs::filesystem_error&) {
    return false;
  }
}

ConfigCache::Status ConfigCache::clear() {
  try {
    // Remove config directory
    if (fs::exists(config_dir_)) {
      fs::remove_all(config_dir_);
    }
    // Also remove cache file and enabled marker
    std::string cache_file = cache_path();
    if (fs::exists(cache_file)) {
      fs::remove(cache_file);
    }
    std::string marker = enabled_marker_path();
    if (fs::exists(marker)) {
      fs::remove(marker);
    }
    files_.clear();
    return Status::Ok;
  } catch (const fs::filesystem_error&) {
    return Status::PermissionDenied;
  }
}

ConfigCache::StatusInfo ConfigCache::get_status() const {
  StatusInfo info;
  info.enabled = is_enabled();
  info.cache_exists = fs::exists(cache_path());

  if (info.cache_exists) {
    // Load cache metadata to get file count and size
    // (files_ is mutable for lazy loading in const context)
    load();

    info.file_count = files_.size();
    info.total_size = 0;
    for (const auto& file : files_) {
      info.total_size += file.size;
    }

    // Use cache file modification time as "last scanned" time
    struct stat st{};
    if (stat(cache_path().c_str(), &st) == 0) {
      info.cache_mtime = static_cast<uint64_t>(st.st_mtime);
    } else {
      info.cache_mtime = 0;
    }
  } else {
    info.file_count = 0;
    info.total_size = 0;
    info.cache_mtime = 0;
  }

  return info;
}

std::vector<ConfigCache::FileChange> ConfigCache::detect_changes(
  const std::unordered_map<std::string, ConfigCache::FileMeta>& current_files
) {
  std::vector<FileChange> changes;

  // Build map of existing files from cache
  std::unordered_map<std::string, std::pair<uint64_t, uint64_t>> old_files;
  for (const auto& file : files_) {
    old_files[file.name] = {file.mtime, file.size};
  }

  // Check for added and modified files
  for (const auto& [path, meta] : current_files) {
    auto it = old_files.find(path);
    if (it == old_files.end()) {
      FileChange change;
      change.type = FileChange::Type::Added;
      change.path = path;
      change.mtime = meta.mtime;
      change.size = meta.size;
      changes.push_back(change);
    } else {
      // Check if modified (different mtime or size)
      if (it->second.first != meta.mtime || it->second.second != meta.size) {
        FileChange change;
        change.type = FileChange::Type::Modified;
        change.path = path;
        change.mtime = meta.mtime;
        change.size = meta.size;
        changes.push_back(change);
      } else {
        FileChange change;
        change.type = FileChange::Type::Unchanged;
        change.path = path;
        change.mtime = meta.mtime;
        change.size = meta.size;
        changes.push_back(change);
      }
    }
  }

  // Check for deleted files
  for (const auto& [path, _] : old_files) {
    if (current_files.find(path) == current_files.end()) {
      FileChange change;
      change.type = FileChange::Type::Deleted;
      change.path = path;
      changes.push_back(change);
    }
  }

  return changes;
}

bool ConfigCache::get_file_stats(const std::string& path, uint64_t& mtime, uint64_t& size) {
  struct stat st{};
  if (stat(path.c_str(), &st) != 0) {
    return false;
  }
  mtime = static_cast<uint64_t>(st.st_mtime);
  size = static_cast<uint64_t>(st.st_size);
  return true;
}

// =============================================================================
// Confirmation implementation
// =============================================================================

bool Confirmation::prompt(const std::string& prompt, bool force) {
  if (force) {
    return true;
  }

  // Check if stdin is a terminal (interactive)
  if (!isatty(STDIN_FILENO)) {
    return false;  // Non-interactive, no terminal
  }

  std::cout << prompt << std::endl << "Type 'yes' to confirm: ";
  std::cout.flush();

  std::string input;
  if (!std::getline(std::cin, input)) {
    return false;
  }

  // Trim whitespace
  const char* whitespace = " \t\n\r";
  size_t start = input.find_first_not_of(whitespace);
  if (start == std::string::npos) {
    return false;
  }
  size_t end = input.find_last_not_of(whitespace);
  input = input.substr(start, end - start + 1);

  return input == "yes";
}

bool Confirmation::prompt(
  const std::string& prompt, bool force, std::chrono::seconds /* timeout */
) {
  // Simplified implementation - timeout not implemented for now
  return Confirmation::prompt(prompt, force);
}

}  // namespace config
}  // namespace axon
