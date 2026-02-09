// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "commands.hpp"

#include <mcap/mcap.hpp>
#include <mcap/reader.hpp>

#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace axon {
namespace config {

namespace fs = std::filesystem;

// Default subdirectories to create
static const std::vector<std::string> DEFAULT_SUBDIRS = {
  "robot",
  "sensors",
};

Commands::Commands()
    : verbose_(false) {
  cache_.set_config_dir(ConfigCache::default_config_dir());
}

int Commands::init() {
  std::string config_dir = ConfigCache::default_config_dir();

  try {
    // Create main directory (and parent directories) if it doesn't exist
    if (!fs::exists(config_dir)) {
      fs::create_directories(config_dir);
      std::cout << "Created " << config_dir << "/" << std::endl;
    }

    // Create subdirectories
    for (const auto& subdir : DEFAULT_SUBDIRS) {
      std::string full_path = config_dir + "/" + subdir;
      if (!fs::exists(full_path)) {
        fs::create_directory(full_path);
        std::cout << "Created " << full_path << "/" << std::endl;
      }
    }

    return 0;

  } catch (const fs::filesystem_error& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }
}

int Commands::scan(bool incremental) {
  if (verbose_) {
    std::cout << "Scanning " << cache_.cache_path() << "..." << std::endl;
  }

  auto result = cache_.scan(incremental);

  if (result.status != ConfigCache::Status::Ok) {
    std::cerr << "Error: " << result.message << std::endl;
    return 1;
  }

  // Print tree structure of scanned files
  std::string config_dir = ConfigCache::default_config_dir();
  if (fs::exists(config_dir)) {
    print_tree(config_dir);
  }

  std::cout << result.message << std::endl;
  return 0;
}

int Commands::enable() {
  if (!cache_.enable()) {
    std::cerr << "Error: Failed to create enabled marker." << std::endl;
    return 1;
  }

  std::cout << "Config injection enabled." << std::endl;

  // Check if cache exists
  auto info = cache_.get_status();
  if (!info.cache_exists) {
    std::cout << "Warning: Config cache not found. Run 'axon_config scan' first." << std::endl;
    std::cout << "Config injection enabled, but no files will be written." << std::endl;
  } else {
    std::cout << "Cache: " << cache_.cache_path() << " (" << info.file_count << " files, "
              << format_size(info.total_size) << ")" << std::endl;
  }

  return 0;
}

int Commands::disable() {
  if (!cache_.disable()) {
    // Check if it was already disabled
    if (cache_.is_enabled()) {
      std::cerr << "Error: Failed to remove enabled marker." << std::endl;
      return 1;
    } else {
      std::cout << "Config injection already disabled." << std::endl;
      return 0;
    }
  }

  std::cout << "Config injection disabled." << std::endl;
  return 0;
}

int Commands::clear(bool force) {
  auto info = cache_.get_status();

  // Show summary
  std::cout << "This will DELETE all configuration data:" << std::endl;
  std::cout << "  Directory: " << ConfigCache::default_config_dir() << "/" << std::endl;
  std::cout << "  Files: " << info.file_count << std::endl;
  std::cout << "  Size: " << format_size(info.total_size) << std::endl;
  std::cout << std::endl;

  // Confirm
  if (!Confirmation::prompt("Are you sure? Type 'yes' to confirm:", force)) {
    std::cout << "Operation cancelled. No changes made." << std::endl;
    return 0;
  }

  // Perform clear
  auto status = cache_.clear();
  if (status != ConfigCache::Status::Ok) {
    std::cerr << "Error: Failed to clear config directory." << std::endl;
    return 1;
  }

  std::cout << "Deleted " << ConfigCache::default_config_dir() << "/ and all contents."
            << std::endl;
  std::cout << "Config injection disabled." << std::endl;
  return 0;
}

int Commands::status() {
  auto info = cache_.get_status();

  std::cout << "Config Status: " << (info.enabled ? "ENABLED" : "DISABLED") << std::endl;
  std::cout << "Config Directory: " << ConfigCache::default_config_dir() << std::endl;
  std::cout << "Cache File: " << cache_.cache_path() << std::endl;

  if (info.cache_exists) {
    std::cout << "Files Cached: " << info.file_count << std::endl;
    std::cout << "Total Size: " << format_size(info.total_size) << std::endl;
    std::cout << "Last Scanned: " << format_time(info.cache_mtime) << std::endl;
  } else {
    std::cout << "Files Cached: 0" << std::endl;
    std::cout << "Total Size: 0 B" << std::endl;
    std::cout << "Last Scanned: Never" << std::endl;
  }

  return 0;
}

int Commands::execute(int argc, char* argv[]) {
  std::string command;

  if (argc > 1) {
    command = argv[1];
  }

  if (command.empty() || command == "help" || command == "-h" || command == "--help") {
    print_usage();
    return 0;
  }

  // Parse flags
  bool incremental = false;
  bool force = false;

  for (int i = 2; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--incremental" || arg == "-i") {
      incremental = true;
    } else if (arg == "--force" || arg == "-f") {
      force = true;
    } else if (arg == "--verbose" || arg == "-v") {
      verbose_ = true;
    }
  }

  // Execute command
  if (command == "init") {
    return init();
  } else if (command == "scan") {
    return scan(incremental);
  } else if (command == "enable") {
    return enable();
  } else if (command == "disable") {
    return disable();
  } else if (command == "clear") {
    return clear(force);
  } else if (command == "status") {
    return status();
  } else {
    std::cerr << "Error: Unknown command '" << command << "'" << std::endl;
    print_usage();
    return 1;
  }
}

void Commands::print_tree(const std::string& dir, const std::string& prefix) {
  try {
    std::vector<fs::directory_entry> entries;

    // Get all entries in directory
    for (const auto& entry : fs::directory_iterator(dir)) {
      entries.push_back(entry);
    }

    // Sort entries: directories first, then alphabetically
    std::sort(
      entries.begin(),
      entries.end(),
      [](const fs::directory_entry& a, const fs::directory_entry& b) {
        if (a.is_directory() != b.is_directory()) {
          return a.is_directory();
        }
        return a.path().filename() < b.path().filename();
      }
    );

    // Print entries
    size_t i = 0;
    for (const auto& entry : entries) {
      // Skip hidden files, marker file, and cache file
      std::string filename = entry.path().filename().string();
      if (filename[0] == '.') {
        continue;
      }
      // Skip cache.mcap file
      if (filename == ConfigCache::CACHE_FILENAME) {
        continue;
      }

      bool is_dir = entry.is_directory();
      std::string connector = (i < entries.size() - 1) ? "├── " : "└── ";
      std::string prefix_add = (i < entries.size() - 1) ? "│   " : "    ";

      std::cout << prefix << connector << filename;

      if (is_dir) {
        // Count files in subdirectory
        size_t file_count = 0;
        for (const auto& sub_entry : fs::recursive_directory_iterator(entry.path())) {
          if (sub_entry.is_regular_file() && sub_entry.path().filename().string()[0] != '.') {
            file_count++;
          }
        }
        std::cout << "/ (" << file_count << " files)";
      } else if (entry.is_regular_file()) {
        // Get file size
        try {
          uint64_t size = fs::file_size(entry.path());
          std::cout << " (" << format_size(size) << ")";
        } catch (...) {
          // Ignore errors
        }
      }

      std::cout << std::endl;

      if (is_dir) {
        print_tree(entry.path().string(), prefix + prefix_add);
      }

      ++i;
    }
  } catch (const fs::filesystem_error&) {
    // Ignore errors
  }
}

std::string Commands::format_size(uint64_t size) {
  const char* units[] = {"B", "KB", "MB", "GB"};
  int unit_index = 0;

  while (size >= 1024 && unit_index < 3) {
    size /= 1024;
    unit_index++;
  }

  std::ostringstream oss;
  oss << size << " " << units[unit_index];

  return oss.str();
}

std::string Commands::format_time(uint64_t timestamp) {
  if (timestamp == 0) {
    return "Never";
  }

  std::time_t time = static_cast<std::time_t>(timestamp);
  std::tm tm = *std::localtime(&time);

  char buffer[64];
  std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &tm);

  return buffer;
}

void Commands::print_usage() {
  std::cout << "Usage: axon_config <command> [options]" << std::endl;
  std::cout << std::endl;
  std::cout << "Commands:" << std::endl;
  std::cout << "  init      Create /axon/config directory structure" << std::endl;
  std::cout << "  scan      Scan config directory and generate cache" << std::endl;
  std::cout << "  enable    Enable config injection in recordings" << std::endl;
  std::cout << "  disable   Disable config injection" << std::endl;
  std::cout << "  clear     Remove config directory and cache" << std::endl;
  std::cout << "  status    Show current configuration status" << std::endl;
  std::cout << "  help      Show this help message" << std::endl;
  std::cout << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "  --incremental, -i    Incremental scan (only changed files)" << std::endl;
  std::cout << "  --force, -f         Skip confirmation prompt (for scripts)" << std::endl;
  std::cout << "  --verbose, -v       Verbose output" << std::endl;
}

}  // namespace config
}  // namespace axon
