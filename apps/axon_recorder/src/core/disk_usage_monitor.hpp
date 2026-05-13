// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_RECORDER_DISK_USAGE_MONITOR_HPP
#define AXON_RECORDER_DISK_USAGE_MONITOR_HPP

#include <nlohmann/json.hpp>

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace axon {
namespace recorder {

enum class DiskUsageState { DISABLED, NORMAL, WARN, HARD_LIMIT, ERROR };

const char* disk_usage_state_to_string(DiskUsageState state);

uint64_t gb_to_bytes(double gb);
double bytes_to_gb(uint64_t bytes);

struct DiskUsageLimitConfig {
  bool enabled = true;
  uint64_t warn_usage_bytes = 0;
  uint64_t hard_limit_bytes = 0;
  uint64_t max_task_size_bytes = 0;
};

struct DiskUsagePathConfig {
  std::string role;
  std::filesystem::path path;
};

struct DiskUsagePathSnapshot {
  std::string role;
  std::string path;
  bool exists = false;
  uint64_t used_bytes = 0;
  uint64_t capacity_bytes = 0;
  uint64_t free_bytes = 0;
  uint64_t available_bytes = 0;
  std::string error;

  nlohmann::json to_json() const;
};

struct DiskUsageSnapshot {
  bool enabled = true;
  DiskUsageState state = DiskUsageState::NORMAL;
  std::string reason;
  uint64_t total_used_bytes = 0;
  uint64_t warn_usage_bytes = 0;
  uint64_t hard_limit_bytes = 0;
  uint64_t max_task_size_bytes = 0;
  uint64_t current_task_bytes = 0;
  std::vector<DiskUsagePathSnapshot> paths;

  bool hard_limit_reached() const {
    return state == DiskUsageState::HARD_LIMIT;
  }

  bool warn_reached() const {
    return state == DiskUsageState::WARN || state == DiskUsageState::HARD_LIMIT;
  }

  nlohmann::json to_json() const;
};

struct DiskCleanupResult {
  size_t files_removed = 0;
  uint64_t bytes_removed = 0;
  uint64_t projected_used_bytes = 0;
  std::vector<std::string> removed_paths;
  std::vector<std::string> errors;

  nlohmann::json to_json() const;
};

class DiskUsageMonitor {
public:
  DiskUsageMonitor(DiskUsageLimitConfig limits, std::vector<DiskUsagePathConfig> paths);

  DiskUsageSnapshot snapshot(uint64_t current_task_bytes = 0) const;

  DiskCleanupResult cleanup_recording_files(
    const std::vector<std::filesystem::path>& roots, const std::filesystem::path& active_path,
    uint64_t current_used_bytes, uint64_t target_used_bytes, int min_age_sec
  ) const;

private:
  static DiskUsagePathSnapshot collect_path_snapshot(const DiskUsagePathConfig& path_config);
  static uint64_t path_used_bytes(const std::filesystem::path& path, std::string& error);

  DiskUsageLimitConfig limits_;
  std::vector<DiskUsagePathConfig> paths_;
};

}  // namespace recorder
}  // namespace axon

#endif  // AXON_RECORDER_DISK_USAGE_MONITOR_HPP
