// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "disk_usage_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <system_error>

namespace axon {
namespace recorder {

namespace {

constexpr long double kBytesPerGb = 1024.0L * 1024.0L * 1024.0L;

std::filesystem::path normalized_absolute(const std::filesystem::path& path) {
  std::error_code ec;
  auto absolute_path = std::filesystem::absolute(path, ec);
  if (ec) {
    absolute_path = path;
  }
  return absolute_path.lexically_normal();
}

std::filesystem::path nearest_existing_path(std::filesystem::path path) {
  std::error_code ec;
  if (path.empty()) {
    return std::filesystem::current_path(ec);
  }

  path = normalized_absolute(path);
  while (!path.empty() && !std::filesystem::exists(path, ec)) {
    auto parent = path.parent_path();
    if (parent == path) {
      break;
    }
    path = parent;
    ec.clear();
  }

  return path.empty() ? std::filesystem::current_path(ec) : path;
}

bool is_recording_cleanup_candidate(const std::filesystem::path& path) {
  const auto ext = path.extension().string();
  return ext == ".mcap" || ext == ".json";
}

bool same_normalized_path(const std::filesystem::path& left, const std::filesystem::path& right) {
  return normalized_absolute(left) == normalized_absolute(right);
}

}  // namespace

const char* disk_usage_state_to_string(DiskUsageState state) {
  switch (state) {
    case DiskUsageState::DISABLED:
      return "disabled";
    case DiskUsageState::NORMAL:
      return "normal";
    case DiskUsageState::WARN:
      return "warn";
    case DiskUsageState::HARD_LIMIT:
      return "hard_limit";
    case DiskUsageState::ERROR:
      return "error";
  }
  return "unknown";
}

uint64_t gb_to_bytes(double gb) {
  if (!std::isfinite(gb) || gb <= 0.0) {
    return 0;
  }

  const long double bytes = static_cast<long double>(gb) * kBytesPerGb;
  const auto max_value = static_cast<long double>(std::numeric_limits<uint64_t>::max());
  if (bytes >= max_value) {
    return std::numeric_limits<uint64_t>::max();
  }
  return static_cast<uint64_t>(bytes);
}

double bytes_to_gb(uint64_t bytes) {
  return static_cast<double>(static_cast<long double>(bytes) / kBytesPerGb);
}

nlohmann::json DiskUsagePathSnapshot::to_json() const {
  nlohmann::json j;
  j["role"] = role;
  j["path"] = path;
  j["exists"] = exists;
  j["used_bytes"] = used_bytes;
  j["used_gb"] = bytes_to_gb(used_bytes);
  j["capacity_bytes"] = capacity_bytes;
  j["free_bytes"] = free_bytes;
  j["available_bytes"] = available_bytes;
  j["available_gb"] = bytes_to_gb(available_bytes);
  if (!error.empty()) {
    j["error"] = error;
  }
  return j;
}

nlohmann::json DiskUsageSnapshot::to_json() const {
  nlohmann::json j;
  j["enabled"] = enabled;
  j["state"] = disk_usage_state_to_string(state);
  j["reason"] = reason;
  j["total_used_bytes"] = total_used_bytes;
  j["total_used_gb"] = bytes_to_gb(total_used_bytes);
  j["warn_usage_bytes"] = warn_usage_bytes;
  j["warn_usage_gb"] = bytes_to_gb(warn_usage_bytes);
  j["hard_limit_bytes"] = hard_limit_bytes;
  j["hard_limit_gb"] = bytes_to_gb(hard_limit_bytes);
  j["max_task_size_bytes"] = max_task_size_bytes;
  j["max_task_size_gb"] = bytes_to_gb(max_task_size_bytes);
  j["current_task_bytes"] = current_task_bytes;
  j["current_task_gb"] = bytes_to_gb(current_task_bytes);

  nlohmann::json path_array = nlohmann::json::array();
  for (const auto& path : paths) {
    path_array.push_back(path.to_json());
  }
  j["paths"] = path_array;
  return j;
}

nlohmann::json DiskCleanupResult::to_json() const {
  nlohmann::json j;
  j["files_removed"] = files_removed;
  j["bytes_removed"] = bytes_removed;
  j["projected_used_bytes"] = projected_used_bytes;
  j["removed_paths"] = removed_paths;
  j["errors"] = errors;
  return j;
}

DiskUsageMonitor::DiskUsageMonitor(
  DiskUsageLimitConfig limits, std::vector<DiskUsagePathConfig> paths
)
    : limits_(limits) {
  std::set<std::string> seen_paths;
  for (auto& path_config : paths) {
    if (path_config.path.empty()) {
      continue;
    }
    path_config.path = normalized_absolute(path_config.path);
    const std::string key = path_config.path.string();
    if (seen_paths.insert(key).second) {
      paths_.push_back(std::move(path_config));
    }
  }
}

DiskUsageSnapshot DiskUsageMonitor::snapshot(uint64_t current_task_bytes) const {
  DiskUsageSnapshot snapshot;
  snapshot.enabled = limits_.enabled;
  snapshot.warn_usage_bytes = limits_.warn_usage_bytes;
  snapshot.hard_limit_bytes = limits_.hard_limit_bytes;
  snapshot.max_task_size_bytes = limits_.max_task_size_bytes;
  snapshot.current_task_bytes = current_task_bytes;

  if (!limits_.enabled) {
    snapshot.state = DiskUsageState::DISABLED;
    snapshot.reason = "disk usage limits disabled";
    return snapshot;
  }

  for (const auto& path_config : paths_) {
    auto path_snapshot = collect_path_snapshot(path_config);
    snapshot.total_used_bytes += path_snapshot.used_bytes;
    snapshot.paths.push_back(std::move(path_snapshot));
  }

  if (snapshot.paths.empty()) {
    snapshot.state = DiskUsageState::ERROR;
    snapshot.reason = "no disk usage paths configured";
    return snapshot;
  }

  if (limits_.max_task_size_bytes > 0 && current_task_bytes >= limits_.max_task_size_bytes) {
    snapshot.state = DiskUsageState::HARD_LIMIT;
    snapshot.reason = "current task size reached hard limit";
    return snapshot;
  }

  if (limits_.hard_limit_bytes > 0 && snapshot.total_used_bytes >= limits_.hard_limit_bytes) {
    snapshot.state = DiskUsageState::HARD_LIMIT;
    snapshot.reason = "monitored disk usage reached hard limit";
    return snapshot;
  }

  if (limits_.warn_usage_bytes > 0 && snapshot.total_used_bytes >= limits_.warn_usage_bytes) {
    snapshot.state = DiskUsageState::WARN;
    snapshot.reason = "monitored disk usage reached warning threshold";
    return snapshot;
  }

  snapshot.state = DiskUsageState::NORMAL;
  snapshot.reason = "capacity within configured limits";
  return snapshot;
}

DiskCleanupResult DiskUsageMonitor::cleanup_recording_files(
  const std::vector<std::filesystem::path>& roots, const std::filesystem::path& active_path,
  uint64_t current_used_bytes, uint64_t target_used_bytes, int min_age_sec
) const {
  struct Candidate {
    std::filesystem::path path;
    std::filesystem::file_time_type modified_at;
    uint64_t size = 0;
  };

  DiskCleanupResult result;
  result.projected_used_bytes = current_used_bytes;
  if (target_used_bytes == 0 || current_used_bytes <= target_used_bytes) {
    return result;
  }

  std::vector<Candidate> candidates;
  std::set<std::string> seen_paths;
  const auto now = std::filesystem::file_time_type::clock::now();
  const auto active_normalized = normalized_absolute(active_path);
  const auto active_sidecar =
    active_normalized.parent_path() / (active_normalized.stem().string() + ".json");

  for (const auto& root : roots) {
    std::error_code ec;
    if (root.empty() || !std::filesystem::exists(root, ec)) {
      continue;
    }

    std::filesystem::recursive_directory_iterator it(
      root, std::filesystem::directory_options::skip_permission_denied, ec
    );
    std::filesystem::recursive_directory_iterator end;
    if (ec) {
      result.errors.push_back(root.string() + ": " + ec.message());
      continue;
    }

    while (it != end) {
      const auto path = it->path();
      std::error_code entry_ec;
      const auto status = it->symlink_status(entry_ec);
      if (!entry_ec && std::filesystem::is_regular_file(status) &&
          is_recording_cleanup_candidate(path) && !same_normalized_path(path, active_normalized) &&
          !same_normalized_path(path, active_sidecar)) {
        const auto normalized = normalized_absolute(path);
        if (seen_paths.insert(normalized.string()).second) {
          const auto modified_at = std::filesystem::last_write_time(path, entry_ec);
          if (!entry_ec && min_age_sec > 0 &&
              now - modified_at < std::chrono::seconds(min_age_sec)) {
            it.increment(ec);
            if (ec) {
              result.errors.push_back(path.string() + ": " + ec.message());
              ec.clear();
            }
            continue;
          }

          const auto size = std::filesystem::file_size(path, entry_ec);
          if (!entry_ec) {
            candidates.push_back({path, modified_at, size});
          }
        }
      }

      it.increment(ec);
      if (ec) {
        result.errors.push_back(path.string() + ": " + ec.message());
        ec.clear();
      }
    }
  }

  std::sort(candidates.begin(), candidates.end(), [](const Candidate& lhs, const Candidate& rhs) {
    return lhs.modified_at < rhs.modified_at;
  });

  for (const auto& candidate : candidates) {
    if (result.projected_used_bytes <= target_used_bytes) {
      break;
    }

    std::error_code ec;
    const bool removed = std::filesystem::remove(candidate.path, ec);
    if (ec || !removed) {
      result.errors.push_back(
        candidate.path.string() + ": " + (ec ? ec.message() : "remove returned false")
      );
      continue;
    }

    ++result.files_removed;
    result.bytes_removed += candidate.size;
    result.removed_paths.push_back(candidate.path.string());
    result.projected_used_bytes = candidate.size >= result.projected_used_bytes
                                    ? 0
                                    : result.projected_used_bytes - candidate.size;
  }

  return result;
}

DiskUsagePathSnapshot DiskUsageMonitor::collect_path_snapshot(
  const DiskUsagePathConfig& path_config
) {
  DiskUsagePathSnapshot snapshot;
  snapshot.role = path_config.role;
  snapshot.path = normalized_absolute(path_config.path).string();

  std::error_code ec;
  snapshot.exists = std::filesystem::exists(path_config.path, ec);
  if (ec) {
    snapshot.error = ec.message();
    ec.clear();
  }

  snapshot.used_bytes = path_used_bytes(path_config.path, snapshot.error);

  const auto space_path = nearest_existing_path(path_config.path);
  auto space = std::filesystem::space(space_path, ec);
  if (!ec) {
    snapshot.capacity_bytes = space.capacity;
    snapshot.free_bytes = space.free;
    snapshot.available_bytes = space.available;
  } else if (snapshot.error.empty()) {
    snapshot.error = ec.message();
  }

  return snapshot;
}

uint64_t DiskUsageMonitor::path_used_bytes(const std::filesystem::path& path, std::string& error) {
  std::error_code ec;
  if (!std::filesystem::exists(path, ec)) {
    if (ec && error.empty()) {
      error = ec.message();
    }
    return 0;
  }

  const auto status = std::filesystem::symlink_status(path, ec);
  if (ec) {
    if (error.empty()) {
      error = ec.message();
    }
    return 0;
  }

  if (std::filesystem::is_regular_file(status)) {
    auto size = std::filesystem::file_size(path, ec);
    if (ec) {
      if (error.empty()) {
        error = ec.message();
      }
      return 0;
    }
    return size;
  }

  if (!std::filesystem::is_directory(status)) {
    return 0;
  }

  uint64_t total = 0;
  std::filesystem::recursive_directory_iterator it(
    path, std::filesystem::directory_options::skip_permission_denied, ec
  );
  std::filesystem::recursive_directory_iterator end;
  if (ec) {
    if (error.empty()) {
      error = ec.message();
    }
    return 0;
  }

  while (it != end) {
    std::error_code entry_ec;
    const auto entry_status = it->symlink_status(entry_ec);
    if (!entry_ec && std::filesystem::is_regular_file(entry_status)) {
      auto size = std::filesystem::file_size(it->path(), entry_ec);
      if (!entry_ec) {
        total += size;
      } else if (error.empty()) {
        error = entry_ec.message();
      }
    }

    it.increment(ec);
    if (ec) {
      if (error.empty()) {
        error = ec.message();
      }
      ec.clear();
    }
  }

  return total;
}

}  // namespace recorder
}  // namespace axon
