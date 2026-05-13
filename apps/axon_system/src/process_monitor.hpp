// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_SYSTEM_PROCESS_MONITOR_HPP
#define AXON_SYSTEM_PROCESS_MONITOR_HPP

#include <nlohmann/json.hpp>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace axon {
namespace system {

struct ProcessHttpProbeConfig {
  std::string type;
  std::string url;
  int timeout_ms = 350;
};

struct ProcessTargetConfig {
  std::string id;
  std::string executable;
  std::filesystem::path pid_file;
  std::vector<std::filesystem::path> pid_file_candidates;
  std::vector<std::string> cmdline_contains;
  std::optional<ProcessHttpProbeConfig> rpc;
};

struct ProcessMonitorOptions {
  std::filesystem::path proc_root = "/proc";
  std::vector<ProcessTargetConfig> targets;
  int process_sample_cadence_ms = 2000;
};

class ProcessMonitor {
public:
  explicit ProcessMonitor(ProcessMonitorOptions options = {});

  nlohmann::json collect();
  nlohmann::json cadence_json() const;

private:
  struct ProcessStat {
    std::uint64_t user_ticks = 0;
    std::uint64_t system_ticks = 0;
    std::uint64_t start_time_ticks = 0;
    char state = '?';
  };

  struct ProcessIo {
    std::uint64_t read_bytes = 0;
    std::uint64_t write_bytes = 0;
    bool available = false;
  };

  struct ProcessCandidate {
    int pid = 0;
    std::string source;
    std::filesystem::path pid_file;
    std::uint64_t start_time_ticks = 0;
    bool command_matches = true;
    std::string message;
  };

  struct PreviousUsage {
    std::uint64_t total_ticks = 0;
    std::chrono::steady_clock::time_point sampled_at;
  };

  nlohmann::json collect_target(const ProcessTargetConfig& target);
  std::vector<ProcessCandidate> discover_candidates(const ProcessTargetConfig& target) const;
  std::optional<ProcessCandidate> candidate_from_pid_file(
    const ProcessTargetConfig& target, const std::filesystem::path& pid_file,
    const std::string& source
  ) const;
  std::vector<std::filesystem::path> expand_candidate_paths(const std::filesystem::path& pattern
  ) const;
  bool process_exists(int pid) const;
  bool command_matches_target(int pid, const ProcessTargetConfig& target) const;
  std::string read_cmdline(int pid) const;
  std::optional<ProcessStat> read_stat(int pid, std::string* error) const;
  std::optional<std::uint64_t> read_rss_bytes(int pid, std::string* error) const;
  ProcessIo read_io(int pid) const;
  nlohmann::json process_resources(int pid, const ProcessStat& stat);
  nlohmann::json probe_health(const ProcessTargetConfig& target) const;

  static std::vector<std::string> split_cmdline(const std::string& cmdline);
  static bool parse_stat_content(const std::string& content, ProcessStat* stat, std::string* error);
  static bool parse_unsigned_file(
    const std::filesystem::path& path, std::uint64_t* value, std::string* error
  );
  static bool wildcard_match(const std::string& pattern, const std::string& value);
  static std::string basename(const std::string& path);

  ProcessMonitorOptions options_;
  long clock_ticks_per_second_;
  long page_size_bytes_;
  std::map<int, PreviousUsage> previous_usage_;
};

std::vector<ProcessTargetConfig> default_process_targets();

}  // namespace system
}  // namespace axon

#endif  // AXON_SYSTEM_PROCESS_MONITOR_HPP
