// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_SYSTEM_RESOURCE_COLLECTOR_HPP
#define AXON_SYSTEM_RESOURCE_COLLECTOR_HPP

#include <nlohmann/json.hpp>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "procfs_reader.hpp"

namespace axon {
namespace system {

struct DiskPathConfig {
  std::string id;
  std::filesystem::path path;
};

struct ResourceCollectorOptions {
  std::filesystem::path proc_root = "/proc";
  std::vector<DiskPathConfig> disk_paths;
  std::vector<std::string> network_interfaces;
  int resource_sample_cadence_ms = 1000;
  int disk_sample_cadence_ms = 5000;
};

class ResourceCollector {
public:
  explicit ResourceCollector(ResourceCollectorOptions options = {});

  nlohmann::json collect(bool refresh_disk = true);
  nlohmann::json cadence_json() const;

private:
  struct CpuTimes {
    std::uint64_t user = 0;
    std::uint64_t nice = 0;
    std::uint64_t system = 0;
    std::uint64_t idle = 0;
    std::uint64_t iowait = 0;
    std::uint64_t irq = 0;
    std::uint64_t softirq = 0;
    std::uint64_t steal = 0;
  };

  struct NetworkCounters {
    std::uint64_t rx_bytes = 0;
    std::uint64_t tx_bytes = 0;
  };

  nlohmann::json collect_cpu();
  nlohmann::json collect_memory() const;
  nlohmann::json collect_disk() const;
  nlohmann::json collect_network();

  static bool parse_cpu_times(const std::string& content, CpuTimes* times, std::string* error);
  static bool parse_loadavg(
    const std::string& content, double* load1, double* load5, double* load15
  );
  static bool parse_meminfo(const std::string& content, nlohmann::json* memory, std::string* error);
  static bool parse_net_dev(
    const std::string& content, std::map<std::string, NetworkCounters>* counters, std::string* error
  );
  static std::uint64_t cpu_total(const CpuTimes& times);
  static std::uint64_t cpu_idle(const CpuTimes& times);
  static std::string now_iso8601();
  static std::string trim(const std::string& value);

  ResourceCollectorOptions options_;
  ProcfsReader procfs_;
  std::optional<CpuTimes> previous_cpu_;
  std::map<std::string, NetworkCounters> previous_network_;
  std::chrono::steady_clock::time_point previous_network_time_;
  nlohmann::json cached_disk_ = nlohmann::json::array();
  bool has_cached_disk_ = false;
  std::string disk_collected_at_;
};

}  // namespace system
}  // namespace axon

#endif  // AXON_SYSTEM_RESOURCE_COLLECTOR_HPP
