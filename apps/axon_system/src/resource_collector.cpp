// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "resource_collector.hpp"

#include <algorithm>
#include <ctime>
#include <sstream>
#include <system_error>
#include <utility>

namespace axon {
namespace system {

namespace fs = std::filesystem;

namespace {

constexpr double kBytesPerKiB = 1024.0;

bool contains_interface(const std::vector<std::string>& interfaces, const std::string& name) {
  return interfaces.empty() ||
         std::find(interfaces.begin(), interfaces.end(), name) != interfaces.end();
}

}  // namespace

ResourceCollector::ResourceCollector(ResourceCollectorOptions options)
    : options_(std::move(options))
    , procfs_(options_.proc_root) {}

nlohmann::json ResourceCollector::collect() {
  return {
    {"collected_at", now_iso8601()},
    {"sample_cadence_ms", cadence_json()},
    {"cpu", collect_cpu()},
    {"memory", collect_memory()},
    {"disk", collect_disk()},
    {"network", collect_network()},
  };
}

nlohmann::json ResourceCollector::cadence_json() const {
  return {
    {"resources", options_.resource_sample_cadence_ms},
    {"disk", options_.disk_sample_cadence_ms},
    {"unit", "milliseconds"},
  };
}

nlohmann::json ResourceCollector::collect_cpu() {
  std::string stat_content;
  std::string error;
  CpuTimes current;
  if (!procfs_.read_file("stat", &stat_content, &error) ||
      !parse_cpu_times(stat_content, &current, &error)) {
    return {{"available", false}, {"error", error}, {"usage_unit", "percent"}};
  }

  nlohmann::json cpu = {
    {"available", true},
    {"usage_unit", "percent"},
  };

  if (previous_cpu_.has_value()) {
    const auto previous_total = cpu_total(previous_cpu_.value());
    const auto current_total = cpu_total(current);
    const auto previous_idle = cpu_idle(previous_cpu_.value());
    const auto current_idle = cpu_idle(current);
    const auto total_delta = current_total > previous_total ? current_total - previous_total : 0;
    const auto idle_delta = current_idle > previous_idle ? current_idle - previous_idle : 0;

    if (total_delta > 0) {
      const auto busy_delta = total_delta > idle_delta ? total_delta - idle_delta : 0;
      cpu["usage_percent"] =
        (static_cast<double>(busy_delta) * 100.0) / static_cast<double>(total_delta);
    } else {
      cpu["usage_percent"] = 0.0;
      cpu["message"] = "cpu counters did not advance";
    }
  } else {
    cpu["usage_percent"] = 0.0;
    cpu["message"] = "initial cpu sample";
  }
  previous_cpu_ = current;

  std::string load_content;
  double load1 = 0.0;
  double load5 = 0.0;
  double load15 = 0.0;
  if (procfs_.read_file("loadavg", &load_content, nullptr) &&
      parse_loadavg(load_content, &load1, &load5, &load15)) {
    cpu["load1"] = load1;
    cpu["load5"] = load5;
    cpu["load15"] = load15;
    cpu["load_unit"] = "load_average";
  }

  return cpu;
}

nlohmann::json ResourceCollector::collect_memory() const {
  std::string content;
  std::string error;
  nlohmann::json memory;
  if (!procfs_.read_file("meminfo", &content, &error) || !parse_meminfo(content, &memory, &error)) {
    return {{"available", false}, {"error", error}, {"unit", "bytes"}};
  }
  return memory;
}

nlohmann::json ResourceCollector::collect_disk() const {
  nlohmann::json disks = nlohmann::json::array();
  for (const auto& config : options_.disk_paths) {
    const auto measured_path = nearest_existing_path(config.path);
    std::error_code ec;
    const auto space = fs::space(measured_path, ec);
    nlohmann::json disk = {
      {"id", config.id},
      {"path", config.path.lexically_normal().string()},
      {"measured_path", measured_path.lexically_normal().string()},
      {"unit", "bytes"},
    };

    if (ec) {
      disk["available"] = false;
      disk["error"] = ec.message();
    } else {
      const auto used_bytes = space.capacity >= space.free ? space.capacity - space.free : 0;
      disk["available"] = true;
      disk["capacity_bytes"] = space.capacity;
      disk["free_bytes"] = space.free;
      disk["available_bytes"] = space.available;
      disk["used_bytes"] = used_bytes;
      disk["used_percent"] = space.capacity > 0 ? (static_cast<double>(used_bytes) * 100.0) /
                                                    static_cast<double>(space.capacity)
                                                : 0.0;
    }
    disks.push_back(disk);
  }
  return disks;
}

nlohmann::json ResourceCollector::collect_network() {
  std::string content;
  std::string error;
  std::map<std::string, NetworkCounters> current;
  if (!procfs_.read_file("net/dev", &content, &error) ||
      !parse_net_dev(content, &current, &error)) {
    return {{"available", false}, {"error", error}, {"unit", "bytes"}};
  }

  const auto now = std::chrono::steady_clock::now();
  const double elapsed_sec = previous_network_time_.time_since_epoch().count() > 0
                               ? std::chrono::duration<double>(now - previous_network_time_).count()
                               : 0.0;

  nlohmann::json interfaces = nlohmann::json::array();
  for (const auto& [name, counters] : current) {
    if (!contains_interface(options_.network_interfaces, name)) {
      continue;
    }

    double rx_rate = 0.0;
    double tx_rate = 0.0;
    const auto previous = previous_network_.find(name);
    if (previous != previous_network_.end() && elapsed_sec > 0.0) {
      const auto rx_delta = counters.rx_bytes >= previous->second.rx_bytes
                              ? counters.rx_bytes - previous->second.rx_bytes
                              : 0;
      const auto tx_delta = counters.tx_bytes >= previous->second.tx_bytes
                              ? counters.tx_bytes - previous->second.tx_bytes
                              : 0;
      rx_rate = static_cast<double>(rx_delta) / elapsed_sec;
      tx_rate = static_cast<double>(tx_delta) / elapsed_sec;
    }

    interfaces.push_back({
      {"interface", name},
      {"available", true},
      {"rx_bytes", counters.rx_bytes},
      {"tx_bytes", counters.tx_bytes},
      {"rx_bytes_per_sec", rx_rate},
      {"tx_bytes_per_sec", tx_rate},
      {"unit", "bytes"},
      {"rate_unit", "bytes_per_second"},
    });
  }

  previous_network_ = current;
  previous_network_time_ = now;

  return interfaces;
}

bool ResourceCollector::parse_cpu_times(
  const std::string& content, CpuTimes* times, std::string* error
) {
  if (times == nullptr) {
    if (error != nullptr) {
      *error = "cpu output pointer is null";
    }
    return false;
  }

  std::istringstream stream(content);
  std::string label;
  if (!(stream >> label) || label != "cpu") {
    if (error != nullptr) {
      *error = "missing aggregate cpu line";
    }
    return false;
  }

  if (!(stream >> times->user >> times->nice >> times->system >> times->idle >> times->iowait >>
        times->irq >> times->softirq >> times->steal)) {
    if (error != nullptr) {
      *error = "failed to parse aggregate cpu counters";
    }
    return false;
  }
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

bool ResourceCollector::parse_loadavg(
  const std::string& content, double* load1, double* load5, double* load15
) {
  std::istringstream stream(content);
  return static_cast<bool>(stream >> *load1 >> *load5 >> *load15);
}

bool ResourceCollector::parse_meminfo(
  const std::string& content, nlohmann::json* memory, std::string* error
) {
  if (memory == nullptr) {
    if (error != nullptr) {
      *error = "memory output pointer is null";
    }
    return false;
  }

  std::map<std::string, std::uint64_t> values_kib;
  std::istringstream stream(content);
  std::string key;
  std::uint64_t value = 0;
  std::string unit;
  while (stream >> key >> value) {
    if (!key.empty() && key.back() == ':') {
      key.pop_back();
    }
    values_kib[key] = value;
    std::getline(stream, unit);
  }

  const auto total_it = values_kib.find("MemTotal");
  if (total_it == values_kib.end()) {
    if (error != nullptr) {
      *error = "MemTotal not found in meminfo";
    }
    return false;
  }

  const auto total_bytes = static_cast<std::uint64_t>(total_it->second * kBytesPerKiB);
  const auto free_bytes = static_cast<std::uint64_t>(values_kib["MemFree"] * kBytesPerKiB);
  const auto available_kib =
    values_kib.count("MemAvailable") > 0 ? values_kib["MemAvailable"] : values_kib["MemFree"];
  const auto available_bytes = static_cast<std::uint64_t>(available_kib * kBytesPerKiB);
  const auto buffers_bytes = static_cast<std::uint64_t>(values_kib["Buffers"] * kBytesPerKiB);
  const auto cached_bytes = static_cast<std::uint64_t>(values_kib["Cached"] * kBytesPerKiB);
  const auto used_bytes = total_bytes >= available_bytes ? total_bytes - available_bytes : 0;

  *memory = {
    {"available", true},
    {"unit", "bytes"},
    {"total_bytes", total_bytes},
    {"free_bytes", free_bytes},
    {"available_bytes", available_bytes},
    {"used_bytes", used_bytes},
    {"used_percent",
     total_bytes > 0 ? (static_cast<double>(used_bytes) * 100.0) / static_cast<double>(total_bytes)
                     : 0.0},
    {"buffers_bytes", buffers_bytes},
    {"cached_bytes", cached_bytes},
  };

  if (values_kib.count("SwapTotal") > 0) {
    const auto swap_total = static_cast<std::uint64_t>(values_kib["SwapTotal"] * kBytesPerKiB);
    const auto swap_free = static_cast<std::uint64_t>(values_kib["SwapFree"] * kBytesPerKiB);
    (*memory)["swap_total_bytes"] = swap_total;
    (*memory)["swap_free_bytes"] = swap_free;
    (*memory)["swap_used_bytes"] = swap_total >= swap_free ? swap_total - swap_free : 0;
  }

  if (error != nullptr) {
    error->clear();
  }
  return true;
}

bool ResourceCollector::parse_net_dev(
  const std::string& content, std::map<std::string, NetworkCounters>* counters, std::string* error
) {
  if (counters == nullptr) {
    if (error != nullptr) {
      *error = "network output pointer is null";
    }
    return false;
  }

  std::istringstream stream(content);
  std::string line;
  std::getline(stream, line);
  std::getline(stream, line);

  counters->clear();
  while (std::getline(stream, line)) {
    const auto colon = line.find(':');
    if (colon == std::string::npos) {
      continue;
    }
    const auto name = trim(line.substr(0, colon));
    std::istringstream values(line.substr(colon + 1));
    NetworkCounters item;
    std::uint64_t ignored = 0;
    if (!(values >> item.rx_bytes >> ignored >> ignored >> ignored >> ignored >> ignored >>
          ignored >> ignored >> item.tx_bytes)) {
      continue;
    }
    (*counters)[name] = item;
  }

  if (counters->empty()) {
    if (error != nullptr) {
      *error = "no network interfaces found";
    }
    return false;
  }
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

std::uint64_t ResourceCollector::cpu_total(const CpuTimes& times) {
  return times.user + times.nice + times.system + times.idle + times.iowait + times.irq +
         times.softirq + times.steal;
}

std::uint64_t ResourceCollector::cpu_idle(const CpuTimes& times) {
  return times.idle + times.iowait;
}

std::string ResourceCollector::now_iso8601() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t time = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  gmtime_r(&time, &tm);

  char buffer[32];
  std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &tm);
  return buffer;
}

std::string ResourceCollector::trim(const std::string& value) {
  const auto begin = value.find_first_not_of(" \t\n\r");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = value.find_last_not_of(" \t\n\r");
  return value.substr(begin, end - begin + 1);
}

std::filesystem::path ResourceCollector::nearest_existing_path(const std::filesystem::path& path) {
  fs::path probe = path;
  std::error_code ec;
  while (!probe.empty() && !fs::exists(probe, ec)) {
    const auto parent = probe.parent_path();
    if (parent.empty() || parent == probe) {
      break;
    }
    probe = parent;
  }
  return probe.empty() ? path : probe;
}

}  // namespace system
}  // namespace axon
