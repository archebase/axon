// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>

#include "resource_collector.hpp"

namespace {

std::filesystem::path make_temp_dir(const std::string& name) {
  const auto base = std::filesystem::temp_directory_path() /
                    (name + "_" + std::to_string(getpid()) + "_" +
                     std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::create_directories(base);
  return base;
}

void require(bool condition, const std::string& message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void write_file(const std::filesystem::path& path, const std::string& content) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("failed to write " + path.string());
  }
  output << content;
}

}  // namespace

int main() {
  const auto root = make_temp_dir("axon_system_resources");
  try {
    const auto proc = root / "proc";
    write_file(
      proc / "stat",
      "cpu  100 0 50 850 0 0 0 0 0 0\n"
      "cpu0 100 0 50 850 0 0 0 0 0 0\n"
    );
    write_file(proc / "loadavg", "0.25 0.50 0.75 1/100 12345\n");
    write_file(
      proc / "meminfo",
      "MemTotal:       100000 kB\n"
      "MemFree:         20000 kB\n"
      "MemAvailable:    40000 kB\n"
      "Buffers:          5000 kB\n"
      "Cached:          15000 kB\n"
      "SwapTotal:       10000 kB\n"
      "SwapFree:         7000 kB\n"
    );
    write_file(
      proc / "net/dev",
      "Inter-|   Receive                                                |  Transmit\n"
      " face |bytes    packets errs drop fifo frame compressed multicast|bytes    packets errs "
      "drop fifo colls carrier compressed\n"
      "  lo: 10 0 0 0 0 0 0 0 20 0 0 0 0 0 0 0\n"
      "eth0: 1000 1 0 0 0 0 0 0 2000 2 0 0 0 0 0 0\n"
    );

    axon::system::ResourceCollectorOptions options;
    options.proc_root = proc;
    options.disk_paths = {{"fixture_disk", root / "missing" / "recording"}};
    options.network_interfaces = {"eth0"};

    axon::system::ResourceCollector collector(options);
    auto first = collector.collect();
    require(first["cpu"]["available"].get<bool>(), "cpu should be available");
    require(first["cpu"]["usage_unit"].get<std::string>() == "percent", "cpu unit mismatch");
    require(first["memory"]["available"].get<bool>(), "memory should be available");
    require(first["memory"]["unit"].get<std::string>() == "bytes", "memory unit mismatch");
    require(first["memory"]["total_bytes"].get<std::uint64_t>() == 102400000, "memory total");
    require(first["disk"].at(0)["available"].get<bool>(), "disk should be available");
    require(
      first["disk"].at(0)["measured_path"].get<std::string>() == root.lexically_normal().string(),
      "disk measured path should fall back to existing parent"
    );
    require(first["network"].size() == 1, "network interface filter mismatch");
    require(first["network"].at(0)["interface"].get<std::string>() == "eth0", "network name");
    require(first["network"].at(0)["rx_bytes"].get<std::uint64_t>() == 1000, "rx bytes");

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    write_file(
      proc / "stat",
      "cpu  150 0 70 880 0 0 0 0 0 0\n"
      "cpu0 150 0 70 880 0 0 0 0 0 0\n"
    );
    write_file(
      proc / "net/dev",
      "Inter-|   Receive                                                |  Transmit\n"
      " face |bytes    packets errs drop fifo frame compressed multicast|bytes    packets errs "
      "drop fifo colls carrier compressed\n"
      "eth0: 1600 1 0 0 0 0 0 0 2600 2 0 0 0 0 0 0\n"
    );

    auto second = collector.collect();
    const auto usage = second["cpu"]["usage_percent"].get<double>();
    require(usage > 69.0 && usage < 71.0, "cpu usage delta mismatch");
    require(second["network"].at(0)["rx_bytes_per_sec"].get<double>() > 0.0, "rx rate missing");
    require(second["network"].at(0)["tx_bytes_per_sec"].get<double>() > 0.0, "tx rate missing");

    std::filesystem::remove(proc / "meminfo");
    auto partial = collector.collect();
    require(!partial["memory"]["available"].get<bool>(), "memory failure should be explicit");
    require(partial["cpu"]["available"].get<bool>(), "cpu should survive memory failure");
    require(partial["network"].is_array(), "network should survive memory failure");

    std::filesystem::remove_all(root);
    return 0;
  } catch (const std::exception& ex) {
    std::cerr << ex.what() << std::endl;
    std::filesystem::remove_all(root);
    return 1;
  }
}
