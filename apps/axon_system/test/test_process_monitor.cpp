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

#include "process_monitor.hpp"

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
  std::ofstream output(path, std::ios::binary);
  if (!output) {
    throw std::runtime_error("failed to write " + path.string());
  }
  output << content;
}

std::string stat_line(
  int pid, const std::string& comm, std::uint64_t utime, std::uint64_t stime,
  std::uint64_t start_time
) {
  return std::to_string(pid) + " (" + comm + ") S 1 1 1 0 0 0 0 0 0 0 " + std::to_string(utime) +
         " " + std::to_string(stime) + " 0 0 20 0 1 0 " + std::to_string(start_time) +
         " 0 0 0 0 0 0 0 0 0\n";
}

void write_process(
  const std::filesystem::path& proc, int pid, const std::string& executable, std::uint64_t utime,
  std::uint64_t stime, std::uint64_t start_time, std::uint64_t rss_pages, std::uint64_t read_bytes,
  std::uint64_t write_bytes
) {
  const auto pid_dir = proc / std::to_string(pid);
  write_file(pid_dir / "cmdline", "/opt/axon/bin/" + executable + std::string("\0--flag\0", 8));
  write_file(pid_dir / "stat", stat_line(pid, executable, utime, stime, start_time));
  write_file(pid_dir / "statm", "1000 " + std::to_string(rss_pages) + " 0 0 0 0 0\n");
  write_file(
    pid_dir / "io",
    "rchar: 0\n"
    "wchar: 0\n"
    "read_bytes: " +
      std::to_string(read_bytes) +
      "\n"
      "write_bytes: " +
      std::to_string(write_bytes) + "\n"
  );
}

}  // namespace

int main() {
  const auto root = make_temp_dir("axon_system_processes");
  try {
    const auto proc = root / "proc";
    const auto agent = root / "agent";
    write_process(proc, 100, "axon-recorder", 10, 5, 1000, 25, 4096, 8192);
    write_process(proc, 200, "axon-transfer", 7, 3, 900, 30, 1024, 2048);
    write_process(proc, 300, "other-process", 1, 1, 1100, 5, 0, 0);
    write_file(agent / "demo_transfer.pid", "200\n");
    write_file(agent / "wrong_recorder.pid", "300\n");

    axon::system::ProcessMonitorOptions options;
    options.proc_root = proc;
    options.targets = {
      {"recorder", "axon-recorder", {}, {}, {}, std::nullopt},
      {"transfer", "axon-transfer", {}, {agent / "*_transfer.pid"}, {}, std::nullopt},
      {"missing", "axon-missing", {}, {}, {}, std::nullopt},
      {"wrong_pid", "axon-recorder", agent / "wrong_recorder.pid", {}, {}, std::nullopt},
    };

    axon::system::ProcessMonitor monitor(options);
    auto first = monitor.collect();
    require(first["recorder"]["status"].get<std::string>() == "healthy", "recorder status");
    require(first["recorder"]["pid"].get<int>() == 100, "recorder pid");
    require(first["recorder"]["source"].get<std::string>() == "process_match", "recorder source");
    require(
      first["recorder"]["resources"]["rss_bytes"].get<std::uint64_t>() ==
        25 * static_cast<std::uint64_t>(sysconf(_SC_PAGESIZE)),
      "recorder rss"
    );
    require(
      first["recorder"]["resources"]["io"]["read_bytes"].get<std::uint64_t>() == 4096,
      "recorder read bytes"
    );
    require(first["transfer"]["status"].get<std::string>() == "healthy", "transfer status");
    require(first["transfer"]["pid"].get<int>() == 200, "transfer pid");
    require(
      first["transfer"]["source"].get<std::string>() == "pid_file_candidate", "transfer source"
    );
    require(first["missing"]["status"].get<std::string>() == "unavailable", "missing status");
    require(first["wrong_pid"]["status"].get<std::string>() == "unhealthy", "wrong pid status");

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    write_process(proc, 100, "axon-recorder", 20, 10, 1000, 25, 4096, 8192);
    auto second = monitor.collect();
    require(
      second["recorder"]["resources"]["cpu_percent"].get<double>() > 0.0, "recorder cpu delta"
    );

    std::filesystem::remove_all(root);
    return 0;
  } catch (const std::exception& ex) {
    std::cerr << ex.what() << std::endl;
    std::filesystem::remove_all(root);
    return 1;
  }
}
