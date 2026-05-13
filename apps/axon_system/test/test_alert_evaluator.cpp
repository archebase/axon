// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unistd.h>

#include "alert_evaluator.hpp"

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

nlohmann::json snapshot(
  const std::string& recorder_status, double memory_used_percent,
  std::uint64_t disk_available_bytes = 1024
) {
  return {
    {"service", {{"name", "axon-system"}, {"pid", static_cast<int>(getpid())}}},
    {"resources",
     {
       {"memory", {{"available", true}, {"used_percent", memory_used_percent}}},
       {"disk",
        nlohmann::json::array({
          {{"id", "recording_data"}, {"available", true}, {"available_bytes", disk_available_bytes}
          },
        })},
     }},
    {"processes",
     {
       {"recorder",
        {
          {"id", "recorder"},
          {"binary", "axon-recorder"},
          {"pid", nullptr},
          {"status", recorder_status},
        }},
     }},
  };
}

std::string read_file(const std::filesystem::path& path) {
  std::ifstream input(path);
  std::ostringstream stream;
  stream << input.rdbuf();
  return stream.str();
}

}  // namespace

int main() {
  const auto root = make_temp_dir("axon_system_alerts");
  try {
    const auto alert_file = root / "alerts.jsonl";

    axon::system::AlertEvaluatorOptions options;
    options.state_dir = root;
    options.sinks = {{"file", alert_file}};
    options.rules = {
      {"recorder_unavailable", "critical", "", "", 0.0, {}, "recorder", "unavailable", 0},
      {"memory_high", "warning", "memory.used_percent", "gt", 90.0, {}, "", "", 10},
      {"disk_low",
       "critical",
       "disk.available_bytes",
       "lt",
       2048.0,
       {{"path_id", "recording_data"}},
       "",
       "",
       0},
    };

    axon::system::AlertEvaluator evaluator(options);
    auto alerts = evaluator.evaluate(snapshot("unavailable", 95.0));
    require(alerts["firing_count"].get<int>() == 2, "firing count");
    require(alerts["pending_count"].get<int>() == 1, "pending count");
    require(
      alerts["rules"]["recorder_unavailable"]["status"].get<std::string>() == "firing",
      "recorder firing"
    );
    require(
      alerts["rules"]["memory_high"]["status"].get<std::string>() == "pending", "memory pending"
    );
    require(alerts["events"].size() == 2, "firing events");
    const auto file_content = read_file(alert_file);
    require(
      file_content.find("recorder_unavailable") != std::string::npos,
      "file sink: " + alerts.dump() + " content: " + file_content
    );

    alerts = evaluator.evaluate(snapshot("healthy", 50.0, 4096));
    require(alerts["resolved_count"].get<int>() == 2, "resolved count: " + alerts.dump());
    require(alerts["events"].size() == 4, "resolved events");
    require(
      alerts["rules"]["recorder_unavailable"]["status"].get<std::string>() == "resolved",
      "recorder resolved"
    );

    axon::system::AlertEvaluatorOptions failing_options;
    failing_options.state_dir = root;
    failing_options.sinks = {{"file", root}};
    failing_options.rules = {
      {"recorder_unavailable", "critical", "", "", 0.0, {}, "recorder", "unavailable", 0},
    };
    axon::system::AlertEvaluator failing(failing_options);
    alerts = failing.evaluate(snapshot("unavailable", 50.0));
    require(alerts["delivery"]["queued_count"].get<std::size_t>() == 1, "queued delivery");
    require(!alerts["last_delivery_error"].get<std::string>().empty(), "delivery error");

    std::filesystem::remove_all(root);
    return 0;
  } catch (const std::exception& ex) {
    std::cerr << ex.what() << std::endl;
    std::filesystem::remove_all(root);
    return 1;
  }
}
