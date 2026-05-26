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
#include <thread>
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
          {{"id", "recording_data"},
           {"available", true},
           {"available_bytes", disk_available_bytes}},
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

axon::system::AlertSinkConfig file_sink(const std::filesystem::path& path) {
  axon::system::AlertSinkConfig sink;
  sink.type = "file";
  sink.path = path;
  return sink;
}

axon::system::AlertSinkConfig ops_http_sink(const std::string& url, int timeout_ms) {
  axon::system::AlertSinkConfig sink;
  sink.type = "ops_http";
  sink.url = url;
  sink.timeout_ms = timeout_ms;
  return sink;
}

}  // namespace

int main() {
  const auto root = make_temp_dir("axon_system_alerts");
  try {
    const auto alert_file = root / "alerts.jsonl";

    axon::system::AlertEvaluatorOptions options;
    options.state_dir = root;
    options.sinks = {file_sink(alert_file)};
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
    failing_options.sinks = {file_sink(root)};
    failing_options.rules = {
      {"recorder_unavailable", "critical", "", "", 0.0, {}, "recorder", "unavailable", 0},
    };
    axon::system::AlertEvaluator failing(failing_options);
    alerts = failing.evaluate(snapshot("unavailable", 50.0));
    require(alerts["delivery"]["queued_count"].get<std::size_t>() == 1, "queued delivery");
    require(!alerts["last_delivery_error"].get<std::string>().empty(), "delivery error");

    const auto recoverable_parent = root / "recoverable_parent";
    {
      std::ofstream blocker(recoverable_parent);
      blocker << "not a directory";
    }
    axon::system::AlertEvaluatorOptions recoverable_options;
    recoverable_options.state_dir = root;
    recoverable_options.sinks = {file_sink(recoverable_parent / "alerts.jsonl")};
    recoverable_options.rules = {
      {"recorder_unavailable", "critical", "", "", 0.0, {}, "recorder", "unavailable", 0},
    };
    axon::system::AlertEvaluator recoverable(recoverable_options);
    alerts = recoverable.evaluate(snapshot("unavailable", 50.0));
    require(alerts["delivery"]["queued_count"].get<std::size_t>() == 1, "recoverable queued");
    require(
      !alerts["last_delivery_error"].get<std::string>().empty(), "recoverable delivery error"
    );

    std::filesystem::remove(recoverable_parent);
    std::filesystem::create_directories(recoverable_parent);
    std::this_thread::sleep_for(std::chrono::milliseconds(2100));
    alerts = recoverable.evaluate(snapshot("unavailable", 50.0));
    require(alerts["delivery"]["queued_count"].get<std::size_t>() == 0, "recoverable drained");
    require(alerts["last_delivery_error"].get<std::string>().empty(), "recoverable error cleared");

    axon::system::AlertEvaluatorOptions ops_http_options;
    ops_http_options.state_dir = root;
    ops_http_options.sinks = {ops_http_sink("http://127.0.0.1:1/ops/alerts", 50)};
    ops_http_options.rules = {
      {"recorder_unavailable", "critical", "", "", 0.0, {}, "recorder", "unavailable", 0},
    };
    axon::system::AlertEvaluator ops_http(ops_http_options);
    alerts = ops_http.evaluate(snapshot("unavailable", 50.0));
    require(alerts["delivery"]["queued_count"].get<std::size_t>() == 1, "ops_http queued");
    require(!alerts["last_delivery_error"].get<std::string>().empty(), "ops_http delivery error");

    axon::system::AlertEvaluatorOptions unsupported_options;
    unsupported_options.state_dir = root;
    axon::system::AlertSinkConfig unsupported_sink;
    unsupported_sink.type = "pagerduty";
    unsupported_options.sinks = {unsupported_sink};
    unsupported_options.rules = {
      {"recorder_unavailable", "critical", "", "", 0.0, {}, "recorder", "unavailable", 0},
    };
    bool unsupported_sink_rejected = false;
    try {
      axon::system::AlertEvaluator unsupported(unsupported_options);
    } catch (const std::invalid_argument& ex) {
      unsupported_sink_rejected =
        std::string(ex.what()).find("unsupported alert sink type: pagerduty") != std::string::npos;
    }
    require(unsupported_sink_rejected, "unsupported sink should be rejected");

    std::filesystem::remove_all(root);
    return 0;
  } catch (const std::exception& ex) {
    std::cerr << ex.what() << std::endl;
    std::filesystem::remove_all(root);
    return 1;
  }
}
