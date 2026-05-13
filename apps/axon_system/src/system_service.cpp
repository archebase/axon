// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "system_service.hpp"

#include <algorithm>
#include <ctime>
#include <exception>
#include <system_error>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

#ifndef AXON_SYSTEM_VERSION
#define AXON_SYSTEM_VERSION "0.4.0-dev"
#endif

namespace axon {
namespace system {

namespace {

ResourceCollectorOptions make_default_resource_options(const std::filesystem::path& state_dir) {
  ResourceCollectorOptions options;
  options.disk_paths = {
    {"state_dir", state_dir},
  };
  return options;
}

ProcessMonitorOptions make_default_process_options() {
  ProcessMonitorOptions options;
  options.targets = default_process_targets();
  return options;
}

AlertEvaluatorOptions make_default_alert_options(const std::filesystem::path& state_dir) {
  AlertEvaluatorOptions options;
  options.state_dir = state_dir;
  options.rules = default_alert_rules();
  options.sinks = default_alert_sinks();
  return options;
}

SystemServiceOptions normalize_options(SystemServiceOptions options) {
  if (options.resource_options.resource_sample_cadence_ms <= 0) {
    options.resource_options.resource_sample_cadence_ms = 1000;
  }
  if (options.resource_options.disk_sample_cadence_ms <= 0) {
    options.resource_options.disk_sample_cadence_ms = 5000;
  }
  if (options.sampler_idle_sleep_ms <= 0) {
    options.sampler_idle_sleep_ms = 100;
  }
  if (options.resource_options.disk_paths.empty()) {
    options.resource_options.disk_paths =
      make_default_resource_options(options.state_dir).disk_paths;
  }
  if (options.process_options.process_sample_cadence_ms <= 0) {
    options.process_options.process_sample_cadence_ms = 2000;
  }
  if (options.process_options.proc_root.empty()) {
    options.process_options.proc_root = options.resource_options.proc_root;
  }
  if (options.process_options.targets.empty()) {
    options.process_options.targets = default_process_targets();
  }
  if (options.alert_options.evaluate_interval_ms <= 0) {
    options.alert_options.evaluate_interval_ms = 5000;
  }
  if (options.alert_options.state_dir.empty()) {
    options.alert_options.state_dir = options.state_dir;
  }
  if (options.alert_options.rules.empty()) {
    options.alert_options.rules = default_alert_rules();
  }
  if (options.alert_options.sinks.empty()) {
    options.alert_options.sinks = default_alert_sinks();
  }
  return options;
}

SystemServiceOptions default_service_options(std::filesystem::path state_dir) {
  SystemServiceOptions options;
  options.state_dir = std::move(state_dir);
  options.resource_options = make_default_resource_options(options.state_dir);
  options.process_options = make_default_process_options();
  options.alert_options = make_default_alert_options(options.state_dir);
  return options;
}

}  // namespace

SystemService::SystemService(std::filesystem::path state_dir)
    : SystemService(default_service_options(std::move(state_dir))) {}

SystemService::SystemService(SystemServiceOptions options)
    : options_(normalize_options(std::move(options)))
    , state_dir_(options_.state_dir)
    , resource_collector_(options_.resource_options)
    , process_monitor_(options_.process_options)
    , alert_evaluator_(options_.alert_options)
    , started_at_(std::chrono::steady_clock::now())
    , started_at_iso_(now_iso8601()) {}

SystemService::~SystemService() {
  stop_sampler();
}

bool SystemService::initialize(std::string* error) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_dir_.empty()) {
      last_error_ = "state_dir is empty";
      state_ = "degraded";
      if (error != nullptr) {
        *error = last_error_;
      }
      return false;
    }

    std::error_code ec;
    std::filesystem::create_directories(state_dir_, ec);
    if (ec) {
      last_error_ = "failed to create state_dir: " + ec.message();
      state_ = "degraded";
      if (error != nullptr) {
        *error = last_error_;
      }
      return false;
    }

    state_ = "running";
    last_error_.clear();
    sampler_stop_requested_ = false;
    shutdown_requested_ = false;
  }

  sample_resources(true);
  sample_processes();
  sample_alerts();
  start_sampler();

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (error != nullptr) {
      *error = last_error_;
    }
    return (state_ == "running" || state_ == "degraded") && sampler_running_;
  }
}

RpcResponse SystemService::get_health() const {
  std::lock_guard<std::mutex> lock(mutex_);
  const bool healthy =
    (state_ == "running" || state_ == "degraded") && sampler_running_ && !shutdown_requested_;
  nlohmann::json data = {
    {"service", "axon-system"},
    {"state", state_},
    {"sampler_running", sampler_running_},
    {"shutdown_requested", shutdown_requested_},
  };
  if (!last_error_.empty()) {
    data["last_error"] = last_error_;
  }
  return {healthy, healthy ? "ok" : "not running", data};
}

RpcResponse SystemService::get_state() {
  std::lock_guard<std::mutex> lock(mutex_);
  nlohmann::json data = {
    {"service", service_state_json()},
    {"resources", cached_resources_or_placeholder()},
    {"processes", cached_processes_or_placeholder()},
    {"alerts", cached_alerts_or_placeholder()},
  };
  if (!last_error_.empty()) {
    data["last_error"] = last_error_;
  }
  return {true, "ok", data};
}

RpcResponse SystemService::get_metrics() {
  std::lock_guard<std::mutex> lock(mutex_);
  auto data = cached_resources_or_placeholder();
  data["processes"] = cached_processes_or_placeholder();
  data["sample_cadence_ms"] = service_cadence_json();
  return {true, "ok", data};
}

RpcResponse SystemService::get_processes() {
  std::lock_guard<std::mutex> lock(mutex_);
  return {true, "ok", cached_processes_or_placeholder()};
}

RpcResponse SystemService::get_alerts() {
  std::lock_guard<std::mutex> lock(mutex_);
  return {true, "ok", cached_alerts_or_placeholder()};
}

RpcResponse SystemService::request_shutdown() {
  nlohmann::json service;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    shutdown_requested_ = true;
    if (state_ != "stopped") {
      state_ = "stopping";
    }
    service = service_state_json();
  }
  stop_sampler();
  return {true, "shutdown requested", {{"service", service}}};
}

bool SystemService::shutdown_requested() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return shutdown_requested_;
}

void SystemService::mark_stopped() {
  stop_sampler();
  std::lock_guard<std::mutex> lock(mutex_);
  state_ = "stopped";
}

void SystemService::start_sampler() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (sampler_thread_.joinable() || sampler_running_) {
    return;
  }

  sampler_stop_requested_ = false;
  sampler_running_ = true;
  try {
    sampler_thread_ = std::thread(&SystemService::sampler_loop, this);
  } catch (const std::exception& ex) {
    sampler_running_ = false;
    sampler_stop_requested_ = true;
    last_error_ = std::string("failed to start resource sampler: ") + ex.what();
    state_ = "degraded";
  }
}

void SystemService::stop_sampler() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    sampler_stop_requested_ = true;
  }
  sampler_cv_.notify_all();

  if (sampler_thread_.joinable() && sampler_thread_.get_id() != std::this_thread::get_id()) {
    sampler_thread_.join();
  }

  std::lock_guard<std::mutex> lock(mutex_);
  sampler_running_ = false;
}

void SystemService::sampler_loop() {
  const auto resource_cadence =
    std::chrono::milliseconds(options_.resource_options.resource_sample_cadence_ms);
  const auto disk_cadence =
    std::chrono::milliseconds(options_.resource_options.disk_sample_cadence_ms);
  const auto process_cadence =
    std::chrono::milliseconds(options_.process_options.process_sample_cadence_ms);
  const auto alert_cadence = std::chrono::milliseconds(options_.alert_options.evaluate_interval_ms);

  auto next_resource_sample = std::chrono::steady_clock::now() + resource_cadence;
  auto next_disk_sample = std::chrono::steady_clock::now() + disk_cadence;
  auto next_process_sample = std::chrono::steady_clock::now() + process_cadence;
  auto next_alert_sample = std::chrono::steady_clock::now() + alert_cadence;

  while (true) {
    const auto next_sample =
      std::min({next_resource_sample, next_process_sample, next_alert_sample});
    {
      std::unique_lock<std::mutex> lock(mutex_);
      if (sampler_cv_.wait_until(lock, next_sample, [this]() {
            return sampler_stop_requested_;
          })) {
        break;
      }
    }

    const auto now = std::chrono::steady_clock::now();
    if (now >= next_resource_sample) {
      const bool refresh_disk = now >= next_disk_sample;
      sample_resources(refresh_disk);

      do {
        next_resource_sample += resource_cadence;
      } while (next_resource_sample <= now);

      if (refresh_disk) {
        do {
          next_disk_sample += disk_cadence;
        } while (next_disk_sample <= now);
      }
    }

    if (now >= next_process_sample) {
      sample_processes();

      do {
        next_process_sample += process_cadence;
      } while (next_process_sample <= now);
    }

    if (now >= next_alert_sample) {
      sample_alerts();

      do {
        next_alert_sample += alert_cadence;
      } while (next_alert_sample <= now);
    }
  }

  std::lock_guard<std::mutex> lock(mutex_);
  sampler_running_ = false;
}

void SystemService::sample_resources(bool refresh_disk) {
  nlohmann::json snapshot;
  try {
    snapshot = resource_collector_.collect(refresh_disk);
  } catch (const std::exception& ex) {
    snapshot = {
      {"collected_at", now_iso8601()},
      {"sample_cadence_ms", resource_collector_.cadence_json()},
      {"cpu", {{"available", false}, {"error", ex.what()}, {"usage_unit", "percent"}}},
      {"memory", {{"available", false}, {"error", ex.what()}, {"unit", "bytes"}}},
      {"disk", nlohmann::json::array()},
      {"network", {{"available", false}, {"error", ex.what()}, {"unit", "bytes"}}},
    };
  }

  std::lock_guard<std::mutex> lock(mutex_);
  cached_resources_ = std::move(snapshot);
  has_resource_snapshot_ = true;
  apply_resource_status_locked();
}

void SystemService::sample_processes() {
  nlohmann::json snapshot;
  try {
    snapshot = process_monitor_.collect();
  } catch (const std::exception& ex) {
    snapshot = {
      {"error", ex.what()},
    };
  }

  std::lock_guard<std::mutex> lock(mutex_);
  cached_processes_ = std::move(snapshot);
  has_process_snapshot_ = true;
}

void SystemService::sample_alerts() {
  nlohmann::json snapshot;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot = {
      {"service", service_state_json()},
      {"resources", cached_resources_or_placeholder()},
      {"processes", cached_processes_or_placeholder()},
    };
  }

  auto alerts = alert_evaluator_.evaluate(snapshot);
  std::lock_guard<std::mutex> lock(mutex_);
  cached_alerts_ = std::move(alerts);
  has_alert_snapshot_ = true;
}

nlohmann::json SystemService::cached_resources_or_placeholder() const {
  if (has_resource_snapshot_) {
    return cached_resources_;
  }

  return {
    {"collected_at", nullptr},
    {"sample_cadence_ms", resource_collector_.cadence_json()},
    {"cpu", {{"available", false}, {"error", "resource snapshot not collected"}}},
    {"memory", {{"available", false}, {"error", "resource snapshot not collected"}}},
    {"disk", nlohmann::json::array()},
    {"network", {{"available", false}, {"error", "resource snapshot not collected"}}},
  };
}

nlohmann::json SystemService::cached_processes_or_placeholder() const {
  if (has_process_snapshot_) {
    return cached_processes_;
  }

  nlohmann::json processes = nlohmann::json::object();
  for (const auto& target : options_.process_options.targets) {
    processes[target.id] = {
      {"id", target.id},
      {"binary", target.executable},
      {"status", "unknown"},
      {"pid", nullptr},
      {"source", "none"},
      {"message", "process snapshot not collected"},
    };
  }
  return processes;
}

nlohmann::json SystemService::cached_alerts_or_placeholder() const {
  if (has_alert_snapshot_) {
    return cached_alerts_;
  }
  return alert_evaluator_.current_state();
}

nlohmann::json SystemService::service_cadence_json() const {
  auto cadence = resource_collector_.cadence_json();
  cadence["processes"] = options_.process_options.process_sample_cadence_ms;
  cadence["alerts"] = options_.alert_options.evaluate_interval_ms;
  return cadence;
}

void SystemService::apply_resource_status_locked() {
  if (state_ == "stopping" || state_ == "stopped") {
    return;
  }

  std::vector<std::string> failures;
  const auto add_if_unavailable = [&failures](const nlohmann::json& object, const char* name) {
    if (object.is_object() && object.value("available", true) == false) {
      failures.emplace_back(name);
    }
  };

  add_if_unavailable(cached_resources_.value("cpu", nlohmann::json::object()), "cpu");
  add_if_unavailable(cached_resources_.value("memory", nlohmann::json::object()), "memory");
  add_if_unavailable(cached_resources_.value("network", nlohmann::json::object()), "network");

  const auto disk = cached_resources_.value("disk", nlohmann::json::array());
  if (disk.is_array()) {
    for (const auto& entry : disk) {
      if (entry.is_object() && entry.value("available", true) == false) {
        failures.emplace_back("disk:" + entry.value("id", "unknown"));
      }
    }
  }

  if (failures.empty()) {
    state_ = "running";
    last_error_.clear();
    return;
  }

  state_ = "degraded";
  last_error_ = "resource collector failures:";
  for (const auto& failure : failures) {
    last_error_ += " ";
    last_error_ += failure;
  }
}

nlohmann::json SystemService::service_state_json() const {
  const auto uptime = std::chrono::duration_cast<std::chrono::seconds>(
    std::chrono::steady_clock::now() - started_at_
  );
  return {
    {"name", "axon-system"},
    {"version", AXON_SYSTEM_VERSION},
    {"state", state_},
    {"pid", static_cast<int>(getpid())},
    {"started_at", started_at_iso_},
    {"uptime_sec", uptime.count()},
    {"state_dir", state_dir_.lexically_normal().string()},
    {"sample_cadence_ms", service_cadence_json()},
    {"shutdown_requested", shutdown_requested_},
  };
}

std::string SystemService::now_iso8601() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t time = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  gmtime_r(&time, &tm);

  char buffer[32];
  std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &tm);
  return buffer;
}

}  // namespace system
}  // namespace axon
