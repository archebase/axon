// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "system_service.hpp"

#include <ctime>
#include <system_error>
#include <unistd.h>
#include <utility>

#ifndef AXON_SYSTEM_VERSION
#define AXON_SYSTEM_VERSION "0.4.0-dev"
#endif

namespace axon {
namespace system {

namespace {

ResourceCollectorOptions default_resource_options(const std::filesystem::path& state_dir) {
  ResourceCollectorOptions options;
  options.disk_paths = {
    {"state_dir", state_dir},
    {"recording_data", "/tmp/axon/recording"},
    {"transfer_state", "/tmp/axon/transfer"},
  };
  return options;
}

}  // namespace

SystemService::SystemService(std::filesystem::path state_dir)
    : state_dir_(std::move(state_dir))
    , resource_collector_(default_resource_options(state_dir_))
    , started_at_(std::chrono::steady_clock::now())
    , started_at_iso_(now_iso8601()) {}

bool SystemService::initialize(std::string* error) {
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
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

RpcResponse SystemService::get_health() const {
  std::lock_guard<std::mutex> lock(mutex_);
  const bool healthy = state_ == "running" || state_ == "degraded";
  nlohmann::json data = {
    {"service", "axon-system"},
    {"state", state_},
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
    {"resources", resource_collector_.collect()},
    {"processes", nlohmann::json::object()},
    {"alerts", {{"firing_count", 0}, {"last_delivery_error", ""}}},
  };
  if (!last_error_.empty()) {
    data["last_error"] = last_error_;
  }
  return {true, "ok", data};
}

RpcResponse SystemService::get_metrics() {
  std::lock_guard<std::mutex> lock(mutex_);
  return {true, "ok", resource_collector_.collect()};
}

RpcResponse SystemService::request_shutdown() {
  std::lock_guard<std::mutex> lock(mutex_);
  shutdown_requested_ = true;
  if (state_ != "stopped") {
    state_ = "stopping";
  }
  return {true, "shutdown requested", {{"service", service_state_json()}}};
}

bool SystemService::shutdown_requested() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return shutdown_requested_;
}

void SystemService::mark_stopped() {
  std::lock_guard<std::mutex> lock(mutex_);
  state_ = "stopped";
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
    {"sample_cadence_ms", resource_collector_.cadence_json()},
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
