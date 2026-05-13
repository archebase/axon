// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_SYSTEM_SYSTEM_SERVICE_HPP
#define AXON_SYSTEM_SYSTEM_SERVICE_HPP

#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <mutex>
#include <string>
#include <thread>

#include "alert_evaluator.hpp"
#include "process_monitor.hpp"
#include "resource_collector.hpp"
#include "rpc_response.hpp"

namespace axon {
namespace system {

struct SystemServiceOptions {
  std::filesystem::path state_dir = "/var/lib/axon/system";
  ResourceCollectorOptions resource_options;
  ProcessMonitorOptions process_options;
  AlertEvaluatorOptions alert_options;
  int sampler_idle_sleep_ms = 100;
};

class SystemService {
public:
  explicit SystemService(std::filesystem::path state_dir);
  explicit SystemService(SystemServiceOptions options);
  ~SystemService();

  SystemService(const SystemService&) = delete;
  SystemService& operator=(const SystemService&) = delete;

  bool initialize(std::string* error);
  RpcResponse get_health() const;
  RpcResponse get_state();
  RpcResponse get_metrics();
  RpcResponse get_processes();
  RpcResponse get_alerts();
  RpcResponse request_shutdown();
  bool shutdown_requested() const;
  void mark_stopped();

private:
  void start_sampler();
  void stop_sampler();
  void sampler_loop();
  void sample_resources(bool refresh_disk);
  void sample_processes();
  void sample_alerts();
  nlohmann::json cached_resources_or_placeholder() const;
  nlohmann::json cached_processes_or_placeholder() const;
  nlohmann::json cached_alerts_or_placeholder() const;
  nlohmann::json service_cadence_json() const;
  void apply_resource_status_locked();
  nlohmann::json service_state_json() const;
  static std::string now_iso8601();

  mutable std::mutex mutex_;
  std::condition_variable sampler_cv_;
  SystemServiceOptions options_;
  std::filesystem::path state_dir_;
  ResourceCollector resource_collector_;
  ProcessMonitor process_monitor_;
  AlertEvaluator alert_evaluator_;
  std::thread sampler_thread_;
  std::chrono::steady_clock::time_point started_at_;
  std::string started_at_iso_;
  std::string state_ = "starting";
  std::string last_error_;
  nlohmann::json cached_resources_;
  nlohmann::json cached_processes_;
  nlohmann::json cached_alerts_;
  bool has_resource_snapshot_ = false;
  bool has_process_snapshot_ = false;
  bool has_alert_snapshot_ = false;
  bool sampler_running_ = false;
  bool sampler_stop_requested_ = false;
  bool shutdown_requested_ = false;
};

}  // namespace system
}  // namespace axon

#endif  // AXON_SYSTEM_SYSTEM_SERVICE_HPP
