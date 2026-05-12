// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_SYSTEM_SYSTEM_SERVICE_HPP
#define AXON_SYSTEM_SYSTEM_SERVICE_HPP

#include <chrono>
#include <filesystem>
#include <mutex>
#include <string>

#include "resource_collector.hpp"
#include "rpc_response.hpp"

namespace axon {
namespace system {

class SystemService {
public:
  explicit SystemService(std::filesystem::path state_dir);

  bool initialize(std::string* error);
  RpcResponse get_health() const;
  RpcResponse get_state();
  RpcResponse get_metrics();
  RpcResponse request_shutdown();
  bool shutdown_requested() const;
  void mark_stopped();

private:
  nlohmann::json service_state_json() const;
  static std::string now_iso8601();

  mutable std::mutex mutex_;
  std::filesystem::path state_dir_;
  ResourceCollector resource_collector_;
  std::chrono::steady_clock::time_point started_at_;
  std::string started_at_iso_;
  std::string state_ = "starting";
  std::string last_error_;
  bool shutdown_requested_ = false;
};

}  // namespace system
}  // namespace axon

#endif  // AXON_SYSTEM_SYSTEM_SERVICE_HPP
