// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_AGENT_ADAPTER_LOADER_HPP
#define AXON_AGENT_ADAPTER_LOADER_HPP

#include "profile_manager.hpp"

#include <axon/agent/robot_adapter.hpp>
#include <nlohmann/json.hpp>

#include <filesystem>
#include <string>

namespace axon {
namespace agent {

class AdapterLoader {
public:
  AdapterLoader() = default;
  ~AdapterLoader();

  AdapterLoader(const AdapterLoader&) = delete;
  AdapterLoader& operator=(const AdapterLoader&) = delete;

  bool load(const RobotProfile& profile, std::string* error);
  void unload();
  bool is_loaded() const;
  bool is_loaded_for_profile(const std::string& profile_id) const;
  nlohmann::json status_to_json() const;
  nlohmann::json runtime_status_to_json(const RobotAdapterContext& context) const;

  bool start(const RobotAdapterContext& context, std::string* error);
  bool stop(const RobotAdapterContext& context, int timeout_ms, std::string* error);
  bool force_stop(const RobotAdapterContext& context, std::string* error);

private:
  bool validate_descriptor(const RobotProfile& profile, const RobotAdapterDescriptor* descriptor, std::string* error);
  static nlohmann::json descriptor_to_json(const RobotAdapterDescriptor& descriptor);
  static nlohmann::json status_to_json(const RobotAdapterStatus& status);

  void* handle_ = nullptr;
  RobotAdapter* adapter_ = nullptr;
  GetRobotAdapterDescriptorFn get_descriptor_fn_ = nullptr;
  CreateRobotAdapterFn create_fn_ = nullptr;
  DestroyRobotAdapterFn destroy_fn_ = nullptr;
  RobotAdapterDescriptor descriptor_;
  std::string loaded_profile_id_;
  std::filesystem::path loaded_library_;
};

}  // namespace agent
}  // namespace axon

#endif  // AXON_AGENT_ADAPTER_LOADER_HPP
