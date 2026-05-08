// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_AGENT_ADAPTER_LOADER_HPP
#define AXON_AGENT_ADAPTER_LOADER_HPP

#include "profile_manager.hpp"

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
  nlohmann::json status_to_json() const;

private:
  void* handle_ = nullptr;
  std::string loaded_profile_id_;
  std::filesystem::path loaded_library_;
};

}  // namespace agent
}  // namespace axon

#endif  // AXON_AGENT_ADAPTER_LOADER_HPP
