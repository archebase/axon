// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "profile_manager.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <exception>

namespace axon {
namespace agent {

ProfileManager::ProfileManager(std::filesystem::path profile_root)
    : profile_root_(std::move(profile_root)) {}

bool ProfileManager::scan(std::string* error) {
  profiles_.clear();

  try {
    if (!std::filesystem::exists(profile_root_)) {
      if (error != nullptr) {
        *error = "profile root does not exist: " + path_string(profile_root_);
      }
      return false;
    }

    for (const auto& entry : std::filesystem::directory_iterator(profile_root_)) {
      if (!entry.is_directory()) {
        continue;
      }

      const auto adapter_yaml = entry.path() / "adapter.yaml";
      if (!std::filesystem::exists(adapter_yaml)) {
        continue;
      }

      RobotProfile profile;
      std::string parse_error;
      if (!parse_profile(entry.path(), &profile, &parse_error)) {
        if (error != nullptr) {
          *error = parse_error;
        }
        return false;
      }
      profiles_.push_back(std::move(profile));
    }

    std::sort(
      profiles_.begin(), profiles_.end(), [](const RobotProfile& lhs, const RobotProfile& rhs) {
        return lhs.profile_id < rhs.profile_id;
      }
    );
    return true;
  } catch (const std::exception& ex) {
    if (error != nullptr) {
      *error = ex.what();
    }
    return false;
  }
}

const std::vector<RobotProfile>& ProfileManager::profiles() const {
  return profiles_;
}

const RobotProfile* ProfileManager::find_profile(const std::string& profile_id) const {
  for (const auto& profile : profiles_) {
    if (profile.profile_id == profile_id || profile.adapter_id == profile_id ||
        profile.robot_model == profile_id) {
      return &profile;
    }
  }
  return nullptr;
}

const RobotProfile* ProfileManager::active_profile() const {
  if (active_profile_id_.empty()) {
    return nullptr;
  }
  return find_profile(active_profile_id_);
}

bool ProfileManager::select_profile(const std::string& profile_id, std::string* error) {
  const auto* profile = find_profile(profile_id);
  if (profile == nullptr) {
    if (error != nullptr) {
      *error = "profile not found: " + profile_id;
    }
    return false;
  }

  active_profile_id_ = profile->profile_id;
  return true;
}

nlohmann::json ProfileManager::profiles_to_json() const {
  nlohmann::json result = nlohmann::json::array();
  for (const auto& profile : profiles_) {
    result.push_back({
      {"profile_id", profile.profile_id},
      {"adapter_id", profile.adapter_id},
      {"robot_model", profile.robot_model},
      {"root_dir", path_string(profile.root_dir)},
      {"adapter_yaml", path_string(profile.adapter_yaml)},
      {"recorder_yaml", path_string(profile.recorder_yaml)},
      {"transfer_yaml", path_string(profile.transfer_yaml)},
      {"library", path_string(profile.library_path)},
      {"entry_symbol", profile.entry_symbol},
      {"abi_version", profile.abi_version},
      {"auto_start", profile.auto_start},
      {"capabilities", profile.capabilities},
    });
  }
  return result;
}

nlohmann::json ProfileManager::active_profile_to_json() const {
  const auto* profile = active_profile();
  if (profile == nullptr) {
    return nullptr;
  }

  return {
    {"profile_id", profile->profile_id},
    {"adapter_id", profile->adapter_id},
    {"robot_model", profile->robot_model},
    {"root_dir", path_string(profile->root_dir)},
    {"recorder_yaml", path_string(profile->recorder_yaml)},
    {"transfer_yaml", path_string(profile->transfer_yaml)},
    {"auto_start", profile->auto_start},
  };
}

bool ProfileManager::parse_profile(
  const std::filesystem::path& profile_dir, RobotProfile* profile, std::string* error
) {
  try {
    const auto adapter_yaml = profile_dir / "adapter.yaml";
    const auto node = YAML::LoadFile(path_string(adapter_yaml));

    profile->root_dir = profile_dir;
    profile->adapter_yaml = adapter_yaml;
    profile->recorder_yaml = profile_dir / "recorder.yaml";
    profile->transfer_yaml = profile_dir / "transfer.yaml";
    profile->profile_id = profile_dir.filename().string();
    profile->adapter_id =
      node["adapter_id"] ? node["adapter_id"].as<std::string>() : profile->profile_id;
    profile->robot_model =
      node["robot_model"] ? node["robot_model"].as<std::string>() : profile->adapter_id;
    profile->abi_version = node["abi_version"] ? node["abi_version"].as<int>() : 1;
    profile->entry_symbol =
      node["entry_symbol"] ? node["entry_symbol"].as<std::string>() : "axon_agent_create_adapter";
    profile->auto_start = node["auto_start"] ? node["auto_start"].as<bool>() : false;

    if (node["library"]) {
      profile->library_path = profile_dir / node["library"].as<std::string>();
    }

    if (node["capabilities"]) {
      for (const auto& capability : node["capabilities"]) {
        profile->capabilities.push_back(capability.as<std::string>());
      }
    }
    return true;
  } catch (const std::exception& ex) {
    if (error != nullptr) {
      *error = "failed to parse " + path_string(profile_dir / "adapter.yaml") + ": " + ex.what();
    }
    return false;
  }
}

std::string ProfileManager::path_string(const std::filesystem::path& path) {
  return path.lexically_normal().string();
}

}  // namespace agent
}  // namespace axon
