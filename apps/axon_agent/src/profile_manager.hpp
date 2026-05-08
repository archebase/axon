// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_AGENT_PROFILE_MANAGER_HPP
#define AXON_AGENT_PROFILE_MANAGER_HPP

#include <nlohmann/json.hpp>

#include <filesystem>
#include <string>
#include <vector>

namespace axon {
namespace agent {

struct RobotProfile {
  std::string profile_id;
  std::string adapter_id;
  std::string robot_model;
  std::filesystem::path root_dir;
  std::filesystem::path adapter_yaml;
  std::filesystem::path recorder_yaml;
  std::filesystem::path transfer_yaml;
  std::filesystem::path library_path;
  std::string entry_symbol = "axon_agent_create_adapter";
  int abi_version = 1;
  bool auto_start = false;
  std::vector<std::string> capabilities;
};

class ProfileManager {
public:
  explicit ProfileManager(std::filesystem::path profile_root);

  bool scan(std::string* error);
  const std::vector<RobotProfile>& profiles() const;
  const RobotProfile* find_profile(const std::string& profile_id) const;
  const RobotProfile* active_profile() const;
  bool select_profile(const std::string& profile_id, std::string* error);
  nlohmann::json profiles_to_json() const;
  nlohmann::json active_profile_to_json() const;

private:
  static bool parse_profile(const std::filesystem::path& profile_dir, RobotProfile* profile, std::string* error);
  static std::string path_string(const std::filesystem::path& path);

  std::filesystem::path profile_root_;
  std::vector<RobotProfile> profiles_;
  std::string active_profile_id_;
};

}  // namespace agent
}  // namespace axon

#endif  // AXON_AGENT_PROFILE_MANAGER_HPP
