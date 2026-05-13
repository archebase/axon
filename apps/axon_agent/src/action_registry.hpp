// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_AGENT_ACTION_REGISTRY_HPP
#define AXON_AGENT_ACTION_REGISTRY_HPP

#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <string>
#include <vector>

namespace axon {
namespace agent {

struct ActionDefinition {
  std::string id;
  std::string title;
  std::string description;
  std::filesystem::path command;
  nlohmann::json args_schema = nlohmann::json::object();
  int timeout_sec = 30;
  std::string run_as;
  std::vector<std::string> allowed_roles;
  bool requires_approval = false;
  std::filesystem::path source_path;
};

struct ActionRegistryDiagnostic {
  std::string severity;
  std::filesystem::path path;
  std::string action_id;
  std::string message;
};

class ActionRegistry {
public:
  ActionRegistry(
    std::filesystem::path manifest_dir = "/etc/axon/actions.d",
    std::filesystem::path approved_command_dir = "/opt/axon/actions"
  );

  bool load(std::string* error);
  const std::vector<ActionDefinition>& actions() const;
  const std::vector<ActionRegistryDiagnostic>& diagnostics() const;
  nlohmann::json to_json() const;

private:
  ActionDefinition parse_manifest(const std::filesystem::path& path) const;
  bool is_command_allowed(const std::filesystem::path& command) const;
  void add_diagnostic(
    std::string severity, std::filesystem::path path, std::string action_id, std::string message
  );
  static nlohmann::json action_to_json(const ActionDefinition& action);
  static nlohmann::json diagnostic_to_json(const ActionRegistryDiagnostic& diagnostic);
  static nlohmann::json yaml_to_json(const YAML::Node& node);
  static std::string required_string(
    const YAML::Node& node, const std::string& field, const std::filesystem::path& path
  );
  static bool required_bool(
    const YAML::Node& node, const std::string& field, const std::filesystem::path& path
  );
  static int required_positive_int(
    const YAML::Node& node, const std::string& field, const std::filesystem::path& path
  );
  static std::vector<std::string> required_string_sequence(
    const YAML::Node& node, const std::string& field, const std::filesystem::path& path
  );
  static std::string path_string(const std::filesystem::path& path);
  static bool has_yaml_extension(const std::filesystem::path& path);

  std::filesystem::path manifest_dir_;
  std::filesystem::path approved_command_dir_;
  std::vector<ActionDefinition> actions_;
  std::vector<ActionRegistryDiagnostic> diagnostics_;
};

}  // namespace agent
}  // namespace axon

#endif  // AXON_AGENT_ACTION_REGISTRY_HPP
