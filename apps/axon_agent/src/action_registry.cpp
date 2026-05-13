// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "action_registry.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdlib>
#include <exception>
#include <map>
#include <stdexcept>
#include <system_error>
#include <utility>

namespace axon {
namespace agent {

namespace {

std::filesystem::path normalize_for_policy(const std::filesystem::path& path) {
  std::error_code ec;
  const auto canonical = std::filesystem::weakly_canonical(path, ec);
  if (!ec) {
    return canonical.lexically_normal();
  }
  return path.lexically_normal();
}

bool path_has_prefix(const std::filesystem::path& candidate, const std::filesystem::path& prefix) {
  const auto normalized_candidate = normalize_for_policy(candidate);
  const auto normalized_prefix = normalize_for_policy(prefix);

  auto candidate_it = normalized_candidate.begin();
  auto prefix_it = normalized_prefix.begin();
  for (; prefix_it != normalized_prefix.end(); ++prefix_it, ++candidate_it) {
    if (candidate_it == normalized_candidate.end() || *candidate_it != *prefix_it) {
      return false;
    }
  }
  return candidate_it != normalized_candidate.end();
}

nlohmann::json scalar_to_json(const std::string& value) {
  if (value == "true") {
    return true;
  }
  if (value == "false") {
    return false;
  }
  if (value == "null" || value == "~") {
    return nullptr;
  }

  char* end = nullptr;
  errno = 0;
  const auto integer = std::strtoll(value.c_str(), &end, 10);
  if (errno == 0 && end != value.c_str() && *end == '\0') {
    return integer;
  }

  errno = 0;
  const auto decimal = std::strtod(value.c_str(), &end);
  if (errno == 0 && end != value.c_str() && *end == '\0') {
    return decimal;
  }

  return value;
}

}  // namespace

ActionRegistry::ActionRegistry(
  std::filesystem::path manifest_dir, std::filesystem::path approved_command_dir
)
    : manifest_dir_(std::move(manifest_dir))
    , approved_command_dir_(std::move(approved_command_dir)) {}

bool ActionRegistry::load(std::string* error) {
  actions_.clear();
  diagnostics_.clear();

  try {
    if (!std::filesystem::exists(manifest_dir_)) {
      add_diagnostic(
        "warning",
        manifest_dir_,
        "",
        "action manifest directory does not exist: " + path_string(manifest_dir_)
      );
      return true;
    }
    if (!std::filesystem::is_directory(manifest_dir_)) {
      add_diagnostic(
        "error",
        manifest_dir_,
        "",
        "action manifest path is not a directory: " + path_string(manifest_dir_)
      );
      return true;
    }

    std::vector<std::filesystem::path> manifests;
    for (const auto& entry : std::filesystem::directory_iterator(manifest_dir_)) {
      if (!entry.is_regular_file() || !has_yaml_extension(entry.path())) {
        continue;
      }
      manifests.push_back(entry.path());
    }
    std::sort(manifests.begin(), manifests.end());

    std::map<std::string, std::filesystem::path> seen_ids;
    for (const auto& manifest : manifests) {
      try {
        auto action = parse_manifest(manifest);
        if (seen_ids.find(action.id) != seen_ids.end()) {
          add_diagnostic(
            "error",
            manifest,
            action.id,
            "duplicate action id '" + action.id + "'; first declared in " +
              path_string(seen_ids[action.id])
          );
          continue;
        }
        seen_ids[action.id] = manifest;
        actions_.push_back(std::move(action));
      } catch (const std::exception& ex) {
        add_diagnostic("error", manifest, "", ex.what());
      }
    }

    std::sort(
      actions_.begin(),
      actions_.end(),
      [](const ActionDefinition& lhs, const ActionDefinition& rhs) {
        return lhs.id < rhs.id;
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

const std::vector<ActionDefinition>& ActionRegistry::actions() const {
  return actions_;
}

const std::vector<ActionRegistryDiagnostic>& ActionRegistry::diagnostics() const {
  return diagnostics_;
}

nlohmann::json ActionRegistry::to_json() const {
  nlohmann::json actions = nlohmann::json::array();
  for (const auto& action : actions_) {
    actions.push_back(action_to_json(action));
  }

  nlohmann::json diagnostics = nlohmann::json::array();
  std::size_t invalid_count = 0;
  for (const auto& diagnostic : diagnostics_) {
    diagnostics.push_back(diagnostic_to_json(diagnostic));
    if (diagnostic.severity == "error") {
      ++invalid_count;
    }
  }

  return {
    {"manifest_dir", path_string(manifest_dir_)},
    {"approved_command_dir", path_string(approved_command_dir_)},
    {"loaded_count", actions_.size()},
    {"invalid_count", invalid_count},
    {"actions", actions},
    {"diagnostics", diagnostics},
  };
}

ActionDefinition ActionRegistry::parse_manifest(const std::filesystem::path& path) const {
  const auto root = YAML::LoadFile(path_string(path));
  if (!root || !root.IsMap()) {
    throw std::runtime_error("action manifest must be a YAML map: " + path_string(path));
  }

  ActionDefinition action;
  action.source_path = path;
  action.id = required_string(root, "id", path);
  action.title = required_string(root, "title", path);
  action.description = required_string(root, "description", path);
  action.command = required_string(root, "command", path);
  action.timeout_sec = required_positive_int(root, "timeout_sec", path);
  action.run_as = required_string(root, "run_as", path);
  action.allowed_roles = required_string_sequence(root, "allowed_roles", path);
  action.requires_approval = required_bool(root, "requires_approval", path);

  if (action.id.empty()) {
    throw std::runtime_error("field id must not be empty: " + path_string(path));
  }
  if (!root["args_schema"]) {
    throw std::runtime_error("missing field args_schema: " + path_string(path));
  }
  if (!root["args_schema"].IsMap()) {
    throw std::runtime_error("field args_schema must be a map: " + path_string(path));
  }
  action.args_schema = yaml_to_json(root["args_schema"]);

  if (!action.command.is_absolute()) {
    throw std::runtime_error(
      "field command must be an absolute path under " + path_string(approved_command_dir_) + ": " +
      action.command.string()
    );
  }
  if (!is_command_allowed(action.command)) {
    throw std::runtime_error(
      "field command must be under approved action directory " +
      path_string(approved_command_dir_) + ": " + action.command.lexically_normal().string()
    );
  }

  action.command = action.command.lexically_normal();
  return action;
}

bool ActionRegistry::is_command_allowed(const std::filesystem::path& command) const {
  return path_has_prefix(command, approved_command_dir_);
}

void ActionRegistry::add_diagnostic(
  std::string severity, std::filesystem::path path, std::string action_id, std::string message
) {
  diagnostics_.push_back(
    {std::move(severity), std::move(path), std::move(action_id), std::move(message)}
  );
}

nlohmann::json ActionRegistry::action_to_json(const ActionDefinition& action) {
  return {
    {"id", action.id},
    {"title", action.title},
    {"description", action.description},
    {"command", path_string(action.command)},
    {"args_schema", action.args_schema},
    {"timeout_sec", action.timeout_sec},
    {"run_as", action.run_as},
    {"allowed_roles", action.allowed_roles},
    {"requires_approval", action.requires_approval},
    {"source_path", path_string(action.source_path)},
  };
}

nlohmann::json ActionRegistry::diagnostic_to_json(const ActionRegistryDiagnostic& diagnostic) {
  nlohmann::json result = {
    {"severity", diagnostic.severity},
    {"path", path_string(diagnostic.path)},
    {"message", diagnostic.message},
  };
  if (!diagnostic.action_id.empty()) {
    result["action_id"] = diagnostic.action_id;
  }
  return result;
}

nlohmann::json ActionRegistry::yaml_to_json(const YAML::Node& node) {
  if (!node || node.IsNull()) {
    return nullptr;
  }
  if (node.IsMap()) {
    nlohmann::json result = nlohmann::json::object();
    for (const auto& item : node) {
      result[item.first.as<std::string>()] = yaml_to_json(item.second);
    }
    return result;
  }
  if (node.IsSequence()) {
    nlohmann::json result = nlohmann::json::array();
    for (const auto& item : node) {
      result.push_back(yaml_to_json(item));
    }
    return result;
  }
  return scalar_to_json(node.as<std::string>());
}

std::string ActionRegistry::required_string(
  const YAML::Node& node, const std::string& field, const std::filesystem::path& path
) {
  if (!node[field]) {
    throw std::runtime_error("missing field " + field + ": " + path_string(path));
  }
  if (!node[field].IsScalar()) {
    throw std::runtime_error("field " + field + " must be a string: " + path_string(path));
  }
  return node[field].as<std::string>();
}

bool ActionRegistry::required_bool(
  const YAML::Node& node, const std::string& field, const std::filesystem::path& path
) {
  if (!node[field]) {
    throw std::runtime_error("missing field " + field + ": " + path_string(path));
  }
  if (!node[field].IsScalar()) {
    throw std::runtime_error("field " + field + " must be a boolean: " + path_string(path));
  }
  try {
    return node[field].as<bool>();
  } catch (const std::exception&) {
    throw std::runtime_error("field " + field + " must be a boolean: " + path_string(path));
  }
}

int ActionRegistry::required_positive_int(
  const YAML::Node& node, const std::string& field, const std::filesystem::path& path
) {
  if (!node[field]) {
    throw std::runtime_error("missing field " + field + ": " + path_string(path));
  }
  if (!node[field].IsScalar()) {
    throw std::runtime_error(
      "field " + field + " must be a positive integer: " + path_string(path)
    );
  }
  try {
    const auto value = node[field].as<int>();
    if (value <= 0) {
      throw std::runtime_error("not positive");
    }
    return value;
  } catch (const std::exception&) {
    throw std::runtime_error(
      "field " + field + " must be a positive integer: " + path_string(path)
    );
  }
}

std::vector<std::string> ActionRegistry::required_string_sequence(
  const YAML::Node& node, const std::string& field, const std::filesystem::path& path
) {
  if (!node[field]) {
    throw std::runtime_error("missing field " + field + ": " + path_string(path));
  }
  if (!node[field].IsSequence()) {
    throw std::runtime_error("field " + field + " must be a sequence: " + path_string(path));
  }

  std::vector<std::string> result;
  for (const auto& item : node[field]) {
    if (!item.IsScalar()) {
      throw std::runtime_error(
        "field " + field + " must contain only strings: " + path_string(path)
      );
    }
    result.push_back(item.as<std::string>());
  }
  return result;
}

std::string ActionRegistry::path_string(const std::filesystem::path& path) {
  return path.lexically_normal().string();
}

bool ActionRegistry::has_yaml_extension(const std::filesystem::path& path) {
  const auto extension = path.extension().string();
  return extension == ".yaml" || extension == ".yml";
}

}  // namespace agent
}  // namespace axon
