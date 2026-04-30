// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "schema_resolver.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>

// Define component name for logging
#define AXON_LOG_COMPONENT "schema_resolver"
#include <axon_log_macros.hpp>

using axon::logging::kv;

namespace axon {
namespace recorder {

// 80 '=' characters as per MCAP ros2msg spec
static const std::string kSeparator(80, '=');

// ROS primitive types (no recursion needed)
static const std::unordered_set<std::string> kPrimitiveTypes = {
  "bool",
  "byte",
  "char",
  "float32",
  "float64",
  "int8",
  "int16",
  "int32",
  "int64",
  "uint8",
  "uint16",
  "uint32",
  "uint64",
  "string",
  "wstring",
};

// =============================================================================
// SchemaResolver
// =============================================================================

SchemaResolver::SchemaResolver(const std::vector<std::string>& search_paths)
    : search_paths_(search_paths) {}

void SchemaResolver::clear_cache() {
  file_cache_.clear();
}

std::string SchemaResolver::resolve(const std::string& message_type) {
  last_error_.clear();

  if (search_paths_.empty()) {
    last_error_ = "No search paths configured for schema resolution";
    return "";
  }

  // Parse the top-level type
  ParsedType top_level;
  if (!parse_type(message_type, "", top_level)) {
    last_error_ = "Failed to parse message type: " + message_type;
    return "";
  }

  // Read the top-level .msg file
  std::string top_content = read_msg_file(top_level);
  if (top_content.empty()) {
    // last_error_ already set by read_msg_file
    return "";
  }

  // Collect all dependencies via BFS
  // Use ordered collection to maintain deterministic output
  std::vector<ParsedType> dep_order;
  std::unordered_set<std::string> visited;
  visited.insert(top_level.canonical_name());

  // Start with dependencies of the top-level type
  auto top_deps = extract_dependencies(top_content, top_level.package);
  std::vector<ParsedType> queue(top_deps.begin(), top_deps.end());

  while (!queue.empty()) {
    ParsedType current = std::move(queue.front());
    queue.erase(queue.begin());

    std::string canonical = current.canonical_name();
    if (visited.count(canonical)) {
      continue;
    }
    visited.insert(canonical);

    std::string content = read_msg_file(current);
    if (content.empty()) {
      AXON_LOG_WARN(
        "Could not resolve dependency " << kv("type", canonical) << " for "
                                        << kv("parent", message_type)
                                        << ", schema will be incomplete"
      );
      continue;
    }

    dep_order.push_back(current);

    // Extract sub-dependencies
    auto sub_deps = extract_dependencies(content, current.package);
    for (auto& dep : sub_deps) {
      if (!visited.count(dep.canonical_name())) {
        queue.push_back(std::move(dep));
      }
    }
  }

  // Build the concatenated definition
  std::ostringstream result;
  result << top_content;

  for (const auto& dep : dep_order) {
    auto it = file_cache_.find(dep.canonical_name());
    if (it != file_cache_.end()) {
      result << "\n" << kSeparator << "\n";
      result << "MSG: " << dep.package << "/" << dep.type_name << "\n";
      result << it->second;
    }
  }

  AXON_LOG_INFO(
    "Resolved schema for " << kv("type", message_type) << " with "
                           << kv("dependencies", dep_order.size()) << " dependencies"
  );

  return result.str();
}

// =============================================================================
// Type Parsing
// =============================================================================

bool SchemaResolver::parse_type(
  const std::string& type_str, const std::string& context_package, ParsedType& out
) {
  if (type_str.empty()) {
    return false;
  }

  // Handle ROS1 built-in types
  if (type_str == "time" || type_str == "Time") {
    out.package = "builtin_interfaces";
    out.type_name = "Time";
    return true;
  }
  if (type_str == "duration" || type_str == "Duration") {
    out.package = "builtin_interfaces";
    out.type_name = "Duration";
    return true;
  }

  // Handle ROS1 special case: "Header" without package → std_msgs/Header
  if (type_str == "Header") {
    out.package = "std_msgs";
    out.type_name = "Header";
    return true;
  }

  // Count slashes to determine format
  size_t first_slash = type_str.find('/');
  if (first_slash == std::string::npos) {
    // No slash: bare type name, use context package
    if (context_package.empty()) {
      return false;  // Can't resolve without context
    }
    out.package = context_package;
    out.type_name = type_str;
    return true;
  }

  size_t second_slash = type_str.find('/', first_slash + 1);
  if (second_slash == std::string::npos) {
    // One slash: ROS1 format "pkg/Type"
    out.package = type_str.substr(0, first_slash);
    out.type_name = type_str.substr(first_slash + 1);
    return true;
  }

  // Two slashes: ROS2 format "pkg/msg/Type"
  out.package = type_str.substr(0, first_slash);
  // Skip the middle segment (msg, srv, action)
  out.type_name = type_str.substr(second_slash + 1);
  return true;
}

// =============================================================================
// File I/O
// =============================================================================

std::string SchemaResolver::find_msg_file(const ParsedType& parsed) const {
  for (const auto& search_path : search_paths_) {
    // Pattern 1: Merged install / system install layout
    //   <path>/<package>/msg/<Type>.msg
    //   e.g., /opt/ros/humble/share/sensor_msgs/msg/Image.msg
    {
      std::string path = search_path + "/" + parsed.package + "/msg/" + parsed.type_name + ".msg";
      std::ifstream file(path);
      if (file.good()) {
        return path;
      }
    }

    // Pattern 2: Isolated colcon install layout
    //   <path>/<package>/share/<package>/msg/<Type>.msg
    //   e.g., install/sensor_msgs/share/sensor_msgs/msg/Image.msg
    {
      std::string path = search_path + "/" + parsed.package + "/share/" + parsed.package + "/msg/" +
                         parsed.type_name + ".msg";
      std::ifstream file(path);
      if (file.good()) {
        return path;
      }
    }

    // Pattern 3: Merged install with share/ prefix
    //   <path>/share/<package>/msg/<Type>.msg
    //   e.g., install/share/sensor_msgs/msg/Image.msg
    {
      std::string path =
        search_path + "/share/" + parsed.package + "/msg/" + parsed.type_name + ".msg";
      std::ifstream file(path);
      if (file.good()) {
        return path;
      }
    }
  }
  return "";
}

std::string SchemaResolver::read_msg_file(const ParsedType& parsed) {
  std::string canonical = parsed.canonical_name();

  // Check cache first
  auto it = file_cache_.find(canonical);
  if (it != file_cache_.end()) {
    return it->second;
  }

  // Find the file
  std::string file_path = find_msg_file(parsed);
  if (file_path.empty()) {
    last_error_ = "Could not find .msg file for: " + canonical + " in search paths";
    AXON_LOG_WARN("Message definition not found: " << kv("type", canonical));
    return "";
  }

  // Read file content
  std::ifstream file(file_path);
  if (!file.is_open()) {
    last_error_ = "Failed to open file: " + file_path;
    return "";
  }

  std::ostringstream content;
  content << file.rdbuf();

  std::string result = content.str();

  // Remove trailing newlines for consistent formatting
  while (!result.empty() && result.back() == '\n') {
    result.pop_back();
  }

  // Cache it
  file_cache_[canonical] = result;

  AXON_LOG_DEBUG("Read .msg file: " << kv("type", canonical) << " from " << kv("path", file_path));

  return result;
}

// =============================================================================
// Dependency Extraction
// =============================================================================

bool SchemaResolver::is_primitive(const std::string& type_name) {
  return kPrimitiveTypes.count(type_name) > 0;
}

bool SchemaResolver::is_ros1_builtin(const std::string& type_name) {
  return type_name == "time" || type_name == "duration" || type_name == "Time" ||
         type_name == "Duration";
}

std::vector<SchemaResolver::ParsedType> SchemaResolver::extract_dependencies(
  const std::string& content, const std::string& context_package
) const {
  std::vector<ParsedType> deps;
  std::unordered_set<std::string> seen;

  std::istringstream stream(content);
  std::string line;

  while (std::getline(stream, line)) {
    // Skip empty lines
    if (line.empty()) {
      continue;
    }

    // Skip comment lines
    size_t first_non_space = line.find_first_not_of(" \t");
    if (first_non_space == std::string::npos || line[first_non_space] == '#') {
      continue;
    }

    // Remove inline comments
    std::string effective_line = line;
    size_t comment_pos = effective_line.find('#');
    if (comment_pos != std::string::npos) {
      effective_line = effective_line.substr(0, comment_pos);
    }

    // Trim whitespace
    size_t start = effective_line.find_first_not_of(" \t");
    size_t end = effective_line.find_last_not_of(" \t\r\n");
    if (start == std::string::npos) {
      continue;
    }
    effective_line = effective_line.substr(start, end - start + 1);

    // Skip constant definitions (lines with '=')
    if (effective_line.find('=') != std::string::npos) {
      continue;
    }

    // Parse the type from the field declaration: "<type> <name>"
    // or "<type>[<size>] <name>"
    size_t space_pos = effective_line.find_first_of(" \t");
    if (space_pos == std::string::npos) {
      continue;
    }

    std::string type_str = effective_line.substr(0, space_pos);

    // Strip array notation: "Type[]" → "Type", "Type[10]" → "Type"
    size_t bracket_pos = type_str.find('[');
    if (bracket_pos != std::string::npos) {
      type_str = type_str.substr(0, bracket_pos);
    }

    // Skip primitive types
    if (is_primitive(type_str)) {
      continue;
    }

    // Parse the type reference
    ParsedType dep;
    if (parse_type(type_str, context_package, dep)) {
      std::string canonical = dep.canonical_name();
      if (!seen.count(canonical)) {
        seen.insert(canonical);
        deps.push_back(std::move(dep));
      }
    }
  }

  return deps;
}

}  // namespace recorder
}  // namespace axon
