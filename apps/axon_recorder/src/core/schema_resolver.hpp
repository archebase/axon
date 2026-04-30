// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_SCHEMA_RESOLVER_HPP
#define AXON_SCHEMA_RESOLVER_HPP

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace axon {
namespace recorder {

/**
 * Resolves ROS message definitions from .msg files on disk.
 *
 * Given a message type (e.g., "sensor_msgs/msg/Image" or "sensor_msgs/Image")
 * and a list of search paths, reads the .msg file, recursively resolves all
 * dependent message types, and produces a concatenated definition string in
 * the MCAP ros2msg format:
 *
 *   <top-level .msg content>
 *   ================================================================================
 *   MSG: std_msgs/Header
 *   <Header .msg content>
 *   ================================================================================
 *   MSG: builtin_interfaces/Time
 *   <Time .msg content>
 *
 * Supports both ROS1 and ROS2 type naming conventions:
 *   - ROS2: "sensor_msgs/msg/Image" → <path>/sensor_msgs/msg/Image.msg
 *   - ROS1: "sensor_msgs/Image"     → <path>/sensor_msgs/msg/Image.msg
 *
 * ROS1 built-in types (time, duration) are mapped to their ROS2 equivalents
 * (builtin_interfaces/msg/Time, builtin_interfaces/msg/Duration).
 */
class SchemaResolver {
public:
  /**
   * Construct a SchemaResolver with the given search paths.
   *
   * @param search_paths Directories to search for ROS message packages.
   *                     Each path should be a share directory containing
   *                     package directories (e.g., /opt/ros/humble/share).
   */
  explicit SchemaResolver(const std::vector<std::string>& search_paths);

  /**
   * Resolve a message type to its full MCAP ros2msg definition string.
   *
   * @param message_type Message type name (ROS1 or ROS2 format)
   * @return Concatenated definition string, or empty string on failure
   */
  std::string resolve(const std::string& message_type);

  /**
   * Get the last error message.
   */
  std::string get_last_error() const {
    return last_error_;
  }

  /**
   * Clear the internal cache. Useful if .msg files may have changed on disk.
   */
  void clear_cache();

private:
  /**
   * Parsed message type with package and type name separated.
   */
  struct ParsedType {
    std::string package;    // e.g., "sensor_msgs"
    std::string type_name;  // e.g., "Image"

    // Canonical ROS2 name: "sensor_msgs/msg/Image"
    std::string canonical_name() const {
      return package + "/msg/" + type_name;
    }
  };

  /**
   * Parse a message type string into package and type name.
   * Handles both "pkg/msg/Type" (ROS2) and "pkg/Type" (ROS1) formats.
   *
   * @param type_str The message type string
   * @param context_package Package name for resolving bare type names (e.g., "Header")
   * @return Parsed type, or nullopt if parsing fails
   */
  static bool parse_type(
    const std::string& type_str, const std::string& context_package, ParsedType& out
  );

  /**
   * Find the .msg file path for a parsed type.
   *
   * @param parsed The parsed message type
   * @return Full file path, or empty string if not found
   */
  std::string find_msg_file(const ParsedType& parsed) const;

  /**
   * Read and cache a .msg file's content.
   *
   * @param parsed The parsed message type
   * @return The file content, or empty string on failure
   */
  std::string read_msg_file(const ParsedType& parsed);

  /**
   * Extract non-primitive type dependencies from a .msg file's content.
   *
   * @param content The .msg file content
   * @param context_package The package containing this .msg file
   * @return List of dependent types (canonical ROS2 names)
   */
  std::vector<ParsedType> extract_dependencies(
    const std::string& content, const std::string& context_package
  ) const;

  /**
   * Check if a type name is a ROS primitive type.
   */
  static bool is_primitive(const std::string& type_name);

  /**
   * Check if a type name is a ROS1 built-in type (time, duration)
   * that needs mapping to builtin_interfaces.
   */
  static bool is_ros1_builtin(const std::string& type_name);

  std::vector<std::string> search_paths_;
  std::unordered_map<std::string, std::string> file_cache_;  // canonical_name -> content
  std::string last_error_;
};

}  // namespace recorder
}  // namespace axon

#endif  // AXON_SCHEMA_RESOLVER_HPP
