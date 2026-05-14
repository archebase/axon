// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "panel_api.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <system_error>
#include <unordered_map>
#include <vector>

#include "config_cache.hpp"
#include "httplib.h"
#include "nlohmann/json.hpp"

namespace axon {
namespace panel {

namespace fs = std::filesystem;
using json = nlohmann::json;

namespace {

constexpr const char* kPanelHistoryFile = ".panel_history.jsonl";
constexpr const char* kPanelHistoryDir = ".panel_history";
constexpr const char* kPanelTaskStateFile = ".panel_tasks.json";
constexpr const char* kPanelLoggingStateFile = ".panel_logging.json";
constexpr size_t kMaxPreviewBytes = 1024 * 1024;
constexpr size_t kMaxUploadBytes = 16 * 1024 * 1024;

std::string env_or_empty(const char* name) {
  const char* value = std::getenv(name);
  return value == nullptr ? "" : std::string(value);
}

std::string to_lower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

bool starts_with(const std::string& value, const std::string& prefix) {
  return value.size() >= prefix.size() && value.compare(0, prefix.size(), prefix) == 0;
}

bool contains_ci(const std::string& value, const std::string& needle) {
  return to_lower(value).find(to_lower(needle)) != std::string::npos;
}

std::time_t file_time_to_time_t(const fs::file_time_type& file_time) {
  const auto system_time = std::chrono::time_point_cast<std::chrono::system_clock::duration>(
    file_time - fs::file_time_type::clock::now() + std::chrono::system_clock::now()
  );
  return std::chrono::system_clock::to_time_t(system_time);
}

std::string format_utc_time(std::time_t time, const char* pattern) {
  std::tm tm{};
#if defined(_WIN32) || defined(_WIN64)
  gmtime_s(&tm, &time);
#else
  gmtime_r(&time, &tm);
#endif
  char buffer[40];
  std::strftime(buffer, sizeof(buffer), pattern, &tm);
  return buffer;
}

std::string iso8601_from_time(std::time_t time) {
  return format_utc_time(time, "%Y-%m-%dT%H:%M:%SZ");
}

std::string compact_timestamp() {
  return format_utc_time(std::time(nullptr), "%Y%m%dT%H%M%SZ");
}

std::string now_iso8601() {
  return iso8601_from_time(std::time(nullptr));
}

std::optional<uint64_t> file_size_or_null(const fs::path& path) {
  std::error_code ec;
  const auto size = fs::file_size(path, ec);
  if (ec) {
    return std::nullopt;
  }
  return static_cast<uint64_t>(size);
}

std::optional<std::time_t> file_mtime_or_null(const fs::path& path) {
  std::error_code ec;
  const auto mtime = fs::last_write_time(path, ec);
  if (ec) {
    return std::nullopt;
  }
  return file_time_to_time_t(mtime);
}

void send_json(httplib::Response& res, const json& body, int status = 200) {
  res.status = status;
  res.set_header("Cache-Control", "no-store");
  res.set_content(body.dump(), "application/json");
}

void send_error(httplib::Response& res, int status, const std::string& message) {
  send_json(res, json{{"success", false}, {"message", message}}, status);
}

bool is_allowed_user_component(const fs::path& component, bool allow_hidden) {
  const std::string part = component.string();
  if (part.empty() || part == "." || part == "..") {
    return false;
  }
  if (!allow_hidden && !part.empty() && part.front() == '.') {
    return false;
  }
  return part.find('/') == std::string::npos && part.find('\\') == std::string::npos;
}

bool is_hidden_path_component(const fs::path& path) {
  const std::string filename = path.filename().string();
  return !filename.empty() && filename.front() == '.';
}

std::optional<fs::path> safe_join(
  const fs::path& base, const std::string& relative, bool allow_hidden, std::string& error
) {
  if (relative.empty()) {
    error = "relative path is required";
    return std::nullopt;
  }

  fs::path rel_path(relative);
  if (rel_path.is_absolute()) {
    error = "absolute paths are not allowed";
    return std::nullopt;
  }

  fs::path clean;
  for (const auto& component : rel_path) {
    if (!is_allowed_user_component(component, allow_hidden)) {
      error = "path contains an unsafe component: " + component.string();
      return std::nullopt;
    }
    clean /= component;
  }

  if (clean.empty()) {
    error = "relative path is required";
    return std::nullopt;
  }

  return base / clean;
}

std::string safe_filename(std::string filename) {
  std::replace(filename.begin(), filename.end(), '\\', '/');
  filename = fs::path(filename).filename().string();
  std::string output;
  output.reserve(filename.size());
  for (unsigned char c : filename) {
    if (std::isalnum(c) || c == '.' || c == '_' || c == '-') {
      output.push_back(static_cast<char>(c));
    } else {
      output.push_back('_');
    }
  }
  if (output.empty() || output == "." || output == "..") {
    return "uploaded_config";
  }
  return output;
}

std::string safe_history_id_for_path(const std::string& rel_path) {
  std::string output;
  output.reserve(rel_path.size());
  for (unsigned char c : rel_path) {
    if (std::isalnum(c) || c == '.' || c == '_' || c == '-') {
      output.push_back(static_cast<char>(c));
    } else {
      output.push_back('_');
    }
  }
  if (output.empty()) {
    return "config";
  }
  return output;
}

std::string read_text_file(const fs::path& path, std::string& error) {
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    error = "failed to open " + path.string();
    return "";
  }
  std::ostringstream body;
  body << file.rdbuf();
  if (!file.good() && !file.eof()) {
    error = "failed to read " + path.string();
    return "";
  }
  return body.str();
}

std::string read_text_file_prefix(
  const fs::path& path, size_t max_bytes, bool& truncated, std::string& error
) {
  truncated = false;
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    error = "failed to open " + path.string();
    return "";
  }

  std::string body(max_bytes + 1, '\0');
  file.read(body.data(), static_cast<std::streamsize>(body.size()));
  const auto read_count = file.gcount();
  if (!file.good() && !file.eof()) {
    error = "failed to read " + path.string();
    return "";
  }

  truncated = static_cast<size_t>(read_count) > max_bytes;
  body.resize(truncated ? max_bytes : static_cast<size_t>(read_count));
  return body;
}

bool write_binary_file(const fs::path& path, const std::string& content, std::string& error) {
  std::error_code ec;
  fs::create_directories(path.parent_path(), ec);
  if (ec) {
    error = "failed to create directory " + path.parent_path().string() + ": " + ec.message();
    return false;
  }

  std::ofstream file(path, std::ios::binary | std::ios::trunc);
  if (!file) {
    error = "failed to open " + path.string() + " for writing";
    return false;
  }
  file.write(content.data(), static_cast<std::streamsize>(content.size()));
  if (!file) {
    error = "failed to write " + path.string();
    return false;
  }
  return true;
}

bool write_json_file(const fs::path& path, const json& value, std::string& error) {
  return write_binary_file(path, value.dump(2) + "\n", error);
}

json read_json_file_or_default(const fs::path& path, const json& fallback) {
  if (!fs::exists(path)) {
    return fallback;
  }
  std::string error;
  const std::string body = read_text_file(path, error);
  if (!error.empty()) {
    return fallback;
  }
  try {
    return json::parse(body);
  } catch (const json::exception&) {
    return fallback;
  }
}

bool is_probably_text(const std::string& content) {
  const size_t sample_size = std::min<size_t>(content.size(), 4096);
  for (size_t i = 0; i < sample_size; ++i) {
    const unsigned char c = static_cast<unsigned char>(content[i]);
    if (c == 0) {
      return false;
    }
    if (c < 0x09 || (c > 0x0D && c < 0x20)) {
      return false;
    }
  }
  return true;
}

std::vector<std::string> split_lines(const std::string& content) {
  std::vector<std::string> lines;
  std::istringstream input(content);
  std::string line;
  while (std::getline(input, line)) {
    lines.push_back(line);
  }
  if (!content.empty() && content.back() == '\n') {
    lines.push_back("");
  }
  return lines;
}

json build_simple_line_diff(const std::string& before, const std::string& after) {
  constexpr size_t kMaxDiffLines = 1600;
  std::vector<std::string> before_lines = split_lines(before);
  std::vector<std::string> after_lines = split_lines(after);

  bool truncated = false;
  if (before_lines.size() > kMaxDiffLines) {
    before_lines.resize(kMaxDiffLines);
    truncated = true;
  }
  if (after_lines.size() > kMaxDiffLines) {
    after_lines.resize(kMaxDiffLines);
    truncated = true;
  }

  const size_t rows = before_lines.size();
  const size_t cols = after_lines.size();
  std::vector<std::vector<uint16_t>> lcs(rows + 1, std::vector<uint16_t>(cols + 1, 0));
  for (size_t i = rows; i > 0; --i) {
    for (size_t j = cols; j > 0; --j) {
      if (before_lines[i - 1] == after_lines[j - 1]) {
        lcs[i - 1][j - 1] = static_cast<uint16_t>(lcs[i][j] + 1);
      } else {
        lcs[i - 1][j - 1] = std::max(lcs[i][j - 1], lcs[i - 1][j]);
      }
    }
  }

  json lines = json::array();
  size_t i = 0;
  size_t j = 0;
  while (i < rows || j < cols) {
    if (i < rows && j < cols && before_lines[i] == after_lines[j]) {
      lines.push_back({{"type", "context"}, {"text", before_lines[i]}});
      ++i;
      ++j;
    } else if (j < cols && (i == rows || lcs[i][j + 1] >= lcs[i + 1][j])) {
      lines.push_back({{"type", "added"}, {"text", after_lines[j]}});
      ++j;
    } else if (i < rows) {
      lines.push_back({{"type", "removed"}, {"text", before_lines[i]}});
      ++i;
    }
  }

  return json{{"lines", lines}, {"truncated", truncated}};
}

std::string config_category_for_path(const std::string& rel_path) {
  const std::string lower = to_lower(rel_path);
  const fs::path path(rel_path);
  const std::string ext = to_lower(path.extension().string());
  if (ext == ".tpl" || starts_with(lower, "templates/")) {
    return "template";
  }
  if (ext == ".urdf" || ext == ".xacro" || contains_ci(lower, "urdf")) {
    return "urdf";
  }
  if (contains_ci(lower, "calib") || contains_ci(lower, "calibration") ||
      contains_ci(lower, "extrinsic") || contains_ci(lower, "intrinsic")) {
    return "calibration";
  }
  if (contains_ci(lower, "sensor") || contains_ci(lower, "camera") || contains_ci(lower, "lidar") ||
      contains_ci(lower, "imu") || contains_ci(lower, "radar")) {
    return "sensor";
  }
  return "config";
}

json validate_config_content(const std::string& rel_path, const std::string& content) {
  json errors = json::array();
  json warnings = json::array();
  const fs::path path(rel_path);
  const std::string ext = to_lower(path.extension().string());
  const std::string category = config_category_for_path(rel_path);

  if (content.empty()) {
    errors.push_back("file is empty");
  }

  if ((ext == ".urdf" || ext == ".xml" || category == "urdf") &&
      content.find("<robot") == std::string::npos) {
    errors.push_back("URDF/XML file does not contain a <robot> element");
  }

  if (ext == ".json") {
    try {
      const auto parsed = json::parse(content);
      (void)parsed;
    } catch (const json::exception& e) {
      errors.push_back(std::string("JSON parse error: ") + e.what());
    }
  }

  if ((ext == ".yaml" || ext == ".yml") && content.find('\t') != std::string::npos) {
    warnings.push_back("YAML contains tab characters; prefer spaces for indentation");
  }

  if (category == "calibration" && !content.empty() &&
      !(contains_ci(content, "camera_matrix") || contains_ci(content, "intrinsic") ||
        contains_ci(content, "extrinsic") || contains_ci(content, "translation") ||
        contains_ci(content, "rotation") || contains_ci(content, "transform"))) {
    warnings.push_back("calibration file has no obvious intrinsic/extrinsic or transform fields");
  }

  if (category == "sensor" && !content.empty() &&
      !(contains_ci(content, "topic") || contains_ci(content, "frame_id") ||
        contains_ci(content, "sensor") || contains_ci(content, "rate"))) {
    warnings.push_back("sensor config has no obvious topic, frame_id, sensor, or rate field");
  }

  return json{{"errors", errors}, {"warnings", warnings}};
}

json validate_config_file(const fs::path& path, const std::string& rel_path) {
  const auto size = file_size_or_null(path).value_or(0);
  if (size > 1024 * 1024) {
    return json{{"errors", json::array()}, {"warnings", json::array({"large file not previewed"})}};
  }

  std::string error;
  const std::string content = read_text_file(path, error);
  if (!error.empty()) {
    return json{{"errors", json::array({error})}, {"warnings", json::array()}};
  }
  if (!is_probably_text(content)) {
    return json{
      {"errors", json::array()}, {"warnings", json::array({"binary file not validated"})}
    };
  }
  return validate_config_content(rel_path, content);
}

json config_file_to_json(const fs::path& root, const fs::directory_entry& entry) {
  std::error_code ec;
  const fs::path rel = fs::relative(entry.path(), root, ec);
  const std::string rel_path = ec ? entry.path().filename().string() : rel.generic_string();
  const auto size = file_size_or_null(entry.path()).value_or(0);
  const auto mtime = file_mtime_or_null(entry.path()).value_or(0);

  json file = {
    {"path", rel_path},
    {"name", entry.path().filename().string()},
    {"category", config_category_for_path(rel_path)},
    {"content_type", axon::config::guess_content_type(rel_path)},
    {"size", size},
    {"modified_at", iso8601_from_time(mtime)},
    {"modified_epoch", static_cast<uint64_t>(mtime)},
  };
  file["validation"] = validate_config_file(entry.path(), rel_path);
  return file;
}

json list_config_files(const fs::path& root) {
  json files = json::array();
  if (!fs::exists(root) || !fs::is_directory(root)) {
    return files;
  }

  std::error_code ec;
  fs::recursive_directory_iterator it(root, fs::directory_options::skip_permission_denied, ec);
  const fs::recursive_directory_iterator end;
  for (; !ec && it != end; it.increment(ec)) {
    const auto& entry = *it;
    if (is_hidden_path_component(entry.path())) {
      std::error_code type_ec;
      if (entry.is_directory(type_ec)) {
        it.disable_recursion_pending();
      }
      continue;
    }

    std::error_code type_ec;
    if (!entry.is_regular_file(type_ec)) {
      continue;
    }

    std::error_code rel_ec;
    const fs::path rel = fs::relative(entry.path(), root, rel_ec);
    const std::string rel_path = rel_ec ? entry.path().filename().string() : rel.generic_string();
    if (starts_with(rel_path, std::string(kPanelHistoryDir) + "/")) {
      continue;
    }

    files.push_back(config_file_to_json(root, entry));
  }

  std::sort(files.begin(), files.end(), [](const json& lhs, const json& rhs) {
    return lhs.value("path", "") < rhs.value("path", "");
  });
  return files;
}

json status_info_to_json(const axon::config::ConfigCache& cache) {
  const auto info = cache.get_status();
  json value = {
    {"enabled", info.enabled},
    {"cache_exists", info.cache_exists},
    {"file_count", info.file_count},
    {"total_size", info.total_size},
    {"cache_path", cache.cache_path()},
    {"enabled_marker_path", cache.enabled_marker_path()},
  };
  if (info.cache_mtime > 0) {
    value["last_scanned_at"] = iso8601_from_time(static_cast<std::time_t>(info.cache_mtime));
    value["cache_mtime"] = info.cache_mtime;
  } else {
    value["last_scanned_at"] = nullptr;
    value["cache_mtime"] = 0;
  }
  return value;
}

fs::path history_file_path(const PanelApiOptions& options) {
  return fs::path(options.config_dir) / kPanelHistoryFile;
}

json read_history(const PanelApiOptions& options, size_t limit = 50) {
  json history = json::array();
  const fs::path path = history_file_path(options);
  if (!fs::exists(path)) {
    return history;
  }

  std::ifstream file(path, std::ios::binary);
  if (!file) {
    return history;
  }

  std::vector<json> items;
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }
    try {
      items.push_back(json::parse(line));
    } catch (const json::exception&) {
      continue;
    }
  }

  const auto begin =
    items.size() > limit ? items.end() - static_cast<std::ptrdiff_t>(limit) : items.begin();
  for (auto it = items.end(); it != begin;) {
    --it;
    history.push_back(*it);
  }
  return history;
}

bool append_history(const PanelApiOptions& options, const json& entry, std::string& error) {
  std::error_code ec;
  fs::create_directories(fs::path(options.config_dir), ec);
  if (ec) {
    error = "failed to create config directory: " + ec.message();
    return false;
  }

  std::ofstream file(history_file_path(options), std::ios::binary | std::ios::app);
  if (!file) {
    error = "failed to open history file";
    return false;
  }
  file << entry.dump() << "\n";
  if (!file) {
    error = "failed to write history file";
    return false;
  }
  return true;
}

json build_diagnostics(
  const json& cache_status, const json& files, const PanelApiOptions& options
) {
  json diagnostics = json::array();
  const fs::path config_dir(options.config_dir);
  if (!fs::exists(config_dir)) {
    diagnostics.push_back(
      {{"severity", "error"},
       {"code", "config_dir_missing"},
       {"message", "Config directory does not exist. Upload a file or run axon-config init."}}
    );
    return diagnostics;
  }

  if (files.empty()) {
    diagnostics.push_back(
      {{"severity", "warning"},
       {"code", "config_empty"},
       {"message", "No robot config files were discovered in the config directory."}}
    );
  }

  if (cache_status.value("enabled", false) && !cache_status.value("cache_exists", false)) {
    diagnostics.push_back(
      {{"severity", "error"},
       {"code", "enabled_without_cache"},
       {"message", "Config injection is enabled but cache.mcap does not exist. Rebuild the cache."}}
    );
  }

  if (!cache_status.value("enabled", false)) {
    diagnostics.push_back(
      {{"severity", "info"},
       {"code", "injection_disabled"},
       {"message", "Config injection is disabled; recordings will not receive config attachments."}}
    );
  }

  std::set<std::string> categories;
  uint64_t newest_file_mtime = 0;
  for (const auto& file : files) {
    categories.insert(file.value("category", "config"));
    const uint64_t modified_epoch = file.value("modified_epoch", uint64_t{0});
    newest_file_mtime = std::max(newest_file_mtime, modified_epoch);
    const json validation = file.value("validation", json::object());
    for (const auto& error : validation.value("errors", json::array())) {
      diagnostics.push_back(
        {{"severity", "error"},
         {"code", "file_validation_error"},
         {"path", file.value("path", "")},
         {"message", error.get<std::string>()}}
      );
    }
    for (const auto& warning : validation.value("warnings", json::array())) {
      diagnostics.push_back(
        {{"severity", "warning"},
         {"code", "file_validation_warning"},
         {"path", file.value("path", "")},
         {"message", warning.get<std::string>()}}
      );
    }
  }

  if (!files.empty() && categories.count("urdf") == 0) {
    diagnostics.push_back(
      {{"severity", "warning"},
       {"code", "urdf_missing"},
       {"message", "No URDF file was discovered. Robot geometry attachments may be incomplete."}}
    );
  }
  if (!files.empty() && categories.count("calibration") == 0) {
    diagnostics.push_back(
      {{"severity", "warning"},
       {"code", "calibration_missing"},
       {"message", "No calibration file was discovered."}}
    );
  }
  if (!files.empty() && categories.count("sensor") == 0) {
    diagnostics.push_back(
      {{"severity", "warning"},
       {"code", "sensor_config_missing"},
       {"message", "No sensor config file was discovered."}}
    );
  }

  const uint64_t cache_mtime = cache_status.value("cache_mtime", 0ULL);
  if (cache_status.value("cache_exists", false) && newest_file_mtime > cache_mtime) {
    diagnostics.push_back(
      {{"severity", "warning"},
       {"code", "cache_stale"},
       {"message", "One or more config files are newer than cache.mcap. Rebuild the cache."}}
    );
  }

  return diagnostics;
}

json build_config_status(const PanelApiOptions& options) {
  axon::config::ConfigCache cache;
  cache.set_config_dir(options.config_dir);
  json cache_status = status_info_to_json(cache);
  json files = list_config_files(options.config_dir);

  json grouped = json::object();
  for (const auto& file : files) {
    const std::string category = file.value("category", "config");
    if (!grouped.contains(category)) {
      grouped[category] = json::array();
    }
    grouped[category].push_back(file);
  }

  json result = {
    {"success", true},
    {"config_dir", options.config_dir},
    {"cache", cache_status},
    {"files", files},
    {"groups", grouped},
    {"history", read_history(options)},
  };
  result["diagnostics"] = build_diagnostics(cache_status, files, options);
  return result;
}

std::string multipart_field(const httplib::Request& req, const std::string& name) {
  if (!req.has_file(name)) {
    return "";
  }
  return req.get_file_value(name).content;
}

std::string target_path_for_upload(
  const httplib::MultipartFormData& upload, const std::string& path_field,
  const std::string& category_field
) {
  if (!path_field.empty()) {
    return fs::path(path_field).generic_string();
  }

  const std::string filename = safe_filename(upload.filename);
  const std::string category = to_lower(category_field);
  if (category == "robot" || category == "urdf") {
    return (fs::path("robot") / filename).generic_string();
  }
  if (category == "sensor" || category == "sensors") {
    return (fs::path("sensors") / filename).generic_string();
  }
  if (category == "calibration" || category == "calib") {
    return (fs::path("calibration") / filename).generic_string();
  }
  return filename;
}

json upload_warnings_for_path(const std::string& rel_path) {
  static const std::set<std::string> allowed = {
    ".urdf",
    ".xacro",
    ".xml",
    ".yaml",
    ".yml",
    ".json",
    ".txt",
    ".cfg",
    ".ini",
    ".tpl",
    ".png",
    ".jpg",
    ".jpeg",
    ".bin",
    ".pdf",
  };
  const std::string ext = to_lower(fs::path(rel_path).extension().string());
  if (ext.empty() || allowed.count(ext) > 0) {
    return json::array();
  }
  return json::array({"file extension is not recognized as a common robot config artifact"});
}

std::optional<json> find_history_item(
  const PanelApiOptions& options, const std::string& history_id
) {
  const json history = read_history(options, 500);
  for (const auto& item : history) {
    if (item.value("id", "") == history_id) {
      return item;
    }
  }
  return std::nullopt;
}

fs::path panel_state_root(const PanelApiOptions& options) {
  const fs::path config_dir(options.config_dir);
  if (config_dir.has_parent_path()) {
    return config_dir.parent_path();
  }
  return fs::temp_directory_path() / "axon";
}

fs::path task_state_file_path(const PanelApiOptions& options) {
  return fs::path(options.recording_dir) / kPanelTaskStateFile;
}

json load_task_state(const PanelApiOptions& options) {
  json state = read_json_file_or_default(task_state_file_path(options), json::object());
  return state.is_object() ? state : json::object();
}

bool save_task_state(const PanelApiOptions& options, const json& state, std::string& error) {
  return write_json_file(task_state_file_path(options), state, error);
}

json list_recording_tasks(const PanelApiOptions& options) {
  json tasks = json::array();
  const fs::path root(options.recording_dir);
  const json state = load_task_state(options);
  if (!fs::exists(root) || !fs::is_directory(root)) {
    return tasks;
  }

  std::error_code ec;
  fs::recursive_directory_iterator it(root, fs::directory_options::skip_permission_denied, ec);
  const fs::recursive_directory_iterator end;
  for (; !ec && it != end; it.increment(ec)) {
    const auto& entry = *it;
    if (is_hidden_path_component(entry.path())) {
      std::error_code type_ec;
      if (entry.is_directory(type_ec)) {
        it.disable_recursion_pending();
      }
      continue;
    }

    std::error_code type_ec;
    if (!entry.is_regular_file(type_ec) || entry.path().extension() != ".mcap") {
      continue;
    }

    std::error_code rel_ec;
    const fs::path rel = fs::relative(entry.path(), root, rel_ec);
    const std::string rel_path = rel_ec ? entry.path().filename().string() : rel.generic_string();
    const std::string task_id = rel_path;
    const fs::path sidecar = entry.path().parent_path() / (entry.path().stem().string() + ".json");
    const auto mtime = file_mtime_or_null(entry.path()).value_or(0);
    const json task_state =
      state.contains(task_id) && state[task_id].is_object() ? state[task_id] : json::object();

    tasks.push_back(
      {{"task_id", task_id},
       {"name", entry.path().stem().string()},
       {"path", rel_path},
       {"absolute_path", entry.path().string()},
       {"sidecar_path", fs::exists(sidecar) ? sidecar.string() : ""},
       {"sidecar_exists", fs::exists(sidecar)},
       {"size", file_size_or_null(entry.path()).value_or(0)},
       {"modified_at", iso8601_from_time(mtime)},
       {"modified_epoch", static_cast<uint64_t>(mtime)},
       {"reviewed", task_state.value("reviewed", false)},
       {"queued", task_state.value("queued", false)},
       {"status", task_state.value("status", "local")}}
    );
  }

  std::sort(tasks.begin(), tasks.end(), [](const json& lhs, const json& rhs) {
    return lhs.value("modified_epoch", 0ULL) > rhs.value("modified_epoch", 0ULL);
  });
  return tasks;
}

fs::path logging_state_file_path(const PanelApiOptions& options) {
  return panel_state_root(options) / kPanelLoggingStateFile;
}

std::string default_log_level() {
  const std::string env_level = env_or_empty("AXON_LOG_LEVEL");
  return env_level.empty() ? "info" : to_lower(env_level);
}

bool is_valid_log_level(const std::string& level) {
  static const std::set<std::string> valid = {"debug", "info", "warning", "error", "fatal"};
  return valid.count(to_lower(level)) > 0;
}

json logging_status(const PanelApiOptions& options) {
  json state = read_json_file_or_default(logging_state_file_path(options), json::object());
  if (!state.is_object()) {
    state = json::object();
  }
  if (!state.contains("level") || !state["level"].is_string()) {
    state["level"] = default_log_level();
  }
  state["levels"] = json::array({"debug", "info", "warning", "error", "fatal"});
  state["state_path"] = logging_state_file_path(options).string();
  return json{{"success", true}, {"logging", state}};
}

void handle_config_upload(
  const PanelApiOptions& options, const httplib::Request& req, httplib::Response& res
) {
  if (!req.is_multipart_form_data()) {
    send_error(res, 400, "multipart/form-data request is required");
    return;
  }
  if (!req.has_file("file")) {
    send_error(res, 400, "missing multipart file field: file");
    return;
  }

  const auto upload = req.get_file_value("file");
  if (upload.content.empty()) {
    send_error(res, 400, "uploaded file is empty");
    return;
  }
  if (upload.content.size() > kMaxUploadBytes) {
    send_error(res, 413, "uploaded file exceeds the 16 MiB limit");
    return;
  }

  const std::string rel_path =
    target_path_for_upload(upload, multipart_field(req, "path"), multipart_field(req, "category"));
  std::string path_error;
  const auto target = safe_join(options.config_dir, rel_path, false, path_error);
  if (!target) {
    send_error(res, 400, path_error);
    return;
  }

  const std::string id = compact_timestamp() + "-" + safe_history_id_for_path(rel_path);
  const fs::path history_dir = fs::path(options.config_dir) / kPanelHistoryDir;
  const fs::path previous_snapshot = history_dir / (id + ".previous");
  bool replaced = false;
  std::string error;

  if (fs::exists(*target)) {
    std::error_code ec;
    fs::create_directories(history_dir, ec);
    if (ec) {
      send_error(res, 500, "failed to create history directory: " + ec.message());
      return;
    }
    fs::copy_file(*target, previous_snapshot, fs::copy_options::overwrite_existing, ec);
    if (ec) {
      send_error(res, 500, "failed to snapshot previous file: " + ec.message());
      return;
    }
    replaced = true;
  }

  if (!write_binary_file(*target, upload.content, error)) {
    send_error(res, 500, error);
    return;
  }

  json validation = validate_config_content(rel_path, upload.content);
  for (const auto& warning : upload_warnings_for_path(rel_path)) {
    validation["warnings"].push_back(warning);
  }

  json history_entry = {
    {"id", id},
    {"action", replaced ? "update" : "upload"},
    {"path", rel_path},
    {"filename", upload.filename},
    {"content_type",
     upload.content_type.empty() ? axon::config::guess_content_type(rel_path)
                                 : upload.content_type},
    {"size", upload.content.size()},
    {"created_at", now_iso8601()},
    {"replaced", replaced},
    {"previous_snapshot",
     replaced ? (fs::path(kPanelHistoryDir) / (id + ".previous")).generic_string() : ""},
    {"validation", validation},
  };

  if (!append_history(options, history_entry, error)) {
    send_error(res, 500, error);
    return;
  }

  send_json(
    res,
    json{
      {"success", true},
      {"message", replaced ? "Configuration file updated" : "Configuration file uploaded"},
      {"file", config_file_to_json(options.config_dir, fs::directory_entry(*target))},
      {"history", history_entry},
      {"status", build_config_status(options)},
    }
  );
}

void handle_get_config_file(
  const PanelApiOptions& options, const httplib::Request& req, httplib::Response& res
) {
  const std::string version = req.get_param_value("version");
  const std::string history_id = req.get_param_value("history_id");
  fs::path path;
  std::string rel_path;

  if (version == "previous") {
    const auto history_item = find_history_item(options, history_id);
    if (!history_item) {
      send_error(res, 404, "history item not found");
      return;
    }
    const std::string snapshot = history_item->value("previous_snapshot", "");
    if (snapshot.empty()) {
      send_error(res, 404, "history item has no previous snapshot");
      return;
    }
    std::string error;
    const auto snapshot_path = safe_join(options.config_dir, snapshot, true, error);
    if (!snapshot_path) {
      send_error(res, 400, error);
      return;
    }
    path = *snapshot_path;
    rel_path = snapshot;
  } else {
    rel_path = req.get_param_value("path");
    std::string error;
    const auto current_path = safe_join(options.config_dir, rel_path, false, error);
    if (!current_path) {
      send_error(res, 400, error);
      return;
    }
    path = *current_path;
  }

  if (!fs::exists(path) || !fs::is_regular_file(path)) {
    send_error(res, 404, "file not found");
    return;
  }

  std::string error;
  bool truncated = false;
  std::string content = read_text_file_prefix(path, kMaxPreviewBytes, truncated, error);
  if (!error.empty()) {
    send_error(res, 500, error);
    return;
  }

  const bool text = is_probably_text(content);

  send_json(
    res,
    json{
      {"success", true},
      {"path", rel_path},
      {"content", text ? content : ""},
      {"binary", !text},
      {"truncated", truncated},
      {"size", file_size_or_null(path).value_or(0)},
      {"content_type", axon::config::guess_content_type(rel_path)},
    }
  );
}

void handle_config_diff(
  const PanelApiOptions& options, const httplib::Request& req, httplib::Response& res
) {
  const std::string history_id = req.get_param_value("history_id");
  const auto history_item = find_history_item(options, history_id);
  if (!history_item) {
    send_error(res, 404, "history item not found");
    return;
  }
  const std::string rel_path = history_item->value("path", "");
  const std::string snapshot = history_item->value("previous_snapshot", "");
  if (rel_path.empty() || snapshot.empty()) {
    send_error(res, 404, "history item has no diffable snapshot");
    return;
  }

  std::string error;
  const auto before_path = safe_join(options.config_dir, snapshot, true, error);
  if (!before_path) {
    send_error(res, 400, error);
    return;
  }
  const auto after_path = safe_join(options.config_dir, rel_path, false, error);
  if (!after_path) {
    send_error(res, 400, error);
    return;
  }
  if (!fs::exists(*before_path) || !fs::exists(*after_path)) {
    send_error(res, 404, "diff file is missing");
    return;
  }
  const auto before_size = file_size_or_null(*before_path).value_or(0);
  const auto after_size = file_size_or_null(*after_path).value_or(0);
  if (before_size > kMaxPreviewBytes || after_size > kMaxPreviewBytes) {
    send_error(res, 413, "diff files exceed the 1 MiB limit");
    return;
  }

  const std::string before = read_text_file(*before_path, error);
  if (!error.empty()) {
    send_error(res, 500, error);
    return;
  }
  const std::string after = read_text_file(*after_path, error);
  if (!error.empty()) {
    send_error(res, 500, error);
    return;
  }
  if (!is_probably_text(before) || !is_probably_text(after)) {
    send_error(res, 400, "binary files cannot be diffed");
    return;
  }

  json diff = build_simple_line_diff(before, after);
  diff["success"] = true;
  diff["path"] = rel_path;
  diff["history_id"] = history_id;
  send_json(res, diff);
}

}  // namespace

PanelApiOptions make_default_panel_api_options() {
  PanelApiOptions options;
  const std::string config_dir = env_or_empty("AXON_PANEL_CONFIG_DIR");
  options.config_dir =
    config_dir.empty() ? axon::config::ConfigCache::default_config_dir() : config_dir;

  const std::string recording_dir = env_or_empty("AXON_PANEL_RECORDING_DIR");
  if (!recording_dir.empty()) {
    options.recording_dir = recording_dir;
  } else {
    const std::string transfer_dir = env_or_empty("AXON_TRANSFER_DATA_DIR");
    options.recording_dir = transfer_dir.empty() ? "/tmp/axon/recording" : transfer_dir;
  }
  return options;
}

void register_panel_api(httplib::Server& server, const PanelApiOptions& options) {
  server.Get(
    "/api/panel/config/status", [options](const httplib::Request&, httplib::Response& res) {
      send_json(res, build_config_status(options));
    }
  );

  server.Post(
    "/api/panel/config/scan", [options](const httplib::Request& req, httplib::Response& res) {
      const std::string incremental_param = to_lower(req.get_param_value("incremental"));
      const bool incremental =
        incremental_param == "1" || incremental_param == "true" || incremental_param == "yes";

      axon::config::ConfigCache cache;
      cache.set_config_dir(options.config_dir);
      if (incremental) {
        (void)cache.load();
      }
      auto result = cache.scan(incremental);
      if (result.status != axon::config::ConfigCache::Status::Ok) {
        send_json(
          res,
          json{
            {"success", false},
            {"message", result.message},
            {"file_count", result.file_count},
            {"total_size", result.total_size},
            {"status", build_config_status(options)},
          },
          result.status == axon::config::ConfigCache::Status::DirNotFound ? 404 : 500
        );
        return;
      }

      json history_entry = {
        {"id", compact_timestamp() + "-scan"},
        {"action", incremental ? "incremental_scan" : "scan"},
        {"path", ""},
        {"created_at", now_iso8601()},
        {"file_count", result.file_count},
        {"total_size", result.total_size},
        {"message", result.message},
      };
      std::string error;
      (void)append_history(options, history_entry, error);

      send_json(
        res,
        json{
          {"success", true},
          {"message", result.message},
          {"file_count", result.file_count},
          {"total_size", result.total_size},
          {"status", build_config_status(options)},
        }
      );
    }
  );

  server.Post(
    "/api/panel/config/injection/enable",
    [options](const httplib::Request&, httplib::Response& res) {
      axon::config::ConfigCache cache;
      cache.set_config_dir(options.config_dir);
      if (!cache.enable()) {
        send_error(res, 500, "failed to enable config injection");
        return;
      }
      std::string error;
      (void)append_history(
        options,
        json{
          {"id", compact_timestamp() + "-enable"},
          {"action", "enable_injection"},
          {"path", ""},
          {"created_at", now_iso8601()}
        },
        error
      );
      send_json(
        res,
        json{
          {"success", true},
          {"message", "Config injection enabled"},
          {"status", build_config_status(options)}
        }
      );
    }
  );

  server.Post(
    "/api/panel/config/injection/disable",
    [options](const httplib::Request&, httplib::Response& res) {
      axon::config::ConfigCache cache;
      cache.set_config_dir(options.config_dir);
      if (!cache.disable() && cache.is_enabled()) {
        send_error(res, 500, "failed to disable config injection");
        return;
      }
      std::string error;
      (void)append_history(
        options,
        json{
          {"id", compact_timestamp() + "-disable"},
          {"action", "disable_injection"},
          {"path", ""},
          {"created_at", now_iso8601()}
        },
        error
      );
      send_json(
        res,
        json{
          {"success", true},
          {"message", "Config injection disabled"},
          {"status", build_config_status(options)}
        }
      );
    }
  );

  server.Post(
    "/api/panel/config/files", [options](const httplib::Request& req, httplib::Response& res) {
      handle_config_upload(options, req, res);
    }
  );

  server.Get(
    "/api/panel/config/file", [options](const httplib::Request& req, httplib::Response& res) {
      handle_get_config_file(options, req, res);
    }
  );

  server.Get(
    "/api/panel/config/diff", [options](const httplib::Request& req, httplib::Response& res) {
      handle_config_diff(options, req, res);
    }
  );

  server.Get("/api/panel/tasks", [options](const httplib::Request&, httplib::Response& res) {
    send_json(
      res,
      json{
        {"success", true},
        {"recording_dir", options.recording_dir},
        {"tasks", list_recording_tasks(options)},
      }
    );
  });

  server.Post(
    "/api/panel/tasks/batch", [options](const httplib::Request& req, httplib::Response& res) {
      json body;
      try {
        body = req.body.empty() ? json::object() : json::parse(req.body);
      } catch (const json::exception& e) {
        send_error(res, 400, std::string("invalid JSON body: ") + e.what());
        return;
      }

      const std::string action = body.value("action", "");
      if (action != "mark_reviewed" && action != "clear_reviewed" && action != "queue_upload") {
        send_error(res, 400, "unsupported batch action");
        return;
      }
      if (!body.contains("task_ids") || !body["task_ids"].is_array()) {
        send_error(res, 400, "task_ids array is required");
        return;
      }

      json current_tasks = list_recording_tasks(options);
      std::set<std::string> existing_task_ids;
      for (const auto& task : current_tasks) {
        existing_task_ids.insert(task.value("task_id", ""));
      }

      std::set<std::string> requested_task_ids;
      json invalid_task_ids = json::array();
      for (const auto& task_id_value : body["task_ids"]) {
        if (!task_id_value.is_string()) {
          send_error(res, 400, "task_ids must contain strings");
          return;
        }
        const std::string task_id = task_id_value.get<std::string>();
        if (task_id.empty()) {
          send_error(res, 400, "task_ids must not contain empty values");
          return;
        }
        if (existing_task_ids.count(task_id) == 0) {
          invalid_task_ids.push_back(task_id);
          continue;
        }
        requested_task_ids.insert(task_id);
      }

      if (!invalid_task_ids.empty()) {
        send_json(
          res,
          json{
            {"success", false},
            {"message", "one or more task IDs were not found"},
            {"invalid_task_ids", invalid_task_ids},
            {"tasks", current_tasks},
          },
          404
        );
        return;
      }
      if (requested_task_ids.empty()) {
        send_error(res, 400, "at least one existing task_id is required");
        return;
      }

      json state = load_task_state(options);
      size_t changed = 0;
      for (const auto& task_id : requested_task_ids) {
        if (!state.contains(task_id) || !state[task_id].is_object()) {
          state[task_id] = json::object();
        }
        if (action == "mark_reviewed") {
          state[task_id]["reviewed"] = true;
          state[task_id]["status"] = "reviewed";
        } else if (action == "clear_reviewed") {
          state[task_id]["reviewed"] = false;
          state[task_id]["status"] = state[task_id].value("queued", false) ? "queued" : "local";
        } else if (action == "queue_upload") {
          state[task_id]["queued"] = true;
          state[task_id]["status"] = "queued";
        }
        state[task_id]["updated_at"] = now_iso8601();
        ++changed;
      }

      std::string error;
      if (!save_task_state(options, state, error)) {
        send_error(res, 500, error);
        return;
      }

      send_json(
        res,
        json{
          {"success", true},
          {"message", "Task batch operation applied"},
          {"changed", changed},
          {"tasks", list_recording_tasks(options)},
        }
      );
    }
  );

  server.Get("/api/panel/logging", [options](const httplib::Request&, httplib::Response& res) {
    send_json(res, logging_status(options));
  });

  server.Post(
    "/api/panel/logging/level", [options](const httplib::Request& req, httplib::Response& res) {
      json body;
      try {
        body = req.body.empty() ? json::object() : json::parse(req.body);
      } catch (const json::exception& e) {
        send_error(res, 400, std::string("invalid JSON body: ") + e.what());
        return;
      }

      const std::string level = to_lower(body.value("level", ""));
      if (!is_valid_log_level(level)) {
        send_error(res, 400, "invalid log level");
        return;
      }

      json state = {
        {"level", level},
        {"updated_at", now_iso8601()},
        {"note",
         "Panel log level preference saved. Recorder process logging still follows recorder "
         "runtime support."},
      };
      std::string error;
      if (!write_json_file(logging_state_file_path(options), state, error)) {
        send_error(res, 500, error);
        return;
      }
      send_json(res, logging_status(options));
    }
  );
}

}  // namespace panel
}  // namespace axon
