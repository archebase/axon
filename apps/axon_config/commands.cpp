// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "commands.hpp"

#include <curl/curl.h>
#include <mcap/mcap.hpp>
#include <mcap/reader.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <system_error>
#include <utility>
#include <vector>

namespace axon {
namespace config {

namespace fs = std::filesystem;

// Default subdirectories to create
static const std::vector<std::string> DEFAULT_SUBDIRS = {
  "robot",
  "sensors",
};

namespace {

constexpr const char* kRegisterPath = "/api/v1/devices/register";
constexpr const char* kConfigPathPrefix = "/configs";

struct HttpResponse {
  long status_code = 0;
  std::string body;
  std::string error;
};

bool is_blank(const std::string& value) {
  return std::all_of(value.begin(), value.end(), [](unsigned char c) {
    return std::isspace(c) != 0;
  });
}

std::string trim_trailing_slashes(std::string value) {
  while (!value.empty() && value.back() == '/') {
    value.pop_back();
  }
  return value;
}

bool is_unreserved_url_char(unsigned char c) {
  return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-' ||
         c == '_' || c == '.' || c == '~';
}

std::string url_encode(const std::string& value) {
  std::ostringstream oss;
  oss << std::uppercase << std::hex;
  for (unsigned char c : value) {
    if (is_unreserved_url_char(c)) {
      oss << static_cast<char>(c);
    } else {
      oss << '%' << std::setw(2) << std::setfill('0') << static_cast<int>(c) << std::setfill(' ');
    }
  }
  return oss.str();
}

std::string json_escape(const std::string& value) {
  std::ostringstream oss;
  for (unsigned char c : value) {
    switch (c) {
      case '"':
        oss << "\\\"";
        break;
      case '\\':
        oss << "\\\\";
        break;
      case '\b':
        oss << "\\b";
        break;
      case '\f':
        oss << "\\f";
        break;
      case '\n':
        oss << "\\n";
        break;
      case '\r':
        oss << "\\r";
        break;
      case '\t':
        oss << "\\t";
        break;
      default:
        if (c < 0x20) {
          oss << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(c)
              << std::dec << std::setfill(' ');
        } else {
          oss << static_cast<char>(c);
        }
        break;
    }
  }
  return oss.str();
}

std::string build_register_payload(const std::string& factory, const std::string& robot_type) {
  std::ostringstream oss;
  oss << "{\"factory\":\"" << json_escape(factory) << "\",\"robot_type\":\""
      << json_escape(robot_type) << "\"}";
  return oss.str();
}

std::string build_register_url(const std::string& keystone_url) {
  return trim_trailing_slashes(keystone_url) + kRegisterPath;
}

std::string build_config_url(
  const std::string& keystone_url, const std::string& factory, const std::string& robot_type,
  const std::string& filename
) {
  return trim_trailing_slashes(keystone_url) + kConfigPathPrefix + "/" + url_encode(factory) + "/" +
         url_encode(robot_type) + "/" + url_encode(filename);
}

size_t write_curl_response(char* ptr, size_t size, size_t nmemb, void* userdata) {
  auto* body = static_cast<std::string*>(userdata);
  const size_t bytes = size * nmemb;
  body->append(ptr, bytes);
  return bytes;
}

HttpResponse get_text(const std::string& url, long timeout_seconds) {
  HttpResponse response;

  CURLcode global_code = curl_global_init(CURL_GLOBAL_DEFAULT);
  if (global_code != CURLE_OK) {
    response.error = curl_easy_strerror(global_code);
    return response;
  }

  CURL* curl = curl_easy_init();
  if (curl == nullptr) {
    response.error = "failed to initialize curl";
    curl_global_cleanup();
    return response;
  }

  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_curl_response);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response.body);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, timeout_seconds);
  curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
  curl_easy_setopt(curl, CURLOPT_USERAGENT, "axon-config/1.0");

  CURLcode code = curl_easy_perform(curl);
  if (code != CURLE_OK) {
    response.error = curl_easy_strerror(code);
  } else {
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response.status_code);
  }

  curl_easy_cleanup(curl);
  curl_global_cleanup();

  return response;
}

HttpResponse post_json(const std::string& url, const std::string& body, long timeout_seconds) {
  HttpResponse response;

  CURLcode global_code = curl_global_init(CURL_GLOBAL_DEFAULT);
  if (global_code != CURLE_OK) {
    response.error = curl_easy_strerror(global_code);
    return response;
  }

  CURL* curl = curl_easy_init();
  if (curl == nullptr) {
    response.error = "failed to initialize curl";
    curl_global_cleanup();
    return response;
  }

  struct curl_slist* headers = nullptr;
  headers = curl_slist_append(headers, "Content-Type: application/json");
  headers = curl_slist_append(headers, "Accept: application/json");

  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_POST, 1L);
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body.c_str());
  curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, static_cast<long>(body.size()));
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_curl_response);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response.body);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, timeout_seconds);
  curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
  curl_easy_setopt(curl, CURLOPT_USERAGENT, "axon-config/1.0");

  CURLcode code = curl_easy_perform(curl);
  if (code != CURLE_OK) {
    response.error = curl_easy_strerror(code);
  } else {
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response.status_code);
  }

  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);
  curl_global_cleanup();

  return response;
}

bool write_text_file_atomic(const fs::path& path, const std::string& content, std::string& error) {
  std::error_code ec;
  fs::create_directories(path.parent_path(), ec);
  if (ec) {
    error = "failed to create directory " + path.parent_path().string() + ": " + ec.message();
    return false;
  }

  fs::path tmp_path = path;
  tmp_path += ".tmp";

  {
    std::ofstream file(tmp_path, std::ios::binary | std::ios::trunc);
    if (!file) {
      error = "failed to open " + tmp_path.string() + " for writing";
      return false;
    }
    file << content;
    if (!file) {
      error = "failed to write " + tmp_path.string();
      return false;
    }
  }

  fs::rename(tmp_path, path, ec);
  if (ec) {
    fs::remove(tmp_path);
    error = "failed to replace " + path.string() + ": " + ec.message();
    return false;
  }

  return true;
}

bool download_keystone_configs(
  const RegisterOptions& options, const std::string& keystone_url, std::string& error
) {
  struct ConfigFile {
    std::string name;
    std::string body;
  };

  std::vector<ConfigFile> files = {{"recorder.yaml", ""}, {"transfer.yaml", ""}};

  for (auto& file : files) {
    const std::string url =
      build_config_url(keystone_url, options.factory, options.robot_type, file.name);
    std::cerr << "Downloading " << url << std::endl;

    HttpResponse response = get_text(url, options.timeout_seconds);
    if (!response.error.empty()) {
      error = "failed to download " + file.name + ": " + response.error;
      return false;
    }

    if (response.status_code == 404) {
      std::cerr << "Warning: Keystone config not found (HTTP 404): " << url << std::endl;
      std::cerr << "Warning: skipped downloading recorder.yaml and transfer.yaml" << std::endl;
      return true;
    }

    if (response.status_code < 200 || response.status_code >= 300) {
      std::ostringstream oss;
      oss << "failed to download " << file.name << " (HTTP " << response.status_code << ")";
      if (!response.body.empty()) {
        oss << ": " << response.body;
      }
      error = oss.str();
      return false;
    }

    file.body = std::move(response.body);
  }

  for (const auto& file : files) {
    const fs::path output_path = fs::path(options.config_dir) / file.name;
    if (!write_text_file_atomic(output_path, file.body, error)) {
      return false;
    }
    std::cerr << "Wrote " << output_path.string() << std::endl;
  }

  return true;
}

}  // namespace

Commands::Commands()
    : verbose_(false) {
  cache_.set_config_dir(ConfigCache::default_config_dir());
}

int Commands::init() {
  std::string config_dir = ConfigCache::default_config_dir();

  try {
    // Create main directory (and parent directories) if it doesn't exist
    if (!fs::exists(config_dir)) {
      fs::create_directories(config_dir);
      std::cout << "Created " << config_dir << "/" << std::endl;
    }

    // Create subdirectories
    for (const auto& subdir : DEFAULT_SUBDIRS) {
      std::string full_path = config_dir + "/" + subdir;
      if (!fs::exists(full_path)) {
        fs::create_directory(full_path);
        std::cout << "Created " << full_path << "/" << std::endl;
      }
    }

    return 0;

  } catch (const fs::filesystem_error& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }
}

int Commands::scan(bool incremental) {
  if (verbose_) {
    std::cout << "Scanning " << cache_.cache_path() << "..." << std::endl;
  }

  auto result = cache_.scan(incremental);

  if (result.status != ConfigCache::Status::Ok) {
    std::cerr << "Error: " << result.message << std::endl;
    return 1;
  }

  // Print tree structure of scanned files
  std::string config_dir = ConfigCache::default_config_dir();
  if (fs::exists(config_dir)) {
    print_tree(config_dir);
  }

  std::cout << result.message << std::endl;
  return 0;
}

int Commands::enable() {
  if (!cache_.enable()) {
    std::cerr << "Error: Failed to create enabled marker." << std::endl;
    return 1;
  }

  std::cout << "Config injection enabled." << std::endl;

  // Check if cache exists
  auto info = cache_.get_status();
  if (!info.cache_exists) {
    std::cout << "Warning: Config cache not found. Run 'axon_config scan' first." << std::endl;
    std::cout << "Config injection enabled, but no files will be written." << std::endl;
  } else {
    std::cout << "Cache: " << cache_.cache_path() << " (" << info.file_count << " files, "
              << format_size(info.total_size) << ")" << std::endl;
  }

  return 0;
}

int Commands::disable() {
  if (!cache_.disable()) {
    // Check if it was already disabled
    if (cache_.is_enabled()) {
      std::cerr << "Error: Failed to remove enabled marker." << std::endl;
      return 1;
    } else {
      std::cout << "Config injection already disabled." << std::endl;
      return 0;
    }
  }

  std::cout << "Config injection disabled." << std::endl;
  return 0;
}

int Commands::clear(bool force) {
  auto info = cache_.get_status();

  // Show summary
  std::cout << "This will DELETE all configuration data:" << std::endl;
  std::cout << "  Directory: " << ConfigCache::default_config_dir() << "/" << std::endl;
  std::cout << "  Files: " << info.file_count << std::endl;
  std::cout << "  Size: " << format_size(info.total_size) << std::endl;
  std::cout << std::endl;

  // Confirm
  if (!Confirmation::prompt("Are you sure? Type 'yes' to confirm:", force)) {
    std::cout << "Operation cancelled. No changes made." << std::endl;
    return 0;
  }

  // Perform clear
  auto status = cache_.clear();
  if (status != ConfigCache::Status::Ok) {
    std::cerr << "Error: Failed to clear config directory." << std::endl;
    return 1;
  }

  std::cout << "Deleted " << ConfigCache::default_config_dir() << "/ and all contents."
            << std::endl;
  std::cout << "Config injection disabled." << std::endl;
  return 0;
}

int Commands::status() {
  auto info = cache_.get_status();

  std::cout << "Config Status: " << (info.enabled ? "ENABLED" : "DISABLED") << std::endl;
  std::cout << "Config Directory: " << ConfigCache::default_config_dir() << std::endl;
  std::cout << "Cache File: " << cache_.cache_path() << std::endl;

  if (info.cache_exists) {
    std::cout << "Files Cached: " << info.file_count << std::endl;
    std::cout << "Total Size: " << format_size(info.total_size) << std::endl;
    std::cout << "Last Scanned: " << format_time(info.cache_mtime) << std::endl;
  } else {
    std::cout << "Files Cached: 0" << std::endl;
    std::cout << "Total Size: 0 B" << std::endl;
    std::cout << "Last Scanned: Never" << std::endl;
  }

  return 0;
}

int Commands::register_device(const RegisterOptions& options) {
  if (options.factory.empty() || is_blank(options.factory)) {
    std::cerr << "Error: factory is required" << std::endl;
    return 1;
  }
  if (options.robot_type.empty() || is_blank(options.robot_type)) {
    std::cerr << "Error: robot_type is required" << std::endl;
    return 1;
  }

  std::string keystone_url = options.keystone_url;
  if (keystone_url.empty()) {
    const char* env_url = std::getenv("AXON_KEYSTONE_URL");
    if (env_url != nullptr) {
      keystone_url = env_url;
    }
  }
  if (keystone_url.empty() || is_blank(keystone_url)) {
    std::cerr << "Error: keystone URL is required. Use --keystone-url or AXON_KEYSTONE_URL."
              << std::endl;
    return 1;
  }
  if (options.fetch_configs && (options.config_dir.empty() || is_blank(options.config_dir))) {
    std::cerr << "Error: config directory is required unless --skip-config-download is used."
              << std::endl;
    return 1;
  }

  const std::string url = build_register_url(keystone_url);
  const std::string payload = build_register_payload(options.factory, options.robot_type);

  if (verbose_) {
    std::cout << "POST " << url << std::endl;
  }

  HttpResponse response = post_json(url, payload, options.timeout_seconds);
  if (!response.error.empty()) {
    std::cerr << "Error: failed to register device: " << response.error << std::endl;
    return 1;
  }

  if (response.status_code == 201) {
    if (options.fetch_configs) {
      std::string config_error;
      if (!download_keystone_configs(options, keystone_url, config_error)) {
        std::cerr << "Error: " << config_error << std::endl;
        return 1;
      }
    }

    if (!response.body.empty()) {
      std::cout << response.body << std::endl;
    } else {
      std::cout << "Device registered." << std::endl;
    }
    return 0;
  }

  std::cerr << "Error: Keystone register failed";
  if (response.status_code != 0) {
    std::cerr << " (HTTP " << response.status_code << ")";
  }
  if (!response.body.empty()) {
    std::cerr << ": " << response.body;
  }
  std::cerr << std::endl;
  return 1;
}

int Commands::execute(int argc, char* argv[]) {
  std::string command;

  if (argc > 1) {
    command = argv[1];
  }

  if (command.empty() || command == "help" || command == "-h" || command == "--help") {
    print_usage();
    return 0;
  }

  // Parse flags
  bool incremental = false;
  bool force = false;
  RegisterOptions register_options;

  for (int i = 2; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--incremental" || arg == "-i") {
      incremental = true;
    } else if (arg == "--force" || arg == "-f") {
      force = true;
    } else if (arg == "--verbose" || arg == "-v") {
      verbose_ = true;
    }
  }

  // Execute command
  if (command == "init") {
    return init();
  } else if (command == "scan") {
    return scan(incremental);
  } else if (command == "enable") {
    return enable();
  } else if (command == "disable") {
    return disable();
  } else if (command == "clear") {
    return clear(force);
  } else if (command == "status") {
    return status();
  } else if (command == "register") {
    if (argc > 2) {
      std::string first_arg = argv[2];
      if (first_arg == "help" || first_arg == "-h" || first_arg == "--help") {
        print_register_usage();
        return 0;
      }
    }
    std::string error;
    if (!parse_register_args(argc, argv, register_options, error)) {
      std::cerr << "Error: " << error << std::endl;
      print_register_usage();
      return 1;
    }
    return register_device(register_options);
  } else {
    std::cerr << "Error: Unknown command '" << command << "'" << std::endl;
    print_usage();
    return 1;
  }
}

void Commands::print_tree(const std::string& dir, const std::string& prefix) {
  try {
    std::vector<fs::directory_entry> entries;

    // Get all entries in directory
    for (const auto& entry : fs::directory_iterator(dir)) {
      entries.push_back(entry);
    }

    // Sort entries: directories first, then alphabetically
    std::sort(
      entries.begin(),
      entries.end(),
      [](const fs::directory_entry& a, const fs::directory_entry& b) {
        if (a.is_directory() != b.is_directory()) {
          return a.is_directory();
        }
        return a.path().filename() < b.path().filename();
      }
    );

    // Print entries
    size_t i = 0;
    for (const auto& entry : entries) {
      // Skip hidden files, marker file, and cache file
      std::string filename = entry.path().filename().string();
      if (filename[0] == '.') {
        continue;
      }
      // Skip cache.mcap file
      if (filename == ConfigCache::CACHE_FILENAME) {
        continue;
      }

      bool is_dir = entry.is_directory();
      std::string connector = (i < entries.size() - 1) ? "├── " : "└── ";
      std::string prefix_add = (i < entries.size() - 1) ? "│   " : "    ";

      std::cout << prefix << connector << filename;

      if (is_dir) {
        // Count files in subdirectory
        size_t file_count = 0;
        for (const auto& sub_entry : fs::recursive_directory_iterator(entry.path())) {
          if (sub_entry.is_regular_file() && sub_entry.path().filename().string()[0] != '.') {
            file_count++;
          }
        }
        std::cout << "/ (" << file_count << " files)";
      } else if (entry.is_regular_file()) {
        // Get file size
        try {
          uint64_t size = fs::file_size(entry.path());
          std::cout << " (" << format_size(size) << ")";
        } catch (...) {
          // Ignore errors
        }
      }

      std::cout << std::endl;

      if (is_dir) {
        print_tree(entry.path().string(), prefix + prefix_add);
      }

      ++i;
    }
  } catch (const fs::filesystem_error&) {
    // Ignore errors
  }
}

std::string Commands::format_size(uint64_t size) {
  const char* units[] = {"B", "KB", "MB", "GB"};
  int unit_index = 0;

  while (size >= 1024 && unit_index < 3) {
    size /= 1024;
    unit_index++;
  }

  std::ostringstream oss;
  oss << size << " " << units[unit_index];

  return oss.str();
}

std::string Commands::format_time(uint64_t timestamp) {
  if (timestamp == 0) {
    return "Never";
  }

  std::time_t time = static_cast<std::time_t>(timestamp);
  std::tm tm = *std::localtime(&time);

  char buffer[64];
  std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &tm);

  return buffer;
}

bool Commands::parse_register_args(
  int argc, char* argv[], RegisterOptions& options, std::string& error
) {
  for (int i = 2; i < argc; ++i) {
    std::string arg = argv[i];

    auto require_value = [&](const std::string& name) -> const char* {
      if (i + 1 >= argc) {
        error = name + " requires a value";
        return nullptr;
      }
      return argv[++i];
    };

    if (arg == "--factory") {
      const char* value = require_value(arg);
      if (value == nullptr) return false;
      options.factory = value;
    } else if (arg == "--robot-type" || arg == "--robot_type" || arg == "--robot-model" ||
               arg == "--robot_model") {
      const char* value = require_value(arg);
      if (value == nullptr) return false;
      options.robot_type = value;
    } else if (arg == "--keystone-url" || arg == "--url") {
      const char* value = require_value(arg);
      if (value == nullptr) return false;
      options.keystone_url = value;
    } else if (arg == "--config-dir") {
      const char* value = require_value(arg);
      if (value == nullptr) return false;
      options.config_dir = value;
    } else if (arg == "--skip-config-download") {
      options.fetch_configs = false;
    } else if (arg == "--timeout") {
      const char* value = require_value(arg);
      if (value == nullptr) return false;
      try {
        options.timeout_seconds = std::stol(value);
      } catch (const std::exception&) {
        error = "--timeout must be an integer number of seconds";
        return false;
      }
      if (options.timeout_seconds <= 0) {
        error = "--timeout must be greater than 0";
        return false;
      }
    } else if (arg == "--verbose" || arg == "-v") {
      verbose_ = true;
    } else {
      error = "unknown register option '" + arg + "'";
      return false;
    }
  }

  return true;
}

void Commands::print_usage() {
  std::cout << "Usage: axon_config <command> [options]" << std::endl;
  std::cout << std::endl;
  std::cout << "Commands:" << std::endl;
  std::cout << "  init      Create /axon/config directory structure" << std::endl;
  std::cout << "  scan      Scan config directory and generate cache" << std::endl;
  std::cout << "  enable    Enable config injection in recordings" << std::endl;
  std::cout << "  disable   Disable config injection" << std::endl;
  std::cout << "  clear     Remove config directory and cache" << std::endl;
  std::cout << "  status    Show current configuration status" << std::endl;
  std::cout << "  register  Register device with Keystone" << std::endl;
  std::cout << "  help      Show this help message" << std::endl;
  std::cout << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "  --incremental, -i    Incremental scan (only changed files)" << std::endl;
  std::cout << "  --force, -f         Skip confirmation prompt (for scripts)" << std::endl;
  std::cout << "  --verbose, -v       Verbose output" << std::endl;
  std::cout << std::endl;
  std::cout << "Register example:" << std::endl;
  std::cout << "  axon_config register --factory \"Factory Shanghai\" --robot-type SynGloves "
               "--keystone-url http://keystone:8080"
            << std::endl;
}

void Commands::print_register_usage() {
  std::cout << "Usage: axon_config register --factory <name> --robot-type <model> [options]"
            << std::endl;
  std::cout << std::endl;
  std::cout << "Registers this device with Keystone:" << std::endl;
  std::cout << "  POST /api/v1/devices/register" << std::endl;
  std::cout << std::endl;
  std::cout << "Required:" << std::endl;
  std::cout << "  --factory NAME        Factory name matching Keystone factories.name" << std::endl;
  std::cout << "  --robot-type MODEL    Robot model matching Keystone robot_types.model"
            << std::endl;
  std::cout << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "  --keystone-url URL    Keystone base URL; can also use AXON_KEYSTONE_URL"
            << std::endl;
  std::cout << "  --config-dir DIR      Directory for recorder.yaml and transfer.yaml "
               "(default: /etc/axon)"
            << std::endl;
  std::cout << "  --skip-config-download"
               "  Register only; do not fetch recorder/transfer configs"
            << std::endl;
  std::cout << "  --timeout SECONDS     HTTP timeout in seconds (default: 10)" << std::endl;
  std::cout << "  --verbose, -v         Verbose output" << std::endl;
  std::cout << std::endl;
  std::cout << "After registration, recorder.yaml and transfer.yaml are downloaded from:"
            << std::endl;
  std::cout << "  <keystone>/configs/<factory>/<robot-type>/" << std::endl;
  std::cout << "HTTP 404 from that config path is treated as a warning and leaves existing "
               "configs unchanged."
            << std::endl;
  std::cout << std::endl;
  std::cout << "Examples:" << std::endl;
  std::cout << "  axon_config register --factory \"Factory Shanghai\" --robot-type SynGloves "
               "--keystone-url http://keystone:8080"
            << std::endl;
  std::cout << "  AXON_KEYSTONE_URL=http://keystone:8080 axon register --factory "
               "\"Factory Shanghai\" --robot-type SynGloves"
            << std::endl;
}

#ifdef AXON_CONFIG_TESTING
std::string Commands::build_register_payload_for_test(
  const std::string& factory, const std::string& robot_type
) {
  return build_register_payload(factory, robot_type);
}

std::string Commands::build_register_url_for_test(const std::string& keystone_url) {
  return build_register_url(keystone_url);
}

std::string Commands::build_config_url_for_test(
  const std::string& keystone_url, const std::string& factory, const std::string& robot_type,
  const std::string& filename
) {
  return build_config_url(keystone_url, factory, robot_type, filename);
}
#endif

}  // namespace config
}  // namespace axon
