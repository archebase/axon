// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "process_monitor.hpp"

#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <system_error>
#include <unistd.h>
#include <utility>

namespace axon {
namespace system {

namespace fs = std::filesystem;
namespace beast = boost::beast;
namespace http = boost::beast::http;
using tcp = boost::asio::ip::tcp;

namespace {

struct ParsedHttpUrl {
  std::string host;
  std::string port = "80";
  std::string target = "/";
};

bool parse_http_url(const std::string& url, ParsedHttpUrl* parsed, std::string* error) {
  constexpr const char* kPrefix = "http://";
  if (parsed == nullptr) {
    if (error != nullptr) {
      *error = "parsed url output pointer is null";
    }
    return false;
  }
  if (url.rfind(kPrefix, 0) != 0) {
    if (error != nullptr) {
      *error = "only http:// health probe URLs are supported";
    }
    return false;
  }

  const auto rest = url.substr(std::string(kPrefix).size());
  const auto slash = rest.find('/');
  const auto authority = slash == std::string::npos ? rest : rest.substr(0, slash);
  parsed->target = slash == std::string::npos ? "/" : rest.substr(slash);
  if (authority.empty()) {
    if (error != nullptr) {
      *error = "health probe URL host is empty";
    }
    return false;
  }

  const auto colon = authority.rfind(':');
  if (colon == std::string::npos) {
    parsed->host = authority;
  } else {
    parsed->host = authority.substr(0, colon);
    parsed->port = authority.substr(colon + 1);
  }

  if (parsed->host.empty() || parsed->port.empty()) {
    if (error != nullptr) {
      *error = "health probe URL host or port is empty";
    }
    return false;
  }
  return true;
}

bool is_pid_dir(const fs::directory_entry& entry) {
  if (!entry.is_directory()) {
    return false;
  }
  const auto name = entry.path().filename().string();
  return !name.empty() && std::all_of(name.begin(), name.end(), [](unsigned char ch) {
    return std::isdigit(ch) != 0;
  });
}

bool read_file(const fs::path& path, std::string* content) {
  if (content == nullptr) {
    return false;
  }
  std::ifstream input(path);
  if (!input) {
    return false;
  }
  std::ostringstream stream;
  stream << input.rdbuf();
  *content = stream.str();
  return true;
}

}  // namespace

ProcessMonitor::ProcessMonitor(ProcessMonitorOptions options)
    : options_(std::move(options))
    , clock_ticks_per_second_(sysconf(_SC_CLK_TCK))
    , page_size_bytes_(sysconf(_SC_PAGESIZE)) {
  if (options_.process_sample_cadence_ms <= 0) {
    options_.process_sample_cadence_ms = 2000;
  }
  if (options_.targets.empty()) {
    options_.targets = default_process_targets();
  }
  if (clock_ticks_per_second_ <= 0) {
    clock_ticks_per_second_ = 100;
  }
  if (page_size_bytes_ <= 0) {
    page_size_bytes_ = 4096;
  }
}

nlohmann::json ProcessMonitor::collect() {
  nlohmann::json processes = nlohmann::json::object();
  for (const auto& target : options_.targets) {
    processes[target.id] = collect_target(target);
  }
  return processes;
}

nlohmann::json ProcessMonitor::cadence_json() const {
  return {
    {"processes", options_.process_sample_cadence_ms},
    {"unit", "milliseconds"},
  };
}

nlohmann::json ProcessMonitor::collect_target(const ProcessTargetConfig& target) {
  nlohmann::json result = {
    {"id", target.id},
    {"binary", target.executable},
    {"status", "unknown"},
    {"pid", nullptr},
    {"source", "none"},
  };

  auto candidates = discover_candidates(target);
  if (!candidates.empty()) {
    nlohmann::json candidate_json = nlohmann::json::array();
    for (const auto& candidate : candidates) {
      candidate_json.push_back({
        {"pid", candidate.pid},
        {"source", candidate.source},
        {"start_time_ticks", candidate.start_time_ticks},
        {"process_present", candidate.process_present},
        {"command_matches", candidate.command_matches},
      });
      if (!candidate.message.empty()) {
        candidate_json.back()["message"] = candidate.message;
      }
    }
    result["candidates"] = std::move(candidate_json);
  }

  if (candidates.empty()) {
    result["status"] = "unavailable";
    result["message"] = "process not found";
    return result;
  }

  auto chosen = candidates.front();
  const auto pid_file_candidate =
    std::find_if(candidates.begin(), candidates.end(), [](const ProcessCandidate& candidate) {
      return (candidate.source == "pid_file" || candidate.source == "pid_file_candidate") &&
             candidate.process_present;
    });
  if (pid_file_candidate != candidates.end()) {
    chosen = *pid_file_candidate;
  } else {
    auto best_process_match = candidates.end();
    for (auto it = candidates.begin(); it != candidates.end(); ++it) {
      if (!it->process_present || !it->command_matches) {
        continue;
      }
      if (best_process_match == candidates.end() ||
          best_process_match->start_time_ticks < it->start_time_ticks) {
        best_process_match = it;
      }
    }
    if (best_process_match != candidates.end()) {
      chosen = *best_process_match;
    }
  }

  result["source"] = chosen.source;
  if (!chosen.process_present) {
    result["status"] = "unavailable";
    result["stale_pid"] = chosen.pid;
    result["message"] = chosen.message.empty() ? "process not found" : chosen.message;
    return result;
  }

  result["pid"] = chosen.pid;
  if (!chosen.command_matches) {
    result["status"] = "unhealthy";
    result["message"] =
      chosen.message.empty() ? "process command does not match target" : chosen.message;
    return result;
  }

  std::string stat_error;
  auto stat = read_stat(chosen.pid, &stat_error);
  if (!stat.has_value()) {
    result["status"] = "unhealthy";
    result["message"] = stat_error.empty() ? "failed to read process stat" : stat_error;
    return result;
  }

  result["resources"] = process_resources(chosen.pid, stat.value());

  if (target.rpc.has_value() && !target.rpc->url.empty()) {
    auto health = probe_health(target);
    const bool reachable = health.value("reachable", false);
    result["health"] = std::move(health);
    result["status"] = reachable ? "healthy" : "unhealthy";
    if (!reachable) {
      result["message"] = result["health"].value("error", "health probe failed");
    }
    return result;
  }

  result["status"] = "healthy";
  return result;
}

std::vector<ProcessMonitor::ProcessCandidate> ProcessMonitor::discover_candidates(
  const ProcessTargetConfig& target
) const {
  std::vector<ProcessCandidate> candidates;

  if (!target.pid_file.empty()) {
    auto candidate = candidate_from_pid_file(target, target.pid_file, "pid_file");
    if (candidate.has_value()) {
      candidates.push_back(candidate.value());
    }
  }

  for (const auto& pattern : target.pid_file_candidates) {
    for (const auto& pid_file : expand_candidate_paths(pattern)) {
      auto candidate = candidate_from_pid_file(target, pid_file, "pid_file_candidate");
      if (candidate.has_value()) {
        candidates.push_back(candidate.value());
      }
    }
  }

  std::error_code ec;
  if (fs::exists(options_.proc_root, ec)) {
    for (const auto& entry : fs::directory_iterator(options_.proc_root, ec)) {
      if (ec) {
        break;
      }
      if (!is_pid_dir(entry)) {
        continue;
      }
      const auto pid = std::stoi(entry.path().filename().string());
      if (!command_matches_target(pid, target)) {
        continue;
      }
      std::string error;
      const auto stat = read_stat(pid, &error);
      candidates.push_back({
        pid,
        "process_match",
        {},
        stat.has_value() ? stat->start_time_ticks : 0,
        true,
        true,
        "",
      });
    }
  }

  return candidates;
}

std::optional<ProcessMonitor::ProcessCandidate> ProcessMonitor::candidate_from_pid_file(
  const ProcessTargetConfig& target, const std::filesystem::path& pid_file,
  const std::string& source
) const {
  std::uint64_t pid_value = 0;
  std::string error;
  if (!parse_unsigned_file(pid_file, &pid_value, &error)) {
    return std::nullopt;
  }
  const auto pid = static_cast<int>(pid_value);
  ProcessCandidate candidate;
  candidate.pid = pid;
  candidate.source = source;
  candidate.pid_file = pid_file;

  if (!process_exists(pid)) {
    candidate.process_present = false;
    candidate.command_matches = false;
    candidate.message = "pid file points to a missing process";
    return candidate;
  }

  std::string stat_error;
  const auto stat = read_stat(pid, &stat_error);
  candidate.start_time_ticks = stat.has_value() ? stat->start_time_ticks : 0;
  if (!command_matches_target(pid, target)) {
    candidate.command_matches = false;
    candidate.message = "pid file process command does not match target";
  }
  return candidate;
}

std::vector<std::filesystem::path> ProcessMonitor::expand_candidate_paths(
  const std::filesystem::path& pattern
) const {
  const auto pattern_string = pattern.string();
  if (pattern_string.find('*') == std::string::npos) {
    std::error_code ec;
    return fs::exists(pattern, ec) ? std::vector<fs::path>{pattern} : std::vector<fs::path>{};
  }

  const auto parent = pattern.parent_path();
  const auto filename_pattern = pattern.filename().string();
  std::vector<fs::path> matches;
  std::error_code ec;
  if (!fs::exists(parent, ec)) {
    return matches;
  }
  for (const auto& entry : fs::directory_iterator(parent, ec)) {
    if (ec) {
      break;
    }
    if (wildcard_match(filename_pattern, entry.path().filename().string())) {
      matches.push_back(entry.path());
    }
  }
  std::sort(matches.begin(), matches.end());
  return matches;
}

bool ProcessMonitor::process_exists(int pid) const {
  std::error_code ec;
  return pid > 0 && fs::exists(options_.proc_root / std::to_string(pid), ec);
}

bool ProcessMonitor::command_matches_target(int pid, const ProcessTargetConfig& target) const {
  const auto cmdline = read_cmdline(pid);
  if (cmdline.empty()) {
    return false;
  }
  const auto args = split_cmdline(cmdline);
  const auto executable_matches = std::any_of(args.begin(), args.end(), [&target](const auto& arg) {
    return basename(arg) == target.executable || arg == target.executable;
  });
  if (!executable_matches) {
    return false;
  }
  return std::all_of(
    target.cmdline_contains.begin(),
    target.cmdline_contains.end(),
    [&cmdline](const auto& needle) {
      return cmdline.find(needle) != std::string::npos;
    }
  );
}

std::string ProcessMonitor::read_cmdline(int pid) const {
  std::string content;
  read_file(options_.proc_root / std::to_string(pid) / "cmdline", &content);
  return content;
}

std::optional<ProcessMonitor::ProcessStat> ProcessMonitor::read_stat(int pid, std::string* error)
  const {
  std::string content;
  if (!read_file(options_.proc_root / std::to_string(pid) / "stat", &content)) {
    if (error != nullptr) {
      *error = "failed to read process stat";
    }
    return std::nullopt;
  }

  ProcessStat stat;
  if (!parse_stat_content(content, &stat, error)) {
    return std::nullopt;
  }
  return stat;
}

std::optional<std::uint64_t> ProcessMonitor::read_rss_bytes(int pid, std::string* error) const {
  std::string content;
  if (!read_file(options_.proc_root / std::to_string(pid) / "statm", &content)) {
    if (error != nullptr) {
      *error = "failed to read process statm";
    }
    return std::nullopt;
  }

  std::istringstream stream(content);
  std::uint64_t size_pages = 0;
  std::uint64_t rss_pages = 0;
  if (!(stream >> size_pages >> rss_pages)) {
    if (error != nullptr) {
      *error = "failed to parse process statm";
    }
    return std::nullopt;
  }
  return rss_pages * static_cast<std::uint64_t>(page_size_bytes_);
}

ProcessMonitor::ProcessIo ProcessMonitor::read_io(int pid) const {
  std::string content;
  ProcessIo io;
  if (!read_file(options_.proc_root / std::to_string(pid) / "io", &content)) {
    return io;
  }

  std::istringstream stream(content);
  std::string key;
  std::uint64_t value = 0;
  while (stream >> key >> value) {
    if (key == "read_bytes:") {
      io.read_bytes = value;
    } else if (key == "write_bytes:") {
      io.write_bytes = value;
    }
  }
  io.available = true;
  return io;
}

nlohmann::json ProcessMonitor::process_resources(int pid, const ProcessStat& stat) {
  const auto now = std::chrono::steady_clock::now();
  const auto total_ticks = stat.user_ticks + stat.system_ticks;
  double cpu_percent = 0.0;
  const auto previous = previous_usage_.find(pid);
  if (previous != previous_usage_.end()) {
    const auto elapsed_sec =
      std::chrono::duration<double>(now - previous->second.sampled_at).count();
    const auto tick_delta =
      total_ticks >= previous->second.total_ticks ? total_ticks - previous->second.total_ticks : 0;
    if (elapsed_sec > 0.0) {
      cpu_percent = (static_cast<double>(tick_delta) * 100.0) /
                    (static_cast<double>(clock_ticks_per_second_) * elapsed_sec);
    }
  }
  previous_usage_[pid] = {total_ticks, now};

  nlohmann::json resources = {
    {"cpu_percent", cpu_percent},
    {"cpu_unit", "percent"},
    {"state", std::string(1, stat.state)},
    {"start_time_ticks", stat.start_time_ticks},
  };

  std::string rss_error;
  const auto rss = read_rss_bytes(pid, &rss_error);
  if (rss.has_value()) {
    resources["rss_bytes"] = rss.value();
  } else {
    resources["rss_available"] = false;
    resources["rss_error"] = rss_error;
  }

  const auto io = read_io(pid);
  resources["io"] = {
    {"available", io.available},
    {"read_bytes", io.read_bytes},
    {"write_bytes", io.write_bytes},
    {"unit", "bytes"},
  };
  return resources;
}

nlohmann::json ProcessMonitor::probe_health(const ProcessTargetConfig& target) const {
  nlohmann::json health = {
    {"type", "http"},
    {"reachable", false},
    {"http_reachable", false},
  };
  if (!target.rpc.has_value() || target.rpc->url.empty()) {
    health["error"] = "health probe is not configured";
    return health;
  }
  const auto probe_type = target.rpc->type.empty() ? "http" : target.rpc->type;
  health["type"] = probe_type;
  if (probe_type != "http") {
    health["error"] = "unsupported health probe type";
    return health;
  }

  ParsedHttpUrl url;
  std::string error;
  if (!parse_http_url(target.rpc->url, &url, &error)) {
    health["url"] = target.rpc->url;
    health["error"] = error;
    return health;
  }

  try {
    boost::asio::io_context io_context;
    tcp::resolver resolver(io_context);
    beast::tcp_stream stream(io_context);
    stream.expires_after(std::chrono::milliseconds(target.rpc->timeout_ms));
    const auto results = resolver.resolve(url.host, url.port);
    stream.connect(results);

    http::request<http::empty_body> request{http::verb::get, url.target, 11};
    request.set(http::field::host, url.host);
    request.set(http::field::user_agent, "axon-system");
    http::write(stream, request);

    beast::flat_buffer buffer;
    http::response<http::string_body> response;
    http::read(stream, buffer, response);

    boost::system::error_code ec;
    stream.socket().shutdown(tcp::socket::shutdown_both, ec);

    const auto status_code = static_cast<unsigned int>(response.result_int());
    health["status_code"] = status_code;
    const bool http_reachable = status_code >= 200 && status_code < 300;
    health["http_reachable"] = http_reachable;
    health["reachable"] = http_reachable;
    if (!http_reachable) {
      health["error"] = "health probe returned HTTP " + std::to_string(status_code);
    }
    try {
      const auto body = nlohmann::json::parse(response.body());
      if (body.is_object()) {
        if (body.contains("success") && body["success"].is_boolean()) {
          const bool rpc_success = body["success"].get<bool>();
          health["rpc_success"] = rpc_success;
          if (http_reachable && !rpc_success) {
            health["reachable"] = false;
            health["error"] = body.value("message", "health probe RPC success false");
          }
        }
        if (body.contains("data") && body["data"].is_object() && body["data"].contains("state")) {
          health["state"] = body["data"]["state"];
        }
      }
    } catch (const nlohmann::json::exception&) {
      // The reachability check does not require a JSON body.
    }
  } catch (const std::exception& ex) {
    health["error"] = ex.what();
  }
  return health;
}

std::vector<std::string> ProcessMonitor::split_cmdline(const std::string& cmdline) {
  std::vector<std::string> args;
  std::string current;
  for (const char ch : cmdline) {
    if (ch == '\0') {
      if (!current.empty()) {
        args.push_back(current);
        current.clear();
      }
      continue;
    }
    current.push_back(ch);
  }
  if (!current.empty()) {
    args.push_back(current);
  }
  return args;
}

bool ProcessMonitor::parse_stat_content(
  const std::string& content, ProcessStat* stat, std::string* error
) {
  if (stat == nullptr) {
    if (error != nullptr) {
      *error = "process stat output pointer is null";
    }
    return false;
  }

  const auto rparen = content.rfind(')');
  if (rparen == std::string::npos || rparen + 2 >= content.size()) {
    if (error != nullptr) {
      *error = "malformed process stat";
    }
    return false;
  }

  std::istringstream stream(content.substr(rparen + 2));
  std::vector<std::string> fields;
  std::string field;
  while (stream >> field) {
    fields.push_back(field);
  }
  if (fields.size() < 20) {
    if (error != nullptr) {
      *error = "process stat has too few fields";
    }
    return false;
  }

  stat->state = fields[0].empty() ? '?' : fields[0][0];
  stat->user_ticks = std::stoull(fields[11]);
  stat->system_ticks = std::stoull(fields[12]);
  stat->start_time_ticks = std::stoull(fields[19]);
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

bool ProcessMonitor::parse_unsigned_file(
  const std::filesystem::path& path, std::uint64_t* value, std::string* error
) {
  if (value == nullptr) {
    if (error != nullptr) {
      *error = "unsigned output pointer is null";
    }
    return false;
  }

  std::ifstream input(path);
  if (!input) {
    if (error != nullptr) {
      *error = "failed to read " + path.string();
    }
    return false;
  }
  input >> *value;
  if (!input) {
    if (error != nullptr) {
      *error = "failed to parse " + path.string();
    }
    return false;
  }
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

bool ProcessMonitor::wildcard_match(const std::string& pattern, const std::string& value) {
  std::size_t pattern_index = 0;
  std::size_t value_index = 0;
  std::size_t star_index = std::string::npos;
  std::size_t match_index = 0;

  while (value_index < value.size()) {
    if (pattern_index < pattern.size() &&
        (pattern[pattern_index] == '?' || pattern[pattern_index] == value[value_index])) {
      ++pattern_index;
      ++value_index;
    } else if (pattern_index < pattern.size() && pattern[pattern_index] == '*') {
      star_index = pattern_index++;
      match_index = value_index;
    } else if (star_index != std::string::npos) {
      pattern_index = star_index + 1;
      value_index = ++match_index;
    } else {
      return false;
    }
  }

  while (pattern_index < pattern.size() && pattern[pattern_index] == '*') {
    ++pattern_index;
  }
  return pattern_index == pattern.size();
}

std::string ProcessMonitor::basename(const std::string& path) {
  const auto slash = path.find_last_of('/');
  return slash == std::string::npos ? path : path.substr(slash + 1);
}

std::vector<ProcessTargetConfig> default_process_targets() {
  return {
    {
      "recorder",
      "axon-recorder",
      {},
      {"/var/lib/axon/agent/*_recorder.pid"},
      {},
      ProcessHttpProbeConfig{"http", "http://127.0.0.1:8080/rpc/state", 350},
    },
    {
      "transfer",
      "axon-transfer",
      {},
      {"/var/lib/axon/agent/*_transfer.pid"},
      {},
      std::nullopt,
    },
  };
}

}  // namespace system
}  // namespace axon
