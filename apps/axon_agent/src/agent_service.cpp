// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "agent_service.hpp"

#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>
#include <yaml-cpp/yaml.h>

#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <exception>
#include <fstream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#ifndef AXON_AGENT_VERSION
#define AXON_AGENT_VERSION "0.4.0-dev"
#endif

namespace axon {
namespace agent {

namespace {

namespace asio = boost::asio;
namespace beast = boost::beast;
namespace http = beast::http;
using tcp = asio::ip::tcp;

struct RecorderRpcEndpoint {
  bool configured = false;
  bool queryable = false;
  std::string mode = "http_server";
  std::string bind_host = "0.0.0.0";
  std::string connect_host = "127.0.0.1";
  std::string auth_token;
  std::filesystem::path config_path;
  std::uint16_t port = 8080;
  std::string url;
  std::string error;
};

std::string path_string(const std::filesystem::path& path) {
  return path.lexically_normal().string();
}

std::string now_iso8601() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t time = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  gmtime_r(&time, &tm);

  char buffer[32];
  std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &tm);
  return buffer;
}

std::string connect_host_for_bind_host(const std::string& host) {
  if (host.empty() || host == "0.0.0.0" || host == "*" || host == "::") {
    return "127.0.0.1";
  }
  return host;
}

std::string endpoint_url(const std::string& host, std::uint16_t port) {
  std::ostringstream stream;
  stream << "http://" << host << ":" << port << "/rpc/state";
  return stream.str();
}

std::string join_strings(const std::vector<std::string>& items, const std::string& delimiter) {
  std::ostringstream stream;
  for (std::size_t i = 0; i < items.size(); ++i) {
    if (i > 0) {
      stream << delimiter;
    }
    stream << items[i];
  }
  return stream.str();
}

RecorderRpcEndpoint load_recorder_rpc_endpoint(const std::filesystem::path& recorder_yaml) {
  RecorderRpcEndpoint endpoint;
  endpoint.config_path = recorder_yaml;

  if (!std::filesystem::exists(recorder_yaml)) {
    endpoint.error = "recorder config not found: " + path_string(recorder_yaml);
    return endpoint;
  }

  endpoint.configured = true;
  try {
    const auto yaml = YAML::LoadFile(path_string(recorder_yaml));
    if (yaml["rpc"] && yaml["rpc"]["mode"]) {
      endpoint.mode = yaml["rpc"]["mode"].as<std::string>();
    }

    if (yaml["http_server"]) {
      const auto http_server = yaml["http_server"];
      if (http_server["host"]) {
        endpoint.bind_host = http_server["host"].as<std::string>();
      }
      if (http_server["port"]) {
        endpoint.port = http_server["port"].as<std::uint16_t>();
      }
      if (http_server["auth_token"]) {
        endpoint.auth_token = http_server["auth_token"].as<std::string>();
      }
    }

    endpoint.connect_host = connect_host_for_bind_host(endpoint.bind_host);
    endpoint.url = endpoint_url(endpoint.connect_host, endpoint.port);

    if (endpoint.mode != "http_server") {
      endpoint.error = "recorder rpc mode is not http_server: " + endpoint.mode;
      return endpoint;
    }

    endpoint.queryable = true;
    return endpoint;
  } catch (const std::exception& ex) {
    endpoint.error = std::string("failed to parse recorder config: ") + ex.what();
    return endpoint;
  }
}

nlohmann::json fetch_recorder_rpc_state(
  const RecorderRpcEndpoint& endpoint, std::chrono::milliseconds timeout
) {
  nlohmann::json result = {
    {"configured", endpoint.configured},
    {"queryable", endpoint.queryable},
    {"mode", endpoint.mode},
    {"bind_host", endpoint.bind_host},
    {"connect_host", endpoint.connect_host},
    {"port", endpoint.port},
    {"url", endpoint.url},
    {"config_path", path_string(endpoint.config_path)},
    {"reachable", false},
  };

  if (!endpoint.error.empty()) {
    result["last_error"] = endpoint.error;
  }
  if (!endpoint.queryable) {
    return result;
  }

  try {
    asio::io_context io_context;
    tcp::resolver resolver(io_context);
    beast::tcp_stream stream(io_context);

    stream.expires_after(timeout);
    const auto resolved = resolver.resolve(endpoint.connect_host, std::to_string(endpoint.port));
    stream.connect(resolved);

    http::request<http::empty_body> request{http::verb::get, "/rpc/state", 11};
    request.set(http::field::host, endpoint.connect_host + ":" + std::to_string(endpoint.port));
    request.set(http::field::user_agent, "axon-agent/" AXON_AGENT_VERSION);
    if (!endpoint.auth_token.empty()) {
      request.set(http::field::authorization, "Bearer " + endpoint.auth_token);
    }

    http::write(stream, request);

    beast::flat_buffer buffer;
    http::response<http::string_body> response;
    http::read(stream, buffer, response);

    beast::error_code shutdown_ec;
    stream.socket().shutdown(tcp::socket::shutdown_both, shutdown_ec);

    result["http_status"] = response.result_int();
    result["reachable"] = response.result_int() >= 200 && response.result_int() < 400;

    auto parsed = nlohmann::json::parse(response.body(), nullptr, false);
    if (parsed.is_discarded()) {
      result["last_error"] = "recorder /rpc/state returned invalid JSON";
      result["body"] = response.body();
      return result;
    }

    result["response"] = parsed;
    if (parsed.contains("success")) {
      result["success"] = parsed["success"];
    }
    if (parsed.contains("message")) {
      result["message"] = parsed["message"];
    }
    if (parsed.contains("data") && parsed["data"].is_object()) {
      result["data"] = parsed["data"];
      const auto& data = parsed["data"];
      if (data.contains("state")) {
        result["state"] = data["state"];
      }
      if (data.contains("version")) {
        result["version"] = data["version"];
      }
      if (data.contains("running")) {
        result["running"] = data["running"];
      }
      if (data.contains("duration_sec")) {
        result["duration_sec"] = data["duration_sec"];
      }
      if (data.contains("message_count")) {
        result["message_count"] = data["message_count"];
      }
      if (data.contains("bytes_written")) {
        result["bytes_written"] = data["bytes_written"];
      }
    }

    return result;
  } catch (const std::exception& ex) {
    result["last_error"] = ex.what();
    return result;
  }
}

nlohmann::json build_components_state(const std::optional<RobotProfile>& active_profile) {
  nlohmann::json components = nlohmann::json::object();

  nlohmann::json recorder = {
    {"kind", "managed_process"},
    {"rpc_state", {{"configured", false}, {"queryable", false}, {"reachable", false}}},
  };
  if (active_profile.has_value()) {
    const auto endpoint = load_recorder_rpc_endpoint(active_profile->recorder_yaml);
    recorder["rpc_state"] = fetch_recorder_rpc_state(endpoint, std::chrono::milliseconds(350));
  } else {
    recorder["rpc_state"]["last_error"] = "no active profile selected";
  }
  components["recorder"] = recorder;

  components["transfer"] = {{"kind", "managed_process"}};
  components["robot_startup"] = {{"kind", "managed_process"}};

  return components;
}

}  // namespace

AgentService::AgentService(std::filesystem::path profile_root, std::filesystem::path state_dir)
  : profile_root_(profile_root)
  , profiles_(std::move(profile_root))
  , processes_(state_dir)
  , state_dir_(std::move(state_dir))
  , started_at_iso_(now_iso8601()) {}

bool AgentService::initialize(std::string* error) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!profiles_.scan(error)) {
    if (error != nullptr) {
      last_error_ = *error;
    }
    return false;
  }

  std::string restore_error;
  if (!restore_active_profile(&restore_error)) {
    last_error_ = restore_error;
    if (error != nullptr) {
      *error = restore_error;
    }
  }
  return true;
}

RpcResponse AgentService::get_state() {
  RpcResponse response;
  std::optional<RobotProfile> active_profile;
  nlohmann::json active_profile_json;
  nlohmann::json adapter_state;
  nlohmann::json processes_state;
  std::string last_error;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto* profile = profiles_.active_profile();
    active_profile_json = profiles_.active_profile_to_json();
    adapter_state = adapter_loader_.status_to_json();
    if (profile != nullptr) {
      active_profile = *profile;
      if (adapter_loader_.is_loaded()) {
        adapter_state["runtime"] = adapter_loader_.runtime_status_to_json(build_adapter_context(*profile));
      }
    }
    processes_state = processes_.state_to_json();
    last_error = last_error_;
  }

  const auto uptime = std::chrono::duration_cast<std::chrono::seconds>(
    std::chrono::steady_clock::now() - started_at_
  );
  const nlohmann::json agent_state = {
    {"name", "axon-agent"},
    {"version", AXON_AGENT_VERSION},
    {"state", "running"},
    {"pid", static_cast<int>(getpid())},
    {"started_at", started_at_iso_},
    {"uptime_sec", uptime.count()},
    {"profile_root", path_string(profile_root_)},
    {"state_dir", path_string(state_dir_)},
    {"active_profile_state_file", path_string(active_profile_state_file())},
  };

  response.message = "ok";
  response.data = {
    {"agent", agent_state},
    {"active_profile", active_profile_json},
    {"adapter", adapter_state},
    {"processes", processes_state},
    {"components", build_components_state(active_profile)},
    {"last_error", last_error},
  };
  return response;
}

RpcResponse AgentService::get_report() {
  std::lock_guard<std::mutex> lock(mutex_);
  RpcResponse response;
  const auto* profile = profiles_.active_profile();
  auto adapter_state = adapter_loader_.status_to_json();
  if (profile != nullptr && adapter_loader_.is_loaded()) {
    adapter_state["runtime"] = adapter_loader_.runtime_status_to_json(build_adapter_context(*profile));
  }
  response.message = "ok";
  response.data = {
    {"profiles", profiles_.profiles_to_json()},
    {"active_profile", profiles_.active_profile_to_json()},
    {"adapter", adapter_state},
    {"processes", processes_.state_to_json()},
  };
  return response;
}

RpcResponse AgentService::list_profiles() {
  std::lock_guard<std::mutex> lock(mutex_);
  RpcResponse response;
  response.message = "ok";
  response.data = {{"profiles", profiles_.profiles_to_json()}, {"active_profile", profiles_.active_profile_to_json()}};
  return response;
}

RpcResponse AgentService::select_profile(const nlohmann::json& params) {
  std::lock_guard<std::mutex> lock(mutex_);
  RpcResponse response;
  try {
    const auto profile_id = require_string(params, "profile_id");
    const auto* target_profile = profiles_.find_profile(profile_id);
    if (target_profile == nullptr) {
      const auto error = "profile not found: " + profile_id;
      last_error_ = error;
      return {false, error, nullptr};
    }

    const auto* current_profile = profiles_.active_profile();
    const bool switching_profile =
      current_profile != nullptr && current_profile->profile_id != target_profile->profile_id;
    if (switching_profile) {
      const auto running_processes = processes_.running_processes();
      if (!running_processes.empty()) {
        const auto error =
          "cannot switch active profile while processes are running: " + join_strings(running_processes, ", ");
        last_error_ = error;
        return {
          false,
          error,
          {
            {"active_profile", profiles_.active_profile_to_json()},
            {"requested_profile_id", target_profile->profile_id},
            {"running_processes", running_processes},
            {"processes", processes_.state_to_json()},
          },
        };
      }
    }

    std::string error;
    if (!persist_active_profile(*target_profile, &error)) {
      last_error_ = error;
      return {false, error, nullptr};
    }

    if (switching_profile) {
      adapter_loader_.unload();
    }

    if (!profiles_.select_profile(target_profile->profile_id, &error)) {
      last_error_ = error;
      return {false, error, nullptr};
    }

    const auto* profile = profiles_.active_profile();
    if (profile == nullptr) {
      return {false, "active profile is missing after selection", nullptr};
    }

    discover_active_processes(*profile);

    nlohmann::json auto_start = {
      {"enabled", profile->auto_start},
      {"trigger", "profile_select"},
      {"attempted", false},
      {"started", false},
      {"skipped", false},
    };
    if (profile->auto_start) {
      const auto auto_start_response = auto_start_robot_process(*profile, "profile_select");
      auto_start = auto_start_response.data;
      if (!auto_start_response.success) {
        return {
          false,
          "profile selected; " + auto_start_response.message,
          {
            {"active_profile", profiles_.active_profile_to_json()},
            {"adapter", adapter_loader_.status_to_json()},
            {"processes", processes_.state_to_json()},
            {"auto_start", auto_start},
          },
        };
      }
    }

    response.message = "profile selected";
    response.data = {
      {"active_profile", profiles_.active_profile_to_json()},
      {"adapter", adapter_loader_.status_to_json()},
      {"processes", processes_.state_to_json()},
      {"auto_start", auto_start},
    };
    return response;
  } catch (const std::exception& ex) {
    last_error_ = ex.what();
    return {false, ex.what(), nullptr};
  }
}

RpcResponse AgentService::start_process(const nlohmann::json& params) {
  std::lock_guard<std::mutex> lock(mutex_);
  try {
    const auto process_id = require_string(params, "process_id");
    const auto* profile = profiles_.active_profile();
    if (profile == nullptr) {
      return {false, "no active profile selected", nullptr};
    }

    std::string error;
    if (process_id == "robot_startup" && !adapter_loader_.is_loaded_for_profile(profile->profile_id)) {
      if (!adapter_loader_.load(*profile, &error)) {
        last_error_ = error;
        return {false, error, nullptr};
      }
    }

    if (process_id == "robot_startup") {
      return start_robot_process(*profile);
    }

    const auto config = build_process_config(*profile, process_id);
    if (!processes_.start(config, &error)) {
      last_error_ = error;
      return {false, error, nullptr};
    }

    return {true, "process started", {{"process_id", process_id}, {"processes", processes_.state_to_json()}}};
  } catch (const std::exception& ex) {
    last_error_ = ex.what();
    return {false, ex.what(), nullptr};
  }
}

RpcResponse AgentService::stop_process(const nlohmann::json& params, bool force) {
  std::lock_guard<std::mutex> lock(mutex_);
  try {
    const auto process_id = require_string(params, "process_id");
    const auto* profile = profiles_.active_profile();
    if (process_id == "robot_startup") {
      if (profile == nullptr) {
        return {false, "no active profile selected", nullptr};
      }
      return stop_robot_process(*profile, force);
    }

    std::string error;
    if (!processes_.stop(process_id, force, &error)) {
      last_error_ = error;
      return {false, error, nullptr};
    }
    return {true, force ? "process force stopped" : "process stopped", processes_.state_to_json()};
  } catch (const std::exception& ex) {
    last_error_ = ex.what();
    return {false, ex.what(), nullptr};
  }
}

RpcResponse AgentService::auto_start_robot_process(const RobotProfile& profile, const std::string& trigger) {
  nlohmann::json auto_start = {
    {"enabled", profile.auto_start},
    {"trigger", trigger},
    {"attempted", profile.auto_start},
    {"started", false},
    {"skipped", false},
  };
  if (!profile.auto_start) {
    auto_start["reason"] = "disabled";
    return {true, "auto_start disabled", auto_start};
  }

  std::string error;
  if (!adapter_loader_.is_loaded_for_profile(profile.profile_id) && !adapter_loader_.load(profile, &error)) {
    last_error_ = error;
    auto_start["error"] = error;
    auto_start["adapter"] = adapter_loader_.status_to_json();
    auto_start["processes"] = processes_.state_to_json();
    return {false, "auto_start failed: " + error, auto_start};
  }

  if (processes_.is_process_running("robot_startup")) {
    auto_start["skipped"] = true;
    auto_start["reason"] = "robot_startup already running";
    auto_start["adapter"] = adapter_loader_.status_to_json();
    auto_start["processes"] = processes_.state_to_json();
    return {true, "auto_start skipped: robot_startup already running", auto_start};
  }

  const auto start_response = start_robot_process(profile);
  auto_start["adapter"] = adapter_loader_.status_to_json();
  auto_start["processes"] = processes_.state_to_json();
  if (!start_response.success) {
    auto_start["error"] = start_response.message;
    return {false, "auto_start failed: " + start_response.message, auto_start};
  }

  auto_start["started"] = true;
  return {true, "auto_start started robot_startup", auto_start};
}

RpcResponse AgentService::start_robot_process(const RobotProfile& profile) {
  std::string error;
  const auto context = build_adapter_context(profile);
  if (!adapter_loader_.start(context, &error)) {
    last_error_ = error;
    return {false, "robot adapter start failed: " + error, nullptr};
  }

  const auto config = build_process_config(profile, "robot_startup");
  if (!processes_.start(config, &error)) {
    std::string cleanup_error;
    (void)adapter_loader_.force_stop(context, &cleanup_error);
    last_error_ = error;
    return {false, error, nullptr};
  }

  return {
    true,
    "robot process started",
    {
      {"process_id", "robot_startup"},
      {"adapter", adapter_loader_.status_to_json()},
      {"processes", processes_.state_to_json()},
    },
  };
}

RpcResponse AgentService::stop_robot_process(const RobotProfile& profile, bool force) {
  std::string error;
  if (!adapter_loader_.is_loaded_for_profile(profile.profile_id) && !adapter_loader_.load(profile, &error)) {
    last_error_ = error;
    return {false, error, nullptr};
  }

  const auto context = build_adapter_context(profile);
  if (!force && !adapter_loader_.stop(context, 5000, &error)) {
    last_error_ = error;
    return {false, "robot adapter stop failed: " + error, nullptr};
  }

  if (!processes_.stop("robot_startup", force, &error)) {
    last_error_ = error;
    return {false, error, nullptr};
  }

  if (force) {
    std::string adapter_error;
    (void)adapter_loader_.force_stop(context, &adapter_error);
  }

  return {
    true,
    force ? "robot process force stopped" : "robot process stopped",
    {
      {"process_id", "robot_startup"},
      {"adapter", adapter_loader_.status_to_json()},
      {"processes", processes_.state_to_json()},
    },
  };
}

ManagedProcessConfig AgentService::build_process_config(const RobotProfile& profile, const std::string& process_id) {
  if (process_id == "recorder") {
    return load_yaml_process_config(profile, process_id, profile.recorder_yaml);
  }
  if (process_id == "transfer") {
    return load_yaml_process_config(profile, process_id, profile.transfer_yaml);
  }
  if (process_id == "robot_startup") {
    return load_adapter_process_config(profile, process_id);
  }
  throw std::runtime_error("unknown process_id: " + process_id);
}

ManagedProcessConfig AgentService::load_yaml_process_config(
  const RobotProfile& profile, const std::string& process_id, const std::filesystem::path& yaml_path
) {
  ManagedProcessConfig config;
  config.process_id = process_id;
  config.pid_file = state_dir_ / (profile.profile_id + "_" + process_id + ".pid");
  config.metadata_file = state_dir_ / (profile.profile_id + "_" + process_id + ".json");
  config.working_directory = profile.root_dir;

  if (!std::filesystem::exists(yaml_path)) {
    throw std::runtime_error("missing yaml for process " + process_id + ": " + yaml_path.string());
  }

  const auto yaml = YAML::LoadFile(yaml_path.string());
  const auto process = yaml["process"] ? yaml["process"] : yaml;
  config.executable =
    process["executable"] ? process["executable"].as<std::string>() : (process_id == "recorder" ? "axon-recorder" : "axon-transfer");

  if (process["args"]) {
    for (const auto& arg : process["args"]) {
      config.args.push_back(arg.as<std::string>());
    }
  } else {
    config.args.push_back("--config");
    config.args.push_back(yaml_path.string());
  }

  if (process["working_directory"]) {
    config.working_directory = profile.root_dir / process["working_directory"].as<std::string>();
  }
  if (process["env"]) {
    for (const auto& item : process["env"]) {
      config.env[item.first.as<std::string>()] = item.second.as<std::string>();
    }
  }
  return config;
}

ManagedProcessConfig AgentService::load_adapter_process_config(
  const RobotProfile& profile, const std::string& process_id
) {
  ManagedProcessConfig config;
  config.process_id = process_id;
  config.pid_file = state_dir_ / (profile.profile_id + "_" + process_id + ".pid");
  config.metadata_file = state_dir_ / (profile.profile_id + "_" + process_id + ".json");
  config.working_directory = profile.root_dir;

  const auto yaml = YAML::LoadFile(profile.adapter_yaml.string());
  const auto process = yaml["managed_processes"] && yaml["managed_processes"][process_id]
    ? yaml["managed_processes"][process_id]
    : YAML::Node();
  if (!process) {
    throw std::runtime_error("adapter.yaml does not declare managed_processes." + process_id);
  }

  config.executable = process["executable"].as<std::string>();
  if (process["args"]) {
    for (const auto& arg : process["args"]) {
      config.args.push_back(arg.as<std::string>());
    }
  }
  if (process["working_directory"]) {
    config.working_directory = profile.root_dir / process["working_directory"].as<std::string>();
  }
  if (process["env"]) {
    for (const auto& item : process["env"]) {
      config.env[item.first.as<std::string>()] = item.second.as<std::string>();
    }
  }
  return config;
}

RobotAdapterContext AgentService::build_adapter_context(const RobotProfile& profile) const {
  return {
    profile.profile_id,
    profile.adapter_id,
    profile.robot_model,
    profile.root_dir.string(),
    profile.adapter_yaml.string(),
    state_dir_.string(),
  };
}

void AgentService::discover_active_processes(const RobotProfile& profile) {
  for (const auto& process_id : {"robot_startup", "recorder", "transfer"}) {
    try {
      processes_.discover(build_process_config(profile, process_id));
    } catch (const std::exception&) {
      if (!processes_.is_process_running(process_id)) {
        processes_.forget(process_id);
      }
      // Optional process configs should not block profile selection.
    }
  }
}

bool AgentService::restore_active_profile(std::string* error) {
  const auto state_file = active_profile_state_file();
  if (!std::filesystem::exists(state_file)) {
    return true;
  }

  try {
    std::ifstream input(state_file);
    if (!input) {
      if (error != nullptr) {
        *error = "failed to read active profile state: " + path_string(state_file);
      }
      return false;
    }

    const auto state = nlohmann::json::parse(input);
    const auto profile_id = state.value("profile_id", std::string());
    if (profile_id.empty()) {
      if (error != nullptr) {
        *error = "active profile state is missing profile_id: " + path_string(state_file);
      }
      return false;
    }

    std::string select_error;
    if (!profiles_.select_profile(profile_id, &select_error)) {
      if (error != nullptr) {
        *error = "failed to restore active profile: " + select_error;
      }
      return false;
    }

    const auto* profile = profiles_.active_profile();
    if (profile == nullptr) {
      if (error != nullptr) {
        *error = "active profile missing after restore: " + profile_id;
      }
      return false;
    }

    discover_active_processes(*profile);
    if (profile->auto_start) {
      const auto auto_start_response = auto_start_robot_process(*profile, "restore");
      if (!auto_start_response.success) {
        if (error != nullptr) {
          *error = "failed to restore active profile auto_start: " + auto_start_response.message;
        }
        return false;
      }
    }
    return true;
  } catch (const std::exception& ex) {
    if (error != nullptr) {
      *error = std::string("failed to restore active profile: ") + ex.what();
    }
    return false;
  }
}

bool AgentService::persist_active_profile(const RobotProfile& profile, std::string* error) const {
  const auto state_file = active_profile_state_file();
  try {
    std::filesystem::create_directories(state_file.parent_path());
    std::ofstream output(state_file);
    if (!output) {
      if (error != nullptr) {
        *error = "failed to write active profile state: " + path_string(state_file);
      }
      return false;
    }

    const nlohmann::json state = {
      {"profile_id", profile.profile_id},
      {"adapter_id", profile.adapter_id},
      {"robot_model", profile.robot_model},
      {"profile_root", path_string(profile.root_dir)},
      {"selected_at", now_iso8601()},
    };
    output << state.dump(2) << '\n';
    return true;
  } catch (const std::exception& ex) {
    if (error != nullptr) {
      *error = std::string("failed to persist active profile: ") + ex.what();
    }
    return false;
  }
}

std::filesystem::path AgentService::active_profile_state_file() const {
  return state_dir_ / "active_profile.json";
}

std::string AgentService::require_string(const nlohmann::json& params, const std::string& key) {
  if (!params.contains(key) || !params[key].is_string()) {
    throw std::runtime_error("missing string parameter: " + key);
  }
  return params[key].get<std::string>();
}

}  // namespace agent
}  // namespace axon
