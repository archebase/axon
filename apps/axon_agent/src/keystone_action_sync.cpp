// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "keystone_action_sync.hpp"

#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <ctime>
#include <exception>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace axon {
namespace agent {

namespace {

namespace asio = boost::asio;
namespace beast = boost::beast;
namespace http = beast::http;
using tcp = asio::ip::tcp;

constexpr const char* kCatalogPathPrefix = "/api/v1/robots/";
constexpr const char* kCatalogPathSuffix = "/action-catalog";
constexpr const char* kPendingPathSuffix = "/action-requests/pending?limit=";
constexpr const char* kRequestPathPrefix = "/api/v1/action-requests/";
constexpr const char* kStatusPathSuffix = "/status";

std::string trim_trailing_slashes(std::string value) {
  while (!value.empty() && value.back() == '/') {
    value.pop_back();
  }
  return value;
}

std::string ensure_leading_slash(std::string value) {
  if (value.empty() || value.front() == '/') {
    return value;
  }
  return "/" + value;
}

std::optional<std::string> json_string(const nlohmann::json& value, const char* key) {
  if (!value.is_object() || !value.contains(key) || !value[key].is_string()) {
    return std::nullopt;
  }
  return value[key].get<std::string>();
}

nlohmann::json json_object_or_empty(const nlohmann::json& value, const char* key) {
  if (!value.is_object() || !value.contains(key) || !value[key].is_object()) {
    return nlohmann::json::object();
  }
  return value[key];
}

nlohmann::json extract_pending_requests(const nlohmann::json& body) {
  if (body.is_array()) {
    return body;
  }
  if (body.is_object()) {
    if (body.contains("requests") && body["requests"].is_array()) {
      return body["requests"];
    }
    if (body.contains("data") && body["data"].is_object()) {
      const auto& data = body["data"];
      if (data.contains("requests") && data["requests"].is_array()) {
        return data["requests"];
      }
    }
  }
  return nlohmann::json::array();
}

nlohmann::json extract_request_detail(const nlohmann::json& body) {
  if (!body.is_object()) {
    return nlohmann::json::object();
  }
  if (body.contains("request") && body["request"].is_object()) {
    return body["request"];
  }
  if (body.contains("data") && body["data"].is_object()) {
    const auto& data = body["data"];
    if (data.contains("request") && data["request"].is_object()) {
      return data["request"];
    }
    if (data.contains("request_id") || data.contains("action_id")) {
      return data;
    }
  }
  return body;
}

std::string response_error(const std::string& operation, const KeystoneHttpResponse& response) {
  if (!response.error.empty()) {
    return operation + " failed: " + response.error;
  }
  std::ostringstream stream;
  stream << operation << " failed with HTTP " << response.status_code;
  if (response.body.is_string()) {
    stream << ": " << response.body.get<std::string>();
  } else if (!response.body.is_null()) {
    stream << ": " << response.body.dump();
  }
  return stream.str();
}

http::verb verb_from_method(const std::string& method) {
  if (method == "GET") {
    return http::verb::get;
  }
  if (method == "PUT") {
    return http::verb::put;
  }
  if (method == "POST") {
    return http::verb::post;
  }
  throw std::runtime_error("unsupported HTTP method: " + method);
}

}  // namespace

std::string keystone_action_sync_now_iso8601() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t time = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  gmtime_r(&time, &tm);

  char buffer[32];
  std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &tm);
  return buffer;
}

std::string keystone_action_status_from_local(const std::string& local_status) {
  if (local_status == "completed" || local_status == "succeeded") {
    return "succeeded";
  }
  if (local_status == "queued" || local_status == "running" || local_status == "expired" ||
      local_status == "cancelled" || local_status == "timed_out") {
    return local_status;
  }
  return "failed";
}

std::string keystone_url_encode_component(const std::string& value) {
  std::ostringstream output;
  output << std::uppercase << std::hex << std::setfill('0');
  for (unsigned char ch : value) {
    if (std::isalnum(ch) || ch == '-' || ch == '_' || ch == '.' || ch == '~') {
      output << static_cast<char>(ch);
    } else {
      output << '%' << std::setw(2) << static_cast<int>(ch);
    }
  }
  return output.str();
}

KeystoneActionHttpTransport::KeystoneActionHttpTransport(
  std::string base_url, std::string auth_token
)
    : auth_token_(std::move(auth_token)) {
  base_url = trim_trailing_slashes(std::move(base_url));
  constexpr const char* http_prefix = "http://";
  constexpr const char* https_prefix = "https://";
  if (base_url.rfind(http_prefix, 0) == 0) {
    scheme_ = "http";
    base_url.erase(0, std::string(http_prefix).size());
  } else if (base_url.rfind(https_prefix, 0) == 0) {
    parse_error_ = "https Keystone URLs are not supported by axon-agent yet";
    return;
  } else {
    parse_error_ = "Keystone URL must start with http://";
    return;
  }

  const auto path_pos = base_url.find('/');
  auto authority = path_pos == std::string::npos ? base_url : base_url.substr(0, path_pos);
  base_path_ = path_pos == std::string::npos ? "" : ensure_leading_slash(base_url.substr(path_pos));

  const auto port_pos = authority.rfind(':');
  if (port_pos == std::string::npos) {
    host_ = authority;
    port_ = "80";
  } else {
    host_ = authority.substr(0, port_pos);
    port_ = authority.substr(port_pos + 1);
  }

  if (host_.empty()) {
    parse_error_ = "Keystone URL host must not be empty";
  }
  if (port_.empty()) {
    parse_error_ = "Keystone URL port must not be empty";
  }
}

KeystoneHttpResponse KeystoneActionHttpTransport::get(
  const std::string& path, std::chrono::milliseconds timeout
) {
  return request("GET", path, nullptr, timeout);
}

KeystoneHttpResponse KeystoneActionHttpTransport::put_json(
  const std::string& path, const nlohmann::json& body, std::chrono::milliseconds timeout
) {
  return request("PUT", path, &body, timeout);
}

KeystoneHttpResponse KeystoneActionHttpTransport::post_json(
  const std::string& path, const nlohmann::json& body, std::chrono::milliseconds timeout
) {
  return request("POST", path, &body, timeout);
}

KeystoneHttpResponse KeystoneActionHttpTransport::request(
  const std::string& method, const std::string& path, const nlohmann::json* body,
  std::chrono::milliseconds timeout
) {
  if (!parse_error_.empty()) {
    return {false, 0, nullptr, parse_error_};
  }

  try {
    asio::io_context io_context;
    tcp::resolver resolver(io_context);
    beast::tcp_stream stream(io_context);

    stream.expires_after(timeout);
    const auto resolved = resolver.resolve(host_, port_);
    stream.connect(resolved);

    const auto target = base_path_ + ensure_leading_slash(path);
    http::request<http::string_body> request{verb_from_method(method), target, 11};
    request.set(http::field::host, host_ + ":" + port_);
    request.set(http::field::user_agent, "axon-agent/" AXON_AGENT_VERSION);
    request.set(http::field::accept, "application/json");
    request.keep_alive(false);
    if (!auth_token_.empty()) {
      request.set(http::field::authorization, "Bearer " + auth_token_);
    }
    if (body != nullptr) {
      request.set(http::field::content_type, "application/json");
      request.body() = body->dump();
      request.prepare_payload();
    }

    http::write(stream, request);

    beast::flat_buffer buffer;
    http::response<http::string_body> response;
    http::read(stream, buffer, response);

    beast::error_code shutdown_ec;
    stream.socket().shutdown(tcp::socket::shutdown_both, shutdown_ec);

    nlohmann::json parsed = nullptr;
    if (!response.body().empty()) {
      parsed = nlohmann::json::parse(response.body(), nullptr, false);
      if (parsed.is_discarded()) {
        return {
          false,
          static_cast<int>(response.result_int()),
          response.body(),
          "Keystone returned invalid JSON"
        };
      }
    }

    const auto status_code = static_cast<int>(response.result_int());
    return {
      status_code >= 200 && status_code < 300,
      status_code,
      parsed,
      status_code >= 200 && status_code < 300
        ? ""
        : std::string(response.reason().data(), response.reason().size())
    };
  } catch (const std::exception& ex) {
    return {false, 0, nullptr, ex.what()};
  }
}

KeystoneActionSync::KeystoneActionSync(
  AgentService& service, KeystoneActionSyncConfig config,
  std::unique_ptr<KeystoneActionTransport> transport
)
    : service_(service)
    , config_(std::move(config))
    , transport_(std::move(transport)) {
  if (!transport_) {
    transport_ =
      std::make_unique<KeystoneActionHttpTransport>(config_.base_url, config_.auth_token);
  }
}

KeystoneActionSync::~KeystoneActionSync() {
  stop();
}

bool KeystoneActionSync::start() {
  if (!config_.enabled || running_) {
    return false;
  }
  running_ = true;
  thread_ = std::make_unique<std::thread>(&KeystoneActionSync::run, this);
  return true;
}

void KeystoneActionSync::stop() {
  if (!running_) {
    return;
  }
  running_ = false;
  if (thread_ && thread_->joinable()) {
    thread_->join();
  }
}

bool KeystoneActionSync::is_running() const {
  return running_;
}

bool KeystoneActionSync::sync_catalog_once(std::string* error) {
  auto robot_id = robot_id_or_error(error);
  if (robot_id.empty()) {
    record_failure(error != nullptr ? *error : "missing robot identity");
    return false;
  }

  const auto encoded_robot_id = keystone_url_encode_component(robot_id);
  const auto payload = service_.build_keystone_action_catalog(robot_id);
  const auto path = std::string(kCatalogPathPrefix) + encoded_robot_id + kCatalogPathSuffix;
  const auto response = transport_->put_json(path, payload, config_.http_timeout);
  if (!response.ok) {
    const auto message = response_error("catalog sync", response);
    if (error != nullptr) {
      *error = message;
    }
    record_failure(message);
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    ++catalog_sync_count_;
    last_catalog_sync_at_ = keystone_action_sync_now_iso8601();
  }
  record_success("catalog");
  return true;
}

bool KeystoneActionSync::poll_once(std::string* error) {
  auto robot_id = robot_id_or_error(error);
  if (robot_id.empty()) {
    record_failure(error != nullptr ? *error : "missing robot identity");
    return false;
  }

  const auto encoded_robot_id = keystone_url_encode_component(robot_id);
  const auto path = std::string(kCatalogPathPrefix) + encoded_robot_id + kPendingPathSuffix +
                    std::to_string(config_.pending_limit);
  const auto response = transport_->get(path, config_.http_timeout);
  if (!response.ok) {
    const auto message = response_error("pending request poll", response);
    if (error != nullptr) {
      *error = message;
    }
    record_failure(message);
    return false;
  }

  for (const auto& pending : extract_pending_requests(response.body)) {
    if (!process_request(robot_id, pending, error)) {
      record_failure(error != nullptr ? *error : "failed to process Keystone action request");
      return false;
    }
  }

  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    ++poll_count_;
    last_poll_at_ = keystone_action_sync_now_iso8601();
  }
  record_success("poll");
  return true;
}

nlohmann::json KeystoneActionSync::status_to_json() const {
  std::lock_guard<std::mutex> lock(stats_mutex_);
  return {
    {"enabled", config_.enabled},
    {"running", running_.load()},
    {"base_url", config_.base_url},
    {"robot_id", config_.robot_id},
    {"pending_limit", config_.pending_limit},
    {"catalog_sync_count", catalog_sync_count_},
    {"poll_count", poll_count_},
    {"action_requests_executed", action_requests_executed_},
    {"status_updates_sent", status_updates_sent_},
    {"consecutive_failures", consecutive_failures_},
    {"current_backoff_ms", backoff_delay_.count()},
    {"last_error", last_error_},
    {"last_catalog_sync_at", last_catalog_sync_at_},
    {"last_poll_at", last_poll_at_},
  };
}

void KeystoneActionSync::run() {
  auto next_catalog_sync = std::chrono::steady_clock::now();
  while (running_) {
    bool ok = true;
    std::string error;
    const auto now = std::chrono::steady_clock::now();
    if (now >= next_catalog_sync) {
      ok = sync_catalog_once(&error);
      next_catalog_sync = now + config_.catalog_sync_interval;
    }
    if (ok) {
      ok = poll_once(&error);
    }
    if (!ok && !error.empty()) {
      std::cerr << "axon-agent Keystone action sync: " << error << std::endl;
    }

    auto sleep_for = ok ? config_.poll_interval : current_backoff();
    if (sleep_for <= std::chrono::milliseconds(0)) {
      sleep_for = config_.poll_interval;
    }

    const auto sleep_until = std::chrono::steady_clock::now() + sleep_for;
    while (running_ && std::chrono::steady_clock::now() < sleep_until) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
}

std::string KeystoneActionSync::robot_id_or_error(std::string* error) {
  auto robot_id = service_.resolve_robot_id(config_.robot_id);
  if (robot_id.empty() && error != nullptr) {
    *error = "Keystone action sync requires --keystone-robot-id or an active robot profile";
  }
  return robot_id;
}

bool KeystoneActionSync::report_status(
  const std::string& robot_id, const std::string& request_id, const std::string& status,
  const nlohmann::json& execution, std::string* error
) {
  nlohmann::json payload = {
    {"schema_version", 1},
    {"robot_id", robot_id},
    {"request_id", request_id},
    {"status", status},
    {"reported_at", keystone_action_sync_now_iso8601()},
  };
  if (!execution.is_null()) {
    payload["execution"] = execution;
    if (execution.is_object() && execution.contains("status") && execution["status"].is_string()) {
      payload["axon_status"] = execution["status"];
    }
  }

  const auto encoded_request_id = keystone_url_encode_component(request_id);
  const auto path = std::string(kRequestPathPrefix) + encoded_request_id + kStatusPathSuffix;
  const auto response = transport_->post_json(path, payload, config_.http_timeout);
  if (!response.ok) {
    const auto message = response_error("status report", response);
    if (error != nullptr) {
      *error = message;
    }
    return false;
  }

  std::lock_guard<std::mutex> lock(stats_mutex_);
  ++status_updates_sent_;
  return true;
}

bool KeystoneActionSync::process_request(
  const std::string& robot_id, const nlohmann::json& pending, std::string* error
) {
  const auto pending_request_id = json_string(pending, "request_id");
  if (!pending_request_id.has_value() || pending_request_id->empty()) {
    if (error != nullptr) {
      *error = "pending action request is missing request_id";
    }
    return false;
  }

  const auto encoded_request_id = keystone_url_encode_component(*pending_request_id);
  const auto detail_path = std::string(kRequestPathPrefix) + encoded_request_id;
  const auto detail_response = transport_->get(detail_path, config_.http_timeout);
  if (!detail_response.ok) {
    const auto message = response_error("request detail fetch", detail_response);
    if (error != nullptr) {
      *error = message;
    }
    return false;
  }

  const auto detail = extract_request_detail(detail_response.body);
  const auto request_id = json_string(detail, "request_id").value_or(*pending_request_id);
  const auto action_id = json_string(detail, "action_id");
  if (!action_id.has_value() || action_id->empty()) {
    if (error != nullptr) {
      *error = "Keystone action request " + request_id + " is missing action_id";
    }
    return false;
  }

  const auto request_status = json_string(detail, "status").value_or("");
  if (request_status == "cancelled") {
    return report_status(robot_id, request_id, "cancelled", detail, error);
  }

  if (!report_status(robot_id, request_id, "queued", nullptr, error)) {
    return false;
  }
  if (!report_status(robot_id, request_id, "running", nullptr, error)) {
    return false;
  }

  nlohmann::json execute_params = {
    {"request_id", request_id},
    {"action_id", *action_id},
    {"args", json_object_or_empty(detail, "args")},
  };
  if (const auto expires_at = json_string(detail, "expires_at")) {
    execute_params["expires_at"] = *expires_at;
  }

  auto execution = service_.execute_action(execute_params);
  nlohmann::json execution_data =
    execution.data.is_null() ? nlohmann::json::object() : execution.data;
  execution_data["success"] = execution.success;
  execution_data["message"] = execution.message;

  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    ++action_requests_executed_;
  }

  const auto local_status =
    json_string(execution_data, "status").value_or(execution.success ? "completed" : "failed");
  const auto status = keystone_action_status_from_local(local_status);
  return report_status(robot_id, request_id, status, execution_data, error);
}

void KeystoneActionSync::record_success(const std::string&) {
  std::lock_guard<std::mutex> lock(stats_mutex_);
  consecutive_failures_ = 0;
  backoff_delay_ = std::chrono::milliseconds(0);
  last_error_.clear();
}

void KeystoneActionSync::record_failure(const std::string& error) {
  std::lock_guard<std::mutex> lock(stats_mutex_);
  ++consecutive_failures_;
  last_error_ = error;
  if (backoff_delay_ <= std::chrono::milliseconds(0)) {
    backoff_delay_ = config_.min_backoff;
  } else {
    backoff_delay_ = std::min(backoff_delay_ * 2, config_.max_backoff);
  }
  if (backoff_delay_ > config_.max_backoff) {
    backoff_delay_ = config_.max_backoff;
  }
}

std::chrono::milliseconds KeystoneActionSync::current_backoff() const {
  std::lock_guard<std::mutex> lock(stats_mutex_);
  return backoff_delay_;
}

}  // namespace agent
}  // namespace axon
