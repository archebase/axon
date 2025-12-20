#include "http_callback_client.hpp"

#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>

#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <regex>
#include <sstream>
#include <thread>

namespace beast = boost::beast;
namespace http = beast::http;
namespace net = boost::asio;
using tcp = net::ip::tcp;

namespace axon {
namespace recorder {

// ============================================================================
// JSON Serialization Helpers
// ============================================================================

namespace {

std::string escape_json_string(const std::string& s) {
  std::ostringstream oss;
  for (char c : s) {
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
        if ('\x00' <= c && c <= '\x1f') {
          oss << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(c);
        } else {
          oss << c;
        }
    }
  }
  return oss.str();
}

std::string string_array_to_json(const std::vector<std::string>& arr) {
  std::ostringstream oss;
  oss << "[";
  for (size_t i = 0; i < arr.size(); ++i) {
    if (i > 0) oss << ", ";
    oss << "\"" << escape_json_string(arr[i]) << "\"";
  }
  oss << "]";
  return oss.str();
}

}  // namespace

// ============================================================================
// Payload JSON Serialization
// ============================================================================

std::string StartCallbackPayload::to_json() const {
  std::ostringstream oss;
  oss << "{";
  oss << "\"task_id\": \"" << escape_json_string(task_id) << "\", ";
  oss << "\"device_id\": \"" << escape_json_string(device_id) << "\", ";
  oss << "\"status\": \"" << escape_json_string(status) << "\", ";
  oss << "\"started_at\": \"" << escape_json_string(started_at) << "\", ";
  oss << "\"topics\": " << string_array_to_json(topics);
  oss << "}";
  return oss.str();
}

std::string FinishCallbackPayload::to_json() const {
  std::ostringstream oss;
  oss << "{";
  oss << "\"task_id\": \"" << escape_json_string(task_id) << "\", ";
  oss << "\"device_id\": \"" << escape_json_string(device_id) << "\", ";
  oss << "\"status\": \"" << escape_json_string(status) << "\", ";
  oss << "\"started_at\": \"" << escape_json_string(started_at) << "\", ";
  oss << "\"finished_at\": \"" << escape_json_string(finished_at) << "\", ";
  oss << "\"duration_sec\": " << std::fixed << std::setprecision(1) << duration_sec << ", ";
  oss << "\"message_count\": " << message_count << ", ";
  oss << "\"file_size_bytes\": " << file_size_bytes << ", ";
  oss << "\"output_path\": \"" << escape_json_string(output_path) << "\", ";
  oss << "\"topics\": " << string_array_to_json(topics) << ", ";
  if (error.empty()) {
    oss << "\"error\": null";
  } else {
    oss << "\"error\": \"" << escape_json_string(error) << "\"";
  }
  oss << "}";
  return oss.str();
}

// ============================================================================
// HttpCallbackClient Implementation
// ============================================================================

HttpCallbackClient::HttpCallbackClient()
    : config_() {}

HttpCallbackClient::HttpCallbackClient(const Config& config)
    : config_(config) {}

HttpCallbackClient::~HttpCallbackClient() = default;

std::string HttpCallbackClient::get_iso8601_timestamp() {
  return get_iso8601_timestamp(std::chrono::system_clock::now());
}

std::string HttpCallbackClient::get_iso8601_timestamp(std::chrono::system_clock::time_point tp) {
  auto time_t = std::chrono::system_clock::to_time_t(tp);
  std::tm tm = *std::gmtime(&time_t);

  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
  return oss.str();
}

bool HttpCallbackClient::parse_url(
  const std::string& url, std::string& host, std::string& port, std::string& path, bool& use_ssl
) {
  // Simple URL parser for http:// and https:// URLs
  // Format: http(s)://host(:port)/path

  std::regex url_regex(R"(^(https?)://([^/:]+)(?::(\d+))?(.*)$)", std::regex::icase);
  std::smatch match;

  if (!std::regex_match(url, match, url_regex)) {
    return false;
  }

  std::string scheme = match[1].str();
  host = match[2].str();
  std::string port_str = match[3].str();
  path = match[4].str();

  // Default path
  if (path.empty()) {
    path = "/";
  }

  // Determine SSL and default port
  use_ssl = (scheme == "https" || scheme == "HTTPS");
  if (port_str.empty()) {
    port = use_ssl ? "443" : "80";
  } else {
    port = port_str;
  }

  return true;
}

HttpCallbackResult HttpCallbackClient::http_post(
  const std::string& url, const std::string& body, const std::string& auth_token
) {
  HttpCallbackResult result;
  result.success = false;
  result.status_code = 0;

  std::string host, port, path;
  bool use_ssl;

  if (!parse_url(url, host, port, path, use_ssl)) {
    result.error_message = "ERR_CALLBACK_FAILED: Invalid URL format: " + url;
    return result;
  }

  // Note: For simplicity, we only support HTTP (not HTTPS) in this implementation.
  // For HTTPS support, you would need to add Boost.Asio SSL context.
  if (use_ssl) {
    result.error_message =
      "ERR_CALLBACK_FAILED: HTTPS not supported in this build. URL: " + url;
    std::cerr << "[HttpCallbackClient] Warning: " << result.error_message << std::endl;
    // Continue anyway with HTTP - in production you'd want proper SSL support
  }

  try {
    // Create IO context and resolver
    net::io_context ioc;
    tcp::resolver resolver(ioc);
    beast::tcp_stream stream(ioc);

    // Set timeout
    stream.expires_after(config_.request_timeout);

    // Resolve and connect
    auto const results = resolver.resolve(host, port);
    stream.connect(results);

    // Build HTTP request
    http::request<http::string_body> req{http::verb::post, path, 11};
    req.set(http::field::host, host);
    req.set(http::field::user_agent, "axon-recorder/1.0");
    req.set(http::field::content_type, "application/json");

    // Add Bearer token authentication
    if (!auth_token.empty()) {
      req.set(http::field::authorization, "Bearer " + auth_token);
    }

    req.body() = body;
    req.prepare_payload();

    // Send request
    http::write(stream, req);

    // Receive response
    beast::flat_buffer buffer;
    http::response<http::string_body> res;
    http::read(stream, buffer, res);

    result.status_code = static_cast<int>(res.result_int());
    result.response_body = res.body();

    // Check for success (2xx status codes)
    if (result.status_code >= 200 && result.status_code < 300) {
      result.success = true;
    } else {
      result.error_message = "ERR_CALLBACK_FAILED: Server returned status " +
                             std::to_string(result.status_code);
    }

    // Gracefully close the socket
    beast::error_code ec;
    stream.socket().shutdown(tcp::socket::shutdown_both, ec);
    // Ignore not_connected error (it's expected)
    if (ec && ec != beast::errc::not_connected) {
      // Log but don't fail
      std::cerr << "[HttpCallbackClient] Socket shutdown warning: " << ec.message() << std::endl;
    }

  } catch (const std::exception& e) {
    result.error_message = std::string("ERR_CALLBACK_FAILED: ") + e.what();
    std::cerr << "[HttpCallbackClient] Exception: " << e.what() << std::endl;
  }

  return result;
}

HttpCallbackResult HttpCallbackClient::post_start_callback(
  const TaskConfig& task_config, const StartCallbackPayload& payload, HttpCallbackHandler handler
) {
  if (task_config.start_callback_url.empty()) {
    HttpCallbackResult result;
    result.success = true;  // No callback configured is not an error
    result.error_message = "No start callback URL configured";
    if (handler) {
      handler(result);
    }
    return result;
  }

  std::string body = payload.to_json();
  HttpCallbackResult result = http_post(task_config.start_callback_url, body, task_config.user_token);

  if (handler) {
    handler(result);
  }

  return result;
}

HttpCallbackResult HttpCallbackClient::post_finish_callback(
  const TaskConfig& task_config, const FinishCallbackPayload& payload, HttpCallbackHandler handler
) {
  if (task_config.finish_callback_url.empty()) {
    HttpCallbackResult result;
    result.success = true;  // No callback configured is not an error
    result.error_message = "No finish callback URL configured";
    if (handler) {
      handler(result);
    }
    return result;
  }

  std::string body = payload.to_json();
  HttpCallbackResult result = http_post(task_config.finish_callback_url, body, task_config.user_token);

  if (handler) {
    handler(result);
  }

  return result;
}

void HttpCallbackClient::post_start_callback_async(
  const TaskConfig& task_config, const StartCallbackPayload& payload
) {
  // Fire-and-forget async call using a detached thread
  // In production, you might want a thread pool or async IO
  std::thread([this, task_config, payload]() {
    auto result = post_start_callback(task_config, payload, nullptr);
    if (!result.success) {
      std::cerr << "[HttpCallbackClient] Start callback failed: " << result.error_message
                << std::endl;
    } else {
      std::cerr << "[HttpCallbackClient] Start callback sent successfully for task: "
                << task_config.task_id << std::endl;
    }
  }).detach();
}

void HttpCallbackClient::post_finish_callback_async(
  const TaskConfig& task_config, const FinishCallbackPayload& payload
) {
  // Fire-and-forget async call using a detached thread
  std::thread([this, task_config, payload]() {
    auto result = post_finish_callback(task_config, payload, nullptr);
    if (!result.success) {
      std::cerr << "[HttpCallbackClient] Finish callback failed: " << result.error_message
                << std::endl;
    } else {
      std::cerr << "[HttpCallbackClient] Finish callback sent successfully for task: "
                << task_config.task_id << std::endl;
    }
  }).detach();
}

}  // namespace recorder
}  // namespace axon

