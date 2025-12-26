#include "http_callback_client.hpp"

#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/ssl.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/ssl.hpp>
#include <boost/beast/version.hpp>

#include <chrono>
#include <ctime>
#include <iomanip>
#include <regex>
#include <sstream>
#include <thread>

// Logging infrastructure
#define AXON_LOG_COMPONENT "http_callback_client"
#include <axon_log_macros.hpp>

namespace beast = boost::beast;
namespace http = beast::http;
namespace net = boost::asio;
namespace ssl = net::ssl;
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
  oss << "\"duration_sec\": " << std::fixed << std::setprecision(3) << duration_sec << ", ";
  oss << "\"message_count\": " << message_count << ", ";
  oss << "\"file_size_bytes\": " << file_size_bytes << ", ";
  oss << "\"output_path\": \"" << escape_json_string(output_path) << "\", ";
  oss << "\"sidecar_path\": \"" << escape_json_string(sidecar_path) << "\", ";
  oss << "\"topics\": " << string_array_to_json(topics) << ", ";
  // Metadata summary
  oss << "\"metadata\": {";
  oss << "\"scene\": \"" << escape_json_string(metadata.scene) << "\", ";
  oss << "\"subscene\": \"" << escape_json_string(metadata.subscene) << "\", ";
  oss << "\"skills\": " << string_array_to_json(metadata.skills) << ", ";
  oss << "\"factory\": \"" << escape_json_string(metadata.factory) << "\"";
  oss << "}, ";
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
  auto time_t_val = std::chrono::system_clock::to_time_t(tp);
  std::tm tm{};

  // Use thread-safe version of gmtime
#ifdef _WIN32
  gmtime_s(&tm, &time_t_val);
#else
  gmtime_r(&time_t_val, &tm);
#endif

  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
  return oss.str();
}

bool HttpCallbackClient::parse_url(
  const std::string& url, std::string& host, std::string& port, std::string& path, bool& use_ssl
) {
  // Simple URL parser for http:// and https:// URLs
  // Format: http(s)://host(:port)/path
  // Note: static regex is compiled once for better performance
  static const std::regex url_regex(R"(^(https?)://([^/:]+)(?::(\d+))?(.*)$)", std::regex::icase);
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

  try {
    // Create IO context and resolver
    net::io_context ioc;
    tcp::resolver resolver(ioc);

    // Build HTTP request (same for HTTP and HTTPS)
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

    // Response buffer and object
    beast::flat_buffer buffer;
    http::response<http::string_body> res;

    if (use_ssl) {
      // HTTPS request with SSL/TLS
      ssl::context ctx(ssl::context::tlsv12_client);
      
      // Use system's default CA certificates for verification
      ctx.set_default_verify_paths();
      ctx.set_verify_mode(ssl::verify_peer);

      beast::ssl_stream<beast::tcp_stream> stream(ioc, ctx);

      // Set SNI hostname (required for most servers)
      if (!SSL_set_tlsext_host_name(stream.native_handle(), host.c_str())) {
        beast::error_code ec{static_cast<int>(::ERR_get_error()), net::error::get_ssl_category()};
        result.error_message = "ERR_CALLBACK_FAILED: SNI hostname failed: " + ec.message();
        return result;
      }

      // Set timeout on the underlying TCP stream
      beast::get_lowest_layer(stream).expires_after(config_.request_timeout);

      // Resolve and connect
      auto const results = resolver.resolve(host, port);
      beast::get_lowest_layer(stream).connect(results);

      // Perform SSL handshake
      stream.handshake(ssl::stream_base::client);

      // Send request
      http::write(stream, req);

      // Receive response
      http::read(stream, buffer, res);

      // Gracefully close the SSL stream
      beast::error_code ec;
      stream.shutdown(ec);
      // Ignore truncated error (peer may close without close_notify)
      if (ec && ec != net::ssl::error::stream_truncated && ec != beast::errc::not_connected) {
        AXON_LOG_WARN("SSL shutdown warning" << axon::logging::kv("error", ec.message()));
      }

    } else {
      // Plain HTTP request
      beast::tcp_stream stream(ioc);

      // Set timeout
      stream.expires_after(config_.request_timeout);

      // Resolve and connect
      auto const results = resolver.resolve(host, port);
      stream.connect(results);

      // Send request
      http::write(stream, req);

      // Receive response
      http::read(stream, buffer, res);

      // Gracefully close the socket
      beast::error_code ec;
      stream.socket().shutdown(tcp::socket::shutdown_both, ec);
      if (ec && ec != beast::errc::not_connected) {
        AXON_LOG_WARN("Socket shutdown warning" << axon::logging::kv("error", ec.message()));
      }
    }

    result.status_code = static_cast<int>(res.result_int());
    result.response_body = res.body();

    // Check for success (2xx status codes)
    if (result.status_code >= 200 && result.status_code < 300) {
      result.success = true;
    } else {
      result.error_message = "ERR_CALLBACK_FAILED: Server returned status " +
                             std::to_string(result.status_code);
    }

  } catch (const std::exception& e) {
    result.error_message = std::string("ERR_CALLBACK_FAILED: ") + e.what();
    AXON_LOG_ERROR("HTTP request exception" << axon::logging::kv("error", e.what()));
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
  // Capture shared_ptr to ensure the client outlives the async operation
  // This prevents use-after-free if HttpCallbackClient is destroyed before thread completes
  auto self = shared_from_this();
  std::thread([self, task_config, payload]() {
    auto result = self->post_start_callback(task_config, payload, nullptr);
    if (!result.success) {
      AXON_LOG_ERROR("Start callback failed" << axon::logging::kv("error", result.error_message));
    } else {
      AXON_LOG_INFO("Start callback sent successfully" << axon::logging::kv("task_id", task_config.task_id));
    }
  }).detach();
}

void HttpCallbackClient::post_finish_callback_async(
  const TaskConfig& task_config, const FinishCallbackPayload& payload
) {
  // Capture shared_ptr to ensure the client outlives the async operation
  // This prevents use-after-free if HttpCallbackClient is destroyed before thread completes
  auto self = shared_from_this();
  std::thread([self, task_config, payload]() {
    auto result = self->post_finish_callback(task_config, payload, nullptr);
    if (!result.success) {
      AXON_LOG_ERROR("Finish callback failed" << axon::logging::kv("error", result.error_message));
    } else {
      AXON_LOG_INFO("Finish callback sent successfully" << axon::logging::kv("task_id", task_config.task_id));
    }
  }).detach();
}

}  // namespace recorder
}  // namespace axon

