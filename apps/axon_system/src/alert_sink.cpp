// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "alert_sink.hpp"

#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>

#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <utility>

namespace axon {
namespace system {

namespace {

namespace beast = boost::beast;
namespace http = boost::beast::http;
using tcp = boost::asio::ip::tcp;

struct HttpUrl {
  std::string host;
  std::string port = "80";
  std::string target = "/";
};

bool parse_http_url(const std::string& url, HttpUrl* parsed, std::string* error) {
  constexpr const char* kScheme = "http://";
  if (parsed == nullptr) {
    return false;
  }
  if (url.rfind(kScheme, 0) != 0) {
    if (error != nullptr) {
      *error = "only http:// alert sink URLs are supported";
    }
    return false;
  }

  const auto without_scheme = url.substr(std::string(kScheme).size());
  const auto slash_pos = without_scheme.find('/');
  const auto authority =
    slash_pos == std::string::npos ? without_scheme : without_scheme.substr(0, slash_pos);
  parsed->target = slash_pos == std::string::npos ? "/" : without_scheme.substr(slash_pos);
  if (authority.empty()) {
    if (error != nullptr) {
      *error = "alert sink URL host is empty";
    }
    return false;
  }

  const auto colon_pos = authority.rfind(':');
  if (colon_pos != std::string::npos) {
    parsed->host = authority.substr(0, colon_pos);
    parsed->port = authority.substr(colon_pos + 1);
  } else {
    parsed->host = authority;
    parsed->port = "80";
  }

  if (parsed->host.empty() || parsed->port.empty()) {
    if (error != nullptr) {
      *error = "alert sink URL host or port is empty";
    }
    return false;
  }
  return true;
}

std::string trim_ascii_whitespace(std::string value) {
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

std::string read_token_file(const std::filesystem::path& path, std::string* error) {
  std::ifstream input(path);
  if (!input) {
    if (error != nullptr) {
      *error = "failed to open alert sink auth token file";
    }
    return "";
  }
  std::ostringstream stream;
  stream << input.rdbuf();
  return trim_ascii_whitespace(stream.str());
}

class LogAlertSink final : public AlertSink {
public:
  AlertDeliveryResult deliver(const nlohmann::json& event) override {
    std::cout << event.dump() << std::endl;
    return {};
  }
};

class FileAlertSink final : public AlertSink {
public:
  explicit FileAlertSink(std::filesystem::path path)
      : path_(std::move(path)) {}

  AlertDeliveryResult deliver(const nlohmann::json& event) override {
    std::error_code ec;
    if (!path_.parent_path().empty()) {
      std::filesystem::create_directories(path_.parent_path(), ec);
      if (ec) {
        return {false, "failed to create alert sink directory: " + ec.message()};
      }
    }

    std::ofstream output(path_, std::ios::app);
    if (!output) {
      return {false, "failed to open alert sink file: " + path_.string()};
    }
    output << event.dump() << '\n';
    if (!output) {
      return {false, "failed to write alert sink file: " + path_.string()};
    }
    return {};
  }

private:
  std::filesystem::path path_;
};

class OpsHttpAlertSink final : public AlertSink {
public:
  explicit OpsHttpAlertSink(AlertSinkConfig config)
      : config_(std::move(config)) {
    if (config_.url.empty()) {
      throw std::invalid_argument("ops_http alert sink requires url");
    }
    if (config_.timeout_ms <= 0) {
      config_.timeout_ms = 2000;
    }
    std::string error;
    if (!parse_http_url(config_.url, &url_, &error)) {
      throw std::invalid_argument(error);
    }
  }

  AlertDeliveryResult deliver(const nlohmann::json& event) override {
    try {
      std::string token = config_.auth_token;
      if (!config_.auth_token_file.empty()) {
        std::string token_error;
        token = read_token_file(config_.auth_token_file, &token_error);
        if (!token_error.empty()) {
          return {false, token_error};
        }
      }

      boost::asio::io_context io_context;
      tcp::resolver resolver(io_context);
      beast::tcp_stream stream(io_context);
      stream.expires_after(std::chrono::milliseconds(config_.timeout_ms));
      const auto results = resolver.resolve(url_.host, url_.port);
      stream.connect(results);

      const std::string body = event.dump();
      http::request<http::string_body> request{http::verb::post, url_.target, 11};
      request.set(http::field::host, url_.host);
      request.set(http::field::user_agent, "axon-system");
      request.set(http::field::content_type, "application/json");
      if (!token.empty()) {
        request.set(http::field::authorization, "Bearer " + token);
      }
      if (event.contains("event_id") && event["event_id"].is_string()) {
        request.set("X-Axon-Alert-Event-Id", event["event_id"].get<std::string>());
      }
      request.body() = body;
      request.prepare_payload();

      http::write(stream, request);
      beast::flat_buffer buffer;
      http::response<http::string_body> response;
      http::read(stream, buffer, response);

      beast::error_code ec;
      stream.socket().shutdown(tcp::socket::shutdown_both, ec);

      const auto status_code = static_cast<unsigned int>(response.result_int());
      if (status_code < 200 || status_code >= 300) {
        return {false, "ops_http alert sink returned HTTP " + std::to_string(status_code)};
      }
      return {};
    } catch (const std::exception& ex) {
      return {false, std::string("ops_http alert sink delivery failed: ") + ex.what()};
    }
  }

private:
  AlertSinkConfig config_;
  HttpUrl url_;
};

}  // namespace

std::unique_ptr<AlertSink> make_alert_sink(
  const AlertSinkConfig& config, const std::filesystem::path& state_dir
) {
  if (config.type == "file") {
    const auto path = config.path.empty() ? state_dir / "alerts.jsonl" : config.path;
    return std::make_unique<FileAlertSink>(path);
  }
  if (config.type == "log") {
    return std::make_unique<LogAlertSink>();
  }
  if (config.type == "ops_http") {
    return std::make_unique<OpsHttpAlertSink>(config);
  }
  throw std::invalid_argument("unsupported alert sink type: " + config.type);
}

}  // namespace system
}  // namespace axon
