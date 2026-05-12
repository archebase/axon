// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_CONFIG_HTTP_CLIENT_HPP
#define AXON_CONFIG_HTTP_CLIENT_HPP

#include <curl/curl.h>

#include <chrono>
#include <functional>
#include <string>

namespace axon {
namespace config {

struct HttpResponse {
  long status_code = 0;
  CURLcode curl_code = CURLE_OK;
  std::string body;
  std::string error;
};

struct HttpClientOptions {
  long timeout_seconds = 10;
  int max_attempts = 3;
  std::chrono::milliseconds initial_retry_delay{200};
  std::chrono::milliseconds max_retry_delay{std::chrono::seconds(2)};
  int retry_jitter_percent = 25;
  std::string user_agent = "axon-config/1.0";
};

class HttpClient {
public:
  HttpClient();
  explicit HttpClient(HttpClientOptions options);

  HttpResponse get_text(const std::string& url) const;
  HttpResponse post_json(const std::string& url, const std::string& body) const;

private:
  HttpResponse request_with_retries(
    const std::string& method, const std::string& url, const std::function<HttpResponse()>& request
  ) const;
  HttpResponse perform_get_text(const std::string& url) const;
  HttpResponse perform_post_json(const std::string& url, const std::string& body) const;
  std::chrono::milliseconds retry_delay_for_attempt(int retry_index) const;

  HttpClientOptions options_;
};

}  // namespace config
}  // namespace axon

#endif  // AXON_CONFIG_HTTP_CLIENT_HPP
