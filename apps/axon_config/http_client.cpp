// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "http_client.hpp"

#include <algorithm>
#include <iostream>
#include <random>
#include <thread>
#include <utility>

namespace axon {
namespace config {

namespace {

size_t write_curl_response(char* ptr, size_t size, size_t nmemb, void* userdata) {
  auto* body = static_cast<std::string*>(userdata);
  const size_t bytes = size * nmemb;
  body->append(ptr, bytes);
  return bytes;
}

class CurlGlobalState {
public:
  CurlGlobalState()
      : code_(curl_global_init(CURL_GLOBAL_DEFAULT)) {}

  ~CurlGlobalState() {
    if (code_ == CURLE_OK) {
      curl_global_cleanup();
    }
  }

  CURLcode code() const {
    return code_;
  }

private:
  CURLcode code_;
};

CURLcode ensure_curl_global_initialized() {
  // Function-local static initialization is thread-safe in C++11 and later.
  static const CurlGlobalState state;
  return state.code();
}

bool is_retryable_curl_error(CURLcode code) {
  switch (code) {
    case CURLE_COULDNT_RESOLVE_PROXY:
    case CURLE_COULDNT_RESOLVE_HOST:
    case CURLE_COULDNT_CONNECT:
    case CURLE_OPERATION_TIMEDOUT:
    case CURLE_SEND_ERROR:
    case CURLE_RECV_ERROR:
    case CURLE_GOT_NOTHING:
    case CURLE_PARTIAL_FILE:
    case CURLE_SSL_CONNECT_ERROR:
      return true;
    default:
      return false;
  }
}

bool is_retryable_http_status(long status_code) {
  return status_code == 429 || (status_code >= 500 && status_code <= 599);
}

bool is_retryable_response(const HttpResponse& response) {
  if (response.curl_code != CURLE_OK) {
    return is_retryable_curl_error(response.curl_code);
  }
  return is_retryable_http_status(response.status_code);
}

std::string retry_reason(const HttpResponse& response) {
  if (response.curl_code != CURLE_OK) {
    return response.error;
  }
  return "HTTP " + std::to_string(response.status_code);
}

}  // namespace

HttpClient::HttpClient()
    : HttpClient(HttpClientOptions{}) {}

HttpClient::HttpClient(HttpClientOptions options)
    : options_(std::move(options)) {
  if (options_.max_attempts < 1) {
    options_.max_attempts = 1;
  }
  if (options_.retry_jitter_percent < 0) {
    options_.retry_jitter_percent = 0;
  }
}

HttpResponse HttpClient::get_text(const std::string& url) const {
  return request_with_retries("GET", url, [&]() {
    return perform_get_text(url);
  });
}

HttpResponse HttpClient::post_json(const std::string& url, const std::string& body) const {
  return request_with_retries("POST", url, [&]() {
    return perform_post_json(url, body);
  });
}

HttpResponse HttpClient::request_with_retries(
  const std::string& method, const std::string& url, const std::function<HttpResponse()>& request
) const {
  HttpResponse response;
  for (int attempt = 1; attempt <= options_.max_attempts; ++attempt) {
    response = request();
    if (!is_retryable_response(response) || attempt == options_.max_attempts) {
      return response;
    }

    const auto delay = retry_delay_for_attempt(attempt - 1);
    std::cerr << "Warning: " << method << " " << url << " failed with " << retry_reason(response)
              << "; retrying in " << delay.count() << " ms (attempt " << (attempt + 1) << "/"
              << options_.max_attempts << ")" << std::endl;
    std::this_thread::sleep_for(delay);
  }
  return response;
}

HttpResponse HttpClient::perform_get_text(const std::string& url) const {
  HttpResponse response;

  CURLcode global_code = ensure_curl_global_initialized();
  if (global_code != CURLE_OK) {
    response.curl_code = global_code;
    response.error = curl_easy_strerror(global_code);
    return response;
  }

  CURL* curl = curl_easy_init();
  if (curl == nullptr) {
    response.error = "failed to initialize curl";
    return response;
  }

  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_curl_response);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response.body);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, options_.timeout_seconds);
  curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
  curl_easy_setopt(curl, CURLOPT_USERAGENT, options_.user_agent.c_str());

  CURLcode code = curl_easy_perform(curl);
  response.curl_code = code;
  if (code != CURLE_OK) {
    response.error = curl_easy_strerror(code);
  } else {
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response.status_code);
  }

  curl_easy_cleanup(curl);

  return response;
}

HttpResponse HttpClient::perform_post_json(const std::string& url, const std::string& body) const {
  HttpResponse response;

  CURLcode global_code = ensure_curl_global_initialized();
  if (global_code != CURLE_OK) {
    response.curl_code = global_code;
    response.error = curl_easy_strerror(global_code);
    return response;
  }

  CURL* curl = curl_easy_init();
  if (curl == nullptr) {
    response.error = "failed to initialize curl";
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
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, options_.timeout_seconds);
  curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
  curl_easy_setopt(curl, CURLOPT_USERAGENT, options_.user_agent.c_str());

  CURLcode code = curl_easy_perform(curl);
  response.curl_code = code;
  if (code != CURLE_OK) {
    response.error = curl_easy_strerror(code);
  } else {
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response.status_code);
  }

  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);

  return response;
}

std::chrono::milliseconds HttpClient::retry_delay_for_attempt(int retry_index) const {
  auto delay = options_.initial_retry_delay;
  for (int i = 0; i < retry_index && delay < options_.max_retry_delay; ++i) {
    delay *= 2;
  }
  if (delay > options_.max_retry_delay) {
    delay = options_.max_retry_delay;
  }

  const auto jitter_range = delay.count() * options_.retry_jitter_percent / 100;
  if (jitter_range <= 0) {
    return delay;
  }

  static thread_local std::mt19937 generator(std::random_device{}());
  std::uniform_int_distribution<long long> distribution(-jitter_range, jitter_range);
  return delay + std::chrono::milliseconds(distribution(generator));
}

}  // namespace config
}  // namespace axon
