// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <chrono>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

#include "http_client.hpp"

namespace axon {
namespace config {
namespace test {

struct MockHttpResponse {
  int status_code;
  std::string body;
};

class MockHttpServer {
public:
  explicit MockHttpServer(std::vector<MockHttpResponse> responses)
      : responses_(std::move(responses))
      , listen_fd_(::socket(AF_INET, SOCK_STREAM, 0)) {
    if (listen_fd_ < 0) {
      throw std::runtime_error("failed to create test socket");
    }

    int reuse = 1;
    ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = 0;

    if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
      ::close(listen_fd_);
      throw std::runtime_error("failed to bind test socket");
    }

    if (::listen(listen_fd_, 1) != 0) {
      ::close(listen_fd_);
      throw std::runtime_error("failed to listen on test socket");
    }

    socklen_t address_len = sizeof(address);
    if (::getsockname(listen_fd_, reinterpret_cast<sockaddr*>(&address), &address_len) != 0) {
      ::close(listen_fd_);
      throw std::runtime_error("failed to inspect test socket");
    }
    port_ = ntohs(address.sin_port);

    thread_ = std::thread([this]() {
      serve();
    });
  }

  ~MockHttpServer() {
    if (thread_.joinable()) {
      poke();
      thread_.join();
    }
    if (listen_fd_ >= 0) {
      ::close(listen_fd_);
    }
  }

  std::string url() const {
    return "http://127.0.0.1:" + std::to_string(port_);
  }

  void wait() {
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  const std::string& request(size_t index) const {
    return requests_.at(index);
  }

  const std::string& body(size_t index) const {
    return bodies_.at(index);
  }

  size_t request_count() const {
    return requests_.size();
  }

private:
  void poke() const {
    int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
      return;
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = htons(port_);
    ::connect(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address));
    ::close(fd);
  }

  void serve() {
    while (next_response_ < responses_.size()) {
      if (!serve_one(responses_[next_response_])) {
        break;
      }
      ++next_response_;
    }
  }

  bool serve_one(const MockHttpResponse& mock_response) {
    int client_fd = ::accept(listen_fd_, nullptr, nullptr);
    if (client_fd < 0) {
      return false;
    }

    std::string request;
    char buffer[1024];
    while (true) {
      ssize_t bytes = ::recv(client_fd, buffer, sizeof(buffer), 0);
      if (bytes <= 0) {
        break;
      }
      request.append(buffer, static_cast<size_t>(bytes));

      size_t header_end = request.find("\r\n\r\n");
      if (header_end == std::string::npos) {
        continue;
      }

      const size_t body_start = header_end + 4;
      const size_t content_length = parse_content_length(request.substr(0, header_end));
      if (request.size() >= body_start + content_length) {
        break;
      }
    }

    if (!request.empty()) {
      requests_.push_back(request);
      size_t header_end = request.find("\r\n\r\n");
      if (header_end != std::string::npos) {
        const size_t body_start = header_end + 4;
        const size_t content_length = parse_content_length(request.substr(0, header_end));
        bodies_.push_back(request.substr(body_start, content_length));
      } else {
        bodies_.emplace_back();
      }

      std::ostringstream response;
      response << "HTTP/1.1 " << mock_response.status_code << " "
               << reason_phrase(mock_response.status_code) << "\r\n";
      response << "Content-Type: application/json\r\n";
      response << "Content-Length: " << mock_response.body.size() << "\r\n";
      response << "Connection: close\r\n\r\n";
      response << mock_response.body;
      const std::string response_text = response.str();
      ::send(client_fd, response_text.data(), response_text.size(), 0);
    }

    ::close(client_fd);
    return !request.empty();
  }

  size_t parse_content_length(const std::string& headers) const {
    const std::string needle = "Content-Length:";
    size_t pos = headers.find(needle);
    if (pos == std::string::npos) {
      return 0;
    }
    pos += needle.size();
    size_t end = headers.find("\r\n", pos);
    std::string value = headers.substr(pos, end - pos);
    return static_cast<size_t>(std::stoul(value));
  }

  const char* reason_phrase(int status_code) const {
    if (status_code == 200) {
      return "OK";
    }
    if (status_code == 404) {
      return "Not Found";
    }
    if (status_code == 429) {
      return "Too Many Requests";
    }
    return "Error";
  }

  std::vector<MockHttpResponse> responses_;
  size_t next_response_ = 0;
  int listen_fd_;
  uint16_t port_ = 0;
  std::thread thread_;
  std::vector<std::string> requests_;
  std::vector<std::string> bodies_;
};

HttpClientOptions fast_retry_options() {
  HttpClientOptions options;
  options.timeout_seconds = 2;
  options.initial_retry_delay = std::chrono::milliseconds(1);
  options.max_retry_delay = std::chrono::milliseconds(1);
  options.retry_jitter_percent = 0;
  return options;
}

TEST(HttpClientTest, PostJsonSendsHeadersAndBody) {
  MockHttpServer server({{200, "{\"ok\":true}"}});
  HttpClient client(fast_retry_options());

  HttpResponse response = client.post_json(server.url() + "/api/v1/devices/register", "{\"x\":1}");
  ASSERT_EQ(response.curl_code, CURLE_OK);
  ASSERT_EQ(response.status_code, 200);
  EXPECT_EQ(response.body, "{\"ok\":true}");

  server.wait();
  ASSERT_EQ(server.request_count(), 1);
  EXPECT_NE(server.request(0).find("POST /api/v1/devices/register "), std::string::npos);
  EXPECT_NE(server.request(0).find("Content-Type: application/json"), std::string::npos);
  EXPECT_NE(server.request(0).find("Accept: application/json"), std::string::npos);
  EXPECT_EQ(server.body(0), "{\"x\":1}");
}

TEST(HttpClientTest, RetriesGetOnRateLimit) {
  MockHttpServer server({{429, "{\"error\":\"slow down\"}"}, {200, "template: true\n"}});
  HttpClient client(fast_retry_options());

  HttpResponse response = client.get_text(server.url() + "/configs/recorder.yaml");
  ASSERT_EQ(response.curl_code, CURLE_OK);
  ASSERT_EQ(response.status_code, 200);
  EXPECT_EQ(response.body, "template: true\n");

  server.wait();
  ASSERT_EQ(server.request_count(), 2);
  EXPECT_NE(server.request(0).find("GET /configs/recorder.yaml "), std::string::npos);
  EXPECT_NE(server.request(1).find("GET /configs/recorder.yaml "), std::string::npos);
}

TEST(HttpClientTest, DoesNotRetryClientError) {
  MockHttpServer server({{404, "{\"error\":\"missing\"}"}});
  HttpClient client(fast_retry_options());

  HttpResponse response = client.get_text(server.url() + "/configs/missing.yaml");
  ASSERT_EQ(response.curl_code, CURLE_OK);
  EXPECT_EQ(response.status_code, 404);
  EXPECT_EQ(response.body, "{\"error\":\"missing\"}");

  server.wait();
  EXPECT_EQ(server.request_count(), 1);
}

}  // namespace test
}  // namespace config
}  // namespace axon
