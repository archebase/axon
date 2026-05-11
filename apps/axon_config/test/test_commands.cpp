// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

#include "commands.hpp"

namespace fs = std::filesystem;

namespace axon {
namespace config {
namespace test {

class MockKeystoneServer {
public:
  MockKeystoneServer(int status_code, std::string response_body)
      : status_code_(status_code)
      , response_body_(std::move(response_body))
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
      serve_once();
    });
  }

  ~MockKeystoneServer() {
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

  const std::string& last_request() const {
    return last_request_;
  }

  const std::string& last_body() const {
    return last_body_;
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

  void serve_once() {
    int client_fd = ::accept(listen_fd_, nullptr, nullptr);
    if (client_fd < 0) {
      return;
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

      size_t body_start = header_end + 4;
      size_t content_length = parse_content_length(request.substr(0, header_end));
      if (request.size() >= body_start + content_length) {
        break;
      }
    }

    if (!request.empty()) {
      last_request_ = request;
      size_t header_end = request.find("\r\n\r\n");
      if (header_end != std::string::npos) {
        size_t body_start = header_end + 4;
        size_t content_length = parse_content_length(request.substr(0, header_end));
        last_body_ = request.substr(body_start, content_length);
      }

      std::ostringstream response;
      response << "HTTP/1.1 " << status_code_ << " " << reason_phrase() << "\r\n";
      response << "Content-Type: application/json\r\n";
      response << "Content-Length: " << response_body_.size() << "\r\n";
      response << "Connection: close\r\n\r\n";
      response << response_body_;
      std::string response_text = response.str();
      ::send(client_fd, response_text.data(), response_text.size(), 0);
    }

    ::close(client_fd);
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

  const char* reason_phrase() const {
    if (status_code_ == 201) {
      return "Created";
    }
    if (status_code_ == 404) {
      return "Not Found";
    }
    return "Error";
  }

  int status_code_;
  std::string response_body_;
  int listen_fd_;
  uint16_t port_ = 0;
  std::thread thread_;
  std::string last_request_;
  std::string last_body_;
};

class CommandsTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Use a temporary test directory
    std::string dir_name = "axon_commands_test_" + std::to_string(std::time(nullptr));
    test_dir_ = fs::temp_directory_path() / dir_name;
    fs::create_directory(test_dir_);

    // Set config dir to test directory by setting environment
    // For testing, we use the default path and create it
    default_config_dir_ = fs::temp_directory_path();
    default_config_dir_ /= "axon_config_" + std::to_string(std::time(nullptr));
    fs::create_directory(default_config_dir_);
  }

  void TearDown() override {
    // Clean up test directories
    if (fs::exists(test_dir_)) {
      fs::remove_all(test_dir_);
    }
    if (fs::exists(default_config_dir_)) {
      fs::remove_all(default_config_dir_);
    }
  }

  // Helper to create a test file
  void create_test_file(
    const fs::path& dir, const std::string& rel_path, const std::string& content
  ) {
    fs::path file_path = dir;
    file_path /= rel_path;
    fs::create_directories(file_path.parent_path());

    std::ofstream file(file_path);
    file << content;
    file.close();
  }

  fs::path test_dir_;
  fs::path default_config_dir_;
};

// =============================================================================
// Test: execute command parsing - basic smoke tests
// =============================================================================

TEST(CommandsExecuteTest, HandlesHelpCommand) {
  Commands commands;
  char argv0[] = "axon_config";
  char* argv[] = {argv0};
  int result = commands.execute(1, argv);

  EXPECT_EQ(result, 0);  // help returns 0
}

TEST(CommandsExecuteTest, HandlesUnknownCommand) {
  Commands commands;
  char argv0[] = "axon_config";
  char argv1[] = "unknown";
  char* argv[] = {argv0, argv1};
  int result = commands.execute(2, argv);

  EXPECT_EQ(result, 1);  // unknown command returns error
}

TEST(CommandsRegisterTest, BuildsKeystoneRegisterPayload) {
  EXPECT_EQ(
    Commands::build_register_payload_for_test("Factory Shanghai", "SynGloves"),
    "{\"factory\":\"Factory Shanghai\",\"robot_type\":\"SynGloves\"}"
  );
}

TEST(CommandsRegisterTest, EscapesRegisterPayloadStrings) {
  EXPECT_EQ(
    Commands::build_register_payload_for_test("Factory \"A\"", "Type\\One\n"),
    "{\"factory\":\"Factory \\\"A\\\"\",\"robot_type\":\"Type\\\\One\\n\"}"
  );
}

TEST(CommandsRegisterTest, BuildsRegisterUrlWithoutDuplicateSlash) {
  EXPECT_EQ(
    Commands::build_register_url_for_test("http://keystone:8080/"),
    "http://keystone:8080/api/v1/devices/register"
  );
}

TEST(CommandsRegisterTest, RejectsMissingFactoryBeforeNetworkCall) {
  unsetenv("AXON_KEYSTONE_URL");
  Commands commands;
  char argv0[] = "axon_config";
  char argv1[] = "register";
  char argv2[] = "--robot-type";
  char argv3[] = "SynGloves";
  char argv4[] = "--keystone-url";
  char argv5[] = "http://127.0.0.1:1";
  char* argv[] = {argv0, argv1, argv2, argv3, argv4, argv5};

  EXPECT_EQ(commands.execute(6, argv), 1);
}

TEST(CommandsRegisterTest, RejectsMissingKeystoneUrlBeforeNetworkCall) {
  unsetenv("AXON_KEYSTONE_URL");
  Commands commands;
  char argv0[] = "axon_config";
  char argv1[] = "register";
  char argv2[] = "--factory";
  char argv3[] = "Factory Shanghai";
  char argv4[] = "--robot-type";
  char argv5[] = "SynGloves";
  char* argv[] = {argv0, argv1, argv2, argv3, argv4, argv5};

  EXPECT_EQ(commands.execute(6, argv), 1);
}

TEST(CommandsRegisterTest, RejectsUnknownRegisterOption) {
  Commands commands;
  char argv0[] = "axon_config";
  char argv1[] = "register";
  char argv2[] = "--factory";
  char argv3[] = "Factory Shanghai";
  char argv4[] = "--robot-type";
  char argv5[] = "SynGloves";
  char argv6[] = "--bad-option";
  char* argv[] = {argv0, argv1, argv2, argv3, argv4, argv5, argv6};

  EXPECT_EQ(commands.execute(7, argv), 1);
}

TEST(CommandsRegisterTest, PostsRegistrationRequestToKeystone) {
  unsetenv("AXON_KEYSTONE_URL");
  MockKeystoneServer server(
    201,
    "{\"device_id\":\"AB-F0001-T0003-000001\",\"factory\":\"Factory Shanghai\","
    "\"factory_id\":\"1\",\"robot_type\":\"SynGloves\",\"robot_type_id\":\"3\","
    "\"robot_id\":\"9\"}"
  );
  Commands commands;
  std::vector<std::string> args = {
    "axon_config",
    "register",
    "--factory",
    "Factory Shanghai",
    "--robot-type",
    "SynGloves",
    "--keystone-url",
    server.url(),
  };
  std::vector<char*> argv;
  for (auto& arg : args) {
    argv.push_back(arg.data());
  }

  EXPECT_EQ(commands.execute(static_cast<int>(argv.size()), argv.data()), 0);
  server.wait();
  EXPECT_NE(server.last_request().find("POST /api/v1/devices/register "), std::string::npos);
  EXPECT_NE(server.last_request().find("Content-Type: application/json"), std::string::npos);
  EXPECT_EQ(server.last_body(), "{\"factory\":\"Factory Shanghai\",\"robot_type\":\"SynGloves\"}");
}

TEST(CommandsRegisterTest, FailsWhenKeystoneRejectsRegistration) {
  unsetenv("AXON_KEYSTONE_URL");
  MockKeystoneServer server(404, "{\"error\":\"factory not found\"}");
  Commands commands;
  std::vector<std::string> args = {
    "axon_config",
    "register",
    "--factory",
    "Missing Factory",
    "--robot-type",
    "SynGloves",
    "--keystone-url",
    server.url(),
  };
  std::vector<char*> argv;
  for (auto& arg : args) {
    argv.push_back(arg.data());
  }

  EXPECT_EQ(commands.execute(static_cast<int>(argv.size()), argv.data()), 1);
  server.wait();
  EXPECT_EQ(server.last_body(), "{\"factory\":\"Missing Factory\",\"robot_type\":\"SynGloves\"}");
}

}  // namespace test
}  // namespace config
}  // namespace axon
