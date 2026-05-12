// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <nlohmann/json.hpp>
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
#include <utility>
#include <vector>

#include "commands.hpp"

namespace fs = std::filesystem;

namespace axon {
namespace config {
namespace test {

struct MockHttpResponse {
  int status_code;
  std::string body;
};

class MockKeystoneServer {
public:
  MockKeystoneServer(int status_code, std::string response_body)
      : MockKeystoneServer(std::vector<MockHttpResponse>{{status_code, std::move(response_body)}}) {
  }

  explicit MockKeystoneServer(std::vector<MockHttpResponse> responses)
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
    static const std::string empty;
    return requests_.empty() ? empty : requests_.back();
  }

  const std::string& last_body() const {
    static const std::string empty;
    return bodies_.empty() ? empty : bodies_.back();
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

      size_t body_start = header_end + 4;
      size_t content_length = parse_content_length(request.substr(0, header_end));
      if (request.size() >= body_start + content_length) {
        break;
      }
    }

    if (!request.empty()) {
      requests_.push_back(request);
      size_t header_end = request.find("\r\n\r\n");
      if (header_end != std::string::npos) {
        size_t body_start = header_end + 4;
        size_t content_length = parse_content_length(request.substr(0, header_end));
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
      std::string response_text = response.str();
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
    if (status_code == 201) {
      return "Created";
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

std::string read_text_file(const fs::path& path) {
  std::ifstream file(path);
  std::stringstream body;
  body << file.rdbuf();
  return body.str();
}

fs::path make_register_temp_dir(const std::string& prefix) {
  fs::path config_dir =
    fs::temp_directory_path() /
    (prefix + "_" + std::to_string(std::time(nullptr)) + "_" + std::to_string(getpid()));
  fs::remove_all(config_dir);
  fs::create_directories(config_dir);
  return config_dir;
}

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

TEST(CommandsRegisterTest, BuildsConfigUrlWithEncodedPathParts) {
  EXPECT_EQ(
    Commands::build_config_url_for_test(
      "http://keystone:8080/", "Factory Shanghai", "Syn/Gloves", "recorder.yaml"
    ),
    "http://keystone:8080/configs/Factory%20Shanghai/Syn%2FGloves/recorder.yaml"
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
  fs::path config_dir = make_register_temp_dir("axon_register_skip_config");

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
    "--config-dir",
    config_dir.string(),
    "--skip-config-download",
  };
  std::vector<char*> argv;
  for (auto& arg : args) {
    argv.push_back(arg.data());
  }

  EXPECT_EQ(commands.execute(static_cast<int>(argv.size()), argv.data()), 0);
  server.wait();
  EXPECT_EQ(server.request_count(), 1);
  EXPECT_NE(server.last_request().find("POST /api/v1/devices/register "), std::string::npos);
  EXPECT_NE(server.last_request().find("Content-Type: application/json"), std::string::npos);
  EXPECT_EQ(server.last_body(), "{\"factory\":\"Factory Shanghai\",\"robot_type\":\"SynGloves\"}");

  const auto device = nlohmann::json::parse(read_text_file(config_dir / "device.json"));
  EXPECT_EQ(device.at("device_id"), "AB-F0001-T0003-000001");
  EXPECT_EQ(device.at("factory"), "Factory Shanghai");
  EXPECT_EQ(device.at("robot_type"), "SynGloves");
  EXPECT_FALSE(fs::exists(config_dir / "templates" / "recorder.yaml.tpl"));
  EXPECT_FALSE(fs::exists(config_dir / "recorder.yaml"));

  fs::remove_all(config_dir);
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

TEST(CommandsRegisterTest, RetriesRegistrationOnTransientServerError) {
  unsetenv("AXON_KEYSTONE_URL");
  MockKeystoneServer server(
    std::vector<MockHttpResponse>{
      {500, "{\"error\":\"temporary failure\"}"},
      {201, "{\"device_id\":\"AB-F0001-T0003-000001\"}"},
    }
  );
  fs::path config_dir = make_register_temp_dir("axon_register_retry");

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
    "--config-dir",
    config_dir.string(),
    "--skip-config-download",
  };
  std::vector<char*> argv;
  for (auto& arg : args) {
    argv.push_back(arg.data());
  }

  EXPECT_EQ(commands.execute(static_cast<int>(argv.size()), argv.data()), 0);
  server.wait();

  EXPECT_EQ(server.request_count(), 2);
  EXPECT_NE(server.request(0).find("POST /api/v1/devices/register "), std::string::npos);
  EXPECT_NE(server.request(1).find("POST /api/v1/devices/register "), std::string::npos);
  EXPECT_EQ(server.body(0), "{\"factory\":\"Factory Shanghai\",\"robot_type\":\"SynGloves\"}");
  EXPECT_EQ(server.body(1), "{\"factory\":\"Factory Shanghai\",\"robot_type\":\"SynGloves\"}");

  const auto device = nlohmann::json::parse(read_text_file(config_dir / "device.json"));
  EXPECT_EQ(device.at("device_id"), "AB-F0001-T0003-000001");

  fs::remove_all(config_dir);
}

TEST(CommandsRegisterTest, DownloadsRecorderAndTransferConfigsAfterRegistration) {
  unsetenv("AXON_KEYSTONE_URL");
  MockKeystoneServer server(
    std::vector<MockHttpResponse>{
      {201,
       "{\"device_id\":\"AB-F0001-T0003-000001\",\"factory\":\"Factory Shanghai\","
       "\"factory_id\":\"1\",\"robot_type\":\"SynGloves\",\"robot_type_id\":\"3\","
       "\"robot_id\":\"9\"}"},
      {200,
       "device_id: \"{{ device_id }}\"\n"
       "robot_model: \"{{ robot_model }}\"\n"
       "robot_id: \"{{ robot_id }}\"\n"
       "rpc_url: \"{{ recorder_rpc_url }}\"\n"
       "nested_rpc_url: \"{{ endpoints.recorder_rpc_url }}\"\n"},
      {200,
       "factory_id: \"{{ factory_id }}\"\n"
       "robot_type_id: \"{{ robot_type_id }}\"\n"
       "ws_url: \"{{ transfer_ws_url }}\"\n"
       "keystone_url: \"{{ keystone_url }}\"\n"},
    }
  );
  fs::path config_dir = make_register_temp_dir("axon_register_config");
  std::ofstream(config_dir / "recorder.yaml") << "old_recorder: true\n";
  std::ofstream(config_dir / "transfer.yaml") << "old_transfer: true\n";

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
    "--config-dir",
    config_dir.string(),
  };
  std::vector<char*> argv;
  for (auto& arg : args) {
    argv.push_back(arg.data());
  }

  EXPECT_EQ(commands.execute(static_cast<int>(argv.size()), argv.data()), 0);
  server.wait();

  EXPECT_EQ(server.request_count(), 3);
  EXPECT_NE(server.request(0).find("POST /api/v1/devices/register "), std::string::npos);
  EXPECT_NE(
    server.request(1).find("GET /configs/Factory%20Shanghai/SynGloves/recorder.yaml "),
    std::string::npos
  );
  EXPECT_NE(
    server.request(2).find("GET /configs/Factory%20Shanghai/SynGloves/transfer.yaml "),
    std::string::npos
  );

  const std::string ws_base = "ws://" + server.url().substr(std::string("http://").size());

  EXPECT_EQ(
    read_text_file(config_dir / "templates" / "recorder.yaml.tpl"),
    "device_id: \"{{ device_id }}\"\n"
    "robot_model: \"{{ robot_model }}\"\n"
    "robot_id: \"{{ robot_id }}\"\n"
    "rpc_url: \"{{ recorder_rpc_url }}\"\n"
    "nested_rpc_url: \"{{ endpoints.recorder_rpc_url }}\"\n"
  );
  EXPECT_EQ(
    read_text_file(config_dir / "templates" / "transfer.yaml.tpl"),
    "factory_id: \"{{ factory_id }}\"\n"
    "robot_type_id: \"{{ robot_type_id }}\"\n"
    "ws_url: \"{{ transfer_ws_url }}\"\n"
    "keystone_url: \"{{ keystone_url }}\"\n"
  );
  EXPECT_EQ(
    read_text_file(config_dir / "recorder.yaml"),
    "device_id: \"AB-F0001-T0003-000001\"\n"
    "robot_model: \"SynGloves\"\n"
    "robot_id: \"9\"\n"
    "rpc_url: \"" +
      ws_base +
      "/rpc\"\n"
      "nested_rpc_url: \"" +
      ws_base + "/rpc\"\n"
  );
  EXPECT_EQ(
    read_text_file(config_dir / "transfer.yaml"),
    "factory_id: \"1\"\n"
    "robot_type_id: \"3\"\n"
    "ws_url: \"" +
      ws_base +
      "/transfer\"\n"
      "keystone_url: \"" +
      server.url() + "\"\n"
  );

  const auto device = nlohmann::json::parse(read_text_file(config_dir / "device.json"));
  EXPECT_EQ(device.at("device_id"), "AB-F0001-T0003-000001");
  EXPECT_EQ(device.at("factory_id"), "1");
  EXPECT_EQ(device.at("robot_id"), "9");
  EXPECT_EQ(device.at("keystone_url"), server.url());
  EXPECT_EQ(device.at("endpoints").at("recorder_rpc_url"), ws_base + "/rpc");
  EXPECT_EQ(device.at("endpoints").at("transfer_ws_url"), ws_base + "/transfer");

  fs::remove_all(config_dir);
}

TEST(CommandsRegisterTest, RetriesConfigDownloadOnRateLimit) {
  unsetenv("AXON_KEYSTONE_URL");
  MockKeystoneServer server(
    std::vector<MockHttpResponse>{
      {201, "{\"device_id\":\"AB-F0001-T0003-000001\",\"robot_id\":\"9\"}"},
      {429, "{\"error\":\"rate limited\"}"},
      {200, "device_id: \"{{ device_id }}\"\nrobot_id: \"{{ robot_id }}\"\n"},
      {200, "transfer_device: \"{{ device_id }}\"\n"},
    }
  );
  fs::path config_dir = make_register_temp_dir("axon_register_config_retry");

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
    "--config-dir",
    config_dir.string(),
  };
  std::vector<char*> argv;
  for (auto& arg : args) {
    argv.push_back(arg.data());
  }

  EXPECT_EQ(commands.execute(static_cast<int>(argv.size()), argv.data()), 0);
  server.wait();

  EXPECT_EQ(server.request_count(), 4);
  EXPECT_NE(
    server.request(1).find("GET /configs/Factory%20Shanghai/SynGloves/recorder.yaml "),
    std::string::npos
  );
  EXPECT_NE(
    server.request(2).find("GET /configs/Factory%20Shanghai/SynGloves/recorder.yaml "),
    std::string::npos
  );
  EXPECT_NE(
    server.request(3).find("GET /configs/Factory%20Shanghai/SynGloves/transfer.yaml "),
    std::string::npos
  );
  EXPECT_EQ(
    read_text_file(config_dir / "recorder.yaml"),
    "device_id: \"AB-F0001-T0003-000001\"\nrobot_id: \"9\"\n"
  );
  EXPECT_EQ(
    read_text_file(config_dir / "transfer.yaml"), "transfer_device: \"AB-F0001-T0003-000001\"\n"
  );

  fs::remove_all(config_dir);
}

TEST(CommandsRegisterTest, WarnsAndSkipsConfigDownloadWhenKeystoneReturns404) {
  unsetenv("AXON_KEYSTONE_URL");
  MockKeystoneServer server(
    std::vector<MockHttpResponse>{
      {201, "{\"device_id\":\"AB-F0001-T0003-000001\"}"},
      {404, "{\"error\":\"config not found\"}"},
    }
  );
  fs::path config_dir = make_register_temp_dir("axon_register_config_404");

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
    "--config-dir",
    config_dir.string(),
  };
  std::vector<char*> argv;
  for (auto& arg : args) {
    argv.push_back(arg.data());
  }

  EXPECT_EQ(commands.execute(static_cast<int>(argv.size()), argv.data()), 0);
  server.wait();

  EXPECT_EQ(server.request_count(), 2);
  EXPECT_TRUE(fs::exists(config_dir / "device.json"));
  EXPECT_FALSE(fs::exists(config_dir / "templates" / "recorder.yaml.tpl"));
  EXPECT_FALSE(fs::exists(config_dir / "templates" / "transfer.yaml.tpl"));
  EXPECT_FALSE(fs::exists(config_dir / "recorder.yaml"));
  EXPECT_FALSE(fs::exists(config_dir / "transfer.yaml"));

  fs::remove_all(config_dir);
}

TEST(CommandsRegisterTest, LeavesExistingConfigsUnchangedWhenTransferConfigReturns404) {
  unsetenv("AXON_KEYSTONE_URL");
  MockKeystoneServer server(
    std::vector<MockHttpResponse>{
      {201, "{\"device_id\":\"AB-F0001-T0003-000001\"}"},
      {200, "new_recorder: true\n"},
      {404, "{\"error\":\"transfer config not found\"}"},
    }
  );
  fs::path config_dir = make_register_temp_dir("axon_register_config_transfer_404");
  std::ofstream(config_dir / "recorder.yaml") << "old_recorder: true\n";
  std::ofstream(config_dir / "transfer.yaml") << "old_transfer: true\n";

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
    "--config-dir",
    config_dir.string(),
  };
  std::vector<char*> argv;
  for (auto& arg : args) {
    argv.push_back(arg.data());
  }

  EXPECT_EQ(commands.execute(static_cast<int>(argv.size()), argv.data()), 0);
  server.wait();

  EXPECT_EQ(server.request_count(), 3);
  EXPECT_TRUE(fs::exists(config_dir / "device.json"));
  EXPECT_FALSE(fs::exists(config_dir / "templates" / "recorder.yaml.tpl"));
  EXPECT_FALSE(fs::exists(config_dir / "templates" / "transfer.yaml.tpl"));
  EXPECT_EQ(read_text_file(config_dir / "recorder.yaml"), "old_recorder: true\n");
  EXPECT_EQ(read_text_file(config_dir / "transfer.yaml"), "old_transfer: true\n");

  fs::remove_all(config_dir);
}

TEST(CommandsRegisterTest, LeavesExistingConfigsUnchangedWhenTemplateRenderingFails) {
  unsetenv("AXON_KEYSTONE_URL");
  MockKeystoneServer server(
    std::vector<MockHttpResponse>{
      {201, "{\"device_id\":\"AB-F0001-T0003-000001\",\"robot_id\":\"9\"}"},
      {200, "device_id: \"{{ device_id }}\"\nmissing: \"{{ missing_value }}\"\n"},
      {200, "robot_id: \"{{ robot_id }}\"\n"},
    }
  );
  fs::path config_dir = make_register_temp_dir("axon_register_config_render_error");
  std::ofstream(config_dir / "recorder.yaml") << "old_recorder: true\n";
  std::ofstream(config_dir / "transfer.yaml") << "old_transfer: true\n";

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
    "--config-dir",
    config_dir.string(),
  };
  std::vector<char*> argv;
  for (auto& arg : args) {
    argv.push_back(arg.data());
  }

  EXPECT_EQ(commands.execute(static_cast<int>(argv.size()), argv.data()), 1);
  server.wait();

  EXPECT_EQ(server.request_count(), 3);
  EXPECT_TRUE(fs::exists(config_dir / "device.json"));
  EXPECT_FALSE(fs::exists(config_dir / "templates" / "recorder.yaml.tpl"));
  EXPECT_FALSE(fs::exists(config_dir / "templates" / "transfer.yaml.tpl"));
  EXPECT_EQ(read_text_file(config_dir / "recorder.yaml"), "old_recorder: true\n");
  EXPECT_EQ(read_text_file(config_dir / "transfer.yaml"), "old_transfer: true\n");

  fs::remove_all(config_dir);
}

TEST(CommandsRefreshTest, RefreshesConfigsFromSavedDeviceState) {
  unsetenv("AXON_KEYSTONE_URL");
  MockKeystoneServer server(
    std::vector<MockHttpResponse>{
      {200,
       "device_id: \"{{ device_id }}\"\n"
       "robot_id: \"{{ robot_id }}\"\n"
       "rpc_url: \"{{ recorder_rpc_url }}\"\n"},
      {200,
       "factory: \"{{ factory }}\"\n"
       "robot_type: \"{{ robot_type }}\"\n"
       "ws_url: \"{{ transfer_ws_url }}\"\n"},
    }
  );
  fs::path config_dir = make_register_temp_dir("axon_refresh_config");
  const std::string device_body =
    "{\n"
    "  \"device_id\": \"AB-F0001-T0003-000001\",\n"
    "  \"factory\": \"Factory Shanghai\",\n"
    "  \"factory_id\": \"1\",\n"
    "  \"robot_type\": \"SynGloves\",\n"
    "  \"robot_type_id\": \"3\",\n"
    "  \"robot_id\": \"9\",\n"
    "  \"keystone_url\": \"" +
    server.url() +
    "\",\n"
    "  \"registered_at\": \"2026-05-12T00:00:00Z\"\n"
    "}\n";
  std::ofstream(config_dir / "device.json") << device_body;
  std::ofstream(config_dir / "recorder.yaml") << "old_recorder: true\n";
  std::ofstream(config_dir / "transfer.yaml") << "old_transfer: true\n";

  Commands commands;
  std::vector<std::string> args = {
    "axon_config",
    "refresh",
    "--config-dir",
    config_dir.string(),
  };
  std::vector<char*> argv;
  for (auto& arg : args) {
    argv.push_back(arg.data());
  }

  EXPECT_EQ(commands.execute(static_cast<int>(argv.size()), argv.data()), 0);
  server.wait();

  const std::string ws_base = "ws://" + server.url().substr(std::string("http://").size());

  EXPECT_EQ(server.request_count(), 2);
  EXPECT_NE(
    server.request(0).find("GET /configs/Factory%20Shanghai/SynGloves/recorder.yaml "),
    std::string::npos
  );
  EXPECT_NE(
    server.request(1).find("GET /configs/Factory%20Shanghai/SynGloves/transfer.yaml "),
    std::string::npos
  );
  EXPECT_EQ(read_text_file(config_dir / "device.json"), device_body);
  EXPECT_EQ(
    read_text_file(config_dir / "recorder.yaml"),
    "device_id: \"AB-F0001-T0003-000001\"\n"
    "robot_id: \"9\"\n"
    "rpc_url: \"" +
      ws_base + "/rpc\"\n"
  );
  EXPECT_EQ(
    read_text_file(config_dir / "transfer.yaml"),
    "factory: \"Factory Shanghai\"\n"
    "robot_type: \"SynGloves\"\n"
    "ws_url: \"" +
      ws_base + "/transfer\"\n"
  );

  fs::remove_all(config_dir);
}

TEST(CommandsRefreshTest, FailsWhenDeviceStateIsMissing) {
  unsetenv("AXON_KEYSTONE_URL");
  fs::path config_dir = make_register_temp_dir("axon_refresh_missing_device");

  Commands commands;
  std::vector<std::string> args = {
    "axon_config",
    "refresh",
    "--config-dir",
    config_dir.string(),
  };
  std::vector<char*> argv;
  for (auto& arg : args) {
    argv.push_back(arg.data());
  }

  EXPECT_EQ(commands.execute(static_cast<int>(argv.size()), argv.data()), 1);

  fs::remove_all(config_dir);
}

TEST(CommandsRefreshTest, LeavesExistingConfigsUnchangedWhenTemplateReturns404) {
  unsetenv("AXON_KEYSTONE_URL");
  MockKeystoneServer server(
    std::vector<MockHttpResponse>{
      {404, "{\"error\":\"config not found\"}"},
    }
  );
  fs::path config_dir = make_register_temp_dir("axon_refresh_config_404");
  std::ofstream(config_dir / "device.json") << "{\n"
                                            << "  \"device_id\": \"AB-F0001-T0003-000001\",\n"
                                            << "  \"factory\": \"Factory Shanghai\",\n"
                                            << "  \"robot_type\": \"SynGloves\",\n"
                                            << "  \"robot_id\": \"9\",\n"
                                            << "  \"keystone_url\": \"" << server.url() << "\"\n"
                                            << "}\n";
  std::ofstream(config_dir / "recorder.yaml") << "old_recorder: true\n";
  std::ofstream(config_dir / "transfer.yaml") << "old_transfer: true\n";

  Commands commands;
  std::vector<std::string> args = {
    "axon_config",
    "refresh",
    "--config-dir",
    config_dir.string(),
  };
  std::vector<char*> argv;
  for (auto& arg : args) {
    argv.push_back(arg.data());
  }

  EXPECT_EQ(commands.execute(static_cast<int>(argv.size()), argv.data()), 0);
  server.wait();

  EXPECT_EQ(server.request_count(), 1);
  EXPECT_FALSE(fs::exists(config_dir / "templates" / "recorder.yaml.tpl"));
  EXPECT_FALSE(fs::exists(config_dir / "templates" / "transfer.yaml.tpl"));
  EXPECT_EQ(read_text_file(config_dir / "recorder.yaml"), "old_recorder: true\n");
  EXPECT_EQ(read_text_file(config_dir / "transfer.yaml"), "old_transfer: true\n");

  fs::remove_all(config_dir);
}

}  // namespace test
}  // namespace config
}  // namespace axon
