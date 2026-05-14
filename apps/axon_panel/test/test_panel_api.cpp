// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <set>
#include <string>
#include <thread>

#include "httplib.h"
#include "nlohmann/json.hpp"
#include "panel_api.hpp"

namespace fs = std::filesystem;
using json = nlohmann::json;

namespace axon {
namespace panel {
namespace test {

namespace {

json parse_json_response(const httplib::Result& response) {
  EXPECT_TRUE(response);
  if (!response) {
    return json::object();
  }
  return json::parse(response->body);
}

class PanelApiTest : public ::testing::Test {
protected:
  void SetUp() override {
    test_root_ = fs::temp_directory_path() /
                 ("axon_panel_api_test_" +
                  std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    config_dir_ = test_root_ / "config";
    recording_dir_ = test_root_ / "recordings";
    home_dir_ = test_root_ / "home";
    fs::create_directories(config_dir_);
    fs::create_directories(recording_dir_);
    fs::create_directories(home_dir_);

    const char* home = std::getenv("HOME");
    original_home_ = home == nullptr ? "" : std::string(home);
    had_home_ = home != nullptr;
    setenv("HOME", home_dir_.string().c_str(), 1);

    PanelApiOptions options;
    options.config_dir = config_dir_.string();
    options.recording_dir = recording_dir_.string();
    register_panel_api(server_, options);

    port_ = server_.bind_to_any_port("127.0.0.1");
    ASSERT_GT(port_, 0);
    server_thread_ = std::thread([this]() {
      server_.listen_after_bind();
    });
    wait_until_ready();
  }

  void TearDown() override {
    server_.stop();
    if (server_thread_.joinable()) {
      server_thread_.join();
    }

    if (had_home_) {
      setenv("HOME", original_home_.c_str(), 1);
    } else {
      unsetenv("HOME");
    }

    if (fs::exists(test_root_)) {
      fs::remove_all(test_root_);
    }
  }

  httplib::Client client() {
    httplib::Client client("127.0.0.1", port_);
    client.set_connection_timeout(2, 0);
    client.set_read_timeout(2, 0);
    client.set_write_timeout(2, 0);
    return client;
  }

  void write_file(const fs::path& path, const std::string& content) {
    fs::create_directories(path.parent_path());
    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    ASSERT_TRUE(file);
    file << content;
  }

  json upload_file(
    const std::string& filename, const std::string& content, const std::string& category,
    const std::string& target_path = ""
  ) {
    httplib::MultipartFormDataItems items = {
      {"file", content, filename, "application/octet-stream"},
      {"category", category, "", ""},
    };
    if (!target_path.empty()) {
      items.push_back({"path", target_path, "", ""});
    }

    auto response = client().Post("/api/panel/config/files", items);
    EXPECT_TRUE(response);
    if (response) {
      EXPECT_EQ(response->status, 200);
    }
    return parse_json_response(response);
  }

  void wait_until_ready() {
    for (int i = 0; i < 50; ++i) {
      auto response = client().Get("/api/panel/config/status");
      if (response && response->status == 200) {
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    FAIL() << "panel API server did not become ready";
  }

  fs::path test_root_;
  fs::path config_dir_;
  fs::path recording_dir_;
  fs::path home_dir_;

private:
  httplib::Server server_;
  std::thread server_thread_;
  int port_ = 0;
  std::string original_home_;
  bool had_home_ = false;
};

}  // namespace

TEST_F(PanelApiTest, UploadUpdatePreviewAndDiffConfigFiles) {
  auto status_response = client().Get("/api/panel/config/status");
  ASSERT_TRUE(status_response);
  ASSERT_EQ(status_response->status, 200);
  json status = parse_json_response(status_response);
  EXPECT_TRUE(status.value("success", false));
  EXPECT_EQ(status.value("config_dir", ""), config_dir_.string());

  json first_upload = upload_file("robot.urdf", "<robot name=\"demo\"></robot>\n", "urdf");
  ASSERT_TRUE(first_upload.value("success", false));
  EXPECT_EQ(first_upload["file"].value("path", ""), "robot/robot.urdf");
  EXPECT_EQ(first_upload["file"].value("category", ""), "urdf");
  EXPECT_FALSE(first_upload["history"].value("replaced", true));

  json second_upload =
    upload_file("robot.urdf", "<robot name=\"demo2\"></robot>\n", "urdf", "robot/robot.urdf");
  ASSERT_TRUE(second_upload.value("success", false));
  EXPECT_TRUE(second_upload["history"].value("replaced", false));
  const std::string history_id = second_upload["history"].value("id", "");
  ASSERT_FALSE(history_id.empty());

  auto file_response = client().Get("/api/panel/config/file?path=robot/robot.urdf");
  ASSERT_TRUE(file_response);
  ASSERT_EQ(file_response->status, 200);
  json file = parse_json_response(file_response);
  EXPECT_FALSE(file.value("binary", true));
  EXPECT_NE(file.value("content", "").find("demo2"), std::string::npos);

  auto diff_response = client().Get(("/api/panel/config/diff?history_id=" + history_id).c_str());
  ASSERT_TRUE(diff_response);
  ASSERT_EQ(diff_response->status, 200);
  json diff = parse_json_response(diff_response);
  ASSERT_TRUE(diff.contains("lines"));
  EXPECT_FALSE(diff["lines"].empty());
  EXPECT_EQ(diff.value("path", ""), "robot/robot.urdf");

  httplib::MultipartFormDataItems unsafe_items = {
    {"file", "<robot name=\"bad\"></robot>\n", "bad.urdf", "application/octet-stream"},
    {"path", "../escape.urdf", "", ""},
  };
  auto unsafe_response = client().Post("/api/panel/config/files", unsafe_items);
  ASSERT_TRUE(unsafe_response);
  EXPECT_EQ(unsafe_response->status, 400);
}

TEST_F(PanelApiTest, ScanAndInjectionToggleUseIsolatedHomeCache) {
  write_file(config_dir_ / "robot" / "type.txt", "AGV-500");

  auto scan_response = client().Post("/api/panel/config/scan?incremental=false", "", "text/plain");
  ASSERT_TRUE(scan_response);
  ASSERT_EQ(scan_response->status, 200);
  json scan = parse_json_response(scan_response);
  EXPECT_TRUE(scan.value("success", false));
  EXPECT_EQ(scan.value("file_count", 0), 1);
  EXPECT_TRUE(fs::exists(home_dir_ / ".axon" / "cache.mcap"));

  auto enable_response = client().Post("/api/panel/config/injection/enable", "", "text/plain");
  ASSERT_TRUE(enable_response);
  ASSERT_EQ(enable_response->status, 200);
  json enabled = parse_json_response(enable_response);
  EXPECT_TRUE(enabled["status"]["cache"].value("enabled", false));
  EXPECT_TRUE(fs::exists(home_dir_ / ".axon" / ".enabled"));

  auto disable_response = client().Post("/api/panel/config/injection/disable", "", "text/plain");
  ASSERT_TRUE(disable_response);
  ASSERT_EQ(disable_response->status, 200);
  json disabled = parse_json_response(disable_response);
  EXPECT_FALSE(disabled["status"]["cache"].value("enabled", true));
  EXPECT_FALSE(fs::exists(home_dir_ / ".axon" / ".enabled"));
}

TEST_F(PanelApiTest, TaskBatchOperationsRequireDiscoveredTaskIds) {
  write_file(recording_dir_ / "a" / "task.mcap", "mcap-a");
  write_file(recording_dir_ / "b" / "task.mcap", "mcap-b");
  write_file(recording_dir_ / "a" / "task.json", "{}");

  auto tasks_response = client().Get("/api/panel/tasks");
  ASSERT_TRUE(tasks_response);
  ASSERT_EQ(tasks_response->status, 200);
  json tasks_body = parse_json_response(tasks_response);
  ASSERT_EQ(tasks_body["tasks"].size(), 2);

  std::set<std::string> task_ids;
  for (const auto& task : tasks_body["tasks"]) {
    task_ids.insert(task.value("task_id", ""));
  }
  EXPECT_TRUE(task_ids.count("a/task.mcap") > 0);
  EXPECT_TRUE(task_ids.count("b/task.mcap") > 0);

  json invalid_body = {{"action", "queue_upload"}, {"task_ids", json::array({"missing.mcap"})}};
  auto invalid_response =
    client().Post("/api/panel/tasks/batch", invalid_body.dump(), "application/json");
  ASSERT_TRUE(invalid_response);
  ASSERT_EQ(invalid_response->status, 404);
  json invalid = parse_json_response(invalid_response);
  EXPECT_FALSE(invalid.value("success", true));

  json valid_body = {{"action", "queue_upload"}, {"task_ids", json::array({"a/task.mcap"})}};
  auto valid_response =
    client().Post("/api/panel/tasks/batch", valid_body.dump(), "application/json");
  ASSERT_TRUE(valid_response);
  ASSERT_EQ(valid_response->status, 200);
  json valid = parse_json_response(valid_response);
  EXPECT_TRUE(valid.value("success", false));
  EXPECT_EQ(valid.value("changed", 0), 1);

  bool queued = false;
  for (const auto& task : valid["tasks"]) {
    if (task.value("task_id", "") == "a/task.mcap") {
      queued = task.value("queued", false) && task.value("status", "") == "queued";
    }
  }
  EXPECT_TRUE(queued);
}

TEST_F(PanelApiTest, LoggingLevelValidationPersistsPanelPreference) {
  json invalid_body = {{"level", "trace"}};
  auto invalid_response =
    client().Post("/api/panel/logging/level", invalid_body.dump(), "application/json");
  ASSERT_TRUE(invalid_response);
  ASSERT_EQ(invalid_response->status, 400);

  json valid_body = {{"level", "debug"}};
  auto valid_response =
    client().Post("/api/panel/logging/level", valid_body.dump(), "application/json");
  ASSERT_TRUE(valid_response);
  ASSERT_EQ(valid_response->status, 200);
  json valid = parse_json_response(valid_response);
  EXPECT_EQ(valid["logging"].value("level", ""), "debug");

  auto logging_response = client().Get("/api/panel/logging");
  ASSERT_TRUE(logging_response);
  ASSERT_EQ(logging_response->status, 200);
  json logging = parse_json_response(logging_response);
  EXPECT_EQ(logging["logging"].value("level", ""), "debug");
}

}  // namespace test
}  // namespace panel
}  // namespace axon
