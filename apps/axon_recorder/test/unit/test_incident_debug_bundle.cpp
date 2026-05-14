// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

#include "../src/core/incident_debug_bundle.hpp"

namespace fs = std::filesystem;
using namespace axon::recorder;

class IncidentDebugBundleTest : public ::testing::Test {
protected:
  void SetUp() override {
    test_dir_ = fs::temp_directory_path() /
                ("incident_bundle_test_" +
                 std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::create_directories(test_dir_);
  }

  void TearDown() override {
    if (fs::exists(test_dir_)) {
      fs::remove_all(test_dir_);
    }
  }

  fs::path write_file(const std::string& name, const std::string& content) {
    fs::path path = test_dir_ / name;
    std::ofstream out(path);
    out << content;
    return path;
  }

  fs::path test_dir_;
};

TEST_F(IncidentDebugBundleTest, CreatesBundleManifestAndRedactsSensitiveFields) {
  fs::path mcap_path = write_file("task_001.mcap", "mcap bytes");

  TaskConfig task_config;
  task_config.task_id = "task_001";
  task_config.device_id = "robot_01";
  task_config.scene = "warehouse";
  task_config.user_token = "must_not_appear";

  RecorderConfig recorder_config;
  recorder_config.recording.sidecar_json_enabled = false;
  recorder_config.rpc.ws_client.auth_token = "also_must_not_appear";

  IncidentDebugBundleRequest request;
  request.config.enabled = true;
  request.config.directory = (test_dir_ / "bundles").string();
  request.mcap_path = mcap_path.string();
  request.mcap_file_size = fs::file_size(mcap_path);
  request.sidecar_enabled = false;
  request.sidecar_generated = false;
  request.task_config = &task_config;
  request.recorder_config = &recorder_config;
  request.diagnostic_snapshot = {
    {"status", "critical"},
    {"api_token", "secret-token"},
    {"nested", {{"access_key", "access-key-value"}, {"safe", "visible"}}}
  };

  IncidentDebugBundleWriter writer;
  auto result = writer.create(request);

  ASSERT_TRUE(result.success);
  ASSERT_TRUE(result.created);
  EXPECT_TRUE(fs::exists(fs::path(result.bundle_path) / "recording.mcap"));
  ASSERT_TRUE(fs::exists(result.manifest_path));

  std::ifstream input(result.manifest_path);
  nlohmann::json manifest;
  input >> manifest;

  EXPECT_EQ(manifest["artifacts"]["sidecar"]["enabled"], false);
  EXPECT_EQ(manifest["config"]["task_config"]["task_id"], "task_001");
  EXPECT_EQ(manifest["diagnostics"]["api_token"], "[REDACTED]");
  EXPECT_EQ(manifest["diagnostics"]["nested"]["access_key"], "[REDACTED]");
  EXPECT_EQ(manifest["diagnostics"]["nested"]["safe"], "visible");

  const std::string manifest_text = manifest.dump();
  EXPECT_EQ(manifest_text.find("must_not_appear"), std::string::npos);
  EXPECT_EQ(manifest_text.find("also_must_not_appear"), std::string::npos);
  EXPECT_EQ(manifest_text.find("secret-token"), std::string::npos);
  EXPECT_EQ(manifest_text.find("access-key-value"), std::string::npos);
}

TEST_F(IncidentDebugBundleTest, FailureDoesNotModifyOriginalMcap) {
  fs::path mcap_path = write_file("task_002.mcap", "original mcap bytes");
  fs::path not_a_directory = write_file("bundle-parent", "plain file");

  IncidentDebugBundleRequest request;
  request.config.enabled = true;
  request.config.directory = not_a_directory.string();
  request.mcap_path = mcap_path.string();
  request.mcap_file_size = fs::file_size(mcap_path);

  IncidentDebugBundleWriter writer;
  auto result = writer.create(request);

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.created);
  EXPECT_TRUE(fs::exists(mcap_path));
  std::ifstream input(mcap_path);
  std::string content((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
  EXPECT_EQ(content, "original mcap bytes");
}
