// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include <boost/filesystem.hpp>
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include "transfer_config.hpp"

namespace fs = boost::filesystem;

class TransferConfigTest : public ::testing::Test {
protected:
  void SetUp() override {
    test_dir_ = fs::temp_directory_path() / "axon_transfer_test";
    fs::create_directories(test_dir_);
    config_file_ = test_dir_ / "config.yaml";
  }

  void TearDown() override {
    fs::remove_all(test_dir_);
  }

  fs::path test_dir_;
  fs::path config_file_;
};

TEST_F(TransferConfigTest, LoadDefaults) {
  std::ofstream(config_file_.string()) << R"(
device_id: "test_device"
factory_id: "test_factory"
)";

  auto config = axon::transfer::load_config(config_file_.string());

  EXPECT_EQ(config.device_id, "test_device");
  EXPECT_EQ(config.factory_id, "test_factory");
  EXPECT_EQ(config.ws.url, "ws://localhost:8090/transfer");
  EXPECT_EQ(config.scanner.data_dir, "/tmp/axon/recording");
  EXPECT_TRUE(config.scanner.require_json_sidecar);
  EXPECT_EQ(config.scanner.completion_marker_suffix, ".done");
  EXPECT_EQ(config.scanner.min_ready_age_ms.count(), 0);
  EXPECT_EQ(config.uploader.state_db_path, "/tmp/axon/transfer/transfer_state.db");
  EXPECT_EQ(config.uploader.failed_uploads_dir, "/tmp/axon/transfer/failed_uploads/");
  EXPECT_EQ(config.uploader.num_workers, 2);
  EXPECT_EQ(config.uploader.cleanup_retry.max_retries, 5);
  EXPECT_EQ(config.uploader.cleanup_retry.initial_delay_ms.count(), 30000);
  EXPECT_EQ(config.uploader.cleanup_retry.backoff_multiplier, 2.0);
  EXPECT_EQ(config.uploader.cleanup_retry.max_delay_ms.count(), 600000);
}

TEST_F(TransferConfigTest, LoadFullConfig) {
  std::ofstream(config_file_.string()) << R"(
device_id: "robot_01"
factory_id: "factory_a"
ws:
  url: "ws://fleet.example.com:9090/transfer"
  ping_interval_ms: 15000
  reconnect:
    initial_delay_ms: 500
    max_delay_ms: 30000
scanner:
  data_dir: "/data/recordings"
  require_json_sidecar: false
  completion_marker_suffix: ".ready"
  min_ready_age_ms: 2500
uploader:
  num_workers: 4
  cleanup_retry:
    max_retries: 9
    initial_delay_ms: 5000
    backoff_multiplier: 1.5
    max_delay_ms: 120000
  s3:
    bucket: "test-bucket"
    region: "us-west-2"
)";

  auto config = axon::transfer::load_config(config_file_.string());

  EXPECT_EQ(config.device_id, "robot_01");
  EXPECT_EQ(config.factory_id, "factory_a");
  EXPECT_EQ(config.ws.url, "ws://fleet.example.com:9090/transfer");
  EXPECT_EQ(config.ws.ping_interval_ms.count(), 15000);
  EXPECT_EQ(config.ws.reconnect.initial_delay_ms.count(), 500);
  EXPECT_EQ(config.ws.reconnect.max_delay_ms.count(), 30000);
  EXPECT_EQ(config.scanner.data_dir, "/data/recordings");
  EXPECT_FALSE(config.scanner.require_json_sidecar);
  EXPECT_EQ(config.scanner.completion_marker_suffix, ".ready");
  EXPECT_EQ(config.scanner.min_ready_age_ms.count(), 2500);
  EXPECT_EQ(config.uploader.num_workers, 4);
  EXPECT_EQ(config.uploader.cleanup_retry.max_retries, 9);
  EXPECT_EQ(config.uploader.cleanup_retry.initial_delay_ms.count(), 5000);
  EXPECT_EQ(config.uploader.cleanup_retry.backoff_multiplier, 1.5);
  EXPECT_EQ(config.uploader.cleanup_retry.max_delay_ms.count(), 120000);
  EXPECT_EQ(config.uploader.s3.bucket, "test-bucket");
  EXPECT_EQ(config.uploader.s3.region, "us-west-2");
}

TEST_F(TransferConfigTest, EnvironmentVariableOverride) {
  std::ofstream(config_file_.string()) << R"(
device_id: "device_from_yaml"
factory_id: "factory_from_yaml"
ws:
  url: "ws://old.example.com"
scanner:
  data_dir: "/old/data"
uploader:
  s3:
    bucket: "yaml-bucket"
)";

  setenv("AXON_DEVICE_ID", "device_from_env", 1);
  setenv("AXON_TRANSFER_WS_URL", "ws://new.example.com", 1);
  setenv("AXON_TRANSFER_DATA_DIR", "/new/data", 1);
  setenv("AXON_S3_BUCKET", "env-bucket", 1);

  auto config = axon::transfer::load_config(config_file_.string());

  EXPECT_EQ(config.device_id, "device_from_env");
  EXPECT_EQ(config.ws.url, "ws://new.example.com");
  EXPECT_EQ(config.scanner.data_dir, "/new/data");
  EXPECT_EQ(config.uploader.s3.bucket, "env-bucket");

  unsetenv("AXON_DEVICE_ID");
  unsetenv("AXON_TRANSFER_WS_URL");
  unsetenv("AXON_TRANSFER_DATA_DIR");
  unsetenv("AXON_S3_BUCKET");
}

TEST_F(TransferConfigTest, MissingConfigFile) {
  EXPECT_THROW(axon::transfer::load_config("/nonexistent/path/config.yaml"), std::exception);
}
