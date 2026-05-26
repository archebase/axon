// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include <boost/filesystem.hpp>
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <chrono>
#include <memory>
#include <string>

#include "upload_observability.hpp"
#include "upload_state_manager.hpp"

namespace fs = boost::filesystem;

class UploadObservabilityTest : public ::testing::Test {
protected:
  void SetUp() override {
    test_dir_ = fs::temp_directory_path() /
                ("axon_transfer_observability_test_" +
                 std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::create_directories(test_dir_);
    state_manager_ =
      std::make_unique<axon::uploader::UploadStateManager>((test_dir_ / "state.db").string());
  }

  void TearDown() override {
    fs::remove_all(test_dir_);
  }

  axon::uploader::UploadRecord insert_record(
    const std::string& task_id, uint64_t size_bytes = 1024
  ) {
    axon::uploader::UploadRecord record;
    record.file_path = (test_dir_ / (task_id + ".mcap")).string();
    record.json_path = (test_dir_ / (task_id + ".json")).string();
    record.s3_key = "factory/device/date/" + task_id + ".mcap";
    record.task_id = task_id;
    record.factory_id = "factory";
    record.device_id = "device";
    record.file_size_bytes = size_bytes;
    record.checksum_sha256 = std::string(64, 'a');
    EXPECT_TRUE(state_manager_->insert(record));
    return record;
  }

  nlohmann::json find_upload(const nlohmann::json& uploads, const std::string& task_id) {
    for (const auto& upload : uploads) {
      if (upload.value("task_id", "") == task_id) {
        return upload;
      }
    }
    return nullptr;
  }

  fs::path test_dir_;
  std::unique_ptr<axon::uploader::UploadStateManager> state_manager_;
};

TEST_F(UploadObservabilityTest, BuildsStatusForAllUploadStatesAndRedactsSecrets) {
  insert_record("task_pending", 100);
  auto active = insert_record("task_active", 200);
  auto retry_wait = insert_record("task_retry", 300);
  auto completed = insert_record("task_completed", 400);
  auto failed = insert_record("task_failed", 500);

  ASSERT_TRUE(state_manager_->updateStatus(active.file_path, axon::uploader::UploadStatus::ACTIVE));
  ASSERT_TRUE(
    state_manager_->markRetryWait(retry_wait.file_path, "2026-05-26T10:00:00Z", "network timeout")
  );
  ASSERT_TRUE(state_manager_->markCompleted(completed.file_path));
  ASSERT_TRUE(state_manager_->markFailed(failed.file_path, "access_key=secret-token"));

  axon::transfer::TransferConfig config;
  config.uploader.state_db_path = (test_dir_ / "state.db").string();
  config.uploader.s3.bucket = "ops-bucket";
  config.uploader.s3.endpoint_url = "https://user:secret@s3.example.com/path?token=hidden";
  config.uploader.s3.region = "us-west-2";
  config.uploader.s3.access_key = "access-key-value";
  config.uploader.s3.secret_key = "secret-key-value";

  axon::uploader::UploaderStats stats;
  stats.bytes_uploaded = 4096;
  stats.current_upload_bytes_per_sec = 2048;
  stats.files_recovered_from_state = 2;

  auto status = axon::transfer::build_upload_observability_status(config, *state_manager_, stats);

  EXPECT_EQ(status["pending_count"], 1);
  EXPECT_EQ(status["active_count"], 1);
  EXPECT_EQ(status["retry_wait_count"], 1);
  EXPECT_EQ(status["completed_count"], 1);
  EXPECT_EQ(status["failed_count"], 1);
  EXPECT_EQ(status["status_counts"]["completed"], 1);
  EXPECT_EQ(status["progress"]["current_bytes_per_sec"], 2048);
  EXPECT_EQ(status["progress"]["bytes_uploaded"], 4096);
  EXPECT_EQ(status["progress"]["tracked_bytes"], 1500);
  EXPECT_EQ(status["destination"]["bucket"], "ops-bucket");
  EXPECT_EQ(status["destination"]["endpoint_url"], "https://s3.example.com/path");
  EXPECT_EQ(status["recovery"]["recovered_from_state"], true);
  EXPECT_EQ(status["recovery"]["recovered_count"], 2);

  const auto retry_upload = find_upload(status["uploads"], "task_retry");
  ASSERT_FALSE(retry_upload.is_null());
  EXPECT_EQ(retry_upload["status"], "retry-wait");
  EXPECT_EQ(retry_upload["retry_count"], 0);
  EXPECT_EQ(retry_upload["last_error_category"], "timeout");
  EXPECT_EQ(retry_upload["next_retry_at"], "2026-05-26T10:00:00Z");

  const auto failed_upload = find_upload(status["uploads"], "task_failed");
  ASSERT_FALSE(failed_upload.is_null());
  EXPECT_EQ(failed_upload["status"], "failed");
  EXPECT_EQ(failed_upload["last_error"], "[REDACTED]");
  EXPECT_EQ(failed_upload["last_error_category"], "authorization");

  const std::string dumped = status.dump();
  EXPECT_EQ(dumped.find("secret-token"), std::string::npos);
  EXPECT_EQ(dumped.find("access-key-value"), std::string::npos);
  EXPECT_EQ(dumped.find("secret-key-value"), std::string::npos);
  EXPECT_EQ(dumped.find("token=hidden"), std::string::npos);
}
