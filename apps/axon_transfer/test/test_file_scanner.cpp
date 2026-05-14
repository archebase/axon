// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include <boost/filesystem.hpp>
#include <gtest/gtest.h>

#include <ctime>
#include <fstream>
#include <memory>

#include "file_scanner.hpp"
#include "transfer_config.hpp"
#include "upload_state_manager.hpp"

namespace fs = boost::filesystem;

class FileScannerTest : public ::testing::Test {
protected:
  void SetUp() override {
    test_dir_ = fs::temp_directory_path() / "axon_transfer_scanner_test";
    fs::create_directories(test_dir_);

    axon::transfer::ScannerConfig config;
    config.data_dir = test_dir_.string();
    config.require_json_sidecar = true;

    db_path_ = test_dir_ / "state.db";
  }

  void TearDown() override {
    fs::remove_all(test_dir_);
  }

  void create_file(const fs::path& path, const std::string& content = "test") {
    std::ofstream(path.string()) << content;
  }

  void mark_mcap_done(const std::string& task_id) {
    create_file(test_dir_ / (task_id + ".mcap.done"), "");
  }

  fs::path test_dir_;
  fs::path db_path_;
};

TEST_F(FileScannerTest, FindExistingFiles) {
  create_file(test_dir_ / "task_001.mcap");
  create_file(test_dir_ / "task_001.json");

  auto state_manager = std::make_shared<axon::uploader::UploadStateManager>(db_path_.string());
  axon::transfer::FileScanner scanner({test_dir_.string(), true}, state_manager);

  auto result = scanner.find("task_001");

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->task_id, "task_001");
  EXPECT_EQ(result->mcap_path.filename().string(), "task_001.mcap");
  EXPECT_EQ(result->json_path.filename().string(), "task_001.json");
  EXPECT_EQ(result->checksum_sha256.size(), 64);
}

TEST_F(FileScannerTest, FindMissingMcap) {
  create_file(test_dir_ / "task_001.json");

  auto state_manager = std::make_shared<axon::uploader::UploadStateManager>(db_path_.string());
  axon::transfer::FileScanner scanner({test_dir_.string(), true}, state_manager);

  auto result = scanner.find("task_001");

  EXPECT_FALSE(result.has_value());
}

TEST_F(FileScannerTest, FindMissingJson) {
  create_file(test_dir_ / "task_001.mcap");

  auto state_manager = std::make_shared<axon::uploader::UploadStateManager>(db_path_.string());
  axon::transfer::FileScanner scanner({test_dir_.string(), true}, state_manager);

  auto result = scanner.find("task_001");

  EXPECT_FALSE(result.has_value());
}

TEST_F(FileScannerTest, FindWithoutJsonRequired) {
  create_file(test_dir_ / "task_001.mcap");
  mark_mcap_done("task_001");

  auto state_manager = std::make_shared<axon::uploader::UploadStateManager>(db_path_.string());
  axon::transfer::FileScanner scanner({test_dir_.string(), false}, state_manager);

  auto result = scanner.find("task_001");

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->task_id, "task_001");
  EXPECT_TRUE(result->json_path.empty());
  EXPECT_EQ(result->completion_marker_path.filename().string(), "task_001.mcap.done");
}

TEST_F(FileScannerTest, FindWithoutJsonRequiredSkipsMissingMarker) {
  create_file(test_dir_ / "task_001.mcap");

  auto state_manager = std::make_shared<axon::uploader::UploadStateManager>(db_path_.string());
  axon::transfer::FileScanner scanner({test_dir_.string(), false}, state_manager);

  auto result = scanner.find("task_001");

  EXPECT_FALSE(result.has_value());
}

TEST_F(FileScannerTest, FindSkipsMcapModifiedAfterMarker) {
  const auto mcap_path = test_dir_ / "task_001.mcap";
  const auto marker_path = test_dir_ / "task_001.mcap.done";
  create_file(mcap_path);
  create_file(marker_path, "");

  const std::time_t now = std::time(nullptr);
  fs::last_write_time(marker_path, now);
  fs::last_write_time(mcap_path, now + 1);

  auto state_manager = std::make_shared<axon::uploader::UploadStateManager>(db_path_.string());
  axon::transfer::FileScanner scanner({test_dir_.string(), false}, state_manager);

  auto result = scanner.find("task_001");

  EXPECT_FALSE(result.has_value());
}

TEST_F(FileScannerTest, ScanAllFindsMultiple) {
  create_file(test_dir_ / "task_001.mcap");
  create_file(test_dir_ / "task_001.json");
  create_file(test_dir_ / "task_002.mcap");
  create_file(test_dir_ / "task_002.json");
  create_file(test_dir_ / "task_003.mcap");
  create_file(test_dir_ / "task_003.json");

  auto state_manager = std::make_shared<axon::uploader::UploadStateManager>(db_path_.string());
  axon::transfer::FileScanner scanner({test_dir_.string(), true}, state_manager);

  auto results = scanner.scan_all();

  ASSERT_EQ(results.size(), 3);
}

TEST_F(FileScannerTest, ScanAllSkipsIncomplete) {
  create_file(test_dir_ / "task_001.mcap");
  create_file(test_dir_ / "task_001.json");
  create_file(test_dir_ / "task_002.mcap");

  auto state_manager = std::make_shared<axon::uploader::UploadStateManager>(db_path_.string());
  axon::transfer::FileScanner scanner({test_dir_.string(), true}, state_manager);

  auto results = scanner.scan_all();

  EXPECT_EQ(results.size(), 1);
  EXPECT_EQ(results[0].task_id, "task_001");
}

TEST_F(FileScannerTest, ScanAllMcapOnlyUsesDoneMarker) {
  create_file(test_dir_ / "task_001.mcap");
  mark_mcap_done("task_001");
  create_file(test_dir_ / "task_002.mcap");

  auto state_manager = std::make_shared<axon::uploader::UploadStateManager>(db_path_.string());
  axon::transfer::FileScanner scanner({test_dir_.string(), false}, state_manager);

  auto results = scanner.scan_all();

  ASSERT_EQ(results.size(), 1);
  EXPECT_EQ(results[0].task_id, "task_001");
  EXPECT_EQ(results[0].checksum_sha256.size(), 64);
}

TEST_F(FileScannerTest, EmptyDirectory) {
  auto state_manager = std::make_shared<axon::uploader::UploadStateManager>(db_path_.string());
  axon::transfer::FileScanner scanner({test_dir_.string(), true}, state_manager);

  auto results = scanner.scan_all();

  EXPECT_TRUE(results.empty());
}

TEST_F(FileScannerTest, FindSkipsUploadedWaitAck) {
  create_file(test_dir_ / "task_001.mcap");
  create_file(test_dir_ / "task_001.json");

  auto state_manager = std::make_shared<axon::uploader::UploadStateManager>(db_path_.string());
  axon::uploader::UploadRecord record;
  record.file_path = (test_dir_ / "task_001.mcap").string();
  record.json_path = (test_dir_ / "task_001.json").string();
  record.s3_key = "factory/device/date/task_001.mcap";
  record.task_id = "task_001";
  ASSERT_TRUE(state_manager->insert(record));
  ASSERT_TRUE(state_manager->markUploadedWaitAck(record.file_path));

  axon::transfer::FileScanner scanner({test_dir_.string(), true}, state_manager);
  auto result = scanner.find("task_001");
  EXPECT_FALSE(result.has_value());
}
