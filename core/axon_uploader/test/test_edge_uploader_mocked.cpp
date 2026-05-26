// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

/**
 * Unit tests for EdgeUploader using GoogleMock
 *
 * These tests use mocks to test error paths that are difficult to simulate
 * with real files, such as I/O errors, permission errors, and file system failures.
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include "edge_uploader_test_helpers.hpp"
#include "uploader_mocks.hpp"

using namespace axon::uploader;
using namespace axon::uploader::test;
using ::testing::_;
using ::testing::ByMove;
using ::testing::DoAll;
using ::testing::InSequence;
using ::testing::Return;
using ::testing::ReturnRef;

namespace {

bool ends_with(const std::string& value, const std::string& suffix) {
  return value.size() >= suffix.size() &&
         value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

UploaderConfig make_test_config(const std::filesystem::path& test_dir) {
  UploaderConfig config;
  config.state_db_path = (test_dir / "state.db").string();
  config.failed_uploads_dir = (test_dir / "failed").string();
  config.s3.bucket = "test-bucket";
  config.s3.region = "us-east-1";
  config.s3.access_key = "test";
  config.s3.secret_key = "test";
  return config;
}

}  // namespace

class EdgeUploaderMockedTest : public ::testing::Test {
protected:
  void SetUp() override {
    mock_filesystem_ = std::make_unique<MockFileSystem>();
    test_dir_ = std::filesystem::temp_directory_path() /
                ("edge_uploader_mocked_" +
                 std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(test_dir_);
  }

  void TearDown() override {
    std::error_code ec;
    std::filesystem::remove_all(test_dir_, ec);
  }

  std::unique_ptr<MockFileSystem> mock_filesystem_;
  std::string test_mcap_path_ = "/test/path/file.mcap";
  std::string test_json_path_ = "/test/path/file.json";
  std::filesystem::path test_dir_;
};

// =============================================================================
// File Validation Error Tests
// =============================================================================

TEST_F(EdgeUploaderMockedTest, ValidateFilesExist_McapNotFound) {
  // Test file existence validation when MCAP file doesn't exist
  EXPECT_CALL(*mock_filesystem_, exists(test_mcap_path_)).WillOnce(Return(false));

  bool result = validateFilesExistImpl(test_mcap_path_, test_json_path_, *mock_filesystem_);
  EXPECT_FALSE(result);
}

TEST_F(EdgeUploaderMockedTest, ValidateFilesExist_JsonNotFound) {
  // Test file existence validation when JSON file doesn't exist
  EXPECT_CALL(*mock_filesystem_, exists(test_mcap_path_)).WillOnce(Return(true));
  EXPECT_CALL(*mock_filesystem_, exists(test_json_path_)).WillOnce(Return(false));

  bool result = validateFilesExistImpl(test_mcap_path_, test_json_path_, *mock_filesystem_);
  EXPECT_FALSE(result);
}

TEST_F(EdgeUploaderMockedTest, ValidateFilesExist_BothExist) {
  // Test file existence validation when both files exist
  EXPECT_CALL(*mock_filesystem_, exists(test_mcap_path_)).WillOnce(Return(true));
  EXPECT_CALL(*mock_filesystem_, exists(test_json_path_)).WillOnce(Return(true));

  bool result = validateFilesExistImpl(test_mcap_path_, test_json_path_, *mock_filesystem_);
  EXPECT_TRUE(result);
}

TEST_F(EdgeUploaderMockedTest, ValidateFilesExist_McapOnly) {
  EXPECT_CALL(*mock_filesystem_, exists(test_mcap_path_)).WillOnce(Return(true));
  EXPECT_CALL(*mock_filesystem_, exists(test_json_path_)).Times(0);

  bool result = validateFilesExistImpl(test_mcap_path_, "", *mock_filesystem_);
  EXPECT_TRUE(result);
}

TEST_F(EdgeUploaderMockedTest, GetFileSize_Success) {
  // Test file size retrieval
  constexpr uint64_t expected_size = 1024;
  EXPECT_CALL(*mock_filesystem_, file_size(test_mcap_path_)).WillOnce(Return(expected_size));

  uint64_t size = getFileSizeImpl(test_mcap_path_, *mock_filesystem_);
  EXPECT_EQ(size, expected_size);
}

// =============================================================================
// Cleanup Local Files Error Tests
// =============================================================================

TEST_F(EdgeUploaderMockedTest, CleanupLocalFiles_McapRemovalFailure) {
  // Test cleanup when MCAP file removal fails
  EXPECT_CALL(*mock_filesystem_, exists(test_mcap_path_)).WillOnce(Return(true));
  EXPECT_CALL(*mock_filesystem_, remove(test_mcap_path_)).WillOnce(Return(false));
  EXPECT_CALL(*mock_filesystem_, exists(test_json_path_)).WillOnce(Return(true));
  EXPECT_CALL(*mock_filesystem_, remove(test_json_path_)).WillOnce(Return(true));

  // Should not throw, just attempt cleanup
  cleanupLocalFilesImpl(test_mcap_path_, test_json_path_, *mock_filesystem_);
}

TEST_F(EdgeUploaderMockedTest, CleanupLocalFiles_JsonRemovalFailure) {
  // Test cleanup when JSON file removal fails
  EXPECT_CALL(*mock_filesystem_, exists(test_mcap_path_)).WillOnce(Return(true));
  EXPECT_CALL(*mock_filesystem_, remove(test_mcap_path_)).WillOnce(Return(true));
  EXPECT_CALL(*mock_filesystem_, exists(test_json_path_)).WillOnce(Return(true));
  EXPECT_CALL(*mock_filesystem_, remove(test_json_path_)).WillOnce(Return(false));

  // Should not throw, just attempt cleanup
  cleanupLocalFilesImpl(test_mcap_path_, test_json_path_, *mock_filesystem_);
}

TEST_F(EdgeUploaderMockedTest, CleanupLocalFiles_FilesNotExist) {
  // Test cleanup when files don't exist (should not attempt removal)
  EXPECT_CALL(*mock_filesystem_, exists(test_mcap_path_)).WillOnce(Return(false));
  EXPECT_CALL(*mock_filesystem_, exists(test_json_path_)).WillOnce(Return(false));

  // Should not call remove() if files don't exist
  EXPECT_CALL(*mock_filesystem_, remove(_)).Times(0);

  cleanupLocalFilesImpl(test_mcap_path_, test_json_path_, *mock_filesystem_);
}

TEST_F(EdgeUploaderMockedTest, CleanupLocalFiles_EmptyPaths) {
  // Test cleanup with empty paths
  EXPECT_CALL(*mock_filesystem_, exists(_)).Times(0);
  EXPECT_CALL(*mock_filesystem_, remove(_)).Times(0);

  cleanupLocalFilesImpl("", "", *mock_filesystem_);
}

// =============================================================================
// Move to Failed Directory Error Tests
// =============================================================================

TEST_F(EdgeUploaderMockedTest, MoveToFailedDir_DirectoryCreationFailure) {
  // Test move to failed dir when directory creation fails
  // Note: Implementation still attempts to move files even if directory creation fails
  std::string failed_dir = "/test/failed";

  EXPECT_CALL(*mock_filesystem_, create_directories(failed_dir)).WillOnce(Return(false));
  // Implementation checks both mcap_path and json_path independently
  EXPECT_CALL(*mock_filesystem_, exists(test_mcap_path_)).WillOnce(Return(true));
  EXPECT_CALL(*mock_filesystem_, exists(test_json_path_)).WillOnce(Return(true));
  // Implementation will still attempt rename even if directory creation failed
  EXPECT_CALL(*mock_filesystem_, rename(test_mcap_path_, _)).WillOnce(Return(false));
  EXPECT_CALL(*mock_filesystem_, rename(test_json_path_, _)).WillOnce(Return(false));

  // Should attempt to create directory but continue even if it fails
  moveToFailedDirImpl(test_mcap_path_, test_json_path_, failed_dir, *mock_filesystem_);
}

TEST_F(EdgeUploaderMockedTest, MoveToFailedDir_McapRenameFailure) {
  // Test move to failed dir when MCAP file rename fails
  std::string failed_dir = "/test/failed";
  std::string expected_dest = failed_dir + "/file.mcap";

  EXPECT_CALL(*mock_filesystem_, create_directories(failed_dir)).WillOnce(Return(true));
  EXPECT_CALL(*mock_filesystem_, exists(test_mcap_path_)).WillOnce(Return(true));
  EXPECT_CALL(*mock_filesystem_, rename(test_mcap_path_, _)).WillOnce(Return(false));
  EXPECT_CALL(*mock_filesystem_, exists(test_json_path_)).WillOnce(Return(true));
  EXPECT_CALL(*mock_filesystem_, rename(test_json_path_, _)).WillOnce(Return(true));

  // Should attempt rename even if one fails
  moveToFailedDirImpl(test_mcap_path_, test_json_path_, failed_dir, *mock_filesystem_);
}

TEST_F(EdgeUploaderMockedTest, MoveToFailedDir_JsonRenameFailure) {
  // Test move to failed dir when JSON file rename fails
  std::string failed_dir = "/test/failed";

  EXPECT_CALL(*mock_filesystem_, create_directories(failed_dir)).WillOnce(Return(true));
  EXPECT_CALL(*mock_filesystem_, exists(test_mcap_path_)).WillOnce(Return(true));
  EXPECT_CALL(*mock_filesystem_, rename(test_mcap_path_, _)).WillOnce(Return(true));
  EXPECT_CALL(*mock_filesystem_, exists(test_json_path_)).WillOnce(Return(true));
  EXPECT_CALL(*mock_filesystem_, rename(test_json_path_, _)).WillOnce(Return(false));

  // Should attempt rename even if one fails
  moveToFailedDirImpl(test_mcap_path_, test_json_path_, failed_dir, *mock_filesystem_);
}

TEST_F(EdgeUploaderMockedTest, MoveToFailedDir_FilesNotExist) {
  // Test move to failed dir when files don't exist
  std::string failed_dir = "/test/failed";

  EXPECT_CALL(*mock_filesystem_, create_directories(failed_dir)).WillOnce(Return(true));
  EXPECT_CALL(*mock_filesystem_, exists(test_mcap_path_)).WillOnce(Return(false));
  EXPECT_CALL(*mock_filesystem_, exists(test_json_path_)).WillOnce(Return(false));

  // Should not attempt rename if files don't exist
  EXPECT_CALL(*mock_filesystem_, rename(_, _)).Times(0);

  moveToFailedDirImpl(test_mcap_path_, test_json_path_, failed_dir, *mock_filesystem_);
}

TEST_F(EdgeUploaderMockedTest, MoveToFailedDir_EmptyPaths) {
  // Test move to failed dir with empty paths
  std::string failed_dir = "/test/failed";

  EXPECT_CALL(*mock_filesystem_, create_directories(failed_dir)).WillOnce(Return(true));
  EXPECT_CALL(*mock_filesystem_, exists(_)).Times(0);
  EXPECT_CALL(*mock_filesystem_, rename(_, _)).Times(0);

  moveToFailedDirImpl("", "", failed_dir, *mock_filesystem_);
}

// =============================================================================
// Upload Single File Error Tests
// =============================================================================

TEST_F(EdgeUploaderMockedTest, UploadSingleFile_FileNotFound) {
  // Test upload when file doesn't exist
  std::string s3_key = "test-key";
  std::string checksum = "abc123";

  EXPECT_CALL(*mock_filesystem_, exists(test_mcap_path_)).WillOnce(Return(false));

  // Create a mock S3Client (we can't easily mock S3Client, so we'll use nullptr
  // and expect the function to handle file existence check first)
  FileUploadResult result =
    uploadSingleFileImpl(test_mcap_path_, s3_key, checksum, *mock_filesystem_, nullptr);

  EXPECT_FALSE(result.success);
  EXPECT_NE(result.error_message.find("not found"), std::string::npos);
  EXPECT_FALSE(result.is_retryable);
}

TEST_F(EdgeUploaderMockedTest, EnqueueMcapOnlyWithoutSidecar) {
  auto mcap_path = test_dir_ / "task_001.mcap";
  std::ofstream(mcap_path, std::ios::binary) << "mcap payload";

  const auto config = make_test_config(test_dir_);

  {
    EdgeUploader uploader(config);
    EXPECT_TRUE(uploader.enqueue(mcap_path.string(), "", "task_001", "factory", "device", ""));
    EXPECT_EQ(uploader.stats().files_pending.load(), 1);
  }

  UploadStateManager state(config.state_db_path);
  auto record = state.get(mcap_path.string());
  ASSERT_TRUE(record.has_value());
  EXPECT_TRUE(record->json_path.empty());
  EXPECT_EQ(record->file_size_bytes, 12);
  EXPECT_EQ(
    record->checksum_sha256, "19518b7a61349c6b387cb8456bca7469d090156f62e21131de5dceb2274c7b40"
  );
  EXPECT_NE(record->s3_key.find("factory/device/"), std::string::npos);
  EXPECT_TRUE(ends_with(record->s3_key, "/task_001.mcap"));
  EXPECT_EQ(record->status, UploadStatus::PENDING);
}

TEST_F(EdgeUploaderMockedTest, EnqueuePairedMcapJsonPreservesSidecarCompatibility) {
  auto mcap_path = test_dir_ / "task_002.mcap";
  auto json_path = test_dir_ / "task_002.json";
  std::ofstream(mcap_path, std::ios::binary) << "paired mcap payload";
  std::ofstream(json_path, std::ios::binary) << R"({"task_id":"task_002"})";

  const auto config = make_test_config(test_dir_);

  {
    EdgeUploader uploader(config);
    EXPECT_TRUE(uploader.enqueue(
      mcap_path.string(), json_path.string(), "task_002", "factory", "device", "provided-checksum"
    ));
    EXPECT_EQ(uploader.stats().files_pending.load(), 1);
  }

  UploadStateManager state(config.state_db_path);
  auto record = state.get(mcap_path.string());
  ASSERT_TRUE(record.has_value());
  EXPECT_EQ(record->json_path, json_path.string());
  EXPECT_EQ(record->checksum_sha256, "provided-checksum");
  EXPECT_EQ(record->file_size_bytes, 19);
  EXPECT_TRUE(ends_with(record->s3_key, "/task_002.mcap"));
  EXPECT_EQ(record->status, UploadStatus::PENDING);
}

TEST_F(EdgeUploaderMockedTest, EnqueueMissingMcapFailsWithoutStateRecord) {
  auto missing_mcap = test_dir_ / "missing.mcap";
  const auto config = make_test_config(test_dir_);

  {
    EdgeUploader uploader(config);
    EXPECT_FALSE(uploader.enqueue(missing_mcap.string(), "", "missing", "factory", "device", ""));
    EXPECT_EQ(uploader.stats().files_pending.load(), 0);
  }

  UploadStateManager state(config.state_db_path);
  EXPECT_FALSE(state.get(missing_mcap.string()).has_value());
}

TEST_F(EdgeUploaderMockedTest, EnqueueMissingJsonFailsWhenJsonPathProvided) {
  auto mcap_path = test_dir_ / "task_003.mcap";
  auto missing_json = test_dir_ / "task_003.json";
  std::ofstream(mcap_path, std::ios::binary) << "mcap payload";
  const auto config = make_test_config(test_dir_);

  {
    EdgeUploader uploader(config);
    EXPECT_FALSE(uploader.enqueue(
      mcap_path.string(), missing_json.string(), "task_003", "factory", "device", ""
    ));
    EXPECT_EQ(uploader.stats().files_pending.load(), 0);
  }

  UploadStateManager state(config.state_db_path);
  EXPECT_FALSE(state.get(mcap_path.string()).has_value());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
