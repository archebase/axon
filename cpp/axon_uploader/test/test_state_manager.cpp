/**
 * Unit tests for UploadStateManager
 */

#include <gtest/gtest.h>

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <thread>

#include "upload_state_manager.hpp"

namespace fs = std::filesystem;
using namespace axon::uploader;

class UploadStateManagerTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Create a unique temp file for each test
    db_path_ = "/tmp/test_uploader_state_" +
               std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".db";
    manager_ = std::make_unique<UploadStateManager>(db_path_);
  }

  void TearDown() override {
    manager_.reset();

    // Clean up database files
    if (fs::exists(db_path_)) {
      fs::remove(db_path_);
    }
    // Also remove WAL files
    if (fs::exists(db_path_ + "-wal")) {
      fs::remove(db_path_ + "-wal");
    }
    if (fs::exists(db_path_ + "-shm")) {
      fs::remove(db_path_ + "-shm");
    }
  }

  UploadRecord createTestRecord(const std::string& file_path, const std::string& task_id) {
    UploadRecord record;
    record.file_path = file_path;
    record.json_path = file_path.substr(0, file_path.size() - 5) + ".json";
    record.s3_key = "factory/device/2025-01-01/" + task_id + ".mcap";
    record.task_id = task_id;
    record.factory_id = "factory_01";
    record.device_id = "device_01";
    record.file_size_bytes = 1000000;
    record.checksum_sha256 = "abc123";
    record.status = UploadStatus::PENDING;
    record.retry_count = 0;
    return record;
  }

  std::string db_path_;
  std::unique_ptr<UploadStateManager> manager_;
};

TEST_F(UploadStateManagerTest, InsertAndGet) {
  auto record = createTestRecord("/tmp/test.mcap", "task_001");

  ASSERT_TRUE(manager_->insert(record));

  auto retrieved = manager_->get("/tmp/test.mcap");
  ASSERT_TRUE(retrieved.has_value());

  EXPECT_EQ(retrieved->file_path, "/tmp/test.mcap");
  EXPECT_EQ(retrieved->task_id, "task_001");
  EXPECT_EQ(retrieved->factory_id, "factory_01");
  EXPECT_EQ(retrieved->status, UploadStatus::PENDING);
  EXPECT_EQ(retrieved->retry_count, 0);
}

TEST_F(UploadStateManagerTest, InsertDuplicate) {
  auto record = createTestRecord("/tmp/test.mcap", "task_001");

  ASSERT_TRUE(manager_->insert(record));
  EXPECT_FALSE(manager_->insert(record));  // Should fail - duplicate
}

TEST_F(UploadStateManagerTest, UpdateStatus) {
  auto record = createTestRecord("/tmp/test.mcap", "task_001");
  ASSERT_TRUE(manager_->insert(record));

  ASSERT_TRUE(manager_->updateStatus("/tmp/test.mcap", UploadStatus::UPLOADING));

  auto retrieved = manager_->get("/tmp/test.mcap");
  ASSERT_TRUE(retrieved.has_value());
  EXPECT_EQ(retrieved->status, UploadStatus::UPLOADING);
}

TEST_F(UploadStateManagerTest, MarkCompleted) {
  auto record = createTestRecord("/tmp/test.mcap", "task_001");
  ASSERT_TRUE(manager_->insert(record));

  ASSERT_TRUE(manager_->markCompleted("/tmp/test.mcap"));

  auto retrieved = manager_->get("/tmp/test.mcap");
  ASSERT_TRUE(retrieved.has_value());
  EXPECT_EQ(retrieved->status, UploadStatus::COMPLETED);
  EXPECT_FALSE(retrieved->completed_at.empty());
}

TEST_F(UploadStateManagerTest, MarkFailed) {
  auto record = createTestRecord("/tmp/test.mcap", "task_001");
  ASSERT_TRUE(manager_->insert(record));

  ASSERT_TRUE(manager_->markFailed("/tmp/test.mcap", "Network error"));

  auto retrieved = manager_->get("/tmp/test.mcap");
  ASSERT_TRUE(retrieved.has_value());
  EXPECT_EQ(retrieved->status, UploadStatus::FAILED);
  EXPECT_EQ(retrieved->last_error, "Network error");
}

TEST_F(UploadStateManagerTest, IncrementRetry) {
  auto record = createTestRecord("/tmp/test.mcap", "task_001");
  ASSERT_TRUE(manager_->insert(record));

  ASSERT_TRUE(manager_->incrementRetry("/tmp/test.mcap", "First failure"));

  auto retrieved = manager_->get("/tmp/test.mcap");
  ASSERT_TRUE(retrieved.has_value());
  EXPECT_EQ(retrieved->retry_count, 1);
  EXPECT_EQ(retrieved->last_error, "First failure");

  ASSERT_TRUE(manager_->incrementRetry("/tmp/test.mcap", "Second failure"));
  retrieved = manager_->get("/tmp/test.mcap");
  EXPECT_EQ(retrieved->retry_count, 2);
}

TEST_F(UploadStateManagerTest, Remove) {
  auto record = createTestRecord("/tmp/test.mcap", "task_001");
  ASSERT_TRUE(manager_->insert(record));

  ASSERT_TRUE(manager_->remove("/tmp/test.mcap"));
  EXPECT_FALSE(manager_->get("/tmp/test.mcap").has_value());

  // Removing non-existent should return false
  EXPECT_FALSE(manager_->remove("/tmp/nonexistent.mcap"));
}

TEST_F(UploadStateManagerTest, GetPending) {
  // Insert multiple records with different statuses
  auto pending1 = createTestRecord("/tmp/pending1.mcap", "task_001");
  auto pending2 = createTestRecord("/tmp/pending2.mcap", "task_002");
  auto completed = createTestRecord("/tmp/completed.mcap", "task_003");

  ASSERT_TRUE(manager_->insert(pending1));
  ASSERT_TRUE(manager_->insert(pending2));
  ASSERT_TRUE(manager_->insert(completed));
  ASSERT_TRUE(manager_->markCompleted("/tmp/completed.mcap"));

  auto pending = manager_->getPending();
  EXPECT_EQ(pending.size(), 2);
}

TEST_F(UploadStateManagerTest, GetFailed) {
  auto pending = createTestRecord("/tmp/pending.mcap", "task_001");
  auto failed1 = createTestRecord("/tmp/failed1.mcap", "task_002");
  auto failed2 = createTestRecord("/tmp/failed2.mcap", "task_003");

  ASSERT_TRUE(manager_->insert(pending));
  ASSERT_TRUE(manager_->insert(failed1));
  ASSERT_TRUE(manager_->insert(failed2));
  ASSERT_TRUE(manager_->markFailed("/tmp/failed1.mcap", "Error 1"));
  ASSERT_TRUE(manager_->markFailed("/tmp/failed2.mcap", "Error 2"));

  auto failed = manager_->getFailed();
  EXPECT_EQ(failed.size(), 2);
}

TEST_F(UploadStateManagerTest, GetIncomplete) {
  auto pending = createTestRecord("/tmp/pending.mcap", "task_001");
  auto uploading = createTestRecord("/tmp/uploading.mcap", "task_002");
  auto completed = createTestRecord("/tmp/completed.mcap", "task_003");
  auto failed = createTestRecord("/tmp/failed.mcap", "task_004");

  ASSERT_TRUE(manager_->insert(pending));
  ASSERT_TRUE(manager_->insert(uploading));
  ASSERT_TRUE(manager_->insert(completed));
  ASSERT_TRUE(manager_->insert(failed));

  ASSERT_TRUE(manager_->updateStatus("/tmp/uploading.mcap", UploadStatus::UPLOADING));
  ASSERT_TRUE(manager_->markCompleted("/tmp/completed.mcap"));
  ASSERT_TRUE(manager_->markFailed("/tmp/failed.mcap", "Error"));

  auto incomplete = manager_->getIncomplete();
  EXPECT_EQ(incomplete.size(), 2);  // pending + uploading
}

TEST_F(UploadStateManagerTest, CountByStatus) {
  auto pending1 = createTestRecord("/tmp/pending1.mcap", "task_001");
  auto pending2 = createTestRecord("/tmp/pending2.mcap", "task_002");
  auto completed = createTestRecord("/tmp/completed.mcap", "task_003");

  ASSERT_TRUE(manager_->insert(pending1));
  ASSERT_TRUE(manager_->insert(pending2));
  ASSERT_TRUE(manager_->insert(completed));
  ASSERT_TRUE(manager_->markCompleted("/tmp/completed.mcap"));

  EXPECT_EQ(manager_->countByStatus(UploadStatus::PENDING), 2);
  EXPECT_EQ(manager_->countByStatus(UploadStatus::COMPLETED), 1);
  EXPECT_EQ(manager_->countByStatus(UploadStatus::FAILED), 0);
}

TEST_F(UploadStateManagerTest, PendingBytes) {
  auto record1 = createTestRecord("/tmp/test1.mcap", "task_001");
  record1.file_size_bytes = 1000;
  auto record2 = createTestRecord("/tmp/test2.mcap", "task_002");
  record2.file_size_bytes = 2000;

  ASSERT_TRUE(manager_->insert(record1));
  ASSERT_TRUE(manager_->insert(record2));

  EXPECT_EQ(manager_->pendingBytes(), 3000);

  ASSERT_TRUE(manager_->markCompleted("/tmp/test1.mcap"));
  EXPECT_EQ(manager_->pendingBytes(), 2000);
}

TEST_F(UploadStateManagerTest, DeleteCompleted) {
  auto pending = createTestRecord("/tmp/pending.mcap", "task_001");
  auto completed1 = createTestRecord("/tmp/completed1.mcap", "task_002");
  auto completed2 = createTestRecord("/tmp/completed2.mcap", "task_003");

  ASSERT_TRUE(manager_->insert(pending));
  ASSERT_TRUE(manager_->insert(completed1));
  ASSERT_TRUE(manager_->insert(completed2));
  ASSERT_TRUE(manager_->markCompleted("/tmp/completed1.mcap"));
  ASSERT_TRUE(manager_->markCompleted("/tmp/completed2.mcap"));

  int deleted = manager_->deleteCompleted();
  EXPECT_EQ(deleted, 2);

  // Pending should still exist
  EXPECT_TRUE(manager_->get("/tmp/pending.mcap").has_value());
  EXPECT_FALSE(manager_->get("/tmp/completed1.mcap").has_value());
  EXPECT_FALSE(manager_->get("/tmp/completed2.mcap").has_value());
}

TEST_F(UploadStateManagerTest, DbPath) { EXPECT_EQ(manager_->dbPath(), db_path_); }

TEST_F(UploadStateManagerTest, StatusConversion) {
  EXPECT_EQ(uploadStatusToString(UploadStatus::PENDING), "pending");
  EXPECT_EQ(uploadStatusToString(UploadStatus::UPLOADING), "uploading");
  EXPECT_EQ(uploadStatusToString(UploadStatus::COMPLETED), "completed");
  EXPECT_EQ(uploadStatusToString(UploadStatus::FAILED), "failed");

  EXPECT_EQ(uploadStatusFromString("pending"), UploadStatus::PENDING);
  EXPECT_EQ(uploadStatusFromString("uploading"), UploadStatus::UPLOADING);
  EXPECT_EQ(uploadStatusFromString("completed"), UploadStatus::COMPLETED);
  EXPECT_EQ(uploadStatusFromString("failed"), UploadStatus::FAILED);
  EXPECT_EQ(uploadStatusFromString("unknown"), UploadStatus::PENDING);  // Default
}

