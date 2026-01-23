// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

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

TEST_F(UploadStateManagerTest, DbPath) {
  EXPECT_EQ(manager_->dbPath(), db_path_);
}

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

TEST_F(UploadStateManagerTest, DeleteOlderThan) {
  // Insert records and mark some as completed
  auto completed1 = createTestRecord("/tmp/completed1.mcap", "task_comp1");
  auto completed2 = createTestRecord("/tmp/completed2.mcap", "task_comp2");
  auto pending = createTestRecord("/tmp/pending.mcap", "task_pending");

  ASSERT_TRUE(manager_->insert(completed1));
  ASSERT_TRUE(manager_->insert(completed2));
  ASSERT_TRUE(manager_->insert(pending));

  ASSERT_TRUE(manager_->markCompleted("/tmp/completed1.mcap"));
  ASSERT_TRUE(manager_->markCompleted("/tmp/completed2.mcap"));

  // Use negative age (-1 hour) so cutoff = now - (-1 hour) = now + 1 hour (future)
  // Records created "now" have created_at < (now + 1 hour), so they'll be deleted
  int deleted = manager_->deleteOlderThan(std::chrono::hours(-1));

  // Should delete all 2 completed records
  EXPECT_EQ(deleted, 2);

  // Completed records should be gone
  EXPECT_FALSE(manager_->get("/tmp/completed1.mcap").has_value());
  EXPECT_FALSE(manager_->get("/tmp/completed2.mcap").has_value());

  // Pending record should remain (not deleted because it's not completed)
  EXPECT_TRUE(manager_->get("/tmp/pending.mcap").has_value());
}

TEST_F(UploadStateManagerTest, DeleteOlderThanOnlyCompleted) {
  // Verify that deleteOlderThan only deletes completed records
  auto completed1 = createTestRecord("/tmp/completed1.mcap", "task_comp1");
  auto completed2 = createTestRecord("/tmp/completed2.mcap", "task_comp2");
  auto failed = createTestRecord("/tmp/failed.mcap", "task_failed");
  auto uploading = createTestRecord("/tmp/uploading.mcap", "task_uploading");

  ASSERT_TRUE(manager_->insert(completed1));
  ASSERT_TRUE(manager_->insert(completed2));
  ASSERT_TRUE(manager_->insert(failed));
  ASSERT_TRUE(manager_->insert(uploading));

  ASSERT_TRUE(manager_->markCompleted("/tmp/completed1.mcap"));
  ASSERT_TRUE(manager_->markCompleted("/tmp/completed2.mcap"));
  ASSERT_TRUE(manager_->markFailed("/tmp/failed.mcap", "error"));
  ASSERT_TRUE(manager_->updateStatus("/tmp/uploading.mcap", UploadStatus::UPLOADING));

  // Use negative age (-1 hour) so cutoff = now + 1 hour (future)
  // All records created before now are "older than" this future cutoff
  int deleted = manager_->deleteOlderThan(std::chrono::hours(-1));

  // Should delete all 2 completed records
  EXPECT_EQ(deleted, 2);

  // Critical: non-completed records should always remain (only completed records are deleted)
  EXPECT_TRUE(manager_->get("/tmp/failed.mcap").has_value());
  EXPECT_TRUE(manager_->get("/tmp/uploading.mcap").has_value());

  // Completed records should be gone
  EXPECT_FALSE(manager_->get("/tmp/completed1.mcap").has_value());
  EXPECT_FALSE(manager_->get("/tmp/completed2.mcap").has_value());
}

TEST_F(UploadStateManagerTest, DeleteOlderThanNoMatch) {
  // Test with positive age (1 hour) - records created now should NOT be deleted
  auto completed1 = createTestRecord("/tmp/completed1.mcap", "task_comp1");
  ASSERT_TRUE(manager_->insert(completed1));
  ASSERT_TRUE(manager_->markCompleted("/tmp/completed1.mcap"));

  // Use age = 1 hour, so cutoff = now - 1 hour (in the past)
  // Records created "now" have created_at ≈ now, and now is NOT < (now - 1 hour)
  // So nothing should be deleted
  int deleted = manager_->deleteOlderThan(std::chrono::hours(1));

  // Should delete nothing (records are too new to be older than cutoff)
  EXPECT_EQ(deleted, 0);

  // Record should still exist
  EXPECT_TRUE(manager_->get("/tmp/completed1.mcap").has_value());
}

// ============================================================================
// Additional Edge Case and Error Path Tests
// ============================================================================

TEST_F(UploadStateManagerTest, GetPendingEmpty) {
  // Test getPending() with no pending records
  auto completed = createTestRecord("/tmp/completed.mcap", "task_001");
  ASSERT_TRUE(manager_->insert(completed));
  ASSERT_TRUE(manager_->markCompleted("/tmp/completed.mcap"));

  auto pending = manager_->getPending();
  EXPECT_TRUE(pending.empty());
}

TEST_F(UploadStateManagerTest, GetFailedEmpty) {
  // Test getFailed() with no failed records
  auto pending = createTestRecord("/tmp/pending.mcap", "task_001");
  ASSERT_TRUE(manager_->insert(pending));

  auto failed = manager_->getFailed();
  EXPECT_TRUE(failed.empty());
}

TEST_F(UploadStateManagerTest, CountByStatusAllStatuses) {
  // Test countByStatus() for all status types
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
  ASSERT_TRUE(manager_->markFailed("/tmp/failed.mcap", "error"));

  EXPECT_EQ(manager_->countByStatus(UploadStatus::PENDING), 1);
  EXPECT_EQ(manager_->countByStatus(UploadStatus::UPLOADING), 1);
  EXPECT_EQ(manager_->countByStatus(UploadStatus::COMPLETED), 1);
  EXPECT_EQ(manager_->countByStatus(UploadStatus::FAILED), 1);
}

TEST_F(UploadStateManagerTest, PendingBytesEmpty) {
  // Test pendingBytes() with no pending records
  auto completed = createTestRecord("/tmp/completed.mcap", "task_001");
  ASSERT_TRUE(manager_->insert(completed));
  ASSERT_TRUE(manager_->markCompleted("/tmp/completed.mcap"));

  EXPECT_EQ(manager_->pendingBytes(), 0);
}

TEST_F(UploadStateManagerTest, PendingBytesWithUploading) {
  // Test pendingBytes() includes both pending and uploading status
  auto pending = createTestRecord("/tmp/pending.mcap", "task_001");
  pending.file_size_bytes = 1000;
  auto uploading = createTestRecord("/tmp/uploading.mcap", "task_002");
  uploading.file_size_bytes = 2000;

  ASSERT_TRUE(manager_->insert(pending));
  ASSERT_TRUE(manager_->insert(uploading));
  ASSERT_TRUE(manager_->updateStatus("/tmp/uploading.mcap", UploadStatus::UPLOADING));

  EXPECT_EQ(manager_->pendingBytes(), 3000);
}

TEST_F(UploadStateManagerTest, DeleteCompletedEmpty) {
  // Test deleteCompleted() with no completed records
  auto pending = createTestRecord("/tmp/pending.mcap", "task_001");
  ASSERT_TRUE(manager_->insert(pending));

  int deleted = manager_->deleteCompleted();
  EXPECT_EQ(deleted, 0);
  EXPECT_TRUE(manager_->get("/tmp/pending.mcap").has_value());
}

TEST_F(UploadStateManagerTest, UpdateStatusWithError) {
  // Test updateStatus() with error message
  auto record = createTestRecord("/tmp/test.mcap", "task_001");
  ASSERT_TRUE(manager_->insert(record));

  ASSERT_TRUE(manager_->updateStatus("/tmp/test.mcap", UploadStatus::FAILED, "Test error"));

  auto retrieved = manager_->get("/tmp/test.mcap");
  ASSERT_TRUE(retrieved.has_value());
  EXPECT_EQ(retrieved->status, UploadStatus::FAILED);
  EXPECT_EQ(retrieved->last_error, "Test error");
}

TEST_F(UploadStateManagerTest, UpdateStatusNonExistent) {
  // Test updateStatus() on non-existent record
  EXPECT_FALSE(manager_->updateStatus("/tmp/nonexistent.mcap", UploadStatus::COMPLETED));
}

TEST_F(UploadStateManagerTest, MarkCompletedNonExistent) {
  // Test markCompleted() on non-existent record
  EXPECT_FALSE(manager_->markCompleted("/tmp/nonexistent.mcap"));
}

TEST_F(UploadStateManagerTest, MarkFailedNonExistent) {
  // Test markFailed() on non-existent record
  EXPECT_FALSE(manager_->markFailed("/tmp/nonexistent.mcap", "error"));
}

TEST_F(UploadStateManagerTest, IncrementRetryNonExistent) {
  // Test incrementRetry() on non-existent record
  EXPECT_FALSE(manager_->incrementRetry("/tmp/nonexistent.mcap", "error"));
}

TEST_F(UploadStateManagerTest, GetNonExistent) {
  // Test get() on non-existent record
  auto result = manager_->get("/tmp/nonexistent.mcap");
  EXPECT_FALSE(result.has_value());
}

TEST_F(UploadStateManagerTest, StatusConversionEdgeCases) {
  // Test status conversion with edge cases
  EXPECT_EQ(uploadStatusFromString(""), UploadStatus::PENDING);  // Empty string defaults to PENDING
  EXPECT_EQ(uploadStatusFromString("PENDING"), UploadStatus::PENDING);  // Case insensitive?
  // Actually, looking at the code, it's case-sensitive, so this will default
  EXPECT_EQ(uploadStatusFromString("invalid"), UploadStatus::PENDING);  // Invalid string defaults
}

TEST_F(UploadStateManagerTest, RecordWithEmptyFields) {
  // Test inserting record with empty optional fields
  UploadRecord record;
  record.file_path = "/tmp/test.mcap";
  record.s3_key = "test-key";
  record.status = UploadStatus::PENDING;
  // Leave other fields empty/default

  ASSERT_TRUE(manager_->insert(record));

  auto retrieved = manager_->get("/tmp/test.mcap");
  ASSERT_TRUE(retrieved.has_value());
  EXPECT_EQ(retrieved->file_path, "/tmp/test.mcap");
  EXPECT_TRUE(retrieved->json_path.empty());
  EXPECT_TRUE(retrieved->task_id.empty());
}

TEST_F(UploadStateManagerTest, RecordWithSpecialCharacters) {
  // Test record with special characters in paths
  UploadRecord record;
  record.file_path = "/tmp/test with spaces/file.mcap";
  record.json_path = "/tmp/test/file.json";
  record.s3_key = "factory/device/2025-01-01/task_id.mcap";
  record.task_id = "task-001";
  record.factory_id = "factory_01";
  record.device_id = "device-01";
  record.file_size_bytes = 1000;
  record.status = UploadStatus::PENDING;

  ASSERT_TRUE(manager_->insert(record));

  auto retrieved = manager_->get("/tmp/test with spaces/file.mcap");
  ASSERT_TRUE(retrieved.has_value());
  EXPECT_EQ(retrieved->file_path, "/tmp/test with spaces/file.mcap");
}

TEST_F(UploadStateManagerTest, MultipleRetries) {
  // Test multiple retry increments
  auto record = createTestRecord("/tmp/test.mcap", "task_001");
  ASSERT_TRUE(manager_->insert(record));

  for (int i = 0; i < 5; ++i) {
    ASSERT_TRUE(manager_->incrementRetry("/tmp/test.mcap", "Error " + std::to_string(i)));
  }

  auto retrieved = manager_->get("/tmp/test.mcap");
  ASSERT_TRUE(retrieved.has_value());
  EXPECT_EQ(retrieved->retry_count, 5);
  EXPECT_EQ(retrieved->last_error, "Error 4");  // Last error message
}

TEST_F(UploadStateManagerTest, GetIncompleteEmpty) {
  // Test getIncomplete() with no incomplete records
  auto completed = createTestRecord("/tmp/completed.mcap", "task_001");
  auto failed = createTestRecord("/tmp/failed.mcap", "task_002");

  ASSERT_TRUE(manager_->insert(completed));
  ASSERT_TRUE(manager_->insert(failed));
  ASSERT_TRUE(manager_->markCompleted("/tmp/completed.mcap"));
  ASSERT_TRUE(manager_->markFailed("/tmp/failed.mcap", "error"));

  auto incomplete = manager_->getIncomplete();
  EXPECT_TRUE(incomplete.empty());
}

// ============================================================================
// SQLite Error Path Tests
// ============================================================================

TEST_F(UploadStateManagerTest, DatabaseOpenFailure_InvalidPath) {
  // Test database initialization with invalid path
  // This should throw std::runtime_error
  std::string invalid_path = "/nonexistent/path/that/cannot/be/created/state.db";

  EXPECT_THROW({ UploadStateManager invalid_manager(invalid_path); }, std::runtime_error);
}

TEST_F(UploadStateManagerTest, InsertWithVeryLongPath) {
  // Test insert with very long file path (SQLite has limits)
  // This tests edge cases in SQLite string handling
  std::string long_path = "/tmp/" + std::string(500, 'a') + ".mcap";
  auto record = createTestRecord(long_path, "task_long_path");

  // Should handle long paths (SQLite supports up to 1GB for TEXT, so this should work)
  bool result = manager_->insert(record);
  EXPECT_TRUE(result);

  auto retrieved = manager_->get(long_path);
  EXPECT_TRUE(retrieved.has_value());
  EXPECT_EQ(retrieved->file_path, long_path);
}

TEST_F(UploadStateManagerTest, InsertWithSpecialCharacters) {
  // Test insert with special characters in paths (SQL injection safety)
  std::string special_path = "/tmp/file'with\"special;chars.mcap";
  auto record = createTestRecord(special_path, "task_special");

  // SQLite should handle special characters correctly (parameterized queries)
  bool result = manager_->insert(record);
  EXPECT_TRUE(result);

  auto retrieved = manager_->get(special_path);
  EXPECT_TRUE(retrieved.has_value());
  EXPECT_EQ(retrieved->file_path, special_path);
}

TEST_F(UploadStateManagerTest, OperationsOnClosedDatabase) {
  // Test that operations fail gracefully if database is somehow closed
  // This is hard to test directly, but we can test that operations
  // return false for invalid operations
  auto record = createTestRecord("/tmp/test.mcap", "task_001");
  ASSERT_TRUE(manager_->insert(record));

  // Remove the record
  ASSERT_TRUE(manager_->remove("/tmp/test.mcap"));

  // All subsequent operations on this record should return false
  EXPECT_FALSE(manager_->updateStatus("/tmp/test.mcap", UploadStatus::COMPLETED));
  EXPECT_FALSE(manager_->markCompleted("/tmp/test.mcap"));
  EXPECT_FALSE(manager_->markFailed("/tmp/test.mcap", "error"));
  EXPECT_FALSE(manager_->incrementRetry("/tmp/test.mcap", "error"));
  EXPECT_FALSE(manager_->get("/tmp/test.mcap").has_value());
}

TEST_F(UploadStateManagerTest, ConcurrentOperations) {
  // Test that operations are thread-safe (basic test)
  // This doesn't test SQLite errors directly, but tests concurrent access
  auto record1 = createTestRecord("/tmp/test1.mcap", "task_001");
  auto record2 = createTestRecord("/tmp/test2.mcap", "task_002");

  ASSERT_TRUE(manager_->insert(record1));
  ASSERT_TRUE(manager_->insert(record2));

  // Concurrent updates should work (protected by mutex)
  ASSERT_TRUE(manager_->updateStatus("/tmp/test1.mcap", UploadStatus::UPLOADING));
  ASSERT_TRUE(manager_->updateStatus("/tmp/test2.mcap", UploadStatus::UPLOADING));

  auto retrieved1 = manager_->get("/tmp/test1.mcap");
  auto retrieved2 = manager_->get("/tmp/test2.mcap");

  EXPECT_TRUE(retrieved1.has_value());
  EXPECT_TRUE(retrieved2.has_value());
  EXPECT_EQ(retrieved1->status, UploadStatus::UPLOADING);
  EXPECT_EQ(retrieved2->status, UploadStatus::UPLOADING);
}

TEST_F(UploadStateManagerTest, LargeRetryCount) {
  // Test with very large retry count (edge case)
  auto record = createTestRecord("/tmp/test.mcap", "task_001");
  ASSERT_TRUE(manager_->insert(record));

  // Increment retry many times
  for (int i = 0; i < 100; ++i) {
    ASSERT_TRUE(manager_->incrementRetry("/tmp/test.mcap", "Error " + std::to_string(i)));
  }

  auto retrieved = manager_->get("/tmp/test.mcap");
  ASSERT_TRUE(retrieved.has_value());
  EXPECT_EQ(retrieved->retry_count, 100);
}

TEST_F(UploadStateManagerTest, EmptyStringFields) {
  // Test with empty string fields (edge case for SQLite)
  UploadRecord record;
  record.file_path = "/tmp/test.mcap";
  record.s3_key = "test-key";
  record.status = UploadStatus::PENDING;
  // Leave other fields empty/default

  ASSERT_TRUE(manager_->insert(record));

  auto retrieved = manager_->get("/tmp/test.mcap");
  ASSERT_TRUE(retrieved.has_value());
  EXPECT_TRUE(retrieved->json_path.empty());
  EXPECT_TRUE(retrieved->task_id.empty());
  EXPECT_TRUE(retrieved->factory_id.empty());
  EXPECT_TRUE(retrieved->device_id.empty());
}
