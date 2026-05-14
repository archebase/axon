// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "upload_state_manager.hpp"

#include <chrono>
#include <iomanip>
#include <mutex>
#include <sqlite3.h>
#include <sstream>
#include <stdexcept>

namespace axon {
namespace uploader {

// Get current ISO 8601 timestamp
static std::string currentTimestamp() {
  auto now = std::chrono::system_clock::now();
  auto time = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
#ifdef _WIN32
  gmtime_s(&tm, &time);  // Windows uses reversed argument order
#else
  gmtime_r(&time, &tm);
#endif

  // LCOV_EXCL_BR_START - String stream operations generate exception-safety branches
  // that are standard library implementation details
  std::ostringstream ss;
  ss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
  return ss.str();
  // LCOV_EXCL_BR_STOP
}

class UploadStateManager::Impl {
public:
  sqlite3* db = nullptr;
  std::string db_path;
  mutable std::mutex mutex;

  ~Impl() {
    if (db) {
      sqlite3_close(db);
    }
  }

  static void executeOrThrow(sqlite3* db, const char* sql, const std::string& context) {
    char* err_msg = nullptr;
    int rc = sqlite3_exec(db, sql, nullptr, nullptr, &err_msg);
    if (rc != SQLITE_OK) {
      std::string error = err_msg ? err_msg : "Unknown error";
      sqlite3_free(err_msg);
      throw std::runtime_error(context + ": " + error);
    }
  }

  bool schemaNeedsMigration() {
    const char* sql = "SELECT sql FROM sqlite_master WHERE type='table' AND name='upload_state'";
    sqlite3_stmt* stmt = nullptr;
    int rc = sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr);
    if (rc != SQLITE_OK) {
      return false;
    }

    bool needs_migration = false;
    if (sqlite3_step(stmt) == SQLITE_ROW) {
      const char* create_sql = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0));
      std::string schema_sql = create_sql ? create_sql : "";
      needs_migration = schema_sql.find("uploaded_wait_ack") == std::string::npos ||
                        schema_sql.find("delete_retry_count") == std::string::npos ||
                        schema_sql.find("next_retry_at") == std::string::npos ||
                        schema_sql.find("'active'") == std::string::npos ||
                        schema_sql.find("'retry-wait'") == std::string::npos;
    }

    sqlite3_finalize(stmt);
    return needs_migration;
  }

  void migrateSchema() {
    executeOrThrow(db, "BEGIN TRANSACTION;", "Failed to begin schema migration transaction");
    try {
      executeOrThrow(
        db,
        R"(
          CREATE TABLE upload_state_new (
            file_path TEXT PRIMARY KEY,
            json_path TEXT,
            s3_key TEXT NOT NULL,
            task_id TEXT,
            factory_id TEXT,
            device_id TEXT,
            file_size_bytes INTEGER,
            checksum_sha256 TEXT,
            status TEXT NOT NULL CHECK(status IN ('pending', 'active', 'retry-wait', 'uploaded_wait_ack', 'completed', 'failed')),
            retry_count INTEGER DEFAULT 0,
            last_error TEXT,
            next_retry_at TEXT,
            created_at TEXT NOT NULL,
            updated_at TEXT NOT NULL,
            completed_at TEXT,
            delete_retry_count INTEGER DEFAULT 0,
            delete_next_retry_at TEXT,
            delete_last_error TEXT
          );
        )",
        "Failed to create migrated upload_state table"
      );

      executeOrThrow(
        db,
        R"(
          INSERT INTO upload_state_new (
            file_path, json_path, s3_key, task_id, factory_id, device_id,
            file_size_bytes, checksum_sha256, status, retry_count, last_error, next_retry_at,
            created_at, updated_at, completed_at,
            delete_retry_count, delete_next_retry_at, delete_last_error
          )
          SELECT
            file_path, json_path, s3_key, task_id, factory_id, device_id,
            file_size_bytes,
            checksum_sha256,
            CASE status
              WHEN 'uploading' THEN 'active'
              WHEN 'retry_wait' THEN 'retry-wait'
              ELSE status
            END,
            retry_count,
            last_error,
            NULL,
            created_at, updated_at, completed_at,
            0, NULL, ''
          FROM upload_state;
        )",
        "Failed to copy upload_state data during migration"
      );

      executeOrThrow(db, "DROP TABLE upload_state;", "Failed to drop old upload_state table");
      executeOrThrow(
        db,
        "ALTER TABLE upload_state_new RENAME TO upload_state;",
        "Failed to rename migrated upload_state table"
      );

      executeOrThrow(
        db,
        R"(
          CREATE INDEX IF NOT EXISTS idx_status ON upload_state(status);
          CREATE INDEX IF NOT EXISTS idx_task_id ON upload_state(task_id);
          CREATE INDEX IF NOT EXISTS idx_created_at ON upload_state(created_at);
        )",
        "Failed to recreate upload_state indexes"
      );

      executeOrThrow(db, "COMMIT;", "Failed to commit schema migration transaction");
    } catch (...) {
      sqlite3_exec(db, "ROLLBACK;", nullptr, nullptr, nullptr);
      throw;
    }
  }

  void initDatabase() {
    int rc = sqlite3_open(db_path.c_str(), &db);
    if (rc != SQLITE_OK) {
      // LCOV_EXCL_BR_START - String concatenation
      throw std::runtime_error("Cannot open SQLite database: " + db_path);
      // LCOV_EXCL_BR_STOP
    }

    // Enable WAL mode for crash safety and concurrent reads
    char* err_msg = nullptr;
    rc = sqlite3_exec(db, "PRAGMA journal_mode=WAL;", nullptr, nullptr, &err_msg);
    if (rc != SQLITE_OK) {
      std::string error = err_msg ? err_msg : "Unknown error";
      sqlite3_free(err_msg);
      // LCOV_EXCL_BR_START - String concatenation
      throw std::runtime_error("Failed to enable WAL mode: " + error);
      // LCOV_EXCL_BR_STOP
    }

    // Set synchronous mode for good balance of safety and performance
    rc = sqlite3_exec(db, "PRAGMA synchronous=NORMAL;", nullptr, nullptr, &err_msg);
    if (rc != SQLITE_OK) {
      sqlite3_free(err_msg);
    }

    // Set busy timeout
    rc = sqlite3_exec(db, "PRAGMA busy_timeout=5000;", nullptr, nullptr, &err_msg);
    if (rc != SQLITE_OK) {
      sqlite3_free(err_msg);
    }

    // Create table
    const char* create_sql = R"(
      CREATE TABLE IF NOT EXISTS upload_state (
        file_path TEXT PRIMARY KEY,
        json_path TEXT,
        s3_key TEXT NOT NULL,
        task_id TEXT,
        factory_id TEXT,
        device_id TEXT,
        file_size_bytes INTEGER,
        checksum_sha256 TEXT,
        status TEXT NOT NULL CHECK(status IN ('pending', 'active', 'retry-wait', 'uploaded_wait_ack', 'completed', 'failed')),
        retry_count INTEGER DEFAULT 0,
        last_error TEXT,
        next_retry_at TEXT,
        created_at TEXT NOT NULL,
        updated_at TEXT NOT NULL,
        completed_at TEXT,
        delete_retry_count INTEGER DEFAULT 0,
        delete_next_retry_at TEXT,
        delete_last_error TEXT
      );
      CREATE INDEX IF NOT EXISTS idx_status ON upload_state(status);
      CREATE INDEX IF NOT EXISTS idx_task_id ON upload_state(task_id);
      CREATE INDEX IF NOT EXISTS idx_created_at ON upload_state(created_at);
    )";

    rc = sqlite3_exec(db, create_sql, nullptr, nullptr, &err_msg);
    if (rc != SQLITE_OK) {
      std::string error = err_msg ? err_msg : "Unknown error";
      sqlite3_free(err_msg);
      // LCOV_EXCL_BR_START - String concatenation
      throw std::runtime_error("Failed to create tables: " + error);
      // LCOV_EXCL_BR_STOP
    }

    if (schemaNeedsMigration()) {
      migrateSchema();
    }
  }

  // Parse a record from a SELECT statement
  static UploadRecord parseRecord(sqlite3_stmt* stmt) {
    UploadRecord record;
    // LCOV_EXCL_BR_START - String assignments generate exception-safety branches
    // that are standard library implementation details
    record.file_path = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0));
    record.json_path = sqlite3_column_text(stmt, 1)
                         ? reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1))
                         : "";
    record.s3_key = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2));
    record.task_id = sqlite3_column_text(stmt, 3)
                       ? reinterpret_cast<const char*>(sqlite3_column_text(stmt, 3))
                       : "";
    record.factory_id = sqlite3_column_text(stmt, 4)
                          ? reinterpret_cast<const char*>(sqlite3_column_text(stmt, 4))
                          : "";
    record.device_id = sqlite3_column_text(stmt, 5)
                         ? reinterpret_cast<const char*>(sqlite3_column_text(stmt, 5))
                         : "";
    record.file_size_bytes = static_cast<uint64_t>(sqlite3_column_int64(stmt, 6));
    record.checksum_sha256 = sqlite3_column_text(stmt, 7)
                               ? reinterpret_cast<const char*>(sqlite3_column_text(stmt, 7))
                               : "";
    record.status =
      uploadStatusFromString(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 8)));
    record.retry_count = sqlite3_column_int(stmt, 9);
    record.last_error = sqlite3_column_text(stmt, 10)
                          ? reinterpret_cast<const char*>(sqlite3_column_text(stmt, 10))
                          : "";
    record.next_retry_at = sqlite3_column_text(stmt, 11)
                             ? reinterpret_cast<const char*>(sqlite3_column_text(stmt, 11))
                             : "";
    record.created_at = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 12));
    record.updated_at = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 13));
    record.completed_at = sqlite3_column_text(stmt, 14)
                            ? reinterpret_cast<const char*>(sqlite3_column_text(stmt, 14))
                            : "";
    record.delete_retry_count = sqlite3_column_int(stmt, 15);
    record.delete_next_retry_at = sqlite3_column_text(stmt, 16)
                                    ? reinterpret_cast<const char*>(sqlite3_column_text(stmt, 16))
                                    : "";
    record.delete_last_error = sqlite3_column_text(stmt, 17)
                                 ? reinterpret_cast<const char*>(sqlite3_column_text(stmt, 17))
                                 : "";
    // LCOV_EXCL_BR_STOP
    return record;
  }
};

UploadStateManager::UploadStateManager(const std::string& db_path)
    // LCOV_EXCL_BR_START - Smart pointer operations generate exception-safety branches
    // that are standard library implementation details
    : impl_(std::make_unique<Impl>())
// LCOV_EXCL_BR_STOP
{
  impl_->db_path = db_path;
  impl_->initDatabase();
}

UploadStateManager::~UploadStateManager() = default;

bool UploadStateManager::insert(const UploadRecord& record) {
  std::lock_guard<std::mutex> lock(impl_->mutex);

  const char* sql = R"(
    INSERT INTO upload_state 
    (file_path, json_path, s3_key, task_id, factory_id, device_id, file_size_bytes, 
     checksum_sha256, status, retry_count, last_error, next_retry_at, created_at, updated_at,
     delete_retry_count, delete_next_retry_at, delete_last_error)
    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
  )";

  sqlite3_stmt* stmt = nullptr;
  int rc = sqlite3_prepare_v2(impl_->db, sql, -1, &stmt, nullptr);
  if (rc != SQLITE_OK) {
    return false;
  }

  // LCOV_EXCL_BR_START - String stream operations generate exception-safety branches
  std::string now = currentTimestamp();
  sqlite3_bind_text(stmt, 1, record.file_path.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 2, record.json_path.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 3, record.s3_key.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 4, record.task_id.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 5, record.factory_id.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 6, record.device_id.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_int64(stmt, 7, static_cast<sqlite3_int64>(record.file_size_bytes));
  sqlite3_bind_text(stmt, 8, record.checksum_sha256.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 9, uploadStatusToString(record.status).c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_int(stmt, 10, record.retry_count);
  sqlite3_bind_text(stmt, 11, record.last_error.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 12, record.next_retry_at.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 13, now.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 14, now.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_int(stmt, 15, record.delete_retry_count);
  sqlite3_bind_text(stmt, 16, record.delete_next_retry_at.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 17, record.delete_last_error.c_str(), -1, SQLITE_TRANSIENT);

  rc = sqlite3_step(stmt);
  sqlite3_finalize(stmt);
  // LCOV_EXCL_BR_STOP

  return rc == SQLITE_DONE;
}

bool UploadStateManager::updateStatus(
  const std::string& file_path, UploadStatus status, const std::string& error
) {
  std::lock_guard<std::mutex> lock(impl_->mutex);

  const char* sql = R"(
    UPDATE upload_state 
    SET status = ?, last_error = ?, next_retry_at = '', updated_at = ?
    WHERE file_path = ?
  )";

  sqlite3_stmt* stmt = nullptr;
  int rc = sqlite3_prepare_v2(impl_->db, sql, -1, &stmt, nullptr);
  if (rc != SQLITE_OK) {
    return false;
  }

  // LCOV_EXCL_BR_START - String stream operations generate exception-safety branches
  std::string now = currentTimestamp();
  sqlite3_bind_text(stmt, 1, uploadStatusToString(status).c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 2, error.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 3, now.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 4, file_path.c_str(), -1, SQLITE_TRANSIENT);

  rc = sqlite3_step(stmt);
  sqlite3_finalize(stmt);
  // LCOV_EXCL_BR_STOP

  return rc == SQLITE_DONE && sqlite3_changes(impl_->db) > 0;
}

bool UploadStateManager::markCompleted(const std::string& file_path) {
  std::lock_guard<std::mutex> lock(impl_->mutex);

  const char* sql = R"(
    UPDATE upload_state 
    SET status = 'completed', completed_at = ?, updated_at = ?,
        next_retry_at = '',
        delete_retry_count = 0, delete_next_retry_at = NULL, delete_last_error = ''
    WHERE file_path = ?
  )";

  sqlite3_stmt* stmt = nullptr;
  int rc = sqlite3_prepare_v2(impl_->db, sql, -1, &stmt, nullptr);
  if (rc != SQLITE_OK) {
    return false;
  }

  // LCOV_EXCL_BR_START - String stream operations generate exception-safety branches
  std::string now = currentTimestamp();
  sqlite3_bind_text(stmt, 1, now.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 2, now.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 3, file_path.c_str(), -1, SQLITE_TRANSIENT);

  rc = sqlite3_step(stmt);
  sqlite3_finalize(stmt);
  // LCOV_EXCL_BR_STOP

  return rc == SQLITE_DONE && sqlite3_changes(impl_->db) > 0;
}

bool UploadStateManager::markUploadedWaitAck(const std::string& file_path) {
  std::lock_guard<std::mutex> lock(impl_->mutex);

  const char* sql = R"(
    UPDATE upload_state
    SET status = 'uploaded_wait_ack', updated_at = ?,
        next_retry_at = '',
        delete_retry_count = 0, delete_next_retry_at = NULL, delete_last_error = ''
    WHERE file_path = ?
  )";

  sqlite3_stmt* stmt = nullptr;
  int rc = sqlite3_prepare_v2(impl_->db, sql, -1, &stmt, nullptr);
  if (rc != SQLITE_OK) {
    return false;
  }

  std::string now = currentTimestamp();
  sqlite3_bind_text(stmt, 1, now.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 2, file_path.c_str(), -1, SQLITE_TRANSIENT);

  rc = sqlite3_step(stmt);
  sqlite3_finalize(stmt);

  return rc == SQLITE_DONE && sqlite3_changes(impl_->db) > 0;
}

bool UploadStateManager::markFailed(const std::string& file_path, const std::string& error) {
  return updateStatus(file_path, UploadStatus::FAILED, error);
}

bool UploadStateManager::markRetryWait(
  const std::string& file_path, const std::string& next_retry_at, const std::string& error
) {
  std::lock_guard<std::mutex> lock(impl_->mutex);

  const char* sql = R"(
    UPDATE upload_state
    SET status = 'retry-wait', last_error = ?, next_retry_at = ?, updated_at = ?
    WHERE file_path = ?
  )";

  sqlite3_stmt* stmt = nullptr;
  int rc = sqlite3_prepare_v2(impl_->db, sql, -1, &stmt, nullptr);
  if (rc != SQLITE_OK) {
    return false;
  }

  std::string now = currentTimestamp();
  sqlite3_bind_text(stmt, 1, error.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 2, next_retry_at.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 3, now.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 4, file_path.c_str(), -1, SQLITE_TRANSIENT);

  rc = sqlite3_step(stmt);
  sqlite3_finalize(stmt);

  return rc == SQLITE_DONE && sqlite3_changes(impl_->db) > 0;
}

bool UploadStateManager::incrementRetry(const std::string& file_path, const std::string& error) {
  std::lock_guard<std::mutex> lock(impl_->mutex);

  const char* sql = R"(
    UPDATE upload_state 
    SET retry_count = retry_count + 1, last_error = ?, updated_at = ?
    WHERE file_path = ?
  )";

  sqlite3_stmt* stmt = nullptr;
  int rc = sqlite3_prepare_v2(impl_->db, sql, -1, &stmt, nullptr);
  if (rc != SQLITE_OK) {
    return false;
  }

  // LCOV_EXCL_BR_START - String stream operations generate exception-safety branches
  std::string now = currentTimestamp();
  sqlite3_bind_text(stmt, 1, error.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 2, now.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 3, file_path.c_str(), -1, SQLITE_TRANSIENT);

  rc = sqlite3_step(stmt);
  sqlite3_finalize(stmt);
  // LCOV_EXCL_BR_STOP

  return rc == SQLITE_DONE && sqlite3_changes(impl_->db) > 0;
}

bool UploadStateManager::remove(const std::string& file_path) {
  std::lock_guard<std::mutex> lock(impl_->mutex);

  const char* sql = "DELETE FROM upload_state WHERE file_path = ?";

  sqlite3_stmt* stmt = nullptr;
  int rc = sqlite3_prepare_v2(impl_->db, sql, -1, &stmt, nullptr);
  if (rc != SQLITE_OK) {
    return false;
  }

  sqlite3_bind_text(stmt, 1, file_path.c_str(), -1, SQLITE_TRANSIENT);

  rc = sqlite3_step(stmt);
  sqlite3_finalize(stmt);

  return rc == SQLITE_DONE && sqlite3_changes(impl_->db) > 0;
}

std::optional<UploadRecord> UploadStateManager::get(const std::string& file_path) {
  std::lock_guard<std::mutex> lock(impl_->mutex);

  const char* sql = "SELECT * FROM upload_state WHERE file_path = ?";

  sqlite3_stmt* stmt = nullptr;
  int rc = sqlite3_prepare_v2(impl_->db, sql, -1, &stmt, nullptr);
  if (rc != SQLITE_OK) {
    return std::nullopt;
  }

  sqlite3_bind_text(stmt, 1, file_path.c_str(), -1, SQLITE_TRANSIENT);

  rc = sqlite3_step(stmt);
  if (rc == SQLITE_ROW) {
    UploadRecord record = Impl::parseRecord(stmt);
    sqlite3_finalize(stmt);
    return record;
  }

  sqlite3_finalize(stmt);
  return std::nullopt;
}

std::optional<UploadRecord> UploadStateManager::getByTaskId(const std::string& task_id) {
  std::lock_guard<std::mutex> lock(impl_->mutex);

  const char* sql = "SELECT * FROM upload_state WHERE task_id = ? ORDER BY created_at DESC LIMIT 1";

  sqlite3_stmt* stmt = nullptr;
  int rc = sqlite3_prepare_v2(impl_->db, sql, -1, &stmt, nullptr);
  if (rc != SQLITE_OK) {
    return std::nullopt;
  }

  sqlite3_bind_text(stmt, 1, task_id.c_str(), -1, SQLITE_TRANSIENT);

  rc = sqlite3_step(stmt);
  if (rc == SQLITE_ROW) {
    UploadRecord record = Impl::parseRecord(stmt);
    sqlite3_finalize(stmt);
    return record;
  }

  sqlite3_finalize(stmt);
  return std::nullopt;
}

std::vector<UploadRecord> UploadStateManager::getByStatus(UploadStatus status) {
  std::lock_guard<std::mutex> lock(impl_->mutex);

  const char* sql = "SELECT * FROM upload_state WHERE status = ? ORDER BY created_at";

  sqlite3_stmt* stmt = nullptr;
  int rc = sqlite3_prepare_v2(impl_->db, sql, -1, &stmt, nullptr);
  if (rc != SQLITE_OK) {
    return {};
  }

  sqlite3_bind_text(stmt, 1, uploadStatusToString(status).c_str(), -1, SQLITE_TRANSIENT);

  std::vector<UploadRecord> records;
  while (sqlite3_step(stmt) == SQLITE_ROW) {
    records.push_back(Impl::parseRecord(stmt));
  }

  sqlite3_finalize(stmt);
  return records;
}

std::vector<UploadRecord> UploadStateManager::getPending() {
  std::lock_guard<std::mutex> lock(impl_->mutex);

  const char* sql = "SELECT * FROM upload_state WHERE status = 'pending' ORDER BY created_at";

  sqlite3_stmt* stmt = nullptr;
  int rc = sqlite3_prepare_v2(impl_->db, sql, -1, &stmt, nullptr);
  if (rc != SQLITE_OK) {
    return {};
  }

  std::vector<UploadRecord> records;
  // LCOV_EXCL_BR_START - Vector operations generate exception-safety branches
  // that are standard library implementation details
  while (sqlite3_step(stmt) == SQLITE_ROW) {
    records.push_back(Impl::parseRecord(stmt));
  }
  // LCOV_EXCL_BR_STOP

  sqlite3_finalize(stmt);
  return records;
}

std::vector<UploadRecord> UploadStateManager::getFailed() {
  std::lock_guard<std::mutex> lock(impl_->mutex);

  const char* sql = "SELECT * FROM upload_state WHERE status = 'failed' ORDER BY created_at";

  sqlite3_stmt* stmt = nullptr;
  int rc = sqlite3_prepare_v2(impl_->db, sql, -1, &stmt, nullptr);
  if (rc != SQLITE_OK) {
    return {};
  }

  std::vector<UploadRecord> records;
  // LCOV_EXCL_BR_START - Vector operations generate exception-safety branches
  // that are standard library implementation details
  while (sqlite3_step(stmt) == SQLITE_ROW) {
    records.push_back(Impl::parseRecord(stmt));
  }
  // LCOV_EXCL_BR_STOP

  sqlite3_finalize(stmt);
  return records;
}

std::vector<UploadRecord> UploadStateManager::getIncomplete() {
  std::lock_guard<std::mutex> lock(impl_->mutex);

  const char* sql =
    "SELECT * FROM upload_state WHERE status IN ('pending', 'active', 'retry-wait') "
    "ORDER BY created_at";

  sqlite3_stmt* stmt = nullptr;
  int rc = sqlite3_prepare_v2(impl_->db, sql, -1, &stmt, nullptr);
  if (rc != SQLITE_OK) {
    return {};
  }

  std::vector<UploadRecord> records;
  // LCOV_EXCL_BR_START - Vector operations generate exception-safety branches
  // that are standard library implementation details
  while (sqlite3_step(stmt) == SQLITE_ROW) {
    records.push_back(Impl::parseRecord(stmt));
  }
  // LCOV_EXCL_BR_STOP

  sqlite3_finalize(stmt);
  return records;
}

size_t UploadStateManager::countByStatus(UploadStatus status) {
  std::lock_guard<std::mutex> lock(impl_->mutex);

  const char* sql = "SELECT COUNT(*) FROM upload_state WHERE status = ?";

  sqlite3_stmt* stmt = nullptr;
  int rc = sqlite3_prepare_v2(impl_->db, sql, -1, &stmt, nullptr);
  if (rc != SQLITE_OK) {
    return 0;
  }

  sqlite3_bind_text(stmt, 1, uploadStatusToString(status).c_str(), -1, SQLITE_TRANSIENT);

  rc = sqlite3_step(stmt);
  size_t count = 0;
  if (rc == SQLITE_ROW) {
    count = static_cast<size_t>(sqlite3_column_int64(stmt, 0));
  }

  sqlite3_finalize(stmt);
  return count;
}

uint64_t UploadStateManager::pendingBytes() {
  std::lock_guard<std::mutex> lock(impl_->mutex);

  const char* sql =
    "SELECT COALESCE(SUM(file_size_bytes), 0) FROM upload_state WHERE status IN ('pending', "
    "'active', 'retry-wait')";

  sqlite3_stmt* stmt = nullptr;
  int rc = sqlite3_prepare_v2(impl_->db, sql, -1, &stmt, nullptr);
  if (rc != SQLITE_OK) {
    return 0;
  }

  rc = sqlite3_step(stmt);
  uint64_t bytes = 0;
  if (rc == SQLITE_ROW) {
    bytes = static_cast<uint64_t>(sqlite3_column_int64(stmt, 0));
  }

  sqlite3_finalize(stmt);
  return bytes;
}

bool UploadStateManager::updateDeleteRetry(
  const std::string& file_path, int retry_count, const std::string& next_retry_at,
  const std::string& error
) {
  std::lock_guard<std::mutex> lock(impl_->mutex);

  const char* sql = R"(
    UPDATE upload_state
    SET delete_retry_count = ?,
        delete_next_retry_at = ?,
        delete_last_error = ?,
        updated_at = ?
    WHERE file_path = ?
  )";

  sqlite3_stmt* stmt = nullptr;
  int rc = sqlite3_prepare_v2(impl_->db, sql, -1, &stmt, nullptr);
  if (rc != SQLITE_OK) {
    return false;
  }

  std::string now = currentTimestamp();
  sqlite3_bind_int(stmt, 1, retry_count);
  sqlite3_bind_text(stmt, 2, next_retry_at.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 3, error.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 4, now.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 5, file_path.c_str(), -1, SQLITE_TRANSIENT);

  rc = sqlite3_step(stmt);
  sqlite3_finalize(stmt);

  return rc == SQLITE_DONE && sqlite3_changes(impl_->db) > 0;
}

int UploadStateManager::deleteOlderThan(std::chrono::hours age) {
  std::lock_guard<std::mutex> lock(impl_->mutex);

  // Calculate cutoff time
  auto cutoff = std::chrono::system_clock::now() - age;
  auto time = std::chrono::system_clock::to_time_t(cutoff);
  std::tm tm{};
#ifdef _WIN32
  gmtime_s(&tm, &time);  // Windows uses reversed argument order
#else
  gmtime_r(&time, &tm);
#endif

  // LCOV_EXCL_BR_START - String stream operations generate exception-safety branches
  // that are standard library implementation details
  std::ostringstream ss;
  ss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
  std::string cutoff_str = ss.str();
  // LCOV_EXCL_BR_STOP

  const char* sql = "DELETE FROM upload_state WHERE status = 'completed' AND created_at < ?";

  sqlite3_stmt* stmt = nullptr;
  int rc = sqlite3_prepare_v2(impl_->db, sql, -1, &stmt, nullptr);
  if (rc != SQLITE_OK) {
    return 0;
  }

  sqlite3_bind_text(stmt, 1, cutoff_str.c_str(), -1, SQLITE_TRANSIENT);

  rc = sqlite3_step(stmt);
  int deleted = sqlite3_changes(impl_->db);
  sqlite3_finalize(stmt);

  return deleted;
}

int UploadStateManager::deleteCompleted() {
  std::lock_guard<std::mutex> lock(impl_->mutex);

  const char* sql = "DELETE FROM upload_state WHERE status = 'completed'";

  char* err_msg = nullptr;
  int rc = sqlite3_exec(impl_->db, sql, nullptr, nullptr, &err_msg);
  if (rc != SQLITE_OK) {
    sqlite3_free(err_msg);
    return 0;
  }

  return sqlite3_changes(impl_->db);
}

const std::string& UploadStateManager::dbPath() const {
  return impl_->db_path;
}

}  // namespace uploader
}  // namespace axon
