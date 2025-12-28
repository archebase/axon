#include "upload_state_manager.hpp"

#include <sqlite3.h>

#include <chrono>
#include <iomanip>
#include <mutex>
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

  std::ostringstream ss;
  ss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
  return ss.str();
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

  void initDatabase() {
    int rc = sqlite3_open(db_path.c_str(), &db);
    if (rc != SQLITE_OK) {
      throw std::runtime_error("Cannot open SQLite database: " + db_path);  // LCOV_EXCL_BR_LINE
    }

    // Enable WAL mode for crash safety and concurrent reads
    char* err_msg = nullptr;
    rc = sqlite3_exec(db, "PRAGMA journal_mode=WAL;", nullptr, nullptr, &err_msg);
    if (rc != SQLITE_OK) {
      std::string error = err_msg ? err_msg : "Unknown error";
      sqlite3_free(err_msg);
      throw std::runtime_error("Failed to enable WAL mode: " + error);  // LCOV_EXCL_BR_LINE
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
        status TEXT NOT NULL CHECK(status IN ('pending', 'uploading', 'completed', 'failed')),
        retry_count INTEGER DEFAULT 0,
        last_error TEXT,
        created_at TEXT NOT NULL,
        updated_at TEXT NOT NULL,
        completed_at TEXT
      );
      CREATE INDEX IF NOT EXISTS idx_status ON upload_state(status);
      CREATE INDEX IF NOT EXISTS idx_task_id ON upload_state(task_id);
      CREATE INDEX IF NOT EXISTS idx_created_at ON upload_state(created_at);
    )";

    rc = sqlite3_exec(db, create_sql, nullptr, nullptr, &err_msg);
    if (rc != SQLITE_OK) {
      std::string error = err_msg ? err_msg : "Unknown error";
      sqlite3_free(err_msg);
      throw std::runtime_error("Failed to create tables: " + error);  // LCOV_EXCL_BR_LINE
    }
  }

  // Parse a record from a SELECT statement
  static UploadRecord parseRecord(sqlite3_stmt* stmt) {
    UploadRecord record;
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
    record.created_at = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 11));
    record.updated_at = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 12));
    record.completed_at = sqlite3_column_text(stmt, 13)
                              ? reinterpret_cast<const char*>(sqlite3_column_text(stmt, 13))
                              : "";
    return record;
  }
};

UploadStateManager::UploadStateManager(const std::string& db_path)
    : impl_(std::make_unique<Impl>())  // LCOV_EXCL_BR_LINE
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
     checksum_sha256, status, retry_count, last_error, created_at, updated_at)
    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
  )";

  sqlite3_stmt* stmt = nullptr;
  int rc = sqlite3_prepare_v2(impl_->db, sql, -1, &stmt, nullptr);
  if (rc != SQLITE_OK) {
    return false;
  }

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
  sqlite3_bind_text(stmt, 12, now.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 13, now.c_str(), -1, SQLITE_TRANSIENT);

  rc = sqlite3_step(stmt);
  sqlite3_finalize(stmt);

  return rc == SQLITE_DONE;
}

bool UploadStateManager::updateStatus(
    const std::string& file_path, UploadStatus status, const std::string& error
) {
  std::lock_guard<std::mutex> lock(impl_->mutex);

  const char* sql = R"(
    UPDATE upload_state 
    SET status = ?, last_error = ?, updated_at = ?
    WHERE file_path = ?
  )";

  sqlite3_stmt* stmt = nullptr;
  int rc = sqlite3_prepare_v2(impl_->db, sql, -1, &stmt, nullptr);
  if (rc != SQLITE_OK) {
    return false;
  }

  std::string now = currentTimestamp();
  sqlite3_bind_text(stmt, 1, uploadStatusToString(status).c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 2, error.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 3, now.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 4, file_path.c_str(), -1, SQLITE_TRANSIENT);

  rc = sqlite3_step(stmt);
  sqlite3_finalize(stmt);

  return rc == SQLITE_DONE && sqlite3_changes(impl_->db) > 0;
}

bool UploadStateManager::markCompleted(const std::string& file_path) {
  std::lock_guard<std::mutex> lock(impl_->mutex);

  const char* sql = R"(
    UPDATE upload_state 
    SET status = 'completed', completed_at = ?, updated_at = ?
    WHERE file_path = ?
  )";

  sqlite3_stmt* stmt = nullptr;
  int rc = sqlite3_prepare_v2(impl_->db, sql, -1, &stmt, nullptr);
  if (rc != SQLITE_OK) {
    return false;
  }

  std::string now = currentTimestamp();
  sqlite3_bind_text(stmt, 1, now.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 2, now.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 3, file_path.c_str(), -1, SQLITE_TRANSIENT);

  rc = sqlite3_step(stmt);
  sqlite3_finalize(stmt);

  return rc == SQLITE_DONE && sqlite3_changes(impl_->db) > 0;
}

bool UploadStateManager::markFailed(const std::string& file_path, const std::string& error) {
  return updateStatus(file_path, UploadStatus::FAILED, error);
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

  std::string now = currentTimestamp();
  sqlite3_bind_text(stmt, 1, error.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 2, now.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 3, file_path.c_str(), -1, SQLITE_TRANSIENT);

  rc = sqlite3_step(stmt);
  sqlite3_finalize(stmt);

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

std::vector<UploadRecord> UploadStateManager::getPending() {
  std::lock_guard<std::mutex> lock(impl_->mutex);

  const char* sql = "SELECT * FROM upload_state WHERE status = 'pending' ORDER BY created_at";

  sqlite3_stmt* stmt = nullptr;
  int rc = sqlite3_prepare_v2(impl_->db, sql, -1, &stmt, nullptr);
  if (rc != SQLITE_OK) {
    return {};
  }

  std::vector<UploadRecord> records;
  while (sqlite3_step(stmt) == SQLITE_ROW) {
    records.push_back(Impl::parseRecord(stmt));
  }

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
  while (sqlite3_step(stmt) == SQLITE_ROW) {
    records.push_back(Impl::parseRecord(stmt));
  }

  sqlite3_finalize(stmt);
  return records;
}

std::vector<UploadRecord> UploadStateManager::getIncomplete() {
  std::lock_guard<std::mutex> lock(impl_->mutex);

  const char* sql =
      "SELECT * FROM upload_state WHERE status IN ('pending', 'uploading') ORDER BY created_at";

  sqlite3_stmt* stmt = nullptr;
  int rc = sqlite3_prepare_v2(impl_->db, sql, -1, &stmt, nullptr);
  if (rc != SQLITE_OK) {
    return {};
  }

  std::vector<UploadRecord> records;
  while (sqlite3_step(stmt) == SQLITE_ROW) {
    records.push_back(Impl::parseRecord(stmt));
  }

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
      "'uploading')";

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

  std::ostringstream ss;
  ss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
  std::string cutoff_str = ss.str();

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

const std::string& UploadStateManager::dbPath() const { return impl_->db_path; }

}  // namespace uploader
}  // namespace axon

