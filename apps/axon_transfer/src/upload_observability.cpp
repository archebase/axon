// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "upload_observability.hpp"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <string>
#include <vector>

namespace axon {
namespace transfer {

namespace {

std::string lowercase_copy(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

bool contains_sensitive_marker(const std::string& value) {
  const std::string lower = lowercase_copy(value);
  return lower.find("token") != std::string::npos || lower.find("secret") != std::string::npos ||
         lower.find("password") != std::string::npos ||
         lower.find("credential") != std::string::npos ||
         lower.find("authorization") != std::string::npos ||
         lower.find("access_key") != std::string::npos ||
         lower.find("access-key") != std::string::npos;
}

std::string sanitize_error(const std::string& error) {
  if (error.empty()) {
    return "";
  }
  return contains_sensitive_marker(error) ? "[REDACTED]" : error;
}

std::string error_category(const std::string& error) {
  if (error.empty()) {
    return "";
  }

  const std::string lower = lowercase_copy(error);
  if (contains_sensitive_marker(error)) {
    return "authorization";
  }
  if (lower.find("timeout") != std::string::npos || lower.find("timed out") != std::string::npos) {
    return "timeout";
  }
  if (lower.find("credential") != std::string::npos ||
      lower.find("authorization") != std::string::npos ||
      lower.find("access denied") != std::string::npos ||
      lower.find("forbidden") != std::string::npos ||
      lower.find("signature") != std::string::npos) {
    return "authorization";
  }
  if (lower.find("checksum") != std::string::npos || lower.find("sha256") != std::string::npos) {
    return "checksum";
  }
  if (lower.find("not found") != std::string::npos ||
      lower.find("no such file") != std::string::npos ||
      lower.find("missing") != std::string::npos) {
    return "missing_file";
  }
  if (lower.find("network") != std::string::npos || lower.find("connect") != std::string::npos ||
      lower.find("resolve") != std::string::npos) {
    return "network";
  }
  if (lower.find("bucket") != std::string::npos || lower.find("s3") != std::string::npos ||
      lower.find("endpoint") != std::string::npos) {
    return "destination";
  }
  return "unknown";
}

std::string sanitize_endpoint_url(const std::string& endpoint_url) {
  if (endpoint_url.empty()) {
    return "";
  }

  std::string sanitized = endpoint_url;
  const auto scheme_pos = sanitized.find("://");
  const auto authority_start = scheme_pos == std::string::npos ? 0 : scheme_pos + 3;
  const auto path_start = sanitized.find('/', authority_start);
  const auto userinfo_end = sanitized.find('@', authority_start);
  if (userinfo_end != std::string::npos &&
      (path_start == std::string::npos || userinfo_end < path_start)) {
    sanitized.erase(authority_start, userinfo_end - authority_start + 1);
  }

  const auto query_start = sanitized.find('?');
  if (query_start != std::string::npos) {
    sanitized.erase(query_start);
  }
  return sanitized;
}

nlohmann::json upload_record_to_json(const axon::uploader::UploadRecord& record) {
  const bool mcap_only = record.json_path.empty();
  return {
    {"task_id", record.task_id},
    {"status", axon::uploader::uploadStatusToString(record.status)},
    {"upload_mode", mcap_only ? "mcap_only" : "mcap_json"},
    {"file_path", record.file_path},
    {"json_path", mcap_only ? nlohmann::json(nullptr) : nlohmann::json(record.json_path)},
    {"s3_key", record.s3_key},
    {"object_key", record.s3_key},
    {"file_size_bytes", record.file_size_bytes},
    {"checksum_sha256", record.checksum_sha256},
    {"retry_count", record.retry_count},
    {"last_error", sanitize_error(record.last_error)},
    {"last_error_category", error_category(record.last_error)},
    {"next_retry_at",
     record.next_retry_at.empty() ? nlohmann::json(nullptr) : nlohmann::json(record.next_retry_at)},
    {"updated_at", record.updated_at},
    {"completed_at",
     record.completed_at.empty() ? nlohmann::json(nullptr) : nlohmann::json(record.completed_at)},
    {"delete_retry_count", record.delete_retry_count},
    {"delete_next_retry_at",
     record.delete_next_retry_at.empty() ? nlohmann::json(nullptr)
                                         : nlohmann::json(record.delete_next_retry_at)},
    {"delete_last_error", sanitize_error(record.delete_last_error)},
    {"delete_last_error_category", error_category(record.delete_last_error)}
  };
}

uint64_t sum_bytes(const std::vector<axon::uploader::UploadRecord>& records) {
  uint64_t total = 0;
  for (const auto& record : records) {
    total += record.file_size_bytes;
  }
  return total;
}

void append_records(
  nlohmann::json& target, const std::vector<axon::uploader::UploadRecord>& records
) {
  for (const auto& record : records) {
    target.push_back(upload_record_to_json(record));
  }
}

}  // namespace

nlohmann::json build_upload_observability_status(
  const TransferConfig& config, axon::uploader::UploadStateManager& state_manager,
  const axon::uploader::UploaderStats& stats
) {
  auto pending_records = state_manager.getByStatus(axon::uploader::UploadStatus::PENDING);
  auto active_records = state_manager.getByStatus(axon::uploader::UploadStatus::ACTIVE);
  auto retry_wait_records = state_manager.getByStatus(axon::uploader::UploadStatus::RETRY_WAIT);
  auto waiting_ack_records =
    state_manager.getByStatus(axon::uploader::UploadStatus::UPLOADED_WAIT_ACK);
  auto completed_records = state_manager.getByStatus(axon::uploader::UploadStatus::COMPLETED);
  auto failed_records = state_manager.getByStatus(axon::uploader::UploadStatus::FAILED);

  nlohmann::json waiting_ack_task_ids = nlohmann::json::array();
  for (const auto& record : waiting_ack_records) {
    if (!record.task_id.empty()) {
      waiting_ack_task_ids.push_back(record.task_id);
    }
  }

  nlohmann::json uploads = nlohmann::json::array();
  append_records(uploads, pending_records);
  append_records(uploads, active_records);
  append_records(uploads, retry_wait_records);
  append_records(uploads, waiting_ack_records);
  append_records(uploads, completed_records);
  append_records(uploads, failed_records);

  const uint64_t pending_bytes = sum_bytes(pending_records);
  const uint64_t active_bytes = sum_bytes(active_records);
  const uint64_t retry_wait_bytes = sum_bytes(retry_wait_records);
  const uint64_t waiting_ack_bytes = sum_bytes(waiting_ack_records);
  const uint64_t completed_bytes = sum_bytes(completed_records);
  const uint64_t failed_bytes = sum_bytes(failed_records);
  const uint64_t tracked_bytes = pending_bytes + active_bytes + retry_wait_bytes +
                                 waiting_ack_bytes + completed_bytes + failed_bytes;
  const uint64_t recovered_count = stats.files_recovered_from_state.load();

  return {
    {"pending_count", static_cast<int>(pending_records.size())},
    {"active_count", static_cast<int>(active_records.size())},
    {"uploading_count", static_cast<int>(active_records.size())},
    {"retry_wait_count", static_cast<int>(retry_wait_records.size())},
    {"waiting_ack_count", static_cast<int>(waiting_ack_records.size())},
    {"waiting_ack_task_ids", waiting_ack_task_ids},
    {"completed_count", static_cast<int>(completed_records.size())},
    {"failed_count", static_cast<int>(failed_records.size())},
    {"status_counts",
     {{"pending", pending_records.size()},
      {"active", active_records.size()},
      {"retry_wait", retry_wait_records.size()},
      {"uploaded_wait_ack", waiting_ack_records.size()},
      {"completed", completed_records.size()},
      {"failed", failed_records.size()}}},
    {"pending_bytes", state_manager.pendingBytes()},
    {"progress",
     {{"bytes_uploaded", stats.bytes_uploaded.load()},
      {"current_bytes_per_sec", stats.current_upload_bytes_per_sec.load()},
      {"pending_bytes", pending_bytes},
      {"active_bytes", active_bytes},
      {"retry_wait_bytes", retry_wait_bytes},
      {"waiting_ack_bytes", waiting_ack_bytes},
      {"completed_bytes", completed_bytes},
      {"failed_bytes", failed_bytes},
      {"tracked_bytes", tracked_bytes}}},
    {"destination",
     {{"bucket", config.uploader.s3.bucket},
      {"endpoint_url", sanitize_endpoint_url(config.uploader.s3.endpoint_url)},
      {"region", config.uploader.s3.region}}},
    {"recovery",
     {{"state_db_path", config.uploader.state_db_path},
      {"recovered_from_state", recovered_count > 0},
      {"recovered_count", recovered_count},
      {"incomplete_count",
       pending_records.size() + active_records.size() + retry_wait_records.size()}}},
    {"uploads", uploads},
    {"bytes_per_sec", stats.current_upload_bytes_per_sec.load()}
  };
}

}  // namespace transfer
}  // namespace axon
