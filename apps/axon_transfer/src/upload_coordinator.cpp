// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "upload_coordinator.hpp"

#include <boost/asio/post.hpp>
#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>

#include "edge_uploader.hpp"
#include "upload_state_manager.hpp"

namespace axon {
namespace transfer {

namespace {

std::string current_timestamp() {
  auto now = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now);
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

  char buf[32];
  std::tm tm{};
#ifdef _WIN32
  gmtime_s(&tm, &time_t_now);
#else
  gmtime_r(&time_t_now, &tm);
#endif
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &tm);
  char ms_buf[8];
  snprintf(ms_buf, sizeof(ms_buf), ".%03ldZ", ms.count());
  return std::string(buf) + ms_buf;
}

std::optional<std::chrono::system_clock::time_point> parse_timestamp(const std::string& value) {
  if (value.empty()) {
    return std::nullopt;
  }

  std::tm tm{};
  std::istringstream iss(value);
  iss >> std::get_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
  if (iss.fail()) {
    return std::nullopt;
  }

#ifdef _WIN32
  std::time_t tt = _mkgmtime(&tm);
#else
  std::time_t tt = timegm(&tm);
#endif
  if (tt < 0) {
    return std::nullopt;
  }
  return std::chrono::system_clock::from_time_t(tt);
}

}  // namespace

UploadCoordinator::UploadCoordinator(
  const TransferConfig& config, WsClient& ws_client, FileScanner& scanner,
  std::shared_ptr<axon::uploader::EdgeUploader> uploader,
  std::shared_ptr<axon::uploader::UploadStateManager> state_manager, boost::asio::io_context& ioc
)
    : config_(config)
    , ws_client_(ws_client)
    , scanner_(scanner)
    , uploader_(std::move(uploader))
    , state_manager_(std::move(state_manager))
    , ioc_(ioc)
    , cleanup_retry_timer_(ioc_) {
  uploader_->setCallback(
    [this, &ioc = ioc_](const std::string& task_id, bool success, const std::string& error) {
      boost::asio::post(ioc, [this, task_id, success, error]() {
        on_upload_complete(task_id, success, error);
      });
    }
  );

  schedule_next_cleanup_timer();
}

void UploadCoordinator::on_upload_request(const std::string& task_id, int priority) {
  auto group = scanner_.find(task_id);
  if (!group) {
    nlohmann::json msg = {
      {"type", "upload_not_found"},
      {"timestamp", current_timestamp()},
      {"data",
       {{"task_id", task_id},
        {"detail", "No MCAP file matching " + task_id + " in " + config_.scanner.data_dir}}}
    };
    ws_client_.send(msg);
    return;
  }

  enqueue(*group, priority);

  nlohmann::json msg = {
    {"type", "upload_started"},
    {"timestamp", current_timestamp()},
    {"data",
     {{"task_id", task_id},
      {"files", {task_id + ".mcap", task_id + ".json"}},
      {"total_bytes", group->mcap_size_bytes}}}
  };
  ws_client_.send(msg);
}

void UploadCoordinator::on_upload_all() {
  auto groups = scanner_.scan_all();

  for (const auto& group : groups) {
    enqueue(group, 0);
  }

  nlohmann::json msg = {
    {"type", "status"},
    {"timestamp", current_timestamp()},
    {"data",
     {{"pending_count", static_cast<int>(groups.size())},
      {"uploading_count", 0},
      {"completed_count", 0},
      {"failed_count", 0}}}
  };
  ws_client_.send(msg);
}

void UploadCoordinator::on_cancel(const std::string& task_id) {}

void UploadCoordinator::on_status_query() {
  send_status();
}

void UploadCoordinator::on_upload_ack(const std::string& task_id) {
  if (task_id.empty()) {
    return;
  }

  auto record = state_manager_->getByTaskId(task_id);
  if (!record) {
    nlohmann::json msg = {
      {"type", "upload_not_found"},
      {"timestamp", current_timestamp()},
      {"data", {{"task_id", task_id}, {"detail", "ACK for unknown task_id"}}}
    };
    ws_client_.send(msg);
    return;
  }

  if (record->status == axon::uploader::UploadStatus::COMPLETED) {
    return;
  }

  if (record->status != axon::uploader::UploadStatus::UPLOADED_WAIT_ACK) {
    return;
  }

  if (!config_.uploader.delete_after_upload) {
    state_manager_->markCompleted(record->file_path);
    return;
  }

  namespace fs = std::filesystem;
  std::error_code ec_mcap;
  std::error_code ec_json;
  const fs::path mcap_path(record->file_path);
  const fs::path json_path(record->json_path);
  const bool mcap_ok = !fs::exists(mcap_path) || fs::remove(mcap_path, ec_mcap);
  const bool json_ok =
    record->json_path.empty() || !fs::exists(json_path) || fs::remove(json_path, ec_json);

  if (mcap_ok && json_ok) {
    state_manager_->markCompleted(record->file_path);
    return;
  }

  std::string err = "cleanup failed";
  if (ec_mcap) {
    err += ": mcap=" + ec_mcap.message();
  }
  if (ec_json) {
    err += ", json=" + ec_json.message();
  }
  schedule_cleanup_retry(*record, err);
}

void UploadCoordinator::send_connected() {
  const auto pending_count =
    static_cast<int>(state_manager_->countByStatus(axon::uploader::UploadStatus::PENDING));
  const auto uploading_count =
    static_cast<int>(state_manager_->countByStatus(axon::uploader::UploadStatus::UPLOADING));
  const auto waiting_ack_count = static_cast<int>(
    state_manager_->countByStatus(axon::uploader::UploadStatus::UPLOADED_WAIT_ACK)
  );
  const auto completed_count =
    static_cast<int>(state_manager_->countByStatus(axon::uploader::UploadStatus::COMPLETED));
  const auto failed_count =
    static_cast<int>(state_manager_->countByStatus(axon::uploader::UploadStatus::FAILED));

  nlohmann::json msg = {
    {"type", "connected"},
    {"timestamp", current_timestamp()},
    {"data",
     {{"version", "0.1.0"},
      {"device_id", config_.device_id},
      {"pending_count", pending_count},
      {"uploading_count", uploading_count},
      {"waiting_ack_count", waiting_ack_count},
      {"completed_count", completed_count},
      {"failed_count", failed_count}}}
  };
  ws_client_.send(msg);
}

void UploadCoordinator::send_status() {
  const auto& stats = uploader_->stats();
  auto waiting_ack_records =
    state_manager_->getByStatus(axon::uploader::UploadStatus::UPLOADED_WAIT_ACK);
  nlohmann::json waiting_ack_task_ids = nlohmann::json::array();
  for (const auto& record : waiting_ack_records) {
    if (!record.task_id.empty()) {
      waiting_ack_task_ids.push_back(record.task_id);
    }
  }

  nlohmann::json msg = {
    {"type", "status"},
    {"timestamp", current_timestamp()},
    {"data",
     {{"pending_count",
       static_cast<int>(state_manager_->countByStatus(axon::uploader::UploadStatus::PENDING))},
      {"uploading_count",
       static_cast<int>(state_manager_->countByStatus(axon::uploader::UploadStatus::UPLOADING))},
      {"waiting_ack_count", static_cast<int>(waiting_ack_task_ids.size())},
      {"waiting_ack_task_ids", waiting_ack_task_ids},
      {"completed_count",
       static_cast<int>(state_manager_->countByStatus(axon::uploader::UploadStatus::COMPLETED))},
      {"failed_count",
       static_cast<int>(state_manager_->countByStatus(axon::uploader::UploadStatus::FAILED))},
      {"bytes_per_sec", static_cast<int>(stats.current_upload_bytes_per_sec.load())}}}
  };
  ws_client_.send(msg);
}

void UploadCoordinator::enqueue(const FileGroup& group, int priority) {
  uploader_->enqueue(
    group.mcap_path.string(),
    group.json_path.string(),
    group.task_id,
    config_.factory_id,
    config_.device_id,
    ""
  );
}

void UploadCoordinator::on_upload_complete(
  const std::string& task_id, bool success, const std::string& error
) {
  if (success) {
    nlohmann::json msg = {
      {"type", "upload_complete"},
      {"timestamp", current_timestamp()},
      {"data", {{"task_id", task_id}, {"bytes_uploaded", 0}, {"duration_ms", 0}}}
    };
    ws_client_.send(msg);
  } else {
    nlohmann::json msg = {
      {"type", "upload_failed"},
      {"timestamp", current_timestamp()},
      {"data", {{"task_id", task_id}, {"reason", error}, {"retry_count", 0}}}
    };
    ws_client_.send(msg);
  }
}

std::chrono::milliseconds UploadCoordinator::cleanup_delay_for_retry(int retry_count) const {
  const auto& cfg = config_.uploader.cleanup_retry;
  long long delay = static_cast<long long>(
    cfg.initial_delay_ms.count() * std::pow(cfg.backoff_multiplier, retry_count)
  );
  delay = std::min(delay, static_cast<long long>(cfg.max_delay_ms.count()));
  return std::chrono::milliseconds(delay);
}

void UploadCoordinator::schedule_cleanup_retry(
  const axon::uploader::UploadRecord& record, const std::string& error
) {
  const int next_retry_count = record.delete_retry_count + 1;
  const int max_retries = config_.uploader.cleanup_retry.max_retries;

  if (next_retry_count > max_retries) {
    state_manager_->markCompleted(record.file_path);
    nlohmann::json msg = {
      {"type", "local_cleanup_failed"},
      {"timestamp", current_timestamp()},
      {"data",
       {{"task_id", record.task_id},
        {"file_path", record.file_path},
        {"retry_count", record.delete_retry_count},
        {"reason", error}}}
    };
    ws_client_.send(msg);
    return;
  }

  const auto next_at =
    std::chrono::system_clock::now() + cleanup_delay_for_retry(record.delete_retry_count);
  const auto next_time = std::chrono::system_clock::to_time_t(next_at);
  std::tm tm{};
#ifdef _WIN32
  gmtime_s(&tm, &next_time);
#else
  gmtime_r(&next_time, &tm);
#endif
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm);
  state_manager_->updateDeleteRetry(record.file_path, next_retry_count, buf, error);
  schedule_next_cleanup_timer();
}

void UploadCoordinator::run_cleanup_retry_pass() {
  auto waiting_ack_records =
    state_manager_->getByStatus(axon::uploader::UploadStatus::UPLOADED_WAIT_ACK);
  const auto now = std::chrono::system_clock::now();

  for (const auto& record : waiting_ack_records) {
    if (record.delete_retry_count <= 0 || record.delete_next_retry_at.empty()) {
      continue;
    }
    const auto next_retry = parse_timestamp(record.delete_next_retry_at);
    if (!next_retry || next_retry.value() > now) {
      continue;
    }

    on_upload_ack(record.task_id);
  }

  schedule_next_cleanup_timer();
}

void UploadCoordinator::schedule_next_cleanup_timer() {
  auto waiting_ack_records =
    state_manager_->getByStatus(axon::uploader::UploadStatus::UPLOADED_WAIT_ACK);
  const auto now = std::chrono::system_clock::now();
  std::optional<std::chrono::system_clock::time_point> earliest;

  for (const auto& record : waiting_ack_records) {
    if (record.delete_retry_count <= 0 || record.delete_next_retry_at.empty()) {
      continue;
    }
    const auto ts = parse_timestamp(record.delete_next_retry_at);
    if (!ts) {
      continue;
    }
    if (!earliest || ts.value() < earliest.value()) {
      earliest = ts;
    }
  }

  cleanup_retry_timer_.cancel();
  if (!earliest) {
    return;
  }

  const auto delay =
    earliest.value() <= now
      ? std::chrono::milliseconds(0)
      : std::chrono::duration_cast<std::chrono::milliseconds>(earliest.value() - now);
  cleanup_retry_timer_.expires_after(delay);
  cleanup_retry_timer_.async_wait([this](const boost::system::error_code& ec) {
    if (!ec) {
      run_cleanup_retry_pass();
    }
  });
}

}  // namespace transfer
}  // namespace axon
