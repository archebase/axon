// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "upload_coordinator.hpp"

#include <boost/asio/post.hpp>

#include <iostream>

#include "edge_uploader.hpp"

namespace axon {
namespace transfer {

namespace {

std::string current_timestamp() {
  auto now = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now);
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", std::localtime(&time_t_now));
  char ms_buf[8];
  snprintf(ms_buf, sizeof(ms_buf), ".%03ldZ", ms.count());
  return std::string(buf) + ms_buf;
}

}  // namespace

UploadCoordinator::UploadCoordinator(
  const TransferConfig& config, WsClient& ws_client, FileScanner& scanner,
  std::shared_ptr<axon::uploader::EdgeUploader> uploader, boost::asio::io_context& ioc
)
    : config_(config)
    , ws_client_(ws_client)
    , scanner_(scanner)
    , uploader_(std::move(uploader))
    , ioc_(ioc) {
  uploader_->setCallback(
    [this, &ioc = ioc_](const std::string& task_id, bool success, const std::string& error) {
      boost::asio::post(ioc, [this, task_id, success, error]() {
        on_upload_complete(task_id, success, error);
      });
    }
  );
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

void UploadCoordinator::send_connected() {
  const auto& stats = uploader_->stats();

  nlohmann::json msg = {
    {"type", "connected"},
    {"timestamp", current_timestamp()},
    {"data",
     {{"version", "0.1.0"},
      {"device_id", config_.device_id},
      {"pending_count", static_cast<int>(stats.files_pending.load())},
      {"uploading_count", static_cast<int>(stats.files_uploading.load())},
      {"failed_count", static_cast<int>(stats.files_failed.load())}}}
  };
  ws_client_.send(msg);
}

void UploadCoordinator::send_status() {
  const auto& stats = uploader_->stats();

  nlohmann::json msg = {
    {"type", "status"},
    {"timestamp", current_timestamp()},
    {"data",
     {{"pending_count", static_cast<int>(stats.files_pending.load())},
      {"uploading_count", static_cast<int>(stats.files_uploading.load())},
      {"completed_count", static_cast<int>(stats.files_completed.load())},
      {"failed_count", static_cast<int>(stats.files_failed.load())},
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

}  // namespace transfer
}  // namespace axon
