// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "file_scanner.hpp"

#include <algorithm>

#include "../../../core/axon_uploader/upload_state_manager.hpp"

namespace axon {
namespace transfer {

namespace fs = boost::filesystem;

FileScanner::FileScanner(
  const ScannerConfig& config, std::shared_ptr<axon::uploader::UploadStateManager> state_manager
)
    : config_(config)
    , state_manager_(std::move(state_manager)) {}

std::optional<FileGroup> FileScanner::find(const std::string& task_id) const {
  fs::path data_dir(config_.data_dir);
  fs::path mcap_path = data_dir / (task_id + ".mcap");
  fs::path json_path = data_dir / (task_id + ".json");

  if (!fs::exists(mcap_path)) {
    return std::nullopt;
  }

  if (config_.require_json_sidecar && !fs::exists(json_path)) {
    return std::nullopt;
  }

  auto record = state_manager_->get(mcap_path.string());
  if (record) {
    if (record->status == axon::uploader::UploadStatus::COMPLETED ||
        record->status == axon::uploader::UploadStatus::UPLOADING ||
        record->status == axon::uploader::UploadStatus::UPLOADED_WAIT_ACK) {
      return std::nullopt;
    }
  }

  FileGroup group;
  group.mcap_path = mcap_path;
  group.json_path = json_path;
  group.task_id = task_id;
  group.mcap_size_bytes = fs::file_size(mcap_path);

  return group;
}

std::vector<FileGroup> FileScanner::scan_all() const {
  std::vector<FileGroup> results;

  fs::path data_dir(config_.data_dir);
  if (!fs::exists(data_dir) || !fs::is_directory(data_dir)) {
    return results;
  }

  for (const auto& entry : fs::recursive_directory_iterator(data_dir)) {
    if (!fs::is_regular_file(entry)) {
      continue;
    }

    if (entry.path().extension() != ".mcap") {
      continue;
    }

    fs::path mcap_path = entry.path();
    fs::path json_path = mcap_path.parent_path() / (mcap_path.stem().string() + ".json");

    if (config_.require_json_sidecar && !fs::exists(json_path)) {
      continue;
    }

    auto record = state_manager_->get(mcap_path.string());
    if (record) {
      if (record->status == axon::uploader::UploadStatus::COMPLETED ||
          record->status == axon::uploader::UploadStatus::UPLOADING ||
          record->status == axon::uploader::UploadStatus::UPLOADED_WAIT_ACK) {
        continue;
      }
    }

    FileGroup group;
    group.mcap_path = mcap_path;
    group.json_path = json_path;
    group.task_id = mcap_path.stem().string();
    group.mcap_size_bytes = fs::file_size(mcap_path);

    results.push_back(group);
  }

  std::sort(results.begin(), results.end(), [](const FileGroup& a, const FileGroup& b) {
    return fs::last_write_time(a.mcap_path) < fs::last_write_time(b.mcap_path);
  });

  return results;
}

}  // namespace transfer
}  // namespace axon
