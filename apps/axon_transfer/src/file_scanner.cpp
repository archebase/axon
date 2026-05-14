// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "file_scanner.hpp"

#include <openssl/evp.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <utility>

#include "upload_state_manager.hpp"

namespace axon {
namespace transfer {

namespace fs = boost::filesystem;

namespace {

fs::path sidecar_path_for(const fs::path& mcap_path, const std::string& extension) {
  return mcap_path.parent_path() / (mcap_path.stem().string() + extension);
}

fs::path marker_path_for(const fs::path& mcap_path, const ScannerConfig& config) {
  return fs::path(mcap_path.string() + config.completion_marker_suffix);
}

bool exists_regular(const fs::path& path) {
  return fs::exists(path) && fs::is_regular_file(path);
}

bool is_old_enough(const fs::path& path, std::chrono::milliseconds min_age) {
  if (min_age.count() <= 0) {
    return true;
  }

  const std::time_t modified_time = fs::last_write_time(path);
  const auto modified_at = std::chrono::system_clock::from_time_t(modified_time);
  return std::chrono::system_clock::now() - modified_at >= min_age;
}

bool completion_sentinel_ready(
  const fs::path& mcap_path, const fs::path& sentinel_path, std::chrono::milliseconds min_age
) {
  if (!exists_regular(sentinel_path) || !is_old_enough(sentinel_path, min_age)) {
    return false;
  }

  return fs::last_write_time(sentinel_path) >= fs::last_write_time(mcap_path);
}

bool is_blocked_by_upload_state(const axon::uploader::UploadRecord& record) {
  switch (record.status) {
    case axon::uploader::UploadStatus::PENDING:
    case axon::uploader::UploadStatus::ACTIVE:
    case axon::uploader::UploadStatus::RETRY_WAIT:
    case axon::uploader::UploadStatus::UPLOADED_WAIT_ACK:
    case axon::uploader::UploadStatus::COMPLETED:
    case axon::uploader::UploadStatus::FAILED:
      return true;
  }
  return true;
}

std::string sha256_file(const fs::path& path) {
  std::ifstream input(path.string(), std::ios::binary);
  if (!input) {
    return "";
  }

  EVP_MD_CTX* context = EVP_MD_CTX_new();
  if (!context) {
    return "";
  }

  std::string checksum;
  if (EVP_DigestInit_ex(context, EVP_sha256(), nullptr) == 1) {
    std::array<char, 64 * 1024> buffer{};
    while (input.good()) {
      input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
      const std::streamsize read_count = input.gcount();
      if (read_count > 0 &&
          EVP_DigestUpdate(context, buffer.data(), static_cast<size_t>(read_count)) != 1) {
        EVP_MD_CTX_free(context);
        return "";
      }
    }

    std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
    unsigned int digest_len = 0;
    if (!input.bad() && EVP_DigestFinal_ex(context, digest.data(), &digest_len) == 1) {
      std::ostringstream out;
      out << std::hex << std::setfill('0');
      for (unsigned int i = 0; i < digest_len; ++i) {
        out << std::setw(2) << static_cast<int>(digest[i]);
      }
      checksum = out.str();
    }
  }

  EVP_MD_CTX_free(context);
  return checksum;
}

}  // namespace

FileScanner::FileScanner(
  const ScannerConfig& config, std::shared_ptr<axon::uploader::UploadStateManager> state_manager
)
    : config_(config)
    , state_manager_(std::move(state_manager)) {}

std::optional<FileGroup> FileScanner::find(const std::string& task_id) const {
  fs::path data_dir(config_.data_dir);
  fs::path mcap_path = data_dir / (task_id + ".mcap");
  fs::path json_path = sidecar_path_for(mcap_path, ".json");
  fs::path marker_path = marker_path_for(mcap_path, config_);

  if (!exists_regular(mcap_path)) {
    return std::nullopt;
  }

  const fs::path& sentinel_path = config_.require_json_sidecar ? json_path : marker_path;
  if (!completion_sentinel_ready(mcap_path, sentinel_path, config_.min_ready_age_ms)) {
    return std::nullopt;
  }

  if (!is_old_enough(mcap_path, config_.min_ready_age_ms)) {
    return std::nullopt;
  }

  auto record = state_manager_->get(mcap_path.string());
  if (record && is_blocked_by_upload_state(*record)) {
    return std::nullopt;
  }

  const std::string checksum = sha256_file(mcap_path);
  if (checksum.empty()) {
    return std::nullopt;
  }

  FileGroup group;
  group.mcap_path = mcap_path;
  group.json_path = config_.require_json_sidecar ? json_path : fs::path();
  group.completion_marker_path = config_.require_json_sidecar ? fs::path() : marker_path;
  group.task_id = task_id;
  group.mcap_size_bytes = fs::file_size(mcap_path);
  group.checksum_sha256 = checksum;

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
    fs::path json_path = sidecar_path_for(mcap_path, ".json");
    fs::path marker_path = marker_path_for(mcap_path, config_);

    const fs::path& sentinel_path = config_.require_json_sidecar ? json_path : marker_path;
    if (!completion_sentinel_ready(mcap_path, sentinel_path, config_.min_ready_age_ms)) {
      continue;
    }

    if (!is_old_enough(mcap_path, config_.min_ready_age_ms)) {
      continue;
    }

    auto record = state_manager_->get(mcap_path.string());
    if (record && is_blocked_by_upload_state(*record)) {
      continue;
    }

    const std::string checksum = sha256_file(mcap_path);
    if (checksum.empty()) {
      continue;
    }

    FileGroup group;
    group.mcap_path = mcap_path;
    group.json_path = config_.require_json_sidecar ? json_path : fs::path();
    group.completion_marker_path = config_.require_json_sidecar ? fs::path() : marker_path;
    group.task_id = mcap_path.stem().string();
    group.mcap_size_bytes = fs::file_size(mcap_path);
    group.checksum_sha256 = checksum;

    results.push_back(group);
  }

  std::sort(results.begin(), results.end(), [](const FileGroup& a, const FileGroup& b) {
    return fs::last_write_time(a.mcap_path) < fs::last_write_time(b.mcap_path);
  });

  return results;
}

}  // namespace transfer
}  // namespace axon
