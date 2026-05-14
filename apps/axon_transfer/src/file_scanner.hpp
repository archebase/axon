// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_TRANSFER_FILE_SCANNER_HPP
#define AXON_TRANSFER_FILE_SCANNER_HPP

#include <boost/filesystem.hpp>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "transfer_config.hpp"

namespace axon {
namespace uploader {
class UploadStateManager;
}

namespace transfer {

struct FileGroup {
  boost::filesystem::path mcap_path;
  boost::filesystem::path json_path;
  boost::filesystem::path completion_marker_path;
  std::string task_id;
  uint64_t mcap_size_bytes;
  std::string checksum_sha256;
};

class FileScanner {
public:
  FileScanner(
    const ScannerConfig& config, std::shared_ptr<axon::uploader::UploadStateManager> state_manager
  );

  std::optional<FileGroup> find(const std::string& task_id) const;
  std::vector<FileGroup> scan_all() const;

private:
  ScannerConfig config_;
  std::shared_ptr<axon::uploader::UploadStateManager> state_manager_;
};

}  // namespace transfer
}  // namespace axon

#endif  // AXON_TRANSFER_FILE_SCANNER_HPP
