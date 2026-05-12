// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_SYSTEM_PROCFS_READER_HPP
#define AXON_SYSTEM_PROCFS_READER_HPP

#include <filesystem>
#include <string>

namespace axon {
namespace system {

class ProcfsReader {
public:
  explicit ProcfsReader(std::filesystem::path root = "/proc");

  bool read_file(
    const std::filesystem::path& relative_path, std::string* output, std::string* error
  ) const;
  std::filesystem::path root() const;

private:
  std::filesystem::path root_;
};

}  // namespace system
}  // namespace axon

#endif  // AXON_SYSTEM_PROCFS_READER_HPP
