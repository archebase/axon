// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "procfs_reader.hpp"

#include <fstream>
#include <sstream>
#include <utility>

namespace axon {
namespace system {

ProcfsReader::ProcfsReader(std::filesystem::path root)
    : root_(std::move(root)) {}

bool ProcfsReader::read_file(
  const std::filesystem::path& relative_path, std::string* output, std::string* error
) const {
  if (output == nullptr) {
    if (error != nullptr) {
      *error = "output pointer is null";
    }
    return false;
  }

  std::ifstream input(root_ / relative_path);
  if (!input) {
    if (error != nullptr) {
      *error = "failed to read " + (root_ / relative_path).string();
    }
    return false;
  }

  std::ostringstream stream;
  stream << input.rdbuf();
  *output = stream.str();
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

std::filesystem::path ProcfsReader::root() const {
  return root_;
}

}  // namespace system
}  // namespace axon
