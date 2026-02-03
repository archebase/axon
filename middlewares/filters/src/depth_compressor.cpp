// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "depth_compressor.hpp"

#include <cstring>
#include <dlz.hpp>

namespace axon {
namespace depth {

bool DepthCompressor::compress(
  const uint8_t* depth_data, size_t width, size_t height, std::vector<uint8_t>& compressed_data
) {
  if (!depth_data || width == 0 || height == 0) {
    return false;
  }

  // Cast to int16_t (16UC1 encoding)
  const int16_t* depth_data_i16 = reinterpret_cast<const int16_t*>(depth_data);
  int num_pixels = static_cast<int>(width * height);

  // Compress using dlz library
  // Note: dlz::CompressEx takes non-const pointer but doesn't modify input (verified by inspection)
  // The const_cast is safe here - dlz only reads from the input buffer
  std::vector<char> compressed_buffer;
  size_t compressed_size;

  bool success = dlz::CompressEx(
    const_cast<int16_t*>(depth_data_i16),
    num_pixels,
    compressed_buffer,
    0,
    compressed_size,
    static_cast<dlz::CompressionLevel>(config_.level),
    static_cast<dlz::Threading>(config_.threading),
    config_.num_threads
  );

  if (!success) {
    return false;
  }

  // Copy to output vector
  compressed_data.assign(compressed_buffer.begin(), compressed_buffer.begin() + compressed_size);

  return true;
}

std::string DepthCompressor::get_compression_format() const {
  switch (config_.level) {
    case CompressionLevel::kFast:
      return "dlz_fast";
    case CompressionLevel::kMedium:
      return "dlz_medium";
    case CompressionLevel::kMax:
      return "dlz_max";
    default:
      return "dlz_medium";
  }
}

void DepthCompressor::set_config(const Config& config) {
  config_ = config;
}

}  // namespace depth
}  // namespace axon
