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

  // Use CompressToFile to include DLZF header with width/height information
  // This is necessary because CompressedImage message doesn't contain image dimensions
  // Note: dlz::CompressToFile takes non-const pointer but doesn't modify input (verified by
  // inspection) The const_cast is safe here - dlz only reads from the input buffer
  std::vector<char> compressed_buffer;

  bool success = dlz::CompressToFile(
    const_cast<int16_t*>(depth_data_i16),
    static_cast<uint32_t>(width),
    static_cast<uint32_t>(height),
    compressed_buffer,
    static_cast<dlz::CompressionLevel>(config_.level),
    static_cast<dlz::Threading>(config_.threading)
  );

  if (!success) {
    return false;
  }

  // Copy to output vector (includes DLZF header with magic, width, height, level)
  compressed_data.assign(compressed_buffer.begin(), compressed_buffer.end());

  return true;
}

std::string DepthCompressor::get_compression_format() const {
  // DLZF file format: [magic(4)][width(4)][height(4)][level(4)][compressed data...]
  // The format is always "dlzf" since the compression level is stored in the header
  return "dlzf";
}

void DepthCompressor::set_config(const Config& config) {
  config_ = config;
}

}  // namespace depth
}  // namespace axon
