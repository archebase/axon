#include "depth_compressor.hpp"

#include <cstring>

namespace axon {
namespace depth {

bool DepthCompressor::compress(
  const uint8_t* depth_data,
  size_t width,
  size_t height,
  std::vector<uint8_t>& compressed_data
) {
  if (!depth_data || width == 0 || height == 0) {
    return false;
  }

  // Cast to int16_t (16UC1 encoding)
  const int16_t* depth_data_i16 = reinterpret_cast<const int16_t*>(depth_data);
  int num_pixels = static_cast<int>(width * height);

  // Compress using dlz library
  std::vector<char> compressed_buffer;
  size_t compressed_size;

  bool success = dlz::CompressEx(
    const_cast<int16_t*>(depth_data_i16),
    num_pixels,
    compressed_buffer,
    0,
    compressed_size,
    config_.level,
    config_.threading,
    config_.num_threads
  );

  if (!success) {
    return false;
  }

  // Copy to output vector
  compressed_data.assign(
    compressed_buffer.begin(),
    compressed_buffer.begin() + compressed_size
  );

  return true;
}

std::string DepthCompressor::get_compression_format() const {
  switch (config_.level) {
    case dlz::CompressionLevel::kFast:
      return "dlz_fast";
    case dlz::CompressionLevel::kMedium:
      return "dlz_medium";
    case dlz::CompressionLevel::kMax:
      return "dlz_max";
    default:
      return "dlz_medium";
  }
}

void DepthCompressor::set_config(const Config& config) {
  config_ = config;
}

} // namespace depth
} // namespace axon
