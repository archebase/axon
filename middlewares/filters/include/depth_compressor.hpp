// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace axon {
namespace depth {

/**
 * @brief Compression level for depth compression
 */
enum class CompressionLevel : int { kFast = 0, kMedium = 1, kMax = 2 };

/**
 * @brief Threading mode for compression
 */
enum class Threading : int { kSingle = 0, kMulti = 1, kAuto = 2 };

/**
 * @brief Depth image compressor using DepthLiteZ library
 *
 * Provides compression for 16-bit depth images with configurable quality levels.
 * Supports RVL (Run-Length Variable Length) encoding with optional LZ4/ZSTD
 * secondary compression.
 */
class DepthCompressor {
public:
  /**
   * @brief Compression configuration
   */
  struct Config {
    CompressionLevel level = CompressionLevel::kMedium;
    Threading threading = Threading::kAuto;
    int num_threads = 0;             // 0 = auto-detect
    std::string encoding = "16UC1";  // Input encoding format
  };

  /**
   * @brief Compress depth image data
   *
   * @param depth_data 16-bit depth data (row-major)
   * @param width Image width
   * @param height Image height
   * @param compressed_data Output compressed data
   * @return true if compression succeeded
   */
  bool compress(
    const uint8_t* depth_data, size_t width, size_t height, std::vector<uint8_t>& compressed_data
  );

  /**
   * @brief Get compression format identifier
   *
   * Returns the format string for CompressedImage.format field
   * @return Format identifier ("dlz_fast", "dlz_medium", "dlz_max")
   */
  std::string get_compression_format() const;

  /**
   * @brief Set compression configuration
   */
  void set_config(const Config& config);

  /**
   * @brief Get current configuration
   */
  const Config& get_config() const {
    return config_;
  }

private:
  Config config_;
};

}  // namespace depth
}  // namespace axon
