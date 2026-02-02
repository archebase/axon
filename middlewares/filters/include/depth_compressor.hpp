#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <dlz.hpp>

namespace axon {
namespace depth {

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
    dlz::CompressionLevel level = dlz::CompressionLevel::kMedium;
    dlz::Threading threading = dlz::Threading::kAuto;
    int num_threads = 0;  // 0 = auto-detect
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
    const uint8_t* depth_data,
    size_t width,
    size_t height,
    std::vector<uint8_t>& compressed_data
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
  const Config& get_config() const { return config_; }

private:
  Config config_;
};

} // namespace depth
} // namespace axon
