#ifndef IMAGE_COMPRESSOR_HPP
#define IMAGE_COMPRESSOR_HPP

#include <arrow/api.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace axon {
namespace compression {

/**
 * Compression codec types supported for image data
 */
enum class CompressionCodec {
  NONE,  // No compression (raw data)
  JPEG,  // Lossy JPEG compression (~10:1 for natural images)
  LZ4,   // Fast lossless compression (~2:1)
  ZSTD,  // High-ratio lossless compression (~3-5:1)
};

/**
 * Compression parameters
 */
struct CompressionParams {
  CompressionCodec codec = CompressionCodec::ZSTD;
  int quality = 85;           // JPEG quality (1-100)
  int compression_level = 3;  // ZSTD/LZ4 level (1-22 for ZSTD, 1-12 for LZ4)

  static CompressionParams jpeg(int quality = 85) {
    return CompressionParams{CompressionCodec::JPEG, quality, 0};
  }

  static CompressionParams lz4(int level = 1) {
    return CompressionParams{CompressionCodec::LZ4, 0, level};
  }

  static CompressionParams zstd(int level = 3) {
    return CompressionParams{CompressionCodec::ZSTD, 0, level};
  }

  static CompressionParams none() {
    return CompressionParams{CompressionCodec::NONE, 0, 0};
  }
};

/**
 * Compression statistics for monitoring
 */
struct CompressionStats {
  size_t input_bytes = 0;
  size_t output_bytes = 0;
  double compression_ratio = 1.0;
  double compression_time_us = 0.0;
  double throughput_mbps = 0.0;

  void calculate() {
    if (output_bytes > 0) {
      compression_ratio = static_cast<double>(input_bytes) / output_bytes;
    }
    if (compression_time_us > 0) {
      throughput_mbps = (input_bytes / 1024.0 / 1024.0) / (compression_time_us / 1e6);
    }
  }
};

/**
 * Image data structure for compression
 */
struct ImageData {
  const uint8_t* data;
  size_t size;
  int width;
  int height;
  int channels;  // 1=grayscale, 3=RGB, 4=RGBA

  ImageData(const uint8_t* d, size_t s, int w, int h, int c)
      : data(d)
      , size(s)
      , width(w)
      , height(h)
      , channels(c) {}

  // Constructor for depth images (16-bit per pixel)
  static ImageData depth(const uint8_t* d, int w, int h) {
    return ImageData(d, w * h * 2, w, h, 1);
  }

  // Constructor for RGB images
  static ImageData rgb(const uint8_t* d, int w, int h) {
    return ImageData(d, w * h * 3, w, h, 3);
  }
};

/**
 * Abstract interface for image compression
 */
class IImageCompressor {
public:
  virtual ~IImageCompressor() = default;

  /**
   * Compress image data
   * @param input Input image data
   * @param output Output buffer (will be resized)
   * @param stats Optional statistics output
   * @return true on success
   */
  virtual bool compress(
    const ImageData& input, std::vector<uint8_t>& output, CompressionStats* stats = nullptr
  ) = 0;

  /**
   * Decompress image data
   * @param input Compressed data
   * @param input_size Size of compressed data
   * @param output Output buffer (must be pre-allocated to expected size)
   * @param output_size Expected output size
   * @return true on success
   */
  virtual bool decompress(
    const uint8_t* input, size_t input_size, uint8_t* output, size_t output_size
  ) = 0;

  /**
   * Get the codec type
   */
  virtual CompressionCodec codec() const = 0;

  /**
   * Get human-readable codec name
   */
  virtual std::string codec_name() const = 0;
};

/**
 * No-op compressor (passthrough)
 */
class NoCompressor : public IImageCompressor {
public:
  bool compress(
    const ImageData& input, std::vector<uint8_t>& output, CompressionStats* stats = nullptr
  ) override;

  bool decompress(
    const uint8_t* input, size_t input_size, uint8_t* output, size_t output_size
  ) override;

  CompressionCodec codec() const override {
    return CompressionCodec::NONE;
  }
  std::string codec_name() const override {
    return "none";
  }
};

/**
 * LZ4 compressor using Arrow's built-in LZ4 codec
 */
class Lz4Compressor : public IImageCompressor {
public:
  explicit Lz4Compressor(int level = 1);

  bool compress(
    const ImageData& input, std::vector<uint8_t>& output, CompressionStats* stats = nullptr
  ) override;

  bool decompress(
    const uint8_t* input, size_t input_size, uint8_t* output, size_t output_size
  ) override;

  CompressionCodec codec() const override {
    return CompressionCodec::LZ4;
  }
  std::string codec_name() const override {
    return "lz4";
  }

private:
  int level_;
  std::unique_ptr<arrow::util::Codec> codec_;
};

/**
 * ZSTD compressor using Arrow's built-in ZSTD codec
 */
class ZstdCompressor : public IImageCompressor {
public:
  explicit ZstdCompressor(int level = 3);

  bool compress(
    const ImageData& input, std::vector<uint8_t>& output, CompressionStats* stats = nullptr
  ) override;

  bool decompress(
    const uint8_t* input, size_t input_size, uint8_t* output, size_t output_size
  ) override;

  CompressionCodec codec() const override {
    return CompressionCodec::ZSTD;
  }
  std::string codec_name() const override {
    return "zstd";
  }

private:
  int level_;
  std::unique_ptr<arrow::util::Codec> codec_;
};

/**
 * JPEG compressor for RGB images
 * Note: Requires libjpeg-turbo for high performance
 * Falls back to basic JPEG if turbo not available
 */
class JpegCompressor : public IImageCompressor {
public:
  explicit JpegCompressor(int quality = 85);
  ~JpegCompressor();

  bool compress(
    const ImageData& input, std::vector<uint8_t>& output, CompressionStats* stats = nullptr
  ) override;

  bool decompress(
    const uint8_t* input, size_t input_size, uint8_t* output, size_t output_size
  ) override;

  CompressionCodec codec() const override {
    return CompressionCodec::JPEG;
  }
  std::string codec_name() const override {
    return "jpeg";
  }

  void set_quality(int quality) {
    quality_ = std::clamp(quality, 1, 100);
  }
  int quality() const {
    return quality_;
  }

private:
  int quality_;
  struct JpegContext;
  std::unique_ptr<JpegContext> ctx_;
};

/**
 * Factory for creating compressors
 */
class CompressorFactory {
public:
  static std::unique_ptr<IImageCompressor> create(const CompressionParams& params);
  static std::unique_ptr<IImageCompressor> create(CompressionCodec codec);
};

/**
 * Multi-format compressor that can handle different image types
 * Automatically selects the best codec based on image characteristics
 */
class AdaptiveCompressor {
public:
  AdaptiveCompressor();

  /**
   * Compress RGB image (uses JPEG by default)
   */
  bool compress_rgb(
    const ImageData& input, std::vector<uint8_t>& output, CompressionStats* stats = nullptr
  );

  /**
   * Compress depth image (uses ZSTD by default - lossless)
   */
  bool compress_depth(
    const ImageData& input, std::vector<uint8_t>& output, CompressionStats* stats = nullptr
  );

  /**
   * Set compression parameters for RGB
   */
  void set_rgb_params(const CompressionParams& params);

  /**
   * Set compression parameters for depth
   */
  void set_depth_params(const CompressionParams& params);

  /**
   * Get current RGB compressor stats
   */
  const CompressionStats& rgb_stats() const {
    return rgb_stats_;
  }

  /**
   * Get current depth compressor stats
   */
  const CompressionStats& depth_stats() const {
    return depth_stats_;
  }

private:
  std::unique_ptr<IImageCompressor> rgb_compressor_;
  std::unique_ptr<IImageCompressor> depth_compressor_;
  CompressionStats rgb_stats_;
  CompressionStats depth_stats_;
};

/**
 * Arrow integration: Compress an Arrow Binary array
 * Useful for compressing image data stored in Arrow format
 */
arrow::Result<std::shared_ptr<arrow::BinaryArray>> compress_binary_array(
  const std::shared_ptr<arrow::BinaryArray>& input, IImageCompressor& compressor, int width = 0,
  int height = 0, int channels = 0
);

/**
 * Benchmark helper: Run compression benchmark
 */
struct CompressionBenchmarkResult {
  std::string codec_name;
  size_t input_size;
  size_t output_size;
  double compression_ratio;
  double compress_time_ms;
  double decompress_time_ms;
  double compress_throughput_mbps;
  double decompress_throughput_mbps;
};

CompressionBenchmarkResult benchmark_compressor(
  IImageCompressor& compressor, const ImageData& test_data, int iterations = 10
);

}  // namespace compression
}  // namespace axon

#endif  // IMAGE_COMPRESSOR_HPP
