#include "image_compressor.hpp"

#include <arrow/buffer.h>
#include <arrow/builder.h>
#include <arrow/util/compression.h>

#include <chrono>
#include <cstring>
#include <iostream>

// Optional: Include libjpeg-turbo if available
#if __has_include(<turbojpeg.h>)
#define HAS_TURBOJPEG 1
#include <turbojpeg.h>
#elif __has_include(<jpeglib.h>)
#define HAS_JPEGLIB 1
#include <jpeglib.h>
#include <setjmp.h>
#else
#define HAS_JPEG 0
#endif

namespace axon {
namespace compression {

// =============================================================================
// NoCompressor Implementation
// =============================================================================

bool NoCompressor::compress(
  const ImageData& input, std::vector<uint8_t>& output, CompressionStats* stats
) {
  auto start = std::chrono::high_resolution_clock::now();

  output.resize(input.size);
  std::memcpy(output.data(), input.data, input.size);

  if (stats) {
    auto end = std::chrono::high_resolution_clock::now();
    stats->input_bytes = input.size;
    stats->output_bytes = output.size();
    stats->compression_time_us = std::chrono::duration<double, std::micro>(end - start).count();
    stats->calculate();
  }

  return true;
}

bool NoCompressor::decompress(
  const uint8_t* input, size_t input_size, uint8_t* output, size_t output_size
) {
  if (input_size != output_size) {
    return false;
  }
  std::memcpy(output, input, input_size);
  return true;
}

// =============================================================================
// Lz4Compressor Implementation
// =============================================================================

Lz4Compressor::Lz4Compressor(int level)
    : level_(level) {
  auto result = arrow::util::Codec::Create(arrow::Compression::LZ4_FRAME);
  if (result.ok()) {
    codec_ = std::move(result.ValueOrDie());
  }
}

bool Lz4Compressor::compress(
  const ImageData& input, std::vector<uint8_t>& output, CompressionStats* stats
) {
  if (!codec_) {
    return false;
  }

  auto start = std::chrono::high_resolution_clock::now();

  // Get max compressed size
  int64_t max_compressed = codec_->MaxCompressedLen(input.size, input.data);
  output.resize(max_compressed);

  // Compress
  auto result = codec_->Compress(input.size, input.data, max_compressed, output.data());
  if (!result.ok()) {
    std::cerr << "LZ4 compression failed: " << result.status().ToString() << std::endl;
    return false;
  }

  output.resize(result.ValueOrDie());

  if (stats) {
    auto end = std::chrono::high_resolution_clock::now();
    stats->input_bytes = input.size;
    stats->output_bytes = output.size();
    stats->compression_time_us = std::chrono::duration<double, std::micro>(end - start).count();
    stats->calculate();
  }

  return true;
}

bool Lz4Compressor::decompress(
  const uint8_t* input, size_t input_size, uint8_t* output, size_t output_size
) {
  if (!codec_) {
    return false;
  }

  auto result = codec_->Decompress(input_size, input, output_size, output);
  return result.ok() && result.ValueOrDie() == static_cast<int64_t>(output_size);
}

// =============================================================================
// ZstdCompressor Implementation
// =============================================================================

ZstdCompressor::ZstdCompressor(int level)
    : level_(level) {
  auto result = arrow::util::Codec::Create(arrow::Compression::ZSTD, level);
  if (result.ok()) {
    codec_ = std::move(result.ValueOrDie());
  }
}

bool ZstdCompressor::compress(
  const ImageData& input, std::vector<uint8_t>& output, CompressionStats* stats
) {
  if (!codec_) {
    return false;
  }

  auto start = std::chrono::high_resolution_clock::now();

  // Get max compressed size
  int64_t max_compressed = codec_->MaxCompressedLen(input.size, input.data);
  output.resize(max_compressed);

  // Compress
  auto result = codec_->Compress(input.size, input.data, max_compressed, output.data());
  if (!result.ok()) {
    std::cerr << "ZSTD compression failed: " << result.status().ToString() << std::endl;
    return false;
  }

  output.resize(result.ValueOrDie());

  if (stats) {
    auto end = std::chrono::high_resolution_clock::now();
    stats->input_bytes = input.size;
    stats->output_bytes = output.size();
    stats->compression_time_us = std::chrono::duration<double, std::micro>(end - start).count();
    stats->calculate();
  }

  return true;
}

bool ZstdCompressor::decompress(
  const uint8_t* input, size_t input_size, uint8_t* output, size_t output_size
) {
  if (!codec_) {
    return false;
  }

  auto result = codec_->Decompress(input_size, input, output_size, output);
  return result.ok() && result.ValueOrDie() == static_cast<int64_t>(output_size);
}

// =============================================================================
// JpegCompressor Implementation
// =============================================================================

struct JpegCompressor::JpegContext {
#if HAS_TURBOJPEG
  tjhandle compress_handle = nullptr;
  tjhandle decompress_handle = nullptr;

  JpegContext() {
    compress_handle = tjInitCompress();
    decompress_handle = tjInitDecompress();
  }

  ~JpegContext() {
    if (compress_handle) tjDestroy(compress_handle);
    if (decompress_handle) tjDestroy(decompress_handle);
  }
#elif HAS_JPEGLIB
  // Standard libjpeg context
  struct jpeg_compress_struct cinfo;
  struct jpeg_decompress_struct dinfo;
  struct jpeg_error_mgr jerr;
  jmp_buf setjmp_buffer;
  bool error_occurred = false;
#endif
};

JpegCompressor::JpegCompressor(int quality)
    : quality_(std::clamp(quality, 1, 100))
    , ctx_(std::make_unique<JpegContext>()) {}

JpegCompressor::~JpegCompressor() = default;

bool JpegCompressor::compress(
  const ImageData& input, std::vector<uint8_t>& output, CompressionStats* stats
) {
  if (input.channels != 3 && input.channels != 1) {
    std::cerr << "JPEG only supports 1 or 3 channel images" << std::endl;
    return false;
  }

  auto start = std::chrono::high_resolution_clock::now();

#if HAS_TURBOJPEG
  if (!ctx_->compress_handle) {
    return false;
  }

  unsigned char* compressed = nullptr;
  unsigned long compressed_size = 0;

  int pixel_format = (input.channels == 3) ? TJPF_RGB : TJPF_GRAY;
  int subsamp = TJSAMP_420;

  int result = tjCompress2(
    ctx_->compress_handle,
    const_cast<uint8_t*>(input.data),
    input.width,
    0,  // pitch (0 = width * channels)
    input.height,
    pixel_format,
    &compressed,
    &compressed_size,
    subsamp,
    quality_,
    TJFLAG_FASTDCT
  );

  if (result != 0) {
    std::cerr << "TurboJPEG compression failed: " << tjGetErrorStr2(ctx_->compress_handle)
              << std::endl;
    if (compressed) tjFree(compressed);
    return false;
  }

  output.resize(compressed_size);
  std::memcpy(output.data(), compressed, compressed_size);
  tjFree(compressed);

#elif HAS_JPEGLIB
  // Standard libjpeg implementation
  ctx_->cinfo.err = jpeg_std_error(&ctx_->jerr);
  jpeg_create_compress(&ctx_->cinfo);

  // Memory destination
  unsigned char* outbuffer = nullptr;
  unsigned long outsize = 0;
  jpeg_mem_dest(&ctx_->cinfo, &outbuffer, &outsize);

  ctx_->cinfo.image_width = input.width;
  ctx_->cinfo.image_height = input.height;
  ctx_->cinfo.input_components = input.channels;
  ctx_->cinfo.in_color_space = (input.channels == 3) ? JCS_RGB : JCS_GRAYSCALE;

  jpeg_set_defaults(&ctx_->cinfo);
  jpeg_set_quality(&ctx_->cinfo, quality_, TRUE);
  jpeg_start_compress(&ctx_->cinfo, TRUE);

  int row_stride = input.width * input.channels;
  JSAMPROW row_pointer[1];

  while (ctx_->cinfo.next_scanline < ctx_->cinfo.image_height) {
    row_pointer[0] = const_cast<JSAMPLE*>(&input.data[ctx_->cinfo.next_scanline * row_stride]);
    jpeg_write_scanlines(&ctx_->cinfo, row_pointer, 1);
  }

  jpeg_finish_compress(&ctx_->cinfo);
  jpeg_destroy_compress(&ctx_->cinfo);

  output.resize(outsize);
  std::memcpy(output.data(), outbuffer, outsize);
  free(outbuffer);

#else
  // No JPEG library available - fall back to no compression
  std::cerr << "JPEG compression not available (libjpeg not found)" << std::endl;
  output.resize(input.size);
  std::memcpy(output.data(), input.data, input.size);
#endif

  if (stats) {
    auto end = std::chrono::high_resolution_clock::now();
    stats->input_bytes = input.size;
    stats->output_bytes = output.size();
    stats->compression_time_us = std::chrono::duration<double, std::micro>(end - start).count();
    stats->calculate();
  }

  return true;
}

bool JpegCompressor::decompress(
  const uint8_t* input, size_t input_size, uint8_t* output, size_t output_size
) {
#if HAS_TURBOJPEG
  if (!ctx_->decompress_handle) {
    return false;
  }

  int width, height, subsamp, colorspace;
  int result = tjDecompressHeader3(
    ctx_->decompress_handle,
    const_cast<uint8_t*>(input),
    input_size,
    &width,
    &height,
    &subsamp,
    &colorspace
  );

  if (result != 0) {
    return false;
  }

  int pixel_format = TJPF_RGB;
  result = tjDecompress2(
    ctx_->decompress_handle,
    const_cast<uint8_t*>(input),
    input_size,
    output,
    width,
    0,
    height,
    pixel_format,
    TJFLAG_FASTDCT
  );

  return result == 0;

#elif HAS_JPEGLIB
  struct jpeg_decompress_struct dinfo;
  struct jpeg_error_mgr jerr;

  dinfo.err = jpeg_std_error(&jerr);
  jpeg_create_decompress(&dinfo);
  jpeg_mem_src(&dinfo, input, input_size);
  jpeg_read_header(&dinfo, TRUE);
  jpeg_start_decompress(&dinfo);

  int row_stride = dinfo.output_width * dinfo.output_components;
  JSAMPROW row_pointer[1];

  while (dinfo.output_scanline < dinfo.output_height) {
    row_pointer[0] = &output[dinfo.output_scanline * row_stride];
    jpeg_read_scanlines(&dinfo, row_pointer, 1);
  }

  jpeg_finish_decompress(&dinfo);
  jpeg_destroy_decompress(&dinfo);
  return true;

#else
  // No JPEG library - just copy
  std::memcpy(output, input, std::min(input_size, output_size));
  return true;
#endif
}

// =============================================================================
// CompressorFactory Implementation
// =============================================================================

std::unique_ptr<IImageCompressor> CompressorFactory::create(const CompressionParams& params) {
  switch (params.codec) {
    case CompressionCodec::NONE:
      return std::make_unique<NoCompressor>();
    case CompressionCodec::JPEG:
      return std::make_unique<JpegCompressor>(params.quality);
    case CompressionCodec::LZ4:
      return std::make_unique<Lz4Compressor>(params.compression_level);
    case CompressionCodec::ZSTD:
      return std::make_unique<ZstdCompressor>(params.compression_level);
    default:
      return std::make_unique<NoCompressor>();
  }
}

std::unique_ptr<IImageCompressor> CompressorFactory::create(CompressionCodec codec) {
  switch (codec) {
    case CompressionCodec::NONE:
      return std::make_unique<NoCompressor>();
    case CompressionCodec::JPEG:
      return std::make_unique<JpegCompressor>();
    case CompressionCodec::LZ4:
      return std::make_unique<Lz4Compressor>();
    case CompressionCodec::ZSTD:
      return std::make_unique<ZstdCompressor>();
    default:
      return std::make_unique<NoCompressor>();
  }
}

// =============================================================================
// AdaptiveCompressor Implementation
// =============================================================================

AdaptiveCompressor::AdaptiveCompressor()
    : rgb_compressor_(std::make_unique<JpegCompressor>(85))
    , depth_compressor_(std::make_unique<ZstdCompressor>(3)) {}

bool AdaptiveCompressor::compress_rgb(
  const ImageData& input, std::vector<uint8_t>& output, CompressionStats* stats
) {
  bool result = rgb_compressor_->compress(input, output, &rgb_stats_);
  if (stats) {
    *stats = rgb_stats_;
  }
  return result;
}

bool AdaptiveCompressor::compress_depth(
  const ImageData& input, std::vector<uint8_t>& output, CompressionStats* stats
) {
  bool result = depth_compressor_->compress(input, output, &depth_stats_);
  if (stats) {
    *stats = depth_stats_;
  }
  return result;
}

void AdaptiveCompressor::set_rgb_params(const CompressionParams& params) {
  rgb_compressor_ = CompressorFactory::create(params);
}

void AdaptiveCompressor::set_depth_params(const CompressionParams& params) {
  depth_compressor_ = CompressorFactory::create(params);
}

// =============================================================================
// Arrow Integration
// =============================================================================

arrow::Result<std::shared_ptr<arrow::BinaryArray>> compress_binary_array(
  const std::shared_ptr<arrow::BinaryArray>& input, IImageCompressor& compressor, int width,
  int height, int channels
) {
  arrow::BinaryBuilder builder;
  ARROW_RETURN_NOT_OK(builder.Reserve(input->length()));

  std::vector<uint8_t> compressed;

  for (int64_t i = 0; i < input->length(); ++i) {
    if (input->IsNull(i)) {
      ARROW_RETURN_NOT_OK(builder.AppendNull());
      continue;
    }

    auto value = input->GetView(i);
    ImageData img(
      reinterpret_cast<const uint8_t*>(value.data()), value.size(), width, height, channels
    );

    if (!compressor.compress(img, compressed)) {
      return arrow::Status::ExecutionError("Compression failed for element ", i);
    }

    ARROW_RETURN_NOT_OK(builder.Append(compressed.data(), compressed.size()));
  }

  std::shared_ptr<arrow::BinaryArray> result;
  ARROW_RETURN_NOT_OK(builder.Finish(&result));
  return result;
}

// =============================================================================
// Benchmark Helper
// =============================================================================

CompressionBenchmarkResult benchmark_compressor(
  IImageCompressor& compressor, const ImageData& test_data, int iterations
) {
  CompressionBenchmarkResult result;
  result.codec_name = compressor.codec_name();
  result.input_size = test_data.size;

  std::vector<uint8_t> compressed;
  std::vector<uint8_t> decompressed(test_data.size);

  // Warmup
  compressor.compress(test_data, compressed);
  compressor.decompress(
    compressed.data(), compressed.size(), decompressed.data(), decompressed.size()
  );

  // Compression benchmark
  double total_compress_time = 0;
  for (int i = 0; i < iterations; ++i) {
    CompressionStats stats;
    compressor.compress(test_data, compressed, &stats);
    total_compress_time += stats.compression_time_us;
  }

  result.output_size = compressed.size();
  result.compression_ratio = static_cast<double>(test_data.size) / compressed.size();
  result.compress_time_ms = (total_compress_time / iterations) / 1000.0;
  result.compress_throughput_mbps =
    (test_data.size / 1024.0 / 1024.0) / (result.compress_time_ms / 1000.0);

  // Decompression benchmark
  auto start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < iterations; ++i) {
    compressor.decompress(
      compressed.data(), compressed.size(), decompressed.data(), decompressed.size()
    );
  }
  auto end = std::chrono::high_resolution_clock::now();

  result.decompress_time_ms =
    std::chrono::duration<double, std::milli>(end - start).count() / iterations;
  result.decompress_throughput_mbps =
    (test_data.size / 1024.0 / 1024.0) / (result.decompress_time_ms / 1000.0);

  return result;
}

}  // namespace compression
}  // namespace axon
