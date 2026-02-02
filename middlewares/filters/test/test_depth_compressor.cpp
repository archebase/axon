// SPDX-FileCopyrightText: 2026 Copyright (c) 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include <gtest/gtest.h>

#include <depth_compressor.hpp>

namespace axon {
namespace depth {
namespace test {

// Test fixture for DepthCompressor
class DepthCompressorTest : public ::testing::Test {
protected:
  void SetUp() override {
    compressor_.set_config(config_);
  }

  // Create synthetic depth data
  std::vector<uint8_t> create_depth_data(size_t width, size_t height) {
    std::vector<uint8_t> data(width * height * 2);  // 16-bit = 2 bytes per pixel

    uint16_t* depth_ptr = reinterpret_cast<uint16_t*>(data.data());
    for (size_t i = 0; i < width * height; ++i) {
      // Create realistic depth pattern (0-10m in mm units)
      depth_ptr[i] = static_cast<uint16_t>(1000 + (i % 9000));
    }

    return data;
  }

  DepthCompressor compressor_;
  DepthCompressor::Config config_;
};

// Test basic compression functionality
TEST_F(DepthCompressorTest, Compress16UC1_Basic) {
  const size_t width = 640;
  const size_t height = 480;

  auto depth_data = create_depth_data(width, height);
  std::vector<uint8_t> compressed;

  ASSERT_TRUE(compressor_.compress(depth_data.data(), width, height, compressed));
  EXPECT_GT(compressed.size(), 0);
}

// Test compression ratio
TEST_F(DepthCompressorTest, CompressionRatio) {
  const size_t width = 640;
  const size_t height = 480;

  auto depth_data = create_depth_data(width, height);
  std::vector<uint8_t> compressed;

  ASSERT_TRUE(compressor_.compress(depth_data.data(), width, height, compressed));

  // Original size: 640 * 480 * 2 = 614400 bytes
  // Compressed should be significantly smaller (at least 2x)
  size_t original_size = width * height * 2;
  EXPECT_LT(compressed.size(), original_size);
  EXPECT_GT(compressed.size(), 0);

  double compression_ratio = static_cast<double>(original_size) / compressed.size();
  EXPECT_GT(compression_ratio, 2.0);  // At least 2x compression
}

// Test compression format strings
TEST_F(DepthCompressorTest, CompressionFormatStrings) {
  config_.level = axon::depth::CompressionLevel::kFast;
  compressor_.set_config(config_);
  EXPECT_EQ(compressor_.get_compression_format(), "dlz_fast");

  config_.level = axon::depth::CompressionLevel::kMedium;
  compressor_.set_config(config_);
  EXPECT_EQ(compressor_.get_compression_format(), "dlz_medium");

  config_.level = axon::depth::CompressionLevel::kMax;
  compressor_.set_config(config_);
  EXPECT_EQ(compressor_.get_compression_format(), "dlz_max");
}

// Test invalid inputs
TEST_F(DepthCompressorTest, InvalidInput) {
  std::vector<uint8_t> compressed;

  // Null data pointer
  EXPECT_FALSE(compressor_.compress(nullptr, 640, 480, compressed));

  // Zero width
  EXPECT_FALSE(compressor_.compress(reinterpret_cast<const uint8_t*>(0x1), 0, 480, compressed));

  // Zero height
  EXPECT_FALSE(compressor_.compress(reinterpret_cast<const uint8_t*>(0x1), 640, 0, compressed));
}

// Test different compression levels
TEST_F(DepthCompressorTest, DifferentCompressionLevels) {
  const size_t width = 640;
  const size_t height = 480;

  auto depth_data = create_depth_data(width, height);

  std::vector<size_t> compressed_sizes;

  for (auto level :
       {axon::depth::CompressionLevel::kFast,
        axon::depth::CompressionLevel::kMedium,
        axon::depth::CompressionLevel::kMax}) {
    config_.level = level;
    compressor_.set_config(config_);

    std::vector<uint8_t> compressed;
    ASSERT_TRUE(compressor_.compress(depth_data.data(), width, height, compressed));
    compressed_sizes.push_back(compressed.size());
  }

  // kFast should be largest (least compression)
  // kMax should be smallest (best compression)
  EXPECT_GE(compressed_sizes[0], compressed_sizes[1]);  // fast >= medium
  EXPECT_GE(compressed_sizes[1], compressed_sizes[2]);  // medium >= max
}

// Test different image sizes
TEST_F(DepthCompressorTest, DifferentImageSizes) {
  std::vector<std::pair<size_t, size_t>> sizes = {
    {320, 240}, {640, 480}, {1280, 720}, {1920, 1080}
  };

  for (const auto& [width, height] : sizes) {
    auto depth_data = create_depth_data(width, height);
    std::vector<uint8_t> compressed;

    ASSERT_TRUE(compressor_.compress(depth_data.data(), width, height, compressed))
      << "Failed for size: " << width << "x" << height;

    EXPECT_GT(compressed.size(), 0) << "Empty output for size: " << width << "x" << height;
  }
}

// Test config getter/setter
TEST_F(DepthCompressorTest, ConfigGetterSetter) {
  DepthCompressor::Config new_config;
  new_config.level = axon::depth::CompressionLevel::kMax;
  new_config.threading = axon::depth::Threading::kMulti;
  new_config.num_threads = 4;
  new_config.encoding = "16UC1";

  compressor_.set_config(new_config);

  const auto& retrieved_config = compressor_.get_config();
  EXPECT_EQ(retrieved_config.level, axon::depth::CompressionLevel::kMax);
  EXPECT_EQ(retrieved_config.threading, axon::depth::Threading::kMulti);
  EXPECT_EQ(retrieved_config.num_threads, 4);
  EXPECT_EQ(retrieved_config.encoding, "16UC1");
}

}  // namespace test
}  // namespace depth
}  // namespace axon
