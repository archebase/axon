// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "depth_compression_filter.hpp"

#include <ros/serialization.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

#include <cstring>

namespace ros1_plugin {

DepthCompressionFilter::DepthCompressionFilter(const DepthCompressionConfig& config)
    : config_(config) {
  // Configure compressor
  axon::depth::DepthCompressor::Config compressor_config;
  if (config.level == "fast") {
    compressor_config.level = dlz::CompressionLevel::kFast;
  } else if (config.level == "max") {
    compressor_config.level = dlz::CompressionLevel::kMax;
  } else {
    compressor_config.level = dlz::CompressionLevel::kMedium;
  }
  compressor_.set_config(compressor_config);
}

void DepthCompressionFilter::filter_and_process(
  const std::string& topic, const std::string& message_type, const std::vector<uint8_t>& data,
  uint64_t timestamp, ProcessedCallback callback
) {
  // Only process sensor_msgs/Image
  if (message_type != "sensor_msgs/Image") {
    // Not an image message, pass through directly
    callback(topic, message_type, data, timestamp);
    return;
  }

  // Try to extract and compress depth image
  try {
    auto [depth_data, width, height] = extract_depth_data(data);

    if (depth_data != nullptr && width > 0 && height > 0) {
      // Compress depth image
      std::vector<uint8_t> compressed_data;
      if (compressor_.compress(depth_data, width, height, compressed_data)) {
        // Build compressed image message
        std::string format = compressor_.get_compression_format();
        std::vector<uint8_t> compressed_msg =
          build_compressed_image_msg(format, compressed_data, timestamp);

        // Callback with processed message
        callback(topic, "sensor_msgs/CompressedImage", compressed_msg, timestamp);
        return;
      }
    }
  } catch (const std::exception& e) {
    // Compression failed, fall back to original message
  }

  // Compression failed or not a depth image, pass through original message
  callback(topic, message_type, data, timestamp);
}

std::tuple<const uint8_t*, size_t, size_t> DepthCompressionFilter::extract_depth_data(
  const std::vector<uint8_t>& image_msg
) {
  const uint8_t* data = nullptr;
  size_t width = 0;
  size_t height = 0;

  // Use ROS1 deserialization
  try {
    sensor_msgs::Image image;
    ros::serialization::IStream stream(const_cast<uint8_t*>(image_msg.data()), image_msg.size());
    ros::serialization::deserialize(stream, image);

    // Check if it's a 16UC1 depth image
    if (image.encoding == "16UC1") {
      data = image.data.data();
      width = image.width;
      height = image.height;
    }
  } catch (const std::exception& e) {
    // Deserialization failed
  }

  return {data, width, height};
}

std::vector<uint8_t> DepthCompressionFilter::build_compressed_image_msg(
  const std::string& format, const std::vector<uint8_t>& compressed_data, uint64_t timestamp
) {
  // Use ROS1 serialization for CompressedImage
  sensor_msgs::CompressedImage compressed_msg;

  // Set header
  compressed_msg.header.stamp.fromNSec(timestamp);
  compressed_msg.format = format;

  // Set compressed data
  compressed_msg.data = compressed_data;

  // Serialize
  uint32_t serial_size = ros::serialization::serializationLength(compressed_msg);
  std::vector<uint8_t> result(serial_size);

  ros::serialization::OStream stream(result.data(), serial_size);
  ros::serialization::serialize(stream, compressed_msg);

  return result;
}

}  // namespace ros1_plugin
