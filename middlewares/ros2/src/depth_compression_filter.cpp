// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "depth_compression_filter.hpp"

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cstring>

namespace ros2_plugin {

DepthCompressionFilter::DepthCompressionFilter(const DepthCompressionConfig& config)
    : config_(config) {
  // Configure compressor
  axon::depth::DepthCompressor::Config compressor_config;
  if (config_.level == "fast") {
    compressor_config.level = axon::depth::CompressionLevel::kFast;
  } else if (config_.level == "max") {
    compressor_config.level = axon::depth::CompressionLevel::kMax;
  } else {
    compressor_config.level = axon::depth::CompressionLevel::kMedium;
  }
  compressor_.set_config(compressor_config);
}

void DepthCompressionFilter::filter_and_process(
  const std::string& topic, const std::string& message_type, const std::vector<uint8_t>& data,
  uint64_t timestamp_ns, ProcessedCallback callback
) {
  // Only process sensor_msgs::msg::Image
  if (message_type != "sensor_msgs::msg::Image" && message_type != "sensor_msgs/msg/Image") {
    // Not an image message, pass through directly
    callback(topic, message_type, data, timestamp_ns);
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
          build_compressed_image_msg(format, compressed_data, timestamp_ns);

        // Callback with processed message
        callback(topic, "sensor_msgs/msg/CompressedImage", compressed_msg, timestamp_ns);
        return;
      }
    }
  } catch (const std::exception& e) {
    // Compression failed, fall back to original message
  }

  // Compression failed or not a depth image, pass through original message
  callback(topic, message_type, data, timestamp_ns);
}

std::tuple<const uint8_t*, size_t, size_t> DepthCompressionFilter::extract_depth_data(
  const std::vector<uint8_t>& image_msg
) {
  const uint8_t* data = nullptr;
  size_t width = 0;
  size_t height = 0;

  // Use ROS2 deserialization
  try {
    rclcpp::Serialization<sensor_msgs::msg::Image> serialization;
    sensor_msgs::msg::Image image;
    rclcpp::SerializedMessage serialized_msg;
    serialized_msg.reserve(image_msg.size());
    std::memcpy(
      serialized_msg.get_rcl_serialized_message().buffer, image_msg.data(), image_msg.size()
    );
    serialized_msg.get_rcl_serialized_message().buffer_length = image_msg.size();

    // Deserialize
    serialization.deserialize_message(&serialized_msg, &image);

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
  const std::string& format, const std::vector<uint8_t>& compressed_data, uint64_t timestamp_ns
) {
  // Use ROS2 serialization for CompressedImage
  sensor_msgs::msg::CompressedImage compressed_msg;

  // Set header
  compressed_msg.header.stamp.sec = static_cast<int32_t>(timestamp_ns / 1000000000ULL);
  compressed_msg.header.stamp.nanosec = static_cast<uint32_t>(timestamp_ns % 1000000000ULL);
  compressed_msg.format = format;

  // Set compressed data
  compressed_msg.data = compressed_data;

  // Serialize
  rclcpp::Serialization<sensor_msgs::msg::CompressedImage> serialization;
  rclcpp::SerializedMessage serialized_msg;
  serialization.serialize_message(&compressed_msg, &serialized_msg);

  // Copy to output vector
  std::vector<uint8_t> result(
    serialized_msg.get_rcl_serialized_message().buffer,
    serialized_msg.get_rcl_serialized_message().buffer +
      serialized_msg.get_rcl_serialized_message().buffer_length
  );

  return result;
}

}  // namespace ros2_plugin
