// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "depth_compression_filter.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <cstring>

namespace ros2_plugin {

DepthCompressionFilter::DepthCompressionFilter(const DepthCompressionConfig& config)
    : config_(config) {
  // 配置压缩器
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
  const std::string& topic,
  const std::string& message_type,
  const std::vector<uint8_t>& data,
  uint64_t timestamp_ns,
  ProcessedCallback callback
) {
  // 只处理 sensor_msgs::msg::Image
  if (message_type != "sensor_msgs::msg::Image" &&
      message_type != "sensor_msgs/msg/Image") {
    // 不是图像消息，直接传递
    callback(topic, message_type, data, timestamp_ns);
    return;
  }

  // 尝试提取并压缩深度图像
  try {
    auto [depth_data, width, height] = extract_depth_data(data);

    if (depth_data != nullptr && width > 0 && height > 0) {
      // 压缩深度图像
      std::vector<uint8_t> compressed_data;
      if (compressor_.compress(depth_data, width, height, compressed_data)) {
        // 构建压缩图像消息
        std::string format = compressor_.get_compression_format();
        std::vector<uint8_t> compressed_msg =
          build_compressed_image_msg(format, compressed_data, timestamp_ns);

        // 回调处理后的消息
        callback(topic, "sensor_msgs::msg::CompressedImage", compressed_msg, timestamp_ns);
        return;
      }
    }
  } catch (const std::exception& e) {
    // 压缩失败，回退到原始消息
  }

  // 压缩失败或不是深度图像，传递原始消息
  callback(topic, message_type, data, timestamp_ns);
}

std::tuple<const uint8_t*, size_t, size_t>
DepthCompressionFilter::extract_depth_data(const std::vector<uint8_t>& image_msg) {
  const uint8_t* data = nullptr;
  size_t width = 0;
  size_t height = 0;

  // 使用 ROS2 反序列化
  try {
    rclcpp::Serialization<sensor_msgs::msg::Image> serialization;
    sensor_msgs::msg::Image image;
    rclcpp::SerializedMessage serialized_msg;
    serialized_msg.reserve(image_msg.size());
    std::memcpy(
      serialized_msg.get_rcl_serialized_message().buffer, image_msg.data(), image_msg.size()
    );
    serialized_msg.get_rcl_serialized_message().buffer_length = image_msg.size();

    // 反序列化
    serialization.deserialize_message(serialized_msg, &image);

    // 检查是否为 16UC1 深度图像
    if (image.encoding == "16UC1") {
      data = image.data.data();
      width = image.width;
      height = image.height;
    }
  } catch (const std::exception& e) {
    // 反序列化失败
  }

  return {data, width, height};
}

std::vector<uint8_t> DepthCompressionFilter::build_compressed_image_msg(
  const std::string& format,
  const std::vector<uint8_t>& compressed_data,
  uint64_t timestamp_ns
) {
  // 使用 ROS2 序列化 CompressedImage
  sensor_msgs::msg::CompressedImage compressed_msg;

  // 设置头部
  compressed_msg.header.stamp.sec = static_cast<int32_t>(timestamp_ns / 1000000000ULL);
  compressed_msg.header.stamp.nanosec = static_cast<uint32_t>(timestamp_ns % 1000000000ULL);
  compressed_msg.format = format;

  // 设置压缩数据
  compressed_msg.data = compressed_data;

  // 序列化
  rclcpp::Serialization<sensor_msgs::msg::CompressedImage> serialization;
  rclcpp::SerializedMessage serialized_msg;
  serialization.serialize_message(compressed_msg, &serialized_msg);

  // 复制到输出向量
  std::vector<uint8_t> result(
    serialized_msg.get_rcl_serialized_message().buffer,
    serialized_msg.get_rcl_serialized_message().buffer +
      serialized_msg.get_rcl_serialized_message().buffer_length
  );

  return result;
}

}  // namespace ros2_plugin
