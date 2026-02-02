// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef ROS1_DEPTH_COMPRESSION_FILTER_HPP
#define ROS1_DEPTH_COMPRESSION_FILTER_HPP

#include <depth_compressor.hpp>
#include <functional>
#include <string>
#include <vector>

namespace ros1_plugin {

/**
 * @brief 深度压缩配置
 */
struct DepthCompressionConfig {
  bool enabled = false;
  std::string level = "medium";  // fast, medium, max
};

/**
 * @brief 深度图像压缩过滤器
 *
 * 在 ROS1 插件中使用，将 sensor_msgs/Image (16UC1) 压缩为
 * sensor_msgs/CompressedImage 格式。
 */
class DepthCompressionFilter {
public:
  /**
   * @brief 处理后的消息回调类型
   */
  using ProcessedCallback = std::function<void(
    const std::string& topic, const std::string& message_type, const std::vector<uint8_t>& data,
    uint64_t timestamp
  )>;

  /**
   * @brief 构造函数
   * @param config 深度压缩配置
   */
  explicit DepthCompressionFilter(const DepthCompressionConfig& config);

  /**
   * @brief 过滤并处理消息
   *
   * @param topic 主题名称
   * @param message_type 消息类型
   * @param data 原始消息数据（ROS 序列化的 sensor_msgs/Image）
   * @param timestamp 时间戳（纳秒）
   * @param callback 处理后的消息回调
   */
  void filter_and_process(
    const std::string& topic, const std::string& message_type, const std::vector<uint8_t>& data,
    uint64_t timestamp, ProcessedCallback callback
  );

private:
  DepthCompressionConfig config_;
  axon::depth::DepthCompressor compressor_;

  /**
   * @brief 从 ROS Image 消息中提取深度数据
   * @return 提取的深度数据指针，宽度，高度
   */
  std::tuple<const uint8_t*, size_t, size_t> extract_depth_data(
    const std::vector<uint8_t>& image_msg
  );

  /**
   * @brief 构建 CompressedImage 消息
   */
  std::vector<uint8_t> build_compressed_image_msg(
    const std::string& format, const std::vector<uint8_t>& compressed_data, uint64_t timestamp
  );
};

}  // namespace ros1_plugin

#endif  // ROS1_DEPTH_COMPRESSION_FILTER_HPP
