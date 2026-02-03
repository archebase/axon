// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef ROS2_DEPTH_COMPRESSION_FILTER_HPP
#define ROS2_DEPTH_COMPRESSION_FILTER_HPP

#include <depth_compressor.hpp>
#include <functional>
#include <string>
#include <vector>

namespace ros2_plugin {

/**
 * @brief Depth compression configuration
 */
struct DepthCompressionConfig {
  bool enabled = false;
  std::string level = "medium";  // fast, medium, max
};

/**
 * @brief Depth image compression filter
 *
 * Used in ROS2 plugin to compress sensor_msgs::msg::Image (16UC1) to
 * sensor_msgs::msg::CompressedImage format.
 */
class DepthCompressionFilter {
public:
  /**
   * @brief Processed message callback type
   */
  using ProcessedCallback = std::function<void(
    const std::string& topic, const std::string& message_type, const std::vector<uint8_t>& data,
    uint64_t timestamp_ns
  )>;

  /**
   * @brief Constructor
   * @param config Depth compression configuration
   */
  explicit DepthCompressionFilter(const DepthCompressionConfig& config);

  /**
   * @brief Filter and process message
   *
   * @param topic Topic name
   * @param message_type Message type
   * @param data Raw message data (CDR serialized sensor_msgs::msg::Image)
   * @param timestamp_ns Timestamp in nanoseconds
   * @param callback Processed message callback
   */
  void filter_and_process(
    const std::string& topic, const std::string& message_type, const std::vector<uint8_t>& data,
    uint64_t timestamp_ns, ProcessedCallback callback
  );

private:
  DepthCompressionConfig config_;
  axon::depth::DepthCompressor compressor_;

  /**
   * @brief Extract depth data from ROS Image message
   * @return Tuple of depth data pointer, width, height
   */
  std::tuple<const uint8_t*, size_t, size_t> extract_depth_data(
    const std::vector<uint8_t>& image_msg
  );

  /**
   * @brief Build CompressedImage message
   */
  std::vector<uint8_t> build_compressed_image_msg(
    const std::string& format, const std::vector<uint8_t>& compressed_data, uint64_t timestamp_ns
  );
};

}  // namespace ros2_plugin

#endif  // ROS2_DEPTH_COMPRESSION_FILTER_HPP
