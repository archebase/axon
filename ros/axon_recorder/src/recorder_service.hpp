#ifndef AXON_RECORDER_SERVICE_HPP
#define AXON_RECORDER_SERVICE_HPP

#include <cstdint>
#include <string>
#include <vector>

namespace axon {
namespace recorder {

/**
 * Service handler for recorder control
 * Works with both ROS 1 and ROS 2 via the RosInterface abstraction
 */
class RecorderServiceHandler {
public:
  /**
   * Handle start recording request
   * @param config_path Optional path to configuration file
   * @param error_msg Output error message if failed
   * @return true if successful
   */
  bool handle_start_recording(const std::string& config_path, std::string& error_msg);

  /**
   * Handle stop recording request
   * @param flushed_batches Output number of batches flushed
   * @param error_msg Output error message if failed
   * @return true if successful
   */
  bool handle_stop_recording(int32_t& flushed_batches, std::string& error_msg);

  /**
   * Handle update configuration request
   * @param config_path Path to new configuration file
   * @param error_msg Output error message if failed
   * @return true if successful
   */
  bool handle_update_config(const std::string& config_path, std::string& error_msg);

  /**
   * Handle get status request
   * @param is_recording Output whether recording is active
   * @param dataset_path Output path to current dataset
   * @param active_topics Output list of active topics
   * @param batch_sizes Output batch sizes for each topic
   * @param pending_batches Output number of pending batches
   * @param disk_usage_gb Output disk usage in GB
   * @param last_error Output last error message
   * @return true if successful
   */
  bool handle_get_status(
    bool& is_recording, std::string& dataset_path, std::vector<std::string>& active_topics,
    std::vector<int32_t>& batch_sizes, int32_t& pending_batches, double& disk_usage_gb,
    std::string& last_error
  );
};

}  // namespace recorder
}  // namespace axon

#endif  // AXON_RECORDER_SERVICE_HPP
