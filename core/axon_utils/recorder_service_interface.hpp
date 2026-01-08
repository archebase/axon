#ifndef AXON_UTILS_RECORDER_SERVICE_INTERFACE_HPP
#define AXON_UTILS_RECORDER_SERVICE_INTERFACE_HPP

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "common_types.hpp"

namespace axon {
namespace utils {

/**
 * Abstract interface for ROS service adapters.
 * This allows the recorder core to work with both ROS1 and ROS2 via plugins.
 *
 * The interface defines methods for handling the four main recorder services:
 * 1. CachedRecordingConfig - Cache task configuration
 * 2. IsRecordingReady - Query if ready to record
 * 3. RecordingControl - Control recording lifecycle
 * 4. RecordingStatus - Query recording status
 */
class IRecorderServiceAdapter {
public:
  virtual ~IRecorderServiceAdapter() = default;

  /**
   * Register all services with the ROS master/node.
   * Called during initialization.
   */
  virtual bool register_services() = 0;

  /**
   * Shutdown all services.
   * Called during cleanup.
   */
  virtual void shutdown() = 0;

  // ==========================================================================
  // Service Handlers - These are implemented by the plugin
  // ==========================================================================

  /**
   * Handle CachedRecordingConfig service request.
   * Caches task configuration and transitions state to READY.
   *
   * @param task_id Task identifier
   * @param device_id Device identifier
   * @param data_collector_id Data collector identifier
   * @param order_id Order identifier (optional)
   * @param operator_name Operator name (optional)
   * @param scene Scene description
   * @param subscene Subscene description
   * @param skills List of skills being demonstrated
   * @param factory Factory identifier
   * @param topics List of topics to record
   * @param start_callback_url URL for start callback
   * @param finish_callback_url URL for finish callback
   * @param user_token JWT token for authentication
   * @param success Output: success status
   * @param message Output: error/success message
   */
  virtual bool handle_cached_recording_config(
    // Request fields
    const std::string& task_id, const std::string& device_id, const std::string& data_collector_id,
    const std::string& order_id, const std::string& operator_name, const std::string& scene,
    const std::string& subscene, const std::vector<std::string>& skills, const std::string& factory,
    const std::vector<std::string>& topics, const std::string& start_callback_url,
    const std::string& finish_callback_url, const std::string& user_token,
    // Response fields
    bool& success, std::string& message
  ) = 0;

  /**
   * Handle IsRecordingReady service request.
   * Returns whether recorder has cached config and is ready.
   *
   * @param success Output: success status
   * @param message Output: error/success message
   * @param is_configured Output: true if config is cached
   * @param is_recording Output: true if currently recording
   * @param task_id Output: cached task ID
   * @param device_id Output: cached device ID
   * @param order_id Output: cached order ID
   * @param operator_name Output: cached operator name
   * @param scene Output: cached scene
   * @param subscene Output: cached subscene
   * @param skills Output: cached skills
   * @param factory Output: cached factory
   * @param data_collector_id Output: cached data collector ID
   * @param topics Output: cached topics
   */
  virtual bool handle_is_recording_ready(
    // Response fields
    bool& success, std::string& message, bool& is_configured, bool& is_recording,
    std::string& task_id, std::string& device_id, std::string& order_id, std::string& operator_name,
    std::string& scene, std::string& subscene, std::vector<std::string>& skills,
    std::string& factory, std::string& data_collector_id, std::vector<std::string>& topics
  ) = 0;

  /**
   * Handle RecordingControl service request.
   * Processes commands: start, pause, resume, cancel, finish, clear
   *
   * @param command The command to execute
   * @param task_id_request Task ID for validation (optional)
   * @param success Output: success status
   * @param message Output: error/success message
   * @param task_id_response Output: task ID (for confirmation)
   */
  virtual bool handle_recording_control(
    // Request fields
    const std::string& command, const std::string& task_id_request,
    // Response fields
    bool& success, std::string& message, std::string& task_id_response
  ) = 0;

  /**
   * Handle RecordingStatus service request.
   * Returns current recording status and metrics.
   *
   * @param task_id_request Task ID for validation (optional)
   * @param success Output: success status
   * @param message Output: error/success message
   * @param status Output: current state as string
   * @param task_id Output: current task ID
   * @param device_id Output: current device ID
   * @param data_collector_id Output: current data collector ID
   * @param order_id Output: current order ID
   * @param operator_name Output: current operator name
   * @param scene Output: current scene
   * @param subscene Output: current subscene
   * @param skills Output: current skills
   * @param factory Output: current factory
   * @param active_topics Output: topics being recorded
   * @param output_path Output: MCAP file path
   * @param disk_usage_gb Output: disk usage in GB
   * @param duration_sec Output: recording duration in seconds
   * @param message_count Output: total messages recorded
   * @param throughput_mb_sec Output: throughput in MB/s
   * @param last_error Output: last error message
   */
  virtual bool handle_recording_status(
    // Request fields
    const std::string& task_id_request,
    // Response fields
    bool& success, std::string& message, std::string& status, std::string& task_id,
    std::string& device_id, std::string& data_collector_id, std::string& order_id,
    std::string& operator_name, std::string& scene, std::string& subscene,
    std::vector<std::string>& skills, std::string& factory, std::vector<std::string>& active_topics,
    std::string& output_path, double& disk_usage_gb, double& duration_sec, int64_t& message_count,
    double& throughput_mb_sec, std::string& last_error
  ) = 0;
};

/**
 * Abstract interface for recorder context.
 * This provides the business logic that the service adapter uses.
 * The core recorder implements this interface.
 */
class IRecorderContext {
public:
  virtual ~IRecorderContext() = default;

  /**
   * Get the current state as a string.
   */
  virtual std::string get_state_string() const = 0;

  /**
   * Check if recording is active (RECORDING or PAUSED).
   */
  virtual bool is_recording_active() const = 0;

  /**
   * Get the cached task configuration.
   */
  virtual std::optional<TaskConfig> get_cached_config() const = 0;

  /**
   * Get recording metrics.
   */
  virtual bool get_recording_metrics(
    std::string& output_path, double& disk_usage_gb, double& duration_sec, int64_t& message_count,
    double& throughput_mb_sec, std::string& last_error
  ) const = 0;

  /**
   * Cache a new task configuration.
   */
  virtual bool cache_task_config(const TaskConfig& config, std::string& error_msg) = 0;

  /**
   * Start recording.
   */
  virtual bool start_recording(std::string& error_msg) = 0;

  /**
   * Pause recording.
   */
  virtual bool pause_recording(std::string& error_msg) = 0;

  /**
   * Resume recording.
   */
  virtual bool resume_recording(std::string& error_msg) = 0;

  /**
   * Cancel recording.
   */
  virtual bool cancel_recording(std::string& error_msg) = 0;

  /**
   * Finish recording.
   */
  virtual bool finish_recording(std::string& error_msg) = 0;

  /**
   * Clear cached configuration.
   */
  virtual bool clear_config(std::string& error_msg) = 0;
};

/**
 * Factory function type for creating service adapters.
 * Used by plugin loading.
 */
using RecorderServiceAdapterFactory = std::function<std::unique_ptr<IRecorderServiceAdapter>(
  std::shared_ptr<IRecorderContext> context, void* ros_node_ptr
)>;

}  // namespace utils
}  // namespace axon

#endif  // AXON_UTILS_RECORDER_SERVICE_INTERFACE_HPP
