#ifndef AXON_RECORDER_RECORDING_SERVICE_IMPL_HPP
#define AXON_RECORDER_RECORDING_SERVICE_IMPL_HPP

#include <memory>
#include <string>

#include "recorder_context.hpp"
#include "state_machine.hpp"
#include "task_config.hpp"

namespace axon {
namespace recorder {

/**
 * RecordingServiceImpl provides the implementation for all recording services.
 * It coordinates between the ROS service layer and the IRecorderContext.
 *
 * Services implemented:
 * - CachedRecordingConfig: Cache task configuration from server
 * - IsRecordingReady: Query if recorder has cached config
 * - RecordingControl: Control recording lifecycle
 * - RecordingStatus: Query recording status
 *
 * This class uses the IRecorderContext interface for dependency injection,
 * enabling easier testing with mock implementations.
 */
class RecordingServiceImpl {
public:
  /**
   * Construct with a shared pointer to the recorder context.
   * Using shared_ptr ensures the context remains valid for the lifetime of this object.
   */
  explicit RecordingServiceImpl(std::shared_ptr<IRecorderContext> context);

  // =========================================================================
  // CachedRecordingConfig Service
  // =========================================================================

  /**
   * Handle CachedRecordingConfig request.
   * Caches the task configuration and transitions to READY state.
   */
  bool handle_cached_recording_config(
    // Request fields
    const std::string& task_id, const std::string& device_id, const std::string& data_collector_id,
    const std::string& order_id, const std::string& operator_name, const std::string& scene,
    const std::string& subscene, const std::vector<std::string>& skills, const std::string& factory,
    const std::vector<std::string>& topics, const std::string& start_callback_url,
    const std::string& finish_callback_url, const std::string& user_token,
    // Response fields
    bool& success, std::string& message
  );

  // =========================================================================
  // IsRecordingReady Service
  // =========================================================================

  /**
   * Handle IsRecordingReady request.
   * Returns whether recorder has cached config and is ready to start.
   */
  bool handle_is_recording_ready(
    // Response fields
    bool& success, std::string& message, bool& is_configured, bool& is_recording,
    std::string& task_id, std::string& device_id, std::string& order_id, std::string& operator_name,
    std::string& scene, std::string& subscene, std::vector<std::string>& skills,
    std::string& factory, std::string& data_collector_id, std::vector<std::string>& topics
  );

  // =========================================================================
  // RecordingControl Service
  // =========================================================================

  /**
   * Handle RecordingControl request.
   * Processes commands: start, pause, resume, cancel, finish, clear
   */
  bool handle_recording_control(
    // Request fields
    const std::string& command, const std::string& task_id_request,
    // Response fields
    bool& success, std::string& message, std::string& task_id_response
  );

  // =========================================================================
  // RecordingStatus Service
  // =========================================================================

  /**
   * Handle RecordingStatus request.
   * Returns current recording status and metrics.
   */
  bool handle_recording_status(
    // Request fields
    const std::string& task_id_request,
    // Response fields
    bool& success, std::string& message, std::string& status, std::string& task_id,
    std::string& device_id, std::string& data_collector_id, std::string& order_id,
    std::string& operator_name, std::string& scene, std::string& subscene,
    std::vector<std::string>& skills, std::string& factory, std::vector<std::string>& active_topics,
    std::string& output_path, double& disk_usage_gb, double& duration_sec, int64_t& message_count,
    double& throughput_mb_sec, std::string& last_error
  );

private:
  // =========================================================================
  // Command Handlers
  // =========================================================================

  bool handle_start_command(std::string& message, std::string& task_id_response);
  bool handle_pause_command(const std::string& task_id, std::string& message);
  bool handle_resume_command(const std::string& task_id, std::string& message);
  bool handle_cancel_command(const std::string& task_id, std::string& message);
  bool handle_finish_command(const std::string& task_id, std::string& message);
  bool handle_clear_command(std::string& message);

  // =========================================================================
  // Helpers
  // =========================================================================

  /**
   * Validate task_id matches the current recording task.
   */
  bool validate_task_id(const std::string& task_id, std::string& error_msg);

  std::shared_ptr<IRecorderContext> context_;
};

}  // namespace recorder
}  // namespace axon

#endif  // AXON_RECORDER_RECORDING_SERVICE_IMPL_HPP
