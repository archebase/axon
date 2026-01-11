#ifndef AXON_RECORDER_CONTEXT_HPP
#define AXON_RECORDER_CONTEXT_HPP

#include <cstdint>
#include <memory>
#include <string>

// Forward declarations
namespace axon {
namespace recorder {
class StateManager;
class TaskConfigCache;
class HttpCallbackClient;
struct TaskConfig;
}  // namespace recorder
}  // namespace axon

namespace axon {
namespace recorder {

/**
 * Recording statistics for status reporting.
 */
struct RecordingStats {
  uint64_t messages_received = 0;
  uint64_t messages_dropped = 0;
  uint64_t messages_written = 0;
  double drop_rate_percent = 0.0;
};

/**
 * IRecorderContext defines the interface for recorder operations.
 *
 * This interface abstracts the recorder functionality, enabling:
 * - Testability: Mock implementations for unit testing
 * - Dependency injection: Services depend on interface, not concrete class
 * - Single source of truth: All state queries go through this interface
 *
 * Components like RecordingServiceImpl and ServiceAdapter depend on this
 * interface rather than directly on RecorderNode, improving decoupling
 * and making the code more maintainable.
 */
class IRecorderContext {
public:
  virtual ~IRecorderContext() = default;

  // =========================================================================
  // State Management (Single Source of Truth)
  // =========================================================================

  /**
   * Get the state manager for state transitions and queries.
   */
  virtual StateManager& get_state_manager() = 0;

  /**
   * Get the task config cache for configuration management.
   */
  virtual TaskConfigCache& get_task_config_cache() = 0;

  /**
   * Get the HTTP callback client for server notifications.
   */
  virtual std::shared_ptr<HttpCallbackClient> get_http_callback_client() = 0;

  // =========================================================================
  // Recording Operations
  // =========================================================================

  /**
   * Start recording.
   * Enables message processing from queues.
   * @return true if recording started successfully
   */
  virtual bool start_recording() = 0;

  /**
   * Stop recording.
   * Disables new message acceptance, drains existing queues, finalizes file.
   */
  virtual void stop_recording() = 0;

  /**
   * Pause recording.
   * Temporarily stops accepting new messages.
   */
  virtual void pause_recording() = 0;

  /**
   * Resume recording.
   * Resumes accepting messages after pause.
   */
  virtual void resume_recording() = 0;

  /**
   * Cancel recording.
   * Stops recording and cleans up without finalizing.
   */
  virtual void cancel_recording() = 0;

  // =========================================================================
  // Configuration and Status
  // =========================================================================

  /**
   * Configure topics from task config.
   * Called when starting a recording from cached configuration.
   * @param config The task configuration containing topics and metadata
   * @return true if configuration was applied successfully
   */
  virtual bool configure_from_task_config(const TaskConfig& config) = 0;

  /**
   * Get the current output file path.
   */
  virtual std::string get_output_path() const = 0;

  /**
   * Get current recording statistics.
   */
  virtual RecordingStats get_stats() const = 0;

  /**
   * Get recording duration in seconds.
   * @return Duration since recording started, or 0.0 if not recording
   */
  virtual double get_recording_duration_sec() const = 0;

  // =========================================================================
  // Logging
  // =========================================================================

  /**
   * Log an informational message.
   */
  virtual void log_info(const std::string& msg) = 0;

  /**
   * Log a warning message.
   */
  virtual void log_warn(const std::string& msg) = 0;

  /**
   * Log an error message.
   */
  virtual void log_error(const std::string& msg) = 0;
};

}  // namespace recorder
}  // namespace axon

#endif  // AXON_RECORDER_CONTEXT_HPP
