#ifndef AXON_RECORDER_STATE_MACHINE_HPP
#define AXON_RECORDER_STATE_MACHINE_HPP

#include <functional>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace axon {
namespace recorder {

/**
 * RecorderState represents the current state of the recorder in the task lifecycle.
 *
 * State transitions:
 * - IDLE -> READY: via CachedRecordingConfig service
 * - READY -> IDLE: via clear command or timeout
 * - READY -> RECORDING: via start command
 * - RECORDING -> PAUSED: via pause command
 * - PAUSED -> RECORDING: via resume command
 * - RECORDING/PAUSED -> IDLE: via finish or cancel command
 */
enum class RecorderState {
  IDLE,       // No active task, waiting for configuration
  READY,      // Task config cached, ready to start recording
  RECORDING,  // Actively recording ROS messages to MCAP file
  PAUSED      // Recording paused, can resume or finish
};

/**
 * Convert RecorderState to string representation.
 */
inline std::string state_to_string(RecorderState state) {
  switch (state) {
    case RecorderState::IDLE:
      return "idle";
    case RecorderState::READY:
      return "ready";
    case RecorderState::RECORDING:
      return "recording";
    case RecorderState::PAUSED:
      return "paused";
    default:
      return "unknown";
  }
}

/**
 * Parse string to RecorderState.
 */
inline RecorderState string_to_state(const std::string& str) {
  if (str == "idle") return RecorderState::IDLE;
  if (str == "ready") return RecorderState::READY;
  if (str == "recording") return RecorderState::RECORDING;
  if (str == "paused") return RecorderState::PAUSED;
  return RecorderState::IDLE;  // Default
}

/**
 * StateTransitionCallback is called when a state transition occurs.
 * Parameters: (from_state, to_state)
 */
using StateTransitionCallback = std::function<void(RecorderState, RecorderState)>;

/**
 * StateManager manages the recorder state machine with thread-safe transitions.
 *
 * It enforces valid state transitions according to the design document and
 * notifies registered callbacks when transitions occur.
 */
class StateManager {
public:
  StateManager();

  // Non-copyable
  StateManager(const StateManager&) = delete;
  StateManager& operator=(const StateManager&) = delete;

  /**
   * Get the current state.
   */
  RecorderState get_state() const;

  /**
   * Get the current state as a string.
   */
  std::string get_state_string() const;

  /**
   * Check if in a specific state.
   */
  bool is_state(RecorderState state) const;

  /**
   * Check if recording is active (RECORDING or PAUSED).
   */
  bool is_recording_active() const;

  /**
   * Attempt a state transition.
   *
   * @param to The target state
   * @param error_msg Output error message if transition fails
   * @return true if transition was successful
   */
  bool transition_to(RecorderState to, std::string& error_msg);

  /**
   * Attempt a state transition from a specific state.
   * This is safer as it verifies the current state before transitioning.
   *
   * @param from Expected current state
   * @param to Target state
   * @param error_msg Output error message if transition fails
   * @return true if transition was successful
   */
  bool transition(RecorderState from, RecorderState to, std::string& error_msg);

  /**
   * Check if a transition is valid.
   *
   * @param from Source state
   * @param to Target state
   * @return true if the transition is allowed
   */
  bool is_valid_transition(RecorderState from, RecorderState to) const;

  /**
   * Get list of valid transitions from the current state.
   */
  std::vector<RecorderState> get_valid_transitions() const;

  /**
   * Register a callback for state transitions.
   * The callback will be invoked after each successful transition.
   *
   * @param callback The callback function
   */
  void register_transition_callback(StateTransitionCallback callback);

  /**
   * Reset to IDLE state (used during shutdown).
   */
  void reset();

private:
  /**
   * Notify all registered callbacks of a state transition.
   */
  void notify_transition(RecorderState from, RecorderState to);

  /**
   * Build the valid transitions map.
   */
  void build_transition_map();

  mutable std::mutex mutex_;
  RecorderState current_state_;
  std::vector<StateTransitionCallback> callbacks_;

  // Map of valid transitions: from_state -> [to_states]
  std::unordered_map<RecorderState, std::vector<RecorderState>> valid_transitions_;
};

}  // namespace recorder
}  // namespace axon

#endif  // AXON_RECORDER_STATE_MACHINE_HPP

