// Copyright (c) 2026 ArcheBase
// Axon is licensed under Mulan PSL v2.
// You can use this software according to the terms and conditions of the Mulan PSL v2.
// You may obtain a copy of Mulan PSL v2 at:
//          http://license.coscl.org.cn/MulanPSL2
// THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
// EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
// MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
// See the Mulan PSL v2 for more details.

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
   * Build the valid transitions map.
   */
  void build_transition_map();

  mutable std::mutex mutex_;
  RecorderState current_state_;
  std::vector<StateTransitionCallback> callbacks_;

  // Map of valid transitions: from_state -> [to_states]
  std::unordered_map<RecorderState, std::vector<RecorderState>> valid_transitions_;
};

/**
 * StateTransactionGuard provides RAII-style transaction semantics for state transitions.
 *
 * When a state transition needs to be followed by additional operations that might fail,
 * use this guard to automatically rollback the state if the operations don't complete
 * successfully.
 *
 * Usage:
 *   StateTransactionGuard guard(state_manager, previous_state);
 *
 *   if (!state_manager.transition_to(RecorderState::RECORDING, error)) {
 *       return false;  // No rollback needed - transition didn't happen
 *   }
 *
 *   if (!start_recording_impl()) {
 *       return false;  // Guard will rollback to previous_state
 *   }
 *
 *   guard.commit();  // Success - don't rollback
 *   return true;
 *
 * Thread Safety:
 * - The guard should be used from a single thread
 * - The underlying StateManager is thread-safe
 */
class StateTransactionGuard {
public:
  /**
   * Create a transaction guard.
   *
   * @param state_manager Reference to the state manager
   * @param rollback_state State to revert to if not committed
   */
  StateTransactionGuard(StateManager& state_manager, RecorderState rollback_state)
      : state_manager_(state_manager)
      , rollback_state_(rollback_state)
      , committed_(false) {}

  /**
   * Destructor - rolls back if not committed.
   */
  ~StateTransactionGuard() {
    if (!committed_) {
      std::string error;
      // Force transition back to rollback state
      // Note: This might fail if rollback_state is not valid from current state,
      // but we try anyway as a best-effort recovery
      state_manager_.transition_to(rollback_state_, error);
    }
  }

  /**
   * Commit the transaction.
   * Call this when all operations completed successfully.
   * After commit, the destructor will not rollback.
   */
  void commit() {
    committed_ = true;
  }

  /**
   * Check if the transaction has been committed.
   */
  bool is_committed() const {
    return committed_;
  }

  /**
   * Get the rollback state.
   */
  RecorderState get_rollback_state() const {
    return rollback_state_;
  }

  // Non-copyable, non-movable
  StateTransactionGuard(const StateTransactionGuard&) = delete;
  StateTransactionGuard& operator=(const StateTransactionGuard&) = delete;
  StateTransactionGuard(StateTransactionGuard&&) = delete;
  StateTransactionGuard& operator=(StateTransactionGuard&&) = delete;

private:
  StateManager& state_manager_;
  RecorderState rollback_state_;
  bool committed_;
};

}  // namespace recorder
}  // namespace axon

#endif  // AXON_RECORDER_STATE_MACHINE_HPP
