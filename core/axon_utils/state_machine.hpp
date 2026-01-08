#ifndef AXON_UTILS_STATE_MACHINE_HPP
#define AXON_UTILS_STATE_MACHINE_HPP

#include <functional>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace axon {
namespace utils {

/**
 * Generic state type for the state machine.
 * Users should define their own enum class State and use it with this framework.
 */
template<typename StateEnum>
class StateMachine {
public:
  using State = StateEnum;
  using StateTransitionCallback = std::function<void(State, State)>;

  /**
   * Construct with initial state and transition map.
   *
   * @param initial_state The starting state
   * @param transitions Map of valid transitions: from_state -> {to_states}
   */
  explicit StateMachine(
    State initial_state, const std::unordered_map<State, std::vector<State>>& transitions
  );

  // Non-copyable
  StateMachine(const StateMachine&) = delete;
  StateMachine& operator=(const StateMachine&) = delete;

  /**
   * Get the current state.
   */
  State get_state() const;

  /**
   * Check if in a specific state.
   */
  bool is_state(State state) const;

  /**
   * Check if state is in a set of states.
   */
  bool is_state_in(const std::vector<State>& states) const;

  /**
   * Attempt a state transition.
   *
   * @param to The target state
   * @param error_msg Output error message if transition fails
   * @return true if transition was successful
   */
  bool transition_to(State to, std::string& error_msg);

  /**
   * Attempt a state transition from a specific state.
   * This is safer as it verifies the current state before transitioning.
   *
   * @param from Expected current state
   * @param to Target state
   * @param error_msg Output error message if transition fails
   * @return true if transition was successful
   */
  bool transition(State from, State to, std::string& error_msg);

  /**
   * Check if a transition is valid.
   *
   * @param from Source state
   * @param to Target state
   * @return true if the transition is allowed
   */
  bool is_valid_transition(State from, State to) const;

  /**
   * Get list of valid transitions from the current state.
   */
  std::vector<State> get_valid_transitions() const;

  /**
   * Register a callback for state transitions.
   * The callback will be invoked after each successful transition.
   *
   * @param callback The callback function
   */
  void register_transition_callback(StateTransitionCallback callback);

  /**
   * Reset to initial state (used during shutdown).
   */
  void reset();

private:
  mutable std::mutex mutex_;
  State current_state_;
  State initial_state_;
  std::vector<StateTransitionCallback> callbacks_;

  // Map of valid transitions: from_state -> [to_states]
  std::unordered_map<State, std::vector<State>> valid_transitions_;
};

/**
 * StateTransactionGuard provides RAII-style transaction semantics for state transitions.
 *
 * When a state transition needs to be followed by additional operations that might fail,
 * use this guard to automatically rollback the state if the operations don't complete
 * successfully.
 *
 * Usage:
 *   StateTransactionGuard guard(state_machine, previous_state);
 *
 *   if (!state_machine.transition_to(State::RECORDING, error)) {
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
 * - The underlying StateMachine is thread-safe
 */
template<typename StateEnum>
class StateTransactionGuard {
public:
  using State = StateEnum;

  /**
   * Create a transaction guard.
   *
   * @param state_machine Reference to the state machine
   * @param rollback_state State to revert to if not committed
   */
  StateTransactionGuard(StateMachine<State>& state_machine, State rollback_state)
      : state_machine_(state_machine)
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
      state_machine_.transition_to(rollback_state_, error);
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
  State get_rollback_state() const {
    return rollback_state_;
  }

  // Non-copyable, non-movable
  StateTransactionGuard(const StateTransactionGuard&) = delete;
  StateTransactionGuard& operator=(const StateTransactionGuard&) = delete;
  StateTransactionGuard(StateTransactionGuard&&) = delete;
  StateTransactionGuard& operator=(StateTransactionGuard&&) = delete;

private:
  StateMachine<State>& state_machine_;
  State rollback_state_;
  bool committed_;
};

// Template implementation

template<typename StateEnum>
StateMachine<StateEnum>::StateMachine(
  State initial_state, const std::unordered_map<State, std::vector<State>>& transitions
)
    : current_state_(initial_state)
    , initial_state_(initial_state)
    , valid_transitions_(transitions) {}

template<typename StateEnum>
StateEnum StateMachine<StateEnum>::get_state() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return current_state_;
}

template<typename StateEnum>
bool StateMachine<StateEnum>::is_state(State state) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return current_state_ == state;
}

template<typename StateEnum>
bool StateMachine<StateEnum>::is_state_in(const std::vector<State>& states) const {
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& s : states) {
    if (current_state_ == s) {
      return true;
    }
  }
  return false;
}

template<typename StateEnum>
bool StateMachine<StateEnum>::is_valid_transition(State from, State to) const {
  auto it = valid_transitions_.find(from);
  if (it == valid_transitions_.end()) {
    return false;
  }

  const auto& valid_targets = it->second;
  return std::find(valid_targets.begin(), valid_targets.end(), to) != valid_targets.end();
}

template<typename StateEnum>
bool StateMachine<StateEnum>::transition_to(State to, std::string& error_msg) {
  std::vector<StateTransitionCallback> callbacks_copy;
  State from;

  {
    std::lock_guard<std::mutex> lock(mutex_);

    from = current_state_;

    // Check if transition is valid
    if (!is_valid_transition(from, to)) {
      error_msg = "ERR_INVALID_STATE: Cannot transition from " +
                  std::to_string(static_cast<int>(from)) + " to " +
                  std::to_string(static_cast<int>(to));
      return false;
    }

    // Perform transition
    current_state_ = to;
    error_msg.clear();

    // Copy callbacks to invoke outside lock (prevents deadlock if callback queries state)
    callbacks_copy = callbacks_;
  }

  // Notify callbacks outside lock
  for (const auto& callback : callbacks_copy) {
    callback(from, to);
  }

  return true;
}

template<typename StateEnum>
bool StateMachine<StateEnum>::transition(State from, State to, std::string& error_msg) {
  std::vector<StateTransitionCallback> callbacks_copy;

  {
    std::lock_guard<std::mutex> lock(mutex_);

    // Verify current state matches expected
    if (current_state_ != from) {
      error_msg = "ERR_INVALID_STATE: Expected state " + std::to_string(static_cast<int>(from)) +
                  " but current is " + std::to_string(static_cast<int>(current_state_));
      return false;
    }

    // Check if transition is valid
    if (!is_valid_transition(from, to)) {
      error_msg = "ERR_INVALID_STATE: Cannot transition from " +
                  std::to_string(static_cast<int>(from)) + " to " +
                  std::to_string(static_cast<int>(to));
      return false;
    }

    // Perform transition
    current_state_ = to;
    error_msg.clear();

    // Copy callbacks to invoke outside lock (prevents deadlock if callback queries state)
    callbacks_copy = callbacks_;
  }

  // Notify callbacks outside lock
  for (const auto& callback : callbacks_copy) {
    callback(from, to);
  }

  return true;
}

template<typename StateEnum>
std::vector<StateEnum> StateMachine<StateEnum>::get_valid_transitions() const {
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = valid_transitions_.find(current_state_);
  if (it == valid_transitions_.end()) {
    return {};
  }
  return it->second;
}

template<typename StateEnum>
void StateMachine<StateEnum>::register_transition_callback(StateTransitionCallback callback) {
  std::lock_guard<std::mutex> lock(mutex_);
  callbacks_.push_back(std::move(callback));
}

template<typename StateEnum>
void StateMachine<StateEnum>::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  current_state_ = initial_state_;
}

}  // namespace utils
}  // namespace axon

#endif  // AXON_UTILS_STATE_MACHINE_HPP
