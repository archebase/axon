#include "state_machine.hpp"

#include <algorithm>

namespace axon {
namespace recorder {

StateManager::StateManager()
    : current_state_(RecorderState::IDLE) {
  build_transition_map();
}

void StateManager::build_transition_map() {
  // Define valid state transitions according to the design document
  //
  // IDLE -> READY: via CachedRecordingConfig service
  // READY -> IDLE: via clear command or timeout
  // READY -> RECORDING: via start command
  // RECORDING -> PAUSED: via pause command
  // PAUSED -> RECORDING: via resume command
  // RECORDING -> IDLE: via finish or cancel command
  // PAUSED -> IDLE: via finish or cancel command

  valid_transitions_[RecorderState::IDLE] = {RecorderState::READY};

  valid_transitions_[RecorderState::READY] = {RecorderState::IDLE, RecorderState::RECORDING};

  valid_transitions_[RecorderState::RECORDING] = {RecorderState::PAUSED, RecorderState::IDLE};

  valid_transitions_[RecorderState::PAUSED] = {RecorderState::RECORDING, RecorderState::IDLE};
}

RecorderState StateManager::get_state() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return current_state_;
}

std::string StateManager::get_state_string() const {
  return state_to_string(get_state());
}

bool StateManager::is_state(RecorderState state) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return current_state_ == state;
}

bool StateManager::is_recording_active() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return current_state_ == RecorderState::RECORDING || current_state_ == RecorderState::PAUSED;
}

bool StateManager::is_valid_transition(RecorderState from, RecorderState to) const {
  auto it = valid_transitions_.find(from);
  if (it == valid_transitions_.end()) {
    return false;
  }

  const auto& valid_targets = it->second;
  return std::find(valid_targets.begin(), valid_targets.end(), to) != valid_targets.end();
}

bool StateManager::transition_to(RecorderState to, std::string& error_msg) {
  std::lock_guard<std::mutex> lock(mutex_);

  RecorderState from = current_state_;

  // Check if transition is valid
  if (!is_valid_transition(from, to)) {
    error_msg = "ERR_INVALID_STATE: Cannot transition from " + state_to_string(from) + " to " +
                state_to_string(to);
    return false;
  }

  // Perform transition
  current_state_ = to;

  // Notify callbacks (outside lock would be better, but simpler this way)
  // Note: callbacks should be quick and non-blocking
  for (const auto& callback : callbacks_) {
    callback(from, to);
  }

  error_msg.clear();
  return true;
}

bool StateManager::transition(RecorderState from, RecorderState to, std::string& error_msg) {
  std::lock_guard<std::mutex> lock(mutex_);

  // Verify current state matches expected
  if (current_state_ != from) {
    error_msg = "ERR_INVALID_STATE: Expected state " + state_to_string(from) + " but current is " +
                state_to_string(current_state_);
    return false;
  }

  // Check if transition is valid
  if (!is_valid_transition(from, to)) {
    error_msg = "ERR_INVALID_STATE: Cannot transition from " + state_to_string(from) + " to " +
                state_to_string(to);
    return false;
  }

  // Perform transition
  current_state_ = to;

  // Notify callbacks
  for (const auto& callback : callbacks_) {
    callback(from, to);
  }

  error_msg.clear();
  return true;
}

std::vector<RecorderState> StateManager::get_valid_transitions() const {
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = valid_transitions_.find(current_state_);
  if (it == valid_transitions_.end()) {
    return {};
  }
  return it->second;
}

void StateManager::register_transition_callback(StateTransitionCallback callback) {
  std::lock_guard<std::mutex> lock(mutex_);
  callbacks_.push_back(std::move(callback));
}

void StateManager::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  current_state_ = RecorderState::IDLE;
}

void StateManager::notify_transition(RecorderState from, RecorderState to) {
  // Called internally with lock held
  for (const auto& callback : callbacks_) {
    callback(from, to);
  }
}

}  // namespace recorder
}  // namespace axon

