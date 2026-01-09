#ifndef AXON_RECORDER_ADAPTER_HPP
#define AXON_RECORDER_ADAPTER_HPP

/**
 * Adapter header to bridge legacy recorder code with core utilities.
 * This file provides type aliases and adapters to use core/axon_utils types.
 */

#include <axon_utils/recorder_service_interface.hpp>
#include <axon_utils/state_machine.hpp>

#include "common_types.hpp"

namespace axon {
namespace recorder {

// Types are now in axon::recorder namespace, no aliases needed

// Recorder state enum (matches core interface)
enum class RecorderState { IDLE, READY, RECORDING, PAUSED };

// State machine using core template
using StateMachine = axon::utils::StateMachine<RecorderState>;
using StateTransactionGuard = axon::utils::StateTransactionGuard<RecorderState>;

// IRecorderContext interface from core
using IRecorderContext = axon::utils::IRecorderContext;

// Helper function to convert RecorderState to string
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

// Helper function to convert string to RecorderState
inline RecorderState string_to_state(const std::string& str) {
  if (str == "idle") return RecorderState::IDLE;
  if (str == "ready") return RecorderState::READY;
  if (str == "recording") return RecorderState::RECORDING;
  if (str == "paused") return RecorderState::PAUSED;
  return RecorderState::IDLE;  // Default
}

}  // namespace recorder
}  // namespace axon

#endif  // AXON_RECORDER_ADAPTER_HPP
