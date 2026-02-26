// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

/**
 * Unit tests for RecorderState and StateManager
 */

#include <gtest/gtest.h>

#include <atomic>
#include <thread>
#include <vector>

#include "../../state_machine.hpp"

using namespace axon::recorder;

// ============================================================================
// RecorderState String Conversion Tests
// ============================================================================

TEST(RecorderStateTest, StateToString) {
  EXPECT_EQ(state_to_string(RecorderState::IDLE), "idle");
  EXPECT_EQ(state_to_string(RecorderState::READY), "ready");
  EXPECT_EQ(state_to_string(RecorderState::RECORDING), "recording");
  EXPECT_EQ(state_to_string(RecorderState::PAUSED), "paused");
}

TEST(RecorderStateTest, StringToState) {
  EXPECT_EQ(string_to_state("idle"), RecorderState::IDLE);
  EXPECT_EQ(string_to_state("ready"), RecorderState::READY);
  EXPECT_EQ(string_to_state("recording"), RecorderState::RECORDING);
  EXPECT_EQ(string_to_state("paused"), RecorderState::PAUSED);

  // Unknown string defaults to IDLE
  EXPECT_EQ(string_to_state("unknown"), RecorderState::IDLE);
  EXPECT_EQ(string_to_state(""), RecorderState::IDLE);
}

// ============================================================================
// StateManager Tests
// ============================================================================

class StateMachineTest : public ::testing::Test {
protected:
  void SetUp() override {
    // StateManager is initialized to IDLE by default
  }

  StateManager state_manager_;
};

TEST_F(StateMachineTest, InitialState) {
  EXPECT_EQ(state_manager_.get_state(), RecorderState::IDLE);
  EXPECT_EQ(state_manager_.get_state_string(), "idle");
  EXPECT_TRUE(state_manager_.is_state(RecorderState::IDLE));
  EXPECT_FALSE(state_manager_.is_recording_active());
}

TEST_F(StateMachineTest, ValidTransitionIdleToReady) {
  std::string error_msg;
  EXPECT_TRUE(state_manager_.transition_to(RecorderState::READY, error_msg));
  EXPECT_TRUE(error_msg.empty());
  EXPECT_EQ(state_manager_.get_state(), RecorderState::READY);
}

TEST_F(StateMachineTest, ValidTransitionReadyToRecording) {
  std::string error_msg;
  ASSERT_TRUE(state_manager_.transition_to(RecorderState::READY, error_msg));

  EXPECT_TRUE(state_manager_.transition_to(RecorderState::RECORDING, error_msg));
  EXPECT_TRUE(error_msg.empty());
  EXPECT_EQ(state_manager_.get_state(), RecorderState::RECORDING);
  EXPECT_TRUE(state_manager_.is_recording_active());
}

TEST_F(StateMachineTest, ValidTransitionRecordingToPaused) {
  std::string error_msg;
  ASSERT_TRUE(state_manager_.transition_to(RecorderState::READY, error_msg));
  ASSERT_TRUE(state_manager_.transition_to(RecorderState::RECORDING, error_msg));

  EXPECT_TRUE(state_manager_.transition_to(RecorderState::PAUSED, error_msg));
  EXPECT_TRUE(error_msg.empty());
  EXPECT_EQ(state_manager_.get_state(), RecorderState::PAUSED);
  EXPECT_TRUE(state_manager_.is_recording_active());
}

TEST_F(StateMachineTest, ValidTransitionPausedToRecording) {
  std::string error_msg;
  ASSERT_TRUE(state_manager_.transition_to(RecorderState::READY, error_msg));
  ASSERT_TRUE(state_manager_.transition_to(RecorderState::RECORDING, error_msg));
  ASSERT_TRUE(state_manager_.transition_to(RecorderState::PAUSED, error_msg));

  EXPECT_TRUE(state_manager_.transition_to(RecorderState::RECORDING, error_msg));
  EXPECT_TRUE(error_msg.empty());
  EXPECT_EQ(state_manager_.get_state(), RecorderState::RECORDING);
  EXPECT_TRUE(state_manager_.is_recording_active());
}

TEST_F(StateMachineTest, ValidTransitionRecordingToIdle) {
  std::string error_msg;
  ASSERT_TRUE(state_manager_.transition_to(RecorderState::READY, error_msg));
  ASSERT_TRUE(state_manager_.transition_to(RecorderState::RECORDING, error_msg));

  EXPECT_TRUE(state_manager_.transition_to(RecorderState::IDLE, error_msg));
  EXPECT_TRUE(error_msg.empty());
  EXPECT_EQ(state_manager_.get_state(), RecorderState::IDLE);
  EXPECT_FALSE(state_manager_.is_recording_active());
}

TEST_F(StateMachineTest, ValidTransitionPausedToIdle) {
  std::string error_msg;
  ASSERT_TRUE(state_manager_.transition_to(RecorderState::READY, error_msg));
  ASSERT_TRUE(state_manager_.transition_to(RecorderState::RECORDING, error_msg));
  ASSERT_TRUE(state_manager_.transition_to(RecorderState::PAUSED, error_msg));

  EXPECT_TRUE(state_manager_.transition_to(RecorderState::IDLE, error_msg));
  EXPECT_TRUE(error_msg.empty());
  EXPECT_EQ(state_manager_.get_state(), RecorderState::IDLE);
  EXPECT_FALSE(state_manager_.is_recording_active());
}

TEST_F(StateMachineTest, ValidTransitionReadyToIdle) {
  std::string error_msg;
  ASSERT_TRUE(state_manager_.transition_to(RecorderState::READY, error_msg));

  EXPECT_TRUE(state_manager_.transition_to(RecorderState::IDLE, error_msg));
  EXPECT_TRUE(error_msg.empty());
  EXPECT_EQ(state_manager_.get_state(), RecorderState::IDLE);
}

TEST_F(StateMachineTest, InvalidTransitionIdleToRecording) {
  std::string error_msg;
  EXPECT_FALSE(state_manager_.transition_to(RecorderState::RECORDING, error_msg));
  EXPECT_FALSE(error_msg.empty());
  EXPECT_EQ(state_manager_.get_state(), RecorderState::IDLE);  // State unchanged
}

TEST_F(StateMachineTest, InvalidTransitionIdleToPaused) {
  std::string error_msg;
  EXPECT_FALSE(state_manager_.transition_to(RecorderState::PAUSED, error_msg));
  EXPECT_FALSE(error_msg.empty());
  EXPECT_EQ(state_manager_.get_state(), RecorderState::IDLE);
}

TEST_F(StateMachineTest, InvalidTransitionReadyToPaused) {
  std::string error_msg;
  ASSERT_TRUE(state_manager_.transition_to(RecorderState::READY, error_msg));

  EXPECT_FALSE(state_manager_.transition_to(RecorderState::PAUSED, error_msg));
  EXPECT_FALSE(error_msg.empty());
  EXPECT_EQ(state_manager_.get_state(), RecorderState::READY);
}

TEST_F(StateMachineTest, TransitionWithExpectedState) {
  std::string error_msg;

  // Correct expected state
  EXPECT_TRUE(state_manager_.transition(RecorderState::IDLE, RecorderState::READY, error_msg));
  EXPECT_TRUE(error_msg.empty());

  // Wrong expected state
  EXPECT_FALSE(state_manager_.transition(RecorderState::IDLE, RecorderState::RECORDING, error_msg));
  EXPECT_FALSE(error_msg.empty());
}

TEST_F(StateMachineTest, IsValidTransition) {
  // Valid transitions from IDLE
  EXPECT_TRUE(state_manager_.is_valid_transition(RecorderState::IDLE, RecorderState::READY));
  EXPECT_FALSE(state_manager_.is_valid_transition(RecorderState::IDLE, RecorderState::RECORDING));
  EXPECT_FALSE(state_manager_.is_valid_transition(RecorderState::IDLE, RecorderState::PAUSED));

  // Valid transitions from READY
  EXPECT_TRUE(state_manager_.is_valid_transition(RecorderState::READY, RecorderState::RECORDING));
  EXPECT_TRUE(state_manager_.is_valid_transition(RecorderState::READY, RecorderState::IDLE));
  EXPECT_FALSE(state_manager_.is_valid_transition(RecorderState::READY, RecorderState::PAUSED));

  // Valid transitions from RECORDING
  EXPECT_TRUE(state_manager_.is_valid_transition(RecorderState::RECORDING, RecorderState::PAUSED));
  EXPECT_TRUE(state_manager_.is_valid_transition(RecorderState::RECORDING, RecorderState::IDLE));
  EXPECT_FALSE(state_manager_.is_valid_transition(RecorderState::RECORDING, RecorderState::READY));

  // Valid transitions from PAUSED
  EXPECT_TRUE(state_manager_.is_valid_transition(RecorderState::PAUSED, RecorderState::RECORDING));
  EXPECT_TRUE(state_manager_.is_valid_transition(RecorderState::PAUSED, RecorderState::IDLE));
  EXPECT_FALSE(state_manager_.is_valid_transition(RecorderState::PAUSED, RecorderState::READY));
}

TEST_F(StateMachineTest, GetValidTransitions) {
  // From IDLE
  auto transitions = state_manager_.get_valid_transitions();
  EXPECT_EQ(transitions.size(), 1);
  EXPECT_EQ(transitions[0], RecorderState::READY);

  // From READY
  std::string error_msg;
  ASSERT_TRUE(state_manager_.transition_to(RecorderState::READY, error_msg));
  transitions = state_manager_.get_valid_transitions();
  EXPECT_EQ(transitions.size(), 2);

  // From RECORDING
  ASSERT_TRUE(state_manager_.transition_to(RecorderState::RECORDING, error_msg));
  transitions = state_manager_.get_valid_transitions();
  EXPECT_EQ(transitions.size(), 2);
}

TEST_F(StateMachineTest, Reset) {
  std::string error_msg;
  ASSERT_TRUE(state_manager_.transition_to(RecorderState::READY, error_msg));
  ASSERT_TRUE(state_manager_.transition_to(RecorderState::RECORDING, error_msg));

  EXPECT_EQ(state_manager_.get_state(), RecorderState::RECORDING);

  state_manager_.reset();

  EXPECT_EQ(state_manager_.get_state(), RecorderState::IDLE);
}

TEST_F(StateMachineTest, TransitionCallback) {
  std::vector<std::pair<RecorderState, RecorderState>> transitions;

  state_manager_.register_transition_callback([&](RecorderState from, RecorderState to) {
    transitions.emplace_back(from, to);
  });

  std::string error_msg;
  state_manager_.transition_to(RecorderState::READY, error_msg);
  state_manager_.transition_to(RecorderState::RECORDING, error_msg);
  state_manager_.transition_to(RecorderState::PAUSED, error_msg);
  state_manager_.transition_to(RecorderState::IDLE, error_msg);

  ASSERT_EQ(transitions.size(), 4);
  EXPECT_EQ(transitions[0], std::make_pair(RecorderState::IDLE, RecorderState::READY));
  EXPECT_EQ(transitions[1], std::make_pair(RecorderState::READY, RecorderState::RECORDING));
  EXPECT_EQ(transitions[2], std::make_pair(RecorderState::RECORDING, RecorderState::PAUSED));
  EXPECT_EQ(transitions[3], std::make_pair(RecorderState::PAUSED, RecorderState::IDLE));
}

TEST_F(StateMachineTest, MultipleCallbacks) {
  std::atomic<int> callback1_count{0};
  std::atomic<int> callback2_count{0};

  state_manager_.register_transition_callback([&](RecorderState, RecorderState) {
    callback1_count++;
  });

  state_manager_.register_transition_callback([&](RecorderState, RecorderState) {
    callback2_count++;
  });

  std::string error_msg;
  state_manager_.transition_to(RecorderState::READY, error_msg);

  EXPECT_EQ(callback1_count.load(), 1);
  EXPECT_EQ(callback2_count.load(), 1);
}

TEST_F(StateMachineTest, ThreadSafety) {
  // 4 threads covers typical multi-core contention without excessive test time
  constexpr int kNumThreads = 4;
  // 100 iterations per thread provides sufficient stress testing
  constexpr int kIterations = 100;

  std::vector<std::thread> threads;
  std::atomic<int> success_count{0};
  std::atomic<int> failure_count{0};

  for (int t = 0; t < kNumThreads; ++t) {
    threads.emplace_back([&]() {
      for (int i = 0; i < kIterations; ++i) {
        std::string error_msg;

        // Attempt various transitions
        if (state_manager_.transition_to(RecorderState::READY, error_msg)) {
          success_count++;
        } else {
          failure_count++;
        }

        if (state_manager_.transition_to(RecorderState::RECORDING, error_msg)) {
          success_count++;
        } else {
          failure_count++;
        }

        if (state_manager_.transition_to(RecorderState::IDLE, error_msg)) {
          success_count++;
        } else {
          failure_count++;
        }

        // Read operations - verify no torn reads
        auto state1 = state_manager_.get_state();
        auto state2 = state_manager_.get_state();
        (void)state1;
        (void)state2;  // Suppress unused warnings
        state_manager_.is_recording_active();
      }
    });
  }

  for (auto& thread : threads) {
    thread.join();
  }

  // Verify operations occurred - some succeed, some fail due to race conditions
  EXPECT_GT(success_count.load() + failure_count.load(), 0);

  // Verify final state is valid (not corrupted)
  auto final_state = state_manager_.get_state();
  EXPECT_TRUE(
    final_state == RecorderState::IDLE || final_state == RecorderState::READY ||
    final_state == RecorderState::RECORDING || final_state == RecorderState::PAUSED
  );

  // Verify state string is consistent
  std::string state_str = state_to_string(state_manager_.get_state());
  EXPECT_FALSE(state_str.empty());
  EXPECT_NE(state_str, "unknown");
}

// ============================================================================
// Full Workflow Tests
// ============================================================================

TEST_F(StateMachineTest, TypicalRecordingWorkflow) {
  std::string error_msg;

  // IDLE -> READY (config pushed)
  EXPECT_TRUE(state_manager_.transition_to(RecorderState::READY, error_msg));
  EXPECT_TRUE(state_manager_.is_state(RecorderState::READY));

  // READY -> RECORDING (start command)
  EXPECT_TRUE(state_manager_.transition_to(RecorderState::RECORDING, error_msg));
  EXPECT_TRUE(state_manager_.is_recording_active());

  // RECORDING -> PAUSED (pause command)
  EXPECT_TRUE(state_manager_.transition_to(RecorderState::PAUSED, error_msg));
  EXPECT_TRUE(state_manager_.is_recording_active());

  // PAUSED -> RECORDING (resume command)
  EXPECT_TRUE(state_manager_.transition_to(RecorderState::RECORDING, error_msg));

  // RECORDING -> IDLE (finish command)
  EXPECT_TRUE(state_manager_.transition_to(RecorderState::IDLE, error_msg));
  EXPECT_FALSE(state_manager_.is_recording_active());
}

TEST_F(StateMachineTest, CancelWorkflow) {
  std::string error_msg;

  // IDLE -> READY -> RECORDING
  state_manager_.transition_to(RecorderState::READY, error_msg);
  state_manager_.transition_to(RecorderState::RECORDING, error_msg);

  // RECORDING -> IDLE (cancel command)
  EXPECT_TRUE(state_manager_.transition_to(RecorderState::IDLE, error_msg));
  EXPECT_TRUE(state_manager_.is_state(RecorderState::IDLE));
}

TEST_F(StateMachineTest, ClearConfigWorkflow) {
  std::string error_msg;

  // IDLE -> READY (config pushed)
  state_manager_.transition_to(RecorderState::READY, error_msg);

  // READY -> IDLE (clear command)
  EXPECT_TRUE(state_manager_.transition_to(RecorderState::IDLE, error_msg));
  EXPECT_TRUE(state_manager_.is_state(RecorderState::IDLE));
}

// ============================================================================
// Enhanced Coverage Tests
// ============================================================================

TEST_F(StateMachineTest, SelfTransitionAttempt) {
  std::string error_msg;

  // Try to transition from IDLE to IDLE (self-transition)
  EXPECT_FALSE(state_manager_.transition_to(RecorderState::IDLE, error_msg));
  EXPECT_FALSE(error_msg.empty());
  EXPECT_TRUE(state_manager_.is_state(RecorderState::IDLE));

  // Transition to READY
  EXPECT_TRUE(state_manager_.transition_to(RecorderState::READY, error_msg));

  // Try self-transition from READY
  EXPECT_FALSE(state_manager_.transition_to(RecorderState::READY, error_msg));
  EXPECT_TRUE(state_manager_.is_state(RecorderState::READY));
}

TEST_F(StateMachineTest, TransitionWithExpectedState_WrongCurrent) {
  std::string error_msg;

  // Current state is IDLE
  EXPECT_TRUE(state_manager_.is_state(RecorderState::IDLE));

  // Try transition expecting READY but current is IDLE
  EXPECT_FALSE(
    state_manager_.transition(RecorderState::READY, RecorderState::RECORDING, error_msg)
  );

  // State should remain IDLE
  EXPECT_TRUE(state_manager_.is_state(RecorderState::IDLE));
}

TEST_F(StateMachineTest, TransitionWithExpectedState_InvalidTransition) {
  std::string error_msg;

  // Correctly expect IDLE but try invalid transition to RECORDING
  EXPECT_FALSE(state_manager_.transition(RecorderState::IDLE, RecorderState::RECORDING, error_msg));
}

TEST_F(StateMachineTest, GetValidTransitionsFromAllStates) {
  std::string error_msg;

  // IDLE state
  auto idle_transitions = state_manager_.get_valid_transitions();
  EXPECT_EQ(idle_transitions.size(), 1);
  EXPECT_EQ(idle_transitions[0], RecorderState::READY);

  // READY state
  state_manager_.transition_to(RecorderState::READY, error_msg);
  auto ready_transitions = state_manager_.get_valid_transitions();
  EXPECT_EQ(ready_transitions.size(), 2);

  // RECORDING state
  state_manager_.transition_to(RecorderState::RECORDING, error_msg);
  auto recording_transitions = state_manager_.get_valid_transitions();
  EXPECT_EQ(recording_transitions.size(), 2);

  // PAUSED state
  state_manager_.transition_to(RecorderState::PAUSED, error_msg);
  auto paused_transitions = state_manager_.get_valid_transitions();
  EXPECT_EQ(paused_transitions.size(), 2);
}

TEST_F(StateMachineTest, CallbackInvokedInCorrectOrder) {
  std::vector<int> order;

  state_manager_.register_transition_callback([&](RecorderState, RecorderState) {
    order.push_back(1);
  });

  state_manager_.register_transition_callback([&](RecorderState, RecorderState) {
    order.push_back(2);
  });

  state_manager_.register_transition_callback([&](RecorderState, RecorderState) {
    order.push_back(3);
  });

  std::string error_msg;
  state_manager_.transition_to(RecorderState::READY, error_msg);

  ASSERT_EQ(order.size(), 3);
  EXPECT_EQ(order[0], 1);
  EXPECT_EQ(order[1], 2);
  EXPECT_EQ(order[2], 3);
}

TEST_F(StateMachineTest, CallbackReceivesCorrectStates) {
  RecorderState captured_from = RecorderState::RECORDING;  // Unlikely default
  RecorderState captured_to = RecorderState::RECORDING;

  state_manager_.register_transition_callback([&](RecorderState from, RecorderState to) {
    captured_from = from;
    captured_to = to;
  });

  std::string error_msg;
  state_manager_.transition_to(RecorderState::READY, error_msg);

  EXPECT_EQ(captured_from, RecorderState::IDLE);
  EXPECT_EQ(captured_to, RecorderState::READY);

  // Continue with more transitions
  state_manager_.transition_to(RecorderState::RECORDING, error_msg);
  EXPECT_EQ(captured_from, RecorderState::READY);
  EXPECT_EQ(captured_to, RecorderState::RECORDING);
}

TEST_F(StateMachineTest, FailedTransitionDoesNotInvokeCallback) {
  int callback_count = 0;

  state_manager_.register_transition_callback([&](RecorderState, RecorderState) {
    callback_count++;
  });

  std::string error_msg;

  // Invalid transition (IDLE -> RECORDING)
  EXPECT_FALSE(state_manager_.transition_to(RecorderState::RECORDING, error_msg));
  EXPECT_EQ(callback_count, 0);  // Callback should not be invoked

  // Valid transition
  EXPECT_TRUE(state_manager_.transition_to(RecorderState::READY, error_msg));
  EXPECT_EQ(callback_count, 1);  // Callback invoked once
}

TEST_F(StateMachineTest, ResetClearsState) {
  std::string error_msg;

  // Move to various states
  state_manager_.transition_to(RecorderState::READY, error_msg);
  state_manager_.transition_to(RecorderState::RECORDING, error_msg);
  state_manager_.transition_to(RecorderState::PAUSED, error_msg);

  EXPECT_TRUE(state_manager_.is_state(RecorderState::PAUSED));
  EXPECT_TRUE(state_manager_.is_recording_active());

  // Reset
  state_manager_.reset();

  EXPECT_TRUE(state_manager_.is_state(RecorderState::IDLE));
  EXPECT_FALSE(state_manager_.is_recording_active());
  EXPECT_EQ(state_manager_.get_state_string(), "idle");
}

TEST_F(StateMachineTest, IsRecordingActiveAllStates) {
  std::string error_msg;

  // IDLE - not active
  EXPECT_FALSE(state_manager_.is_recording_active());

  // READY - not active
  state_manager_.transition_to(RecorderState::READY, error_msg);
  EXPECT_FALSE(state_manager_.is_recording_active());

  // RECORDING - active
  state_manager_.transition_to(RecorderState::RECORDING, error_msg);
  EXPECT_TRUE(state_manager_.is_recording_active());

  // PAUSED - active
  state_manager_.transition_to(RecorderState::PAUSED, error_msg);
  EXPECT_TRUE(state_manager_.is_recording_active());

  // Back to IDLE - not active
  state_manager_.transition_to(RecorderState::IDLE, error_msg);
  EXPECT_FALSE(state_manager_.is_recording_active());
}

TEST_F(StateMachineTest, ConcurrentReadsAndWrites) {
  const int num_readers = 4;
  const int num_writers = 2;
  const int iterations = 100;

  std::atomic<bool> stop{false};
  std::atomic<int> read_count{0};
  std::atomic<int> write_count{0};

  std::vector<std::thread> threads;

  // Reader threads
  for (int i = 0; i < num_readers; ++i) {
    threads.emplace_back([&]() {
      while (!stop.load()) {
        state_manager_.get_state();
        state_manager_.get_state_string();
        state_manager_.is_recording_active();
        state_manager_.get_valid_transitions();
        read_count.fetch_add(1);
      }
    });
  }

  // Writer threads
  for (int i = 0; i < num_writers; ++i) {
    threads.emplace_back([&]() {
      for (int j = 0; j < iterations; ++j) {
        std::string error_msg;

        // Try various transitions
        state_manager_.transition_to(RecorderState::READY, error_msg);
        state_manager_.transition_to(RecorderState::RECORDING, error_msg);
        state_manager_.transition_to(RecorderState::PAUSED, error_msg);
        state_manager_.transition_to(RecorderState::IDLE, error_msg);
        write_count.fetch_add(4);
      }
    });
  }

  // Let writers complete
  for (int i = num_readers; i < static_cast<int>(threads.size()); ++i) {
    threads[i].join();
  }

  // Stop readers
  stop.store(true);
  for (int i = 0; i < num_readers; ++i) {
    threads[i].join();
  }

  // Verify operations completed
  EXPECT_GT(read_count.load(), 0);
  EXPECT_EQ(write_count.load(), num_writers * iterations * 4);
}

// ============================================================================
// String Conversion Edge Cases
// ============================================================================

TEST(RecorderStateStringTest, UnknownStateValue) {
  // Cast an invalid int to RecorderState
  RecorderState unknown = static_cast<RecorderState>(999);
  std::string result = state_to_string(unknown);
  EXPECT_EQ(result, "unknown");
}

TEST(RecorderStateStringTest, StringToStateUnknown) {
  // Various unknown strings
  EXPECT_EQ(string_to_state(""), RecorderState::IDLE);
  EXPECT_EQ(string_to_state("unknown"), RecorderState::IDLE);
  EXPECT_EQ(string_to_state("IDLE"), RecorderState::IDLE);  // Case sensitive
  EXPECT_EQ(string_to_state("READY"), RecorderState::IDLE);
  EXPECT_EQ(string_to_state("123"), RecorderState::IDLE);
}

TEST(RecorderStateStringTest, ValidConversions) {
  EXPECT_EQ(string_to_state("idle"), RecorderState::IDLE);
  EXPECT_EQ(string_to_state("ready"), RecorderState::READY);
  EXPECT_EQ(string_to_state("recording"), RecorderState::RECORDING);
  EXPECT_EQ(string_to_state("paused"), RecorderState::PAUSED);
}
