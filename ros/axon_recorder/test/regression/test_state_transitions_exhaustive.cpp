/**
 * @file test_state_transitions_exhaustive.cpp
 * @brief Exhaustive regression tests for state machine transitions
 *
 * Tests every valid and invalid state transition to ensure the state machine
 * behaves correctly under all conditions.
 */

#include <gtest/gtest.h>

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "state_machine.hpp"

using namespace axon::recorder;

// ============================================================================
// Test Fixture
// ============================================================================

class StateTransitionsExhaustiveTest : public ::testing::Test {
protected:
  void SetUp() override {
    state_manager_ = std::make_unique<StateManager>();
  }

  void TearDown() override {
    state_manager_.reset();
  }

  std::unique_ptr<StateManager> state_manager_;
};

// ============================================================================
// Valid Transitions - Comprehensive Coverage
// ============================================================================

TEST_F(StateTransitionsExhaustiveTest, IdleToReady) {
  ASSERT_TRUE(state_manager_->is_state(RecorderState::IDLE));

  std::string error;
  bool result = state_manager_->transition_to(RecorderState::READY, error);

  EXPECT_TRUE(result);
  EXPECT_TRUE(error.empty());
  EXPECT_TRUE(state_manager_->is_state(RecorderState::READY));
}

TEST_F(StateTransitionsExhaustiveTest, ReadyToRecording) {
  std::string error;
  state_manager_->transition_to(RecorderState::READY, error);
  ASSERT_TRUE(state_manager_->is_state(RecorderState::READY));

  bool result = state_manager_->transition_to(RecorderState::RECORDING, error);

  EXPECT_TRUE(result);
  EXPECT_TRUE(error.empty());
  EXPECT_TRUE(state_manager_->is_state(RecorderState::RECORDING));
}

TEST_F(StateTransitionsExhaustiveTest, RecordingToPaused) {
  std::string error;
  state_manager_->transition_to(RecorderState::READY, error);
  state_manager_->transition_to(RecorderState::RECORDING, error);
  ASSERT_TRUE(state_manager_->is_state(RecorderState::RECORDING));

  bool result = state_manager_->transition_to(RecorderState::PAUSED, error);

  EXPECT_TRUE(result);
  EXPECT_TRUE(error.empty());
  EXPECT_TRUE(state_manager_->is_state(RecorderState::PAUSED));
}

TEST_F(StateTransitionsExhaustiveTest, PausedToRecording) {
  std::string error;
  state_manager_->transition_to(RecorderState::READY, error);
  state_manager_->transition_to(RecorderState::RECORDING, error);
  state_manager_->transition_to(RecorderState::PAUSED, error);
  ASSERT_TRUE(state_manager_->is_state(RecorderState::PAUSED));

  bool result = state_manager_->transition_to(RecorderState::RECORDING, error);

  EXPECT_TRUE(result);
  EXPECT_TRUE(error.empty());
  EXPECT_TRUE(state_manager_->is_state(RecorderState::RECORDING));
}

TEST_F(StateTransitionsExhaustiveTest, RecordingToIdle) {
  std::string error;
  state_manager_->transition_to(RecorderState::READY, error);
  state_manager_->transition_to(RecorderState::RECORDING, error);
  ASSERT_TRUE(state_manager_->is_state(RecorderState::RECORDING));

  bool result = state_manager_->transition_to(RecorderState::IDLE, error);

  EXPECT_TRUE(result);
  EXPECT_TRUE(error.empty());
  EXPECT_TRUE(state_manager_->is_state(RecorderState::IDLE));
}

TEST_F(StateTransitionsExhaustiveTest, PausedToIdle) {
  std::string error;
  state_manager_->transition_to(RecorderState::READY, error);
  state_manager_->transition_to(RecorderState::RECORDING, error);
  state_manager_->transition_to(RecorderState::PAUSED, error);
  ASSERT_TRUE(state_manager_->is_state(RecorderState::PAUSED));

  bool result = state_manager_->transition_to(RecorderState::IDLE, error);

  EXPECT_TRUE(result);
  EXPECT_TRUE(error.empty());
  EXPECT_TRUE(state_manager_->is_state(RecorderState::IDLE));
}

TEST_F(StateTransitionsExhaustiveTest, ReadyToIdle) {
  std::string error;
  state_manager_->transition_to(RecorderState::READY, error);
  ASSERT_TRUE(state_manager_->is_state(RecorderState::READY));

  bool result = state_manager_->transition_to(RecorderState::IDLE, error);

  EXPECT_TRUE(result);
  EXPECT_TRUE(error.empty());
  EXPECT_TRUE(state_manager_->is_state(RecorderState::IDLE));
}

// ============================================================================
// Invalid Transitions - Comprehensive Coverage
// ============================================================================

TEST_F(StateTransitionsExhaustiveTest, IdleToRecording_Invalid) {
  ASSERT_TRUE(state_manager_->is_state(RecorderState::IDLE));

  std::string error;
  bool result = state_manager_->transition_to(RecorderState::RECORDING, error);

  EXPECT_FALSE(result);
  EXPECT_FALSE(error.empty());
  EXPECT_TRUE(error.find("ERR_INVALID_STATE") != std::string::npos);
  EXPECT_TRUE(state_manager_->is_state(RecorderState::IDLE));  // Unchanged
}

TEST_F(StateTransitionsExhaustiveTest, IdleToPaused_Invalid) {
  ASSERT_TRUE(state_manager_->is_state(RecorderState::IDLE));

  std::string error;
  bool result = state_manager_->transition_to(RecorderState::PAUSED, error);

  EXPECT_FALSE(result);
  EXPECT_FALSE(error.empty());
  EXPECT_TRUE(error.find("ERR_INVALID_STATE") != std::string::npos);
  EXPECT_TRUE(state_manager_->is_state(RecorderState::IDLE));
}

TEST_F(StateTransitionsExhaustiveTest, ReadyToPaused_Invalid) {
  std::string error;
  state_manager_->transition_to(RecorderState::READY, error);
  ASSERT_TRUE(state_manager_->is_state(RecorderState::READY));

  bool result = state_manager_->transition_to(RecorderState::PAUSED, error);

  EXPECT_FALSE(result);
  EXPECT_FALSE(error.empty());
  EXPECT_TRUE(error.find("ERR_INVALID_STATE") != std::string::npos);
  EXPECT_TRUE(state_manager_->is_state(RecorderState::READY));
}

TEST_F(StateTransitionsExhaustiveTest, RecordingToReady_Invalid) {
  std::string error;
  state_manager_->transition_to(RecorderState::READY, error);
  state_manager_->transition_to(RecorderState::RECORDING, error);
  ASSERT_TRUE(state_manager_->is_state(RecorderState::RECORDING));

  bool result = state_manager_->transition_to(RecorderState::READY, error);

  EXPECT_FALSE(result);
  EXPECT_FALSE(error.empty());
  EXPECT_TRUE(error.find("ERR_INVALID_STATE") != std::string::npos);
  EXPECT_TRUE(state_manager_->is_state(RecorderState::RECORDING));
}

TEST_F(StateTransitionsExhaustiveTest, PausedToReady_Invalid) {
  std::string error;
  state_manager_->transition_to(RecorderState::READY, error);
  state_manager_->transition_to(RecorderState::RECORDING, error);
  state_manager_->transition_to(RecorderState::PAUSED, error);
  ASSERT_TRUE(state_manager_->is_state(RecorderState::PAUSED));

  bool result = state_manager_->transition_to(RecorderState::READY, error);

  EXPECT_FALSE(result);
  EXPECT_FALSE(error.empty());
  EXPECT_TRUE(state_manager_->is_state(RecorderState::PAUSED));
}

TEST_F(StateTransitionsExhaustiveTest, IdleToIdle_Invalid) {
  ASSERT_TRUE(state_manager_->is_state(RecorderState::IDLE));

  std::string error;
  bool result = state_manager_->transition_to(RecorderState::IDLE, error);

  EXPECT_FALSE(result);
  EXPECT_FALSE(error.empty());
  EXPECT_TRUE(state_manager_->is_state(RecorderState::IDLE));
}

TEST_F(StateTransitionsExhaustiveTest, ReadyToReady_Invalid) {
  std::string error;
  state_manager_->transition_to(RecorderState::READY, error);
  ASSERT_TRUE(state_manager_->is_state(RecorderState::READY));

  bool result = state_manager_->transition_to(RecorderState::READY, error);

  EXPECT_FALSE(result);
  EXPECT_FALSE(error.empty());
  EXPECT_TRUE(state_manager_->is_state(RecorderState::READY));
}

TEST_F(StateTransitionsExhaustiveTest, RecordingToRecording_Invalid) {
  std::string error;
  state_manager_->transition_to(RecorderState::READY, error);
  state_manager_->transition_to(RecorderState::RECORDING, error);
  ASSERT_TRUE(state_manager_->is_state(RecorderState::RECORDING));

  bool result = state_manager_->transition_to(RecorderState::RECORDING, error);

  EXPECT_FALSE(result);
  EXPECT_FALSE(error.empty());
  EXPECT_TRUE(state_manager_->is_state(RecorderState::RECORDING));
}

TEST_F(StateTransitionsExhaustiveTest, PausedToPaused_Invalid) {
  std::string error;
  state_manager_->transition_to(RecorderState::READY, error);
  state_manager_->transition_to(RecorderState::RECORDING, error);
  state_manager_->transition_to(RecorderState::PAUSED, error);
  ASSERT_TRUE(state_manager_->is_state(RecorderState::PAUSED));

  bool result = state_manager_->transition_to(RecorderState::PAUSED, error);

  EXPECT_FALSE(result);
  EXPECT_FALSE(error.empty());
  EXPECT_TRUE(state_manager_->is_state(RecorderState::PAUSED));
}

// ============================================================================
// Transition with Expected State Verification
// ============================================================================

TEST_F(StateTransitionsExhaustiveTest, TransitionWithCorrectFromState) {
  ASSERT_TRUE(state_manager_->is_state(RecorderState::IDLE));

  std::string error;
  bool result = state_manager_->transition(RecorderState::IDLE, RecorderState::READY, error);

  EXPECT_TRUE(result);
  EXPECT_TRUE(error.empty());
  EXPECT_TRUE(state_manager_->is_state(RecorderState::READY));
}

TEST_F(StateTransitionsExhaustiveTest, TransitionWithIncorrectFromState) {
  ASSERT_TRUE(state_manager_->is_state(RecorderState::IDLE));

  std::string error;
  // Expect READY but current is IDLE
  bool result = state_manager_->transition(RecorderState::READY, RecorderState::RECORDING, error);

  EXPECT_FALSE(result);
  EXPECT_FALSE(error.empty());
  EXPECT_TRUE(error.find("Expected state") != std::string::npos);
  EXPECT_TRUE(state_manager_->is_state(RecorderState::IDLE));  // Unchanged
}

// ============================================================================
// Full Workflow Tests
// ============================================================================

TEST_F(StateTransitionsExhaustiveTest, CompleteRecordingWorkflow) {
  std::string error;

  // IDLE -> READY (cache config)
  EXPECT_TRUE(state_manager_->transition_to(RecorderState::READY, error));
  EXPECT_TRUE(state_manager_->is_state(RecorderState::READY));

  // READY -> RECORDING (start)
  EXPECT_TRUE(state_manager_->transition_to(RecorderState::RECORDING, error));
  EXPECT_TRUE(state_manager_->is_state(RecorderState::RECORDING));
  EXPECT_TRUE(state_manager_->is_recording_active());

  // RECORDING -> PAUSED (pause)
  EXPECT_TRUE(state_manager_->transition_to(RecorderState::PAUSED, error));
  EXPECT_TRUE(state_manager_->is_state(RecorderState::PAUSED));
  EXPECT_TRUE(state_manager_->is_recording_active());

  // PAUSED -> RECORDING (resume)
  EXPECT_TRUE(state_manager_->transition_to(RecorderState::RECORDING, error));
  EXPECT_TRUE(state_manager_->is_state(RecorderState::RECORDING));

  // RECORDING -> IDLE (finish)
  EXPECT_TRUE(state_manager_->transition_to(RecorderState::IDLE, error));
  EXPECT_TRUE(state_manager_->is_state(RecorderState::IDLE));
  EXPECT_FALSE(state_manager_->is_recording_active());
}

TEST_F(StateTransitionsExhaustiveTest, CancelWorkflow) {
  std::string error;

  // IDLE -> READY -> RECORDING
  state_manager_->transition_to(RecorderState::READY, error);
  state_manager_->transition_to(RecorderState::RECORDING, error);
  EXPECT_TRUE(state_manager_->is_state(RecorderState::RECORDING));

  // RECORDING -> IDLE (cancel)
  EXPECT_TRUE(state_manager_->transition_to(RecorderState::IDLE, error));
  EXPECT_TRUE(state_manager_->is_state(RecorderState::IDLE));
}

TEST_F(StateTransitionsExhaustiveTest, ClearConfigWorkflow) {
  std::string error;

  // IDLE -> READY (cache config)
  state_manager_->transition_to(RecorderState::READY, error);
  EXPECT_TRUE(state_manager_->is_state(RecorderState::READY));

  // READY -> IDLE (clear)
  EXPECT_TRUE(state_manager_->transition_to(RecorderState::IDLE, error));
  EXPECT_TRUE(state_manager_->is_state(RecorderState::IDLE));
}

// ============================================================================
// Concurrent Access Tests
// ============================================================================

TEST_F(StateTransitionsExhaustiveTest, ConcurrentTransitions_ThreadSafe) {
  const int NUM_THREADS = 8;
  const int ITERATIONS = 100;

  std::atomic<int> success_count{0};
  std::atomic<int> failure_count{0};

  auto worker = [&](int /* thread_id */) {
    for (int i = 0; i < ITERATIONS; ++i) {
      std::string error;

      // Try IDLE -> READY
      if (state_manager_->transition_to(RecorderState::READY, error)) {
        success_count++;

        // Try READY -> RECORDING
        if (state_manager_->transition_to(RecorderState::RECORDING, error)) {
          // Try RECORDING -> IDLE
          if (state_manager_->transition_to(RecorderState::IDLE, error)) {
            // Full cycle completed
          } else {
            failure_count++;
          }
        } else {
          // Try READY -> IDLE (clear)
          state_manager_->transition_to(RecorderState::IDLE, error);
        }
      } else {
        failure_count++;
      }
    }
  };

  std::vector<std::thread> threads;
  for (int t = 0; t < NUM_THREADS; ++t) {
    threads.emplace_back(worker, t);
  }

  for (auto& thread : threads) {
    thread.join();
  }

  // Should not crash and state should be consistent
  // Final state should be valid (IDLE, READY, RECORDING, or PAUSED)
  auto state = state_manager_->get_state();
  EXPECT_TRUE(
    state == RecorderState::IDLE ||
    state == RecorderState::READY ||
    state == RecorderState::RECORDING ||
    state == RecorderState::PAUSED
  );
}

TEST_F(StateTransitionsExhaustiveTest, ConcurrentReads) {
  std::string error;
  state_manager_->transition_to(RecorderState::READY, error);

  const int NUM_THREADS = 8;
  const int ITERATIONS = 1000;

  std::atomic<int> read_count{0};

  auto reader = [&]() {
    for (int i = 0; i < ITERATIONS; ++i) {
      auto state = state_manager_->get_state();
      auto state_str = state_manager_->get_state_string();
      auto is_active = state_manager_->is_recording_active();
      auto valid = state_manager_->get_valid_transitions();

      // Suppress unused variable warnings
      (void)state;
      (void)state_str;
      (void)is_active;
      (void)valid;

      read_count++;
    }
  };

  std::vector<std::thread> threads;
  for (int t = 0; t < NUM_THREADS; ++t) {
    threads.emplace_back(reader);
  }

  for (auto& thread : threads) {
    thread.join();
  }

  EXPECT_EQ(read_count.load(), NUM_THREADS * ITERATIONS);
}

// ============================================================================
// Callback Tests
// ============================================================================

TEST_F(StateTransitionsExhaustiveTest, TransitionCallbackInvoked) {
  RecorderState from_state = RecorderState::IDLE;
  RecorderState to_state = RecorderState::IDLE;
  int callback_count = 0;

  state_manager_->register_transition_callback(
    [&](RecorderState from, RecorderState to) {
      from_state = from;
      to_state = to;
      callback_count++;
    }
  );

  std::string error;
  state_manager_->transition_to(RecorderState::READY, error);

  EXPECT_EQ(callback_count, 1);
  EXPECT_EQ(from_state, RecorderState::IDLE);
  EXPECT_EQ(to_state, RecorderState::READY);
}

TEST_F(StateTransitionsExhaustiveTest, MultipleCallbacksInvoked) {
  int callback1_count = 0;
  int callback2_count = 0;

  state_manager_->register_transition_callback(
    [&](RecorderState, RecorderState) { callback1_count++; }
  );
  state_manager_->register_transition_callback(
    [&](RecorderState, RecorderState) { callback2_count++; }
  );

  std::string error;
  state_manager_->transition_to(RecorderState::READY, error);

  EXPECT_EQ(callback1_count, 1);
  EXPECT_EQ(callback2_count, 1);
}

TEST_F(StateTransitionsExhaustiveTest, CallbackNotInvokedOnFailure) {
  int callback_count = 0;

  state_manager_->register_transition_callback(
    [&](RecorderState, RecorderState) { callback_count++; }
  );

  std::string error;
  // Invalid transition
  state_manager_->transition_to(RecorderState::RECORDING, error);

  EXPECT_EQ(callback_count, 0);  // Not invoked
}

// ============================================================================
// Reset Tests
// ============================================================================

TEST_F(StateTransitionsExhaustiveTest, ResetToIdle) {
  std::string error;
  state_manager_->transition_to(RecorderState::READY, error);
  state_manager_->transition_to(RecorderState::RECORDING, error);
  EXPECT_TRUE(state_manager_->is_state(RecorderState::RECORDING));

  state_manager_->reset();

  EXPECT_TRUE(state_manager_->is_state(RecorderState::IDLE));
}

// ============================================================================
// Helper Function Tests
// ============================================================================

TEST_F(StateTransitionsExhaustiveTest, StateToString) {
  EXPECT_EQ(state_to_string(RecorderState::IDLE), "idle");
  EXPECT_EQ(state_to_string(RecorderState::READY), "ready");
  EXPECT_EQ(state_to_string(RecorderState::RECORDING), "recording");
  EXPECT_EQ(state_to_string(RecorderState::PAUSED), "paused");
}

TEST_F(StateTransitionsExhaustiveTest, StringToState) {
  EXPECT_EQ(string_to_state("idle"), RecorderState::IDLE);
  EXPECT_EQ(string_to_state("ready"), RecorderState::READY);
  EXPECT_EQ(string_to_state("recording"), RecorderState::RECORDING);
  EXPECT_EQ(string_to_state("paused"), RecorderState::PAUSED);
  EXPECT_EQ(string_to_state("unknown"), RecorderState::IDLE);  // Default
  EXPECT_EQ(string_to_state(""), RecorderState::IDLE);  // Default
}

TEST_F(StateTransitionsExhaustiveTest, GetValidTransitions) {
  // From IDLE
  EXPECT_TRUE(state_manager_->is_state(RecorderState::IDLE));
  auto valid = state_manager_->get_valid_transitions();
  EXPECT_EQ(valid.size(), 1u);
  EXPECT_EQ(valid[0], RecorderState::READY);

  // From READY
  std::string error;
  state_manager_->transition_to(RecorderState::READY, error);
  valid = state_manager_->get_valid_transitions();
  EXPECT_EQ(valid.size(), 2u);

  // From RECORDING
  state_manager_->transition_to(RecorderState::RECORDING, error);
  valid = state_manager_->get_valid_transitions();
  EXPECT_EQ(valid.size(), 2u);

  // From PAUSED
  state_manager_->transition_to(RecorderState::PAUSED, error);
  valid = state_manager_->get_valid_transitions();
  EXPECT_EQ(valid.size(), 2u);
}

TEST_F(StateTransitionsExhaustiveTest, IsValidTransitionDirectCheck) {
  // Valid transitions
  EXPECT_TRUE(state_manager_->is_valid_transition(RecorderState::IDLE, RecorderState::READY));
  EXPECT_TRUE(state_manager_->is_valid_transition(RecorderState::READY, RecorderState::IDLE));
  EXPECT_TRUE(state_manager_->is_valid_transition(RecorderState::READY, RecorderState::RECORDING));
  EXPECT_TRUE(state_manager_->is_valid_transition(RecorderState::RECORDING, RecorderState::PAUSED));
  EXPECT_TRUE(state_manager_->is_valid_transition(RecorderState::RECORDING, RecorderState::IDLE));
  EXPECT_TRUE(state_manager_->is_valid_transition(RecorderState::PAUSED, RecorderState::RECORDING));
  EXPECT_TRUE(state_manager_->is_valid_transition(RecorderState::PAUSED, RecorderState::IDLE));

  // Invalid transitions
  EXPECT_FALSE(state_manager_->is_valid_transition(RecorderState::IDLE, RecorderState::RECORDING));
  EXPECT_FALSE(state_manager_->is_valid_transition(RecorderState::IDLE, RecorderState::PAUSED));
  EXPECT_FALSE(state_manager_->is_valid_transition(RecorderState::READY, RecorderState::PAUSED));
  EXPECT_FALSE(state_manager_->is_valid_transition(RecorderState::RECORDING, RecorderState::READY));
  EXPECT_FALSE(state_manager_->is_valid_transition(RecorderState::PAUSED, RecorderState::READY));

  // Self-transitions (invalid)
  EXPECT_FALSE(state_manager_->is_valid_transition(RecorderState::IDLE, RecorderState::IDLE));
  EXPECT_FALSE(state_manager_->is_valid_transition(RecorderState::READY, RecorderState::READY));
  EXPECT_FALSE(state_manager_->is_valid_transition(RecorderState::RECORDING, RecorderState::RECORDING));
  EXPECT_FALSE(state_manager_->is_valid_transition(RecorderState::PAUSED, RecorderState::PAUSED));
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

