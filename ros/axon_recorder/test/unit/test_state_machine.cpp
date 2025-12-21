/**
 * Unit tests for RecorderState and StateManager
 */

#include <gtest/gtest.h>

#include <atomic>
#include <thread>
#include <vector>

#include "state_machine.hpp"

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

class StateManagerTest : public ::testing::Test {
protected:
  StateManager state_manager_;
};

TEST_F(StateManagerTest, InitialState) {
  EXPECT_EQ(state_manager_.get_state(), RecorderState::IDLE);
  EXPECT_EQ(state_manager_.get_state_string(), "idle");
  EXPECT_TRUE(state_manager_.is_state(RecorderState::IDLE));
  EXPECT_FALSE(state_manager_.is_recording_active());
}

TEST_F(StateManagerTest, ValidTransitionIdleToReady) {
  std::string error_msg;
  EXPECT_TRUE(state_manager_.transition_to(RecorderState::READY, error_msg));
  EXPECT_TRUE(error_msg.empty());
  EXPECT_EQ(state_manager_.get_state(), RecorderState::READY);
}

TEST_F(StateManagerTest, ValidTransitionReadyToRecording) {
  std::string error_msg;
  ASSERT_TRUE(state_manager_.transition_to(RecorderState::READY, error_msg));

  EXPECT_TRUE(state_manager_.transition_to(RecorderState::RECORDING, error_msg));
  EXPECT_TRUE(error_msg.empty());
  EXPECT_EQ(state_manager_.get_state(), RecorderState::RECORDING);
  EXPECT_TRUE(state_manager_.is_recording_active());
}

TEST_F(StateManagerTest, ValidTransitionRecordingToPaused) {
  std::string error_msg;
  ASSERT_TRUE(state_manager_.transition_to(RecorderState::READY, error_msg));
  ASSERT_TRUE(state_manager_.transition_to(RecorderState::RECORDING, error_msg));

  EXPECT_TRUE(state_manager_.transition_to(RecorderState::PAUSED, error_msg));
  EXPECT_TRUE(error_msg.empty());
  EXPECT_EQ(state_manager_.get_state(), RecorderState::PAUSED);
  EXPECT_TRUE(state_manager_.is_recording_active());
}

TEST_F(StateManagerTest, ValidTransitionPausedToRecording) {
  std::string error_msg;
  ASSERT_TRUE(state_manager_.transition_to(RecorderState::READY, error_msg));
  ASSERT_TRUE(state_manager_.transition_to(RecorderState::RECORDING, error_msg));
  ASSERT_TRUE(state_manager_.transition_to(RecorderState::PAUSED, error_msg));

  EXPECT_TRUE(state_manager_.transition_to(RecorderState::RECORDING, error_msg));
  EXPECT_TRUE(error_msg.empty());
  EXPECT_EQ(state_manager_.get_state(), RecorderState::RECORDING);
}

TEST_F(StateManagerTest, ValidTransitionRecordingToIdle) {
  std::string error_msg;
  ASSERT_TRUE(state_manager_.transition_to(RecorderState::READY, error_msg));
  ASSERT_TRUE(state_manager_.transition_to(RecorderState::RECORDING, error_msg));

  EXPECT_TRUE(state_manager_.transition_to(RecorderState::IDLE, error_msg));
  EXPECT_TRUE(error_msg.empty());
  EXPECT_EQ(state_manager_.get_state(), RecorderState::IDLE);
  EXPECT_FALSE(state_manager_.is_recording_active());
}

TEST_F(StateManagerTest, ValidTransitionPausedToIdle) {
  std::string error_msg;
  ASSERT_TRUE(state_manager_.transition_to(RecorderState::READY, error_msg));
  ASSERT_TRUE(state_manager_.transition_to(RecorderState::RECORDING, error_msg));
  ASSERT_TRUE(state_manager_.transition_to(RecorderState::PAUSED, error_msg));

  EXPECT_TRUE(state_manager_.transition_to(RecorderState::IDLE, error_msg));
  EXPECT_TRUE(error_msg.empty());
  EXPECT_EQ(state_manager_.get_state(), RecorderState::IDLE);
}

TEST_F(StateManagerTest, ValidTransitionReadyToIdle) {
  std::string error_msg;
  ASSERT_TRUE(state_manager_.transition_to(RecorderState::READY, error_msg));

  EXPECT_TRUE(state_manager_.transition_to(RecorderState::IDLE, error_msg));
  EXPECT_TRUE(error_msg.empty());
  EXPECT_EQ(state_manager_.get_state(), RecorderState::IDLE);
}

TEST_F(StateManagerTest, InvalidTransitionIdleToRecording) {
  std::string error_msg;
  EXPECT_FALSE(state_manager_.transition_to(RecorderState::RECORDING, error_msg));
  EXPECT_FALSE(error_msg.empty());
  EXPECT_TRUE(error_msg.find("ERR_INVALID_STATE") != std::string::npos);
  EXPECT_EQ(state_manager_.get_state(), RecorderState::IDLE);  // State unchanged
}

TEST_F(StateManagerTest, InvalidTransitionIdleToPaused) {
  std::string error_msg;
  EXPECT_FALSE(state_manager_.transition_to(RecorderState::PAUSED, error_msg));
  EXPECT_FALSE(error_msg.empty());
  EXPECT_EQ(state_manager_.get_state(), RecorderState::IDLE);
}

TEST_F(StateManagerTest, InvalidTransitionReadyToPaused) {
  std::string error_msg;
  ASSERT_TRUE(state_manager_.transition_to(RecorderState::READY, error_msg));

  EXPECT_FALSE(state_manager_.transition_to(RecorderState::PAUSED, error_msg));
  EXPECT_FALSE(error_msg.empty());
  EXPECT_EQ(state_manager_.get_state(), RecorderState::READY);
}

TEST_F(StateManagerTest, TransitionWithExpectedState) {
  std::string error_msg;

  // Correct expected state
  EXPECT_TRUE(
    state_manager_.transition(RecorderState::IDLE, RecorderState::READY, error_msg));
  EXPECT_TRUE(error_msg.empty());

  // Wrong expected state
  EXPECT_FALSE(
    state_manager_.transition(RecorderState::IDLE, RecorderState::RECORDING, error_msg));
  EXPECT_FALSE(error_msg.empty());
  EXPECT_TRUE(error_msg.find("Expected state") != std::string::npos);
}

TEST_F(StateManagerTest, IsValidTransition) {
  // Valid transitions from IDLE
  EXPECT_TRUE(state_manager_.is_valid_transition(RecorderState::IDLE, RecorderState::READY));
  EXPECT_FALSE(
    state_manager_.is_valid_transition(RecorderState::IDLE, RecorderState::RECORDING));
  EXPECT_FALSE(state_manager_.is_valid_transition(RecorderState::IDLE, RecorderState::PAUSED));

  // Valid transitions from READY
  EXPECT_TRUE(
    state_manager_.is_valid_transition(RecorderState::READY, RecorderState::RECORDING));
  EXPECT_TRUE(state_manager_.is_valid_transition(RecorderState::READY, RecorderState::IDLE));
  EXPECT_FALSE(state_manager_.is_valid_transition(RecorderState::READY, RecorderState::PAUSED));

  // Valid transitions from RECORDING
  EXPECT_TRUE(
    state_manager_.is_valid_transition(RecorderState::RECORDING, RecorderState::PAUSED));
  EXPECT_TRUE(
    state_manager_.is_valid_transition(RecorderState::RECORDING, RecorderState::IDLE));
  EXPECT_FALSE(
    state_manager_.is_valid_transition(RecorderState::RECORDING, RecorderState::READY));

  // Valid transitions from PAUSED
  EXPECT_TRUE(
    state_manager_.is_valid_transition(RecorderState::PAUSED, RecorderState::RECORDING));
  EXPECT_TRUE(state_manager_.is_valid_transition(RecorderState::PAUSED, RecorderState::IDLE));
  EXPECT_FALSE(state_manager_.is_valid_transition(RecorderState::PAUSED, RecorderState::READY));
}

TEST_F(StateManagerTest, GetValidTransitions) {
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

TEST_F(StateManagerTest, Reset) {
  std::string error_msg;
  ASSERT_TRUE(state_manager_.transition_to(RecorderState::READY, error_msg));
  ASSERT_TRUE(state_manager_.transition_to(RecorderState::RECORDING, error_msg));

  EXPECT_EQ(state_manager_.get_state(), RecorderState::RECORDING);

  state_manager_.reset();

  EXPECT_EQ(state_manager_.get_state(), RecorderState::IDLE);
}

TEST_F(StateManagerTest, TransitionCallback) {
  std::vector<std::pair<RecorderState, RecorderState>> transitions;

  state_manager_.register_transition_callback(
    [&](RecorderState from, RecorderState to) { transitions.emplace_back(from, to); });

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

TEST_F(StateManagerTest, MultipleCallbacks) {
  std::atomic<int> callback1_count{0};
  std::atomic<int> callback2_count{0};

  state_manager_.register_transition_callback(
    [&](RecorderState, RecorderState) { callback1_count++; });

  state_manager_.register_transition_callback(
    [&](RecorderState, RecorderState) { callback2_count++; });

  std::string error_msg;
  state_manager_.transition_to(RecorderState::READY, error_msg);

  EXPECT_EQ(callback1_count.load(), 1);
  EXPECT_EQ(callback2_count.load(), 1);
}

TEST_F(StateManagerTest, ThreadSafety) {
  const int num_threads = 4;
  const int iterations = 100;

  std::vector<std::thread> threads;
  std::atomic<int> success_count{0};
  std::atomic<int> failure_count{0};

  for (int t = 0; t < num_threads; ++t) {
    threads.emplace_back([&]() {
      for (int i = 0; i < iterations; ++i) {
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

        // Read operations
        state_manager_.get_state();
        state_manager_.is_recording_active();
      }
    });
  }

  for (auto& thread : threads) {
    thread.join();
  }

  // Some transitions succeed, some fail due to race conditions - that's expected
  // The key is that it doesn't crash
  EXPECT_GT(success_count.load() + failure_count.load(), 0);
}

// ============================================================================
// Full Workflow Tests
// ============================================================================

TEST_F(StateManagerTest, TypicalRecordingWorkflow) {
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
  EXPECT_TRUE(state_manager_.is_state(RecorderState::PAUSED));

  // PAUSED -> RECORDING (resume command)
  EXPECT_TRUE(state_manager_.transition_to(RecorderState::RECORDING, error_msg));

  // RECORDING -> IDLE (finish command)
  EXPECT_TRUE(state_manager_.transition_to(RecorderState::IDLE, error_msg));
  EXPECT_FALSE(state_manager_.is_recording_active());
  EXPECT_TRUE(state_manager_.is_state(RecorderState::IDLE));
}

TEST_F(StateManagerTest, CancelWorkflow) {
  std::string error_msg;

  // IDLE -> READY -> RECORDING
  state_manager_.transition_to(RecorderState::READY, error_msg);
  state_manager_.transition_to(RecorderState::RECORDING, error_msg);

  // RECORDING -> IDLE (cancel command)
  EXPECT_TRUE(state_manager_.transition_to(RecorderState::IDLE, error_msg));
  EXPECT_TRUE(state_manager_.is_state(RecorderState::IDLE));
}

TEST_F(StateManagerTest, ClearConfigWorkflow) {
  std::string error_msg;

  // IDLE -> READY (config pushed)
  state_manager_.transition_to(RecorderState::READY, error_msg);

  // READY -> IDLE (clear command)
  EXPECT_TRUE(state_manager_.transition_to(RecorderState::IDLE, error_msg));
  EXPECT_TRUE(state_manager_.is_state(RecorderState::IDLE));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

