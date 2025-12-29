/**
 * @file test_state_transaction_guard.cpp
 * @brief Unit tests for StateTransactionGuard class
 */

#include <gtest/gtest.h>

#include <string>

#include "state_machine.hpp"

namespace axon {
namespace recorder {
namespace {

class StateTransactionGuardTest : public ::testing::Test {
protected:
  void SetUp() override {
    state_manager_ = std::make_unique<StateManager>();
  }

  std::unique_ptr<StateManager> state_manager_;
};

TEST_F(StateTransactionGuardTest, CommitPreventsRollback) {
  // Start in IDLE
  EXPECT_TRUE(state_manager_->is_state(RecorderState::IDLE));

  // Transition to READY
  std::string error;
  ASSERT_TRUE(state_manager_->transition_to(RecorderState::READY, error));

  {
    // Create guard with IDLE as rollback state
    StateTransactionGuard guard(*state_manager_, RecorderState::IDLE);

    // Transition to RECORDING
    ASSERT_TRUE(state_manager_->transition_to(RecorderState::RECORDING, error));

    // Commit - should NOT rollback
    guard.commit();
  }  // Guard destructor runs here

  // Should still be RECORDING (committed, no rollback)
  EXPECT_TRUE(state_manager_->is_state(RecorderState::RECORDING));
}

TEST_F(StateTransactionGuardTest, DestructorRollsBackWithoutCommit) {
  // Start in IDLE
  EXPECT_TRUE(state_manager_->is_state(RecorderState::IDLE));

  // Transition to READY
  std::string error;
  ASSERT_TRUE(state_manager_->transition_to(RecorderState::READY, error));

  {
    // Create guard with IDLE as rollback state
    StateTransactionGuard guard(*state_manager_, RecorderState::IDLE);

    // Transition to RECORDING
    ASSERT_TRUE(state_manager_->transition_to(RecorderState::RECORDING, error));

    // Don't commit - should rollback on destruction
  }  // Guard destructor runs here

  // Should have rolled back to IDLE
  EXPECT_TRUE(state_manager_->is_state(RecorderState::IDLE));
}

TEST_F(StateTransactionGuardTest, IsCommittedReturnsCorrectValue) {
  StateTransactionGuard guard(*state_manager_, RecorderState::IDLE);

  EXPECT_FALSE(guard.is_committed());

  guard.commit();

  EXPECT_TRUE(guard.is_committed());
}

TEST_F(StateTransactionGuardTest, GetRollbackState) {
  StateTransactionGuard guard(*state_manager_, RecorderState::READY);

  EXPECT_EQ(guard.get_rollback_state(), RecorderState::READY);
}

TEST_F(StateTransactionGuardTest, UseInTypicalServicePattern) {
  // Simulate the pattern used in RecordingServiceImpl::handle_start_command

  // Setup: IDLE -> READY
  std::string error;
  ASSERT_TRUE(state_manager_->transition_to(RecorderState::READY, error));

  // Function that simulates start command
  auto start_recording = [this]() -> bool {
    RecorderState previous_state = state_manager_->get_state();
    StateTransactionGuard guard(*state_manager_, previous_state);

    std::string err;
    if (!state_manager_->transition_to(RecorderState::RECORDING, err)) {
      return false;  // Guard will rollback
    }

    // Simulate recording start success
    bool recording_started = true;

    if (!recording_started) {
      return false;  // Guard will rollback
    }

    guard.commit();
    return true;
  };

  EXPECT_TRUE(start_recording());
  EXPECT_TRUE(state_manager_->is_state(RecorderState::RECORDING));
}

TEST_F(StateTransactionGuardTest, RollbackOnFailure) {
  // Setup: IDLE -> READY -> RECORDING
  std::string error;
  ASSERT_TRUE(state_manager_->transition_to(RecorderState::READY, error));
  ASSERT_TRUE(state_manager_->transition_to(RecorderState::RECORDING, error));

  // Function that simulates a recording operation failure
  // When recording fails, it should go back to IDLE (waiting for new config)
  auto recording_operation_with_failure = [this]() -> bool {
    // Guard will rollback to IDLE on failure (recording failures go to IDLE)
    StateTransactionGuard guard(*state_manager_, RecorderState::IDLE);

    // Simulate an operation that fails during recording
    bool operation_succeeded = false;

    if (!operation_succeeded) {
      // Don't commit - guard will rollback to IDLE
      return false;
    }

    guard.commit();
    return true;
  };

  EXPECT_FALSE(recording_operation_with_failure());
  // Should have rolled back to IDLE (recording failures go to IDLE, not READY)
  EXPECT_TRUE(state_manager_->is_state(RecorderState::IDLE));
}

TEST_F(StateTransactionGuardTest, NestedGuards) {
  // Start in IDLE
  std::string error;

  {
    // First guard: rollback to IDLE
    StateTransactionGuard outer_guard(*state_manager_, RecorderState::IDLE);

    // Transition to READY
    ASSERT_TRUE(state_manager_->transition_to(RecorderState::READY, error));

    {
      // Second guard: rollback to READY
      StateTransactionGuard inner_guard(*state_manager_, RecorderState::READY);

      // Transition to RECORDING
      ASSERT_TRUE(state_manager_->transition_to(RecorderState::RECORDING, error));

      // Inner guard commits
      inner_guard.commit();
    }

    // Should still be RECORDING after inner guard commits
    EXPECT_TRUE(state_manager_->is_state(RecorderState::RECORDING));

    // Outer guard does NOT commit - should rollback to IDLE
  }

  // Should have rolled back to IDLE
  EXPECT_TRUE(state_manager_->is_state(RecorderState::IDLE));
}

TEST_F(StateTransactionGuardTest, MultipleCommitsAreSafe) {
  StateTransactionGuard guard(*state_manager_, RecorderState::IDLE);

  guard.commit();
  guard.commit();  // Should be safe to call multiple times
  guard.commit();

  EXPECT_TRUE(guard.is_committed());
}

}  // namespace
}  // namespace recorder
}  // namespace axon

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
