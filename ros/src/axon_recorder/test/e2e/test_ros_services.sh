#!/bin/bash
# ============================================================================
# E2E TEST: ROS Recording Services
# ============================================================================
# This is an E2E TEST that verifies the complete recording workflow
# using actual ROS service calls against a running axon_recorder node.
#
# This test is run automatically by:
#   - CI: .github/workflows/ci.yml (e2e-tests job)
#   - Docker: docker/scripts/run_e2e_tests.sh (Part 3)
#
# Supports: ROS 1 (Noetic) and ROS 2 (Humble/Jazzy/Rolling)
# ============================================================================

set -eo pipefail

# ============================================================================
# Source Library Functions
# ============================================================================
# Libraries are in docker/scripts/lib/ relative to repo root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIB_DIR="${SCRIPT_DIR}/../../../../docker/scripts/lib"

# Source workspace library for ROS version detection
source "${LIB_DIR}/ros_workspace_lib.sh"

# ============================================================================
# ROS Version Detection
# ============================================================================
if [ -n "$ROS_VERSION" ]; then
    echo "Detected ROS_VERSION=$ROS_VERSION"
else
    ROS_VERSION=$(ros_workspace_detect_ros_version)
    if [ -z "$ROS_VERSION" ]; then
        echo "ERROR: Cannot detect ROS version. Please set ROS_DISTRO environment variable." >&2
        exit 1
    fi
    echo "Auto-detected ROS_VERSION=$ROS_VERSION from ROS_DISTRO=${ROS_DISTRO}"
fi

# ============================================================================
# Colors and Counters
# ============================================================================
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

TESTS_PASSED=0
TESTS_FAILED=0

# ============================================================================
# ROS Version-Agnostic Helper Functions
# ============================================================================

# Service call wrapper
svc_call() {
    local service=$1
    local type=$2
    local request=$3
    
    if [ "$ROS_VERSION" = "2" ]; then
        ros2 service call "$service" "axon_recorder/srv/$type" "$request" 2>&1
    else
        rosservice call "$service" "$request" 2>&1
    fi
}

# Service list check
svc_list() {
    if [ "$ROS_VERSION" = "2" ]; then
        ros2 service list 2>/dev/null
    else
        rosservice list 2>/dev/null
    fi
}

# Check if service command available
check_svc_cmd() {
    if [ "$ROS_VERSION" = "2" ]; then
        command -v ros2 &> /dev/null
    else
        command -v rosservice &> /dev/null
    fi
}

# ============================================================================
# Logging Functions
# ============================================================================
log_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }
log_test() { echo -e "\n${GREEN}[TEST]${NC} $1"; }

# ============================================================================
# Assertion Functions
# ============================================================================
assert_success() {
    local result=$1
    local test_name=$2
    # Handle both ROS 1 (success: True) and ROS 2 (success=true) formats
    if echo "$result" | grep -qi "success.*true\|success: True"; then
        log_info "✓ $test_name PASSED"
        TESTS_PASSED=$((TESTS_PASSED + 1))
    else
        log_error "✗ $test_name FAILED"
        log_error "Result: $result"
        TESTS_FAILED=$((TESTS_FAILED + 1))
    fi
}

assert_failure() {
    local result=$1
    local test_name=$2
    if echo "$result" | grep -qi "success.*false\|success: False"; then
        log_info "✓ $test_name PASSED (expected failure)"
        TESTS_PASSED=$((TESTS_PASSED + 1))
    else
        log_error "✗ $test_name FAILED (expected failure but got success)"
        log_error "Result: $result"
        TESTS_FAILED=$((TESTS_FAILED + 1))
    fi
}

assert_status() {
    local result=$1
    local expected=$2
    local test_name=$3
    # Case insensitive status check
    if echo "$result" | grep -qi "status.*$expected"; then
        log_info "✓ $test_name PASSED (status: $expected)"
        TESTS_PASSED=$((TESTS_PASSED + 1))
    else
        log_error "✗ $test_name FAILED (expected status: $expected)"
        log_error "Result: $result"
        TESTS_FAILED=$((TESTS_FAILED + 1))
    fi
}

assert_configured() {
    local result=$1
    local expected=$2  # "true" or "false"
    local test_name=$3
    if echo "$result" | grep -qi "is_configured.*$expected"; then
        log_info "✓ $test_name"
        TESTS_PASSED=$((TESTS_PASSED + 1))
    else
        log_error "✗ $test_name"
        TESTS_FAILED=$((TESTS_FAILED + 1))
    fi
}

# ============================================================================
# Prerequisites Check
# ============================================================================
log_info "Checking prerequisites for ROS $ROS_VERSION..."

if ! check_svc_cmd; then
    log_error "ROS service command not found. Is ROS sourced?"
    exit 1
fi

# Wait for services to be available (with timeout)
TIMEOUT=10
WAITED=0
while ! svc_list | grep -q "axon_recorder"; do
    if [ $WAITED -ge $TIMEOUT ]; then
        log_error "axon_recorder services not found after ${TIMEOUT}s. Is the node running?"
        exit 1
    fi
    log_info "Waiting for axon_recorder services... ($WAITED/$TIMEOUT)"
    sleep 1
    WAITED=$((WAITED + 1))
done

log_info "Prerequisites OK - axon_recorder services found"

# ============================================================================
# Test 1: Initial Status (should be IDLE)
# ============================================================================
log_test "1. Check initial status (should be IDLE)"
RESULT=$(svc_call /axon_recorder/recording_status RecordingStatus "{task_id: ''}")
assert_status "$RESULT" "idle" "Initial status check"

# ============================================================================
# Test 2: IsRecordingReady (should not be configured)
# ============================================================================
log_test "2. Check IsRecordingReady (should not be configured)"
RESULT=$(svc_call /axon_recorder/is_recording_ready IsRecordingReady "{}")
assert_success "$RESULT" "IsRecordingReady query"
assert_configured "$RESULT" "false" "Not configured initially"

# ============================================================================
# Test 3: Start without config (should fail)
# ============================================================================
log_test "3. Start recording without config (should fail)"
RESULT=$(svc_call /axon_recorder/recording_control RecordingControl "{command: 'start', task_id: ''}")
assert_failure "$RESULT" "Start without config"

# ============================================================================
# Test 4: Cache recording config
# ============================================================================
log_test "4. Cache recording config"
RESULT=$(svc_call /axon_recorder/cached_recording_config CachedRecordingConfig "{
  task_id: 'test_task_001',
  device_id: 'test_robot',
  data_collector_id: 'test_collector',
  order_id: '',
  operator_name: '',
  scene: 'integration_test',
  subscene: 'basic_recording',
  skills: ['test'],
  factory: 'test_factory',
  topics: [],
  start_callback_url: '',
  finish_callback_url: '',
  user_token: ''
}")
assert_success "$RESULT" "Cache recording config"

# ============================================================================
# Test 5: Verify config is cached
# ============================================================================
log_test "5. Verify config is cached"
RESULT=$(svc_call /axon_recorder/is_recording_ready IsRecordingReady "{}")
assert_success "$RESULT" "IsRecordingReady after config"
assert_configured "$RESULT" "true" "Config is cached"

# ============================================================================
# Test 6: Check status (should be READY)
# ============================================================================
log_test "6. Check status (should be READY)"
RESULT=$(svc_call /axon_recorder/recording_status RecordingStatus "{task_id: ''}")
assert_status "$RESULT" "ready" "Status after config"

# ============================================================================
# Test 7: Start recording
# ============================================================================
log_test "7. Start recording"
RESULT=$(svc_call /axon_recorder/recording_control RecordingControl "{command: 'start', task_id: ''}")
assert_success "$RESULT" "Start recording"

# ============================================================================
# Test 8: Check status (should be RECORDING)
# ============================================================================
log_test "8. Check status (should be RECORDING)"
sleep 0.5
RESULT=$(svc_call /axon_recorder/recording_status RecordingStatus "{task_id: ''}")
assert_status "$RESULT" "recording" "Status after start"

# ============================================================================
# Test 9: Pause recording
# ============================================================================
log_test "9. Pause recording"
RESULT=$(svc_call /axon_recorder/recording_control RecordingControl "{command: 'pause', task_id: 'test_task_001'}")
assert_success "$RESULT" "Pause recording"

# ============================================================================
# Test 10: Check status (should be PAUSED)
# ============================================================================
log_test "10. Check status (should be PAUSED)"
RESULT=$(svc_call /axon_recorder/recording_status RecordingStatus "{task_id: ''}")
assert_status "$RESULT" "paused" "Status after pause"

# ============================================================================
# Test 11: Resume recording
# ============================================================================
log_test "11. Resume recording"
RESULT=$(svc_call /axon_recorder/recording_control RecordingControl "{command: 'resume', task_id: 'test_task_001'}")
assert_success "$RESULT" "Resume recording"

# ============================================================================
# Test 12: Check status (should be RECORDING)
# ============================================================================
log_test "12. Check status (should be RECORDING)"
RESULT=$(svc_call /axon_recorder/recording_status RecordingStatus "{task_id: ''}")
assert_status "$RESULT" "recording" "Status after resume"

# ============================================================================
# Test 13: Pause with wrong task_id (should fail)
# ============================================================================
log_test "13. Pause with wrong task_id (should fail)"
RESULT=$(svc_call /axon_recorder/recording_control RecordingControl "{command: 'pause', task_id: 'wrong_task_id'}")
assert_failure "$RESULT" "Pause with wrong task_id"

# ============================================================================
# Test 14: Finish recording
# ============================================================================
log_test "14. Finish recording"
RESULT=$(svc_call /axon_recorder/recording_control RecordingControl "{command: 'finish', task_id: 'test_task_001'}")
assert_success "$RESULT" "Finish recording"

# ============================================================================
# Test 15: Check status (should be IDLE)
# ============================================================================
log_test "15. Check status (should be IDLE)"
RESULT=$(svc_call /axon_recorder/recording_status RecordingStatus "{task_id: ''}")
assert_status "$RESULT" "idle" "Status after finish"

# ============================================================================
# Test 16: Test Clear workflow
# ============================================================================
log_test "16. Test Clear workflow"

# Cache config
svc_call /axon_recorder/cached_recording_config CachedRecordingConfig "{
  task_id: 'test_task_002',
  device_id: 'test_robot',
  data_collector_id: 'test_collector',
  order_id: '',
  operator_name: '',
  scene: 'integration_test',
  subscene: 'clear_test',
  skills: [],
  factory: 'test_factory',
  topics: [],
  start_callback_url: '',
  finish_callback_url: '',
  user_token: ''
}" > /dev/null

# Clear without starting
RESULT=$(svc_call /axon_recorder/recording_control RecordingControl "{command: 'clear', task_id: ''}")
assert_success "$RESULT" "Clear config"

# Verify back to idle
RESULT=$(svc_call /axon_recorder/recording_status RecordingStatus "{task_id: ''}")
assert_status "$RESULT" "idle" "Status after clear"

# ============================================================================
# Test 17: Test Cancel workflow
# ============================================================================
log_test "17. Test Cancel workflow"

# Cache config
svc_call /axon_recorder/cached_recording_config CachedRecordingConfig "{
  task_id: 'test_task_003',
  device_id: 'test_robot',
  data_collector_id: 'test_collector',
  order_id: '',
  operator_name: '',
  scene: 'integration_test',
  subscene: 'cancel_test',
  skills: [],
  factory: 'test_factory',
  topics: [],
  start_callback_url: '',
  finish_callback_url: '',
  user_token: ''
}" > /dev/null

# Start recording
svc_call /axon_recorder/recording_control RecordingControl "{command: 'start', task_id: ''}" > /dev/null
sleep 0.5

# Cancel recording
RESULT=$(svc_call /axon_recorder/recording_control RecordingControl "{command: 'cancel', task_id: 'test_task_003'}")
assert_success "$RESULT" "Cancel recording"

# Verify back to idle
RESULT=$(svc_call /axon_recorder/recording_status RecordingStatus "{task_id: ''}")
assert_status "$RESULT" "idle" "Status after cancel"

# ============================================================================
# Summary
# ============================================================================
echo ""
echo "============================================================================"
echo "Integration Test Summary (ROS $ROS_VERSION)"
echo "============================================================================"
echo -e "Tests Passed: ${GREEN}${TESTS_PASSED}${NC}"
echo -e "Tests Failed: ${RED}${TESTS_FAILED}${NC}"
echo ""

if [ $TESTS_FAILED -eq 0 ]; then
    log_info "All integration tests passed!"
    exit 0
else
    log_error "Some integration tests failed!"
    exit 1
fi
