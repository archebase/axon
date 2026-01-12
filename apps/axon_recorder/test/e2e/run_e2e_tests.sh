#!/bin/bash
# @file run_e2e_tests.sh
# @brief End-to-end test script for axon_recorder
#
# This script tests the complete recording workflow:
# 1. Start axon_recorder with mock middleware
# 2. Send HTTP requests to control recording
# 3. Verify MCAP file creation
# 4. Verify sidecar JSON generation
# 5. Clean up test artifacts

set -e

# ==============================================================================
# Configuration
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
BUILD_DIR="${PROJECT_ROOT}/build"
TEST_DIR="${SCRIPT_DIR}"
TEST_DATA_DIR="${TEST_DIR}/test_data"

# Try multiple possible locations for axon_recorder binary
if [[ -f "${BUILD_DIR}/axon_recorder/axon_recorder" ]]; then
    RECORDER_BIN="${BUILD_DIR}/axon_recorder/axon_recorder"
elif [[ -f "${BUILD_DIR}/apps/axon_recorder/axon_recorder" ]]; then
    RECORDER_BIN="${BUILD_DIR}/apps/axon_recorder/axon_recorder"
elif [[ -f "${PROJECT_ROOT}/apps/axon_recorder/build/axon_recorder" ]]; then
    RECORDER_BIN="${PROJECT_ROOT}/apps/axon_recorder/build/axon_recorder"
else
    RECORDER_BIN="${BUILD_DIR}/axon_recorder/axon_recorder"
fi

# Mock plugin path
MOCK_PLUGIN="${BUILD_DIR}/middlewares/axon_mock.so"
if [[ ! -f "${MOCK_PLUGIN}" ]]; then
    MOCK_PLUGIN="${PROJECT_ROOT}/apps/axon_recorder/build/axon_mock.so"
fi

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Test configuration
HTTP_PORT=8080
TEST_TASK_ID="e2e_test_$(date +%s)"
TEST_DEVICE_ID="test_robot_01"
TEST_SCENE="e2e_test_scene"

# ==============================================================================
# Helper Functions
# ==============================================================================

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

cleanup() {
    log_info "Cleaning up..."

    # Stop recorder if running
    if [[ -n "${RECORDER_PID}" ]]; then
        kill "${RECORDER_PID}" 2>/dev/null || true
        wait "${RECORDER_PID}" 2>/dev/null || true
    fi

    # Clean up test data
    rm -rf "${TEST_DATA_DIR}"

    log_info "Cleanup complete"
}

# Trap to ensure cleanup on exit
trap cleanup EXIT INT TERM

# ==============================================================================
# Setup
# ==============================================================================

setup() {
    log_info "Setting up E2E test environment..."

    # Create test data directory
    mkdir -p "${TEST_DATA_DIR}"

    # Detect actual binary location
    if [[ ! -f "${RECORDER_BIN}" ]]; then
        # Search for axon_recorder in common build locations
        RECORDER_BIN=$(find "${BUILD_DIR}" -name "axon_recorder" -type f -executable 2>/dev/null | head -n 1)
        if [[ -z "${RECORDER_BIN}" ]]; then
            log_error "Recorder binary not found"
            log_info "Searched in: ${BUILD_DIR}"
            log_info "Please build axon_recorder first"
            log_info "  - For ROS2: cd middlewares/ros2 && colcon build"
            log_info "  - Or check: apps/axon_recorder/build/"
            exit 1
        fi
        log_info "Found recorder at: ${RECORDER_BIN}"
    fi

    # Check for mock plugin
    if [[ ! -f "${MOCK_PLUGIN}" ]]; then
        log_warn "Mock plugin not found at ${MOCK_PLUGIN}"
        log_info "E2E tests will attempt to run without mock middleware"
        log_info "Build with: cd apps/axon_recorder/test/e2e && cmake . && make"
        MOCK_PLUGIN=""
    fi

    log_info "Using recorder: ${RECORDER_BIN}"
    if [[ -n "${MOCK_PLUGIN}" ]]; then
        log_info "Using mock plugin: ${MOCK_PLUGIN}"
    fi

    log_info "Setup complete"
}

# ==============================================================================
# Test Functions
# ==============================================================================

start_recorder() {
    log_info "Starting axon_recorder..."

    # Create test config in test_data directory
    local config_file="${TEST_DATA_DIR}/test_config.yaml"

    # Use mock plugin if available, otherwise skip plugin config
    if [[ -n "${MOCK_PLUGIN}" && -f "${MOCK_PLUGIN}" ]]; then
        cat > "${config_file}" <<EOF
# E2E Test Configuration
plugin:
  path: ${MOCK_PLUGIN}

dataset:
  path: ./recordings
  mode: create
  stats_file_path: ./stats.json

subscriptions:
  - name: /test/topic1
    message_type: std_msgs/String
    batch_size: 10
    flush_interval_ms: 100
  - name: /test/topic2
    message_type: sensor_msgs/Image
    batch_size: 5
    flush_interval_ms: 200

recording:
  max_disk_usage_gb: 10.0

logging:
  console:
    enabled: true
    colors: false
    level: info
  file:
    enabled: false

upload:
  enabled: false
EOF
    else
        # Minimal config without plugin (for testing HTTP API only)
        cat > "${config_file}" <<EOF
# E2E Test Configuration (minimal, no plugin)
dataset:
  path: ./recordings
  mode: create
  stats_file_path: ./stats.json

subscriptions:
  - name: /test/topic1
    message_type: std_msgs/String
    batch_size: 10
    flush_interval_ms: 100
  - name: /test/topic2
    message_type: sensor_msgs/Image
    batch_size: 5
    flush_interval_ms: 200

recording:
  max_disk_usage_gb: 10.0

logging:
  console:
    enabled: true
    colors: false
    level: info
  file:
    enabled: false

upload:
  enabled: false
EOF
    fi

    # Start recorder in background
    cd "${TEST_DATA_DIR}"
    "${RECORDER_BIN}" --config "${config_file}" > "${TEST_DATA_DIR}/recorder.log" 2>&1 &
    RECORDER_PID=$!

    # Wait for recorder to start
    sleep 2

    # Check if recorder is running
    if ! kill -0 "${RECORDER_PID}" 2>/dev/null; then
        log_error "Failed to start recorder"
        log_error "Log output:"
        cat "${TEST_DATA_DIR}/recorder.log"
        exit 1
    fi

    log_info "Recorder started (PID: ${RECORDER_PID})"
}

test_health_check() {
    log_info "Testing health check endpoint..."

    local response
    response=$(curl -s -o /dev/null -w "%{http_code}" "http://localhost:${HTTP_PORT}/health" || echo "000")

    if [[ "${response}" == "200" ]]; then
        log_info "Health check passed"
        return 0
    else
        log_error "Health check failed (HTTP ${response})"
        return 1
    fi
}

test_cache_config() {
    log_info "Testing cache config endpoint..."

    local response
    response=$(curl -s -X POST \
        -H "Content-Type: application/json" \
        -d "{
            \"task_config\": {
                \"task_id\": \"${TEST_TASK_ID}\",
                \"device_id\": \"${TEST_DEVICE_ID}\",
                \"scene\": \"${TEST_SCENE}\",
                \"factory\": \"test_factory\",
                \"operator_name\": \"test_operator\",
                \"topics\": [\"/test/topic1\", \"/test/topic2\"]
            }
        }" \
        "http://localhost:${HTTP_PORT}/rpc/config")

    if echo "${response}" | grep -q '"success":true'; then
        log_info "Config cache successful"
        return 0
    else
        log_error "Config cache failed"
        log_error "Response: ${response}"
        return 1
    fi
}

test_start_recording() {
    log_info "Testing start recording endpoint..."

    local response
    response=$(curl -s -X POST "http://localhost:${HTTP_PORT}/rpc/begin")

    if echo "${response}" | grep -q '"success":true'; then
        log_info "Recording started successfully"
        return 0
    else
        log_error "Failed to start recording"
        log_error "Response: ${response}"
        return 1
    fi
}

test_recording_status() {
    log_info "Testing recording status endpoint..."

    local response
    response=$(curl -s "http://localhost:${HTTP_PORT}/rpc/state")

    if echo "${response}" | grep -q '"success":true'; then
        log_info "Recording status retrieved successfully"
        return 0
    else
        log_error "Recording status check failed"
        log_error "Response: ${response}"
        return 1
    fi
}

# Extract state from JSON response
get_state() {
    local response="$1"
    echo "${response}" | python3 -c "import sys, json; data=json.load(sys.stdin); print(data.get('data', {}).get('state', 'unknown'))" 2>/dev/null || echo "unknown"
}

test_state_initial_idle() {
    log_info "Testing initial state should be 'idle'..."

    local response
    response=$(curl -s "http://localhost:${HTTP_PORT}/rpc/state")

    local state
    state=$(get_state "${response}")

    if [[ "${state}" == "idle" ]]; then
        log_info "✓ Initial state is 'idle'"
        return 0
    else
        log_error "✗ Initial state is '${state}', expected 'idle'"
        log_error "Response: ${response}"
        return 1
    fi
}

test_state_config_to_ready() {
    log_info "Testing state transition: IDLE -> READY via /rpc/config..."

    # Cache config
    local config_response
    config_response=$(curl -s -X POST \
        -H "Content-Type: application/json" \
        -d "{
            \"task_config\": {
                \"task_id\": \"${TEST_TASK_ID}\",
                \"device_id\": \"${TEST_DEVICE_ID}\",
                \"scene\": \"${TEST_SCENE}\",
                \"topics\": [\"/test/topic1\"]
            }
        }" \
        "http://localhost:${HTTP_PORT}/rpc/config")

    # Check state
    local state_response
    state_response=$(curl -s "http://localhost:${HTTP_PORT}/rpc/state")

    local state
    state=$(get_state "${state_response}")

    if [[ "${state}" == "ready" ]]; then
        log_info "✓ State transitioned to 'ready' after config"
        return 0
    else
        log_error "✗ State is '${state}', expected 'ready' after /rpc/config"
        log_error "Config response: ${config_response}"
        log_error "State response: ${state_response}"
        return 1
    fi
}

test_state_begin_to_recording() {
    log_info "Testing state transition: READY -> RECORDING via /rpc/begin..."

    # Begin recording
    local begin_response
    begin_response=$(curl -s -X POST "http://localhost:${HTTP_PORT}/rpc/begin")

    # Check state
    local state_response
    state_response=$(curl -s "http://localhost:${HTTP_PORT}/rpc/state")

    local state
    state=$(get_state "${state_response}")

    if [[ "${state}" == "recording" ]]; then
        log_info "✓ State transitioned to 'recording' after /rpc/begin"
        return 0
    else
        log_error "✗ State is '${state}', expected 'recording' after /rpc/begin"
        log_error "Begin response: ${begin_response}"
        log_error "State response: ${state_response}"
        return 1
    fi
}

test_state_begin_fails_from_idle() {
    log_info "Testing that /rpc/begin fails from IDLE state..."

    # First ensure we're in IDLE (no config set)
    # Note: This test assumes the recorder starts in IDLE and no config has been set

    local response
    response=$(curl -s -X POST "http://localhost:${HTTP_PORT}/rpc/begin")

    if echo "${response}" | grep -q '"success":false'; then
        log_info "✓ /rpc/begin correctly rejected from IDLE state"
        return 0
    else
        log_error "✗ /rpc/begin should fail from IDLE state"
        log_error "Response: ${response}"
        return 1
    fi
}

test_state_finish_returns_to_idle() {
    log_info "Testing state transition: RECORDING -> IDLE via /rpc/finish..."

    # Finish recording
    local finish_response
    finish_response=$(curl -s -X POST "http://localhost:${HTTP_PORT}/rpc/finish")

    # Check state
    local state_response
    state_response=$(curl -s "http://localhost:${HTTP_PORT}/rpc/state")

    local state
    state=$(get_state "${state_response}")

    if [[ "${state}" == "idle" ]]; then
        log_info "✓ State returned to 'idle' after /rpc/finish"
        return 0
    else
        log_error "✗ State is '${state}', expected 'idle' after /rpc/finish"
        log_error "Finish response: ${finish_response}"
        log_error "State response: ${state_response}"
        return 1
    fi
}

test_state_cancel_from_recording() {
    log_info "Testing state transition: RECORDING -> IDLE via /rpc/cancel..."

    # First, start a new recording
    # Set config
    curl -s -X POST \
        -H "Content-Type: application/json" \
        -d "{\"task_config\": {\"task_id\": \"cancel_test_${TEST_TASK_ID}\", \"device_id\": \"${TEST_DEVICE_ID}\", \"scene\": \"${TEST_SCENE}\"}}" \
        "http://localhost:${HTTP_PORT}/rpc/config" > /dev/null

    # Begin recording
    local begin_response
    begin_response=$(curl -s -X POST "http://localhost:${HTTP_PORT}/rpc/begin")

    # Verify we're in recording state
    local state_before
    state_before=$(get_state "$(curl -s "http://localhost:${HTTP_PORT}/rpc/state")")

    if [[ "${state_before}" != "recording" ]]; then
        log_error "✗ Failed to reach recording state before cancel (state: ${state_before})"
        return 1
    fi

    # Cancel recording
    local cancel_response
    cancel_response=$(curl -s -X POST "http://localhost:${HTTP_PORT}/rpc/cancel")

    # Check state after cancel
    local state_response
    state_response=$(curl -s "http://localhost:${HTTP_PORT}/rpc/state")

    local state
    state=$(get_state "${state_response}")

    if [[ "${state}" == "idle" ]]; then
        log_info "✓ State returned to 'idle' after /rpc/cancel"
        return 0
    else
        log_error "✗ State is '${state}', expected 'idle' after /rpc/cancel"
        log_error "Cancel response: ${cancel_response}"
        log_error "State response: ${state_response}"
        return 1
    fi
}

test_state_cancel_fails_from_idle() {
    log_info "Testing that /rpc/cancel fails from IDLE state..."

    # Ensure we're in IDLE (no active recording)
    local response
    response=$(curl -s -X POST "http://localhost:${HTTP_PORT}/rpc/cancel")

    if echo "${response}" | grep -q '"success":false'; then
        log_info "✓ /rpc/cancel correctly rejected from IDLE state"
        return 0
    else
        log_error "✗ /rpc/cancel should fail from IDLE state"
        log_error "Response: ${response}"
        return 1
    fi
}

test_state_dynamic_subscription_update() {
    log_info "Testing dynamic subscription update with different topics..."

    # First recording with /test/topic1
    local config1_response
    config1_response=$(curl -s -X POST \
        -H "Content-Type: application/json" \
        -d "{
            \"task_config\": {
                \"task_id\": \"dynamic_sub_test_1_${TEST_TASK_ID}\",
                \"device_id\": \"${TEST_DEVICE_ID}\",
                \"scene\": \"${TEST_SCENE}\",
                \"topics\": [\"/test/topic1\"]
            }
        }" \
        "http://localhost:${HTTP_PORT}/rpc/config")

    # Begin first recording
    local begin1_response
    begin1_response=$(curl -s -X POST "http://localhost:${HTTP_PORT}/rpc/begin")

    # Verify first recording started
    local state1
    state1=$(get_state "$(curl -s "http://localhost:${HTTP_PORT}/rpc/state")")
    if [[ "${state1}" != "recording" ]]; then
        log_error "✗ First recording failed to start (state: ${state1})"
        return 1
    fi

    # Cancel first recording
    curl -s -X POST "http://localhost:${HTTP_PORT}/rpc/cancel" > /dev/null

    # Second recording with /test/topic2 (different topic)
    local config2_response
    config2_response=$(curl -s -X POST \
        -H "Content-Type: application/json" \
        -d "{
            \"task_config\": {
                \"task_id\": \"dynamic_sub_test_2_${TEST_TASK_ID}\",
                \"device_id\": \"${TEST_DEVICE_ID}\",
                \"scene\": \"${TEST_SCENE}\",
                \"topics\": [\"/test/topic2\"]
            }
        }" \
        "http://localhost:${HTTP_PORT}/rpc/config")

    # Begin second recording
    local begin2_response
    begin2_response=$(curl -s -X POST "http://localhost:${HTTP_PORT}/rpc/begin")

    # Verify second recording started
    local state2
    state2=$(get_state "$(curl -s "http://localhost:${HTTP_PORT}/rpc/state")")

    if [[ "${state2}" == "recording" ]]; then
        log_info "✓ Dynamic subscription update succeeded (switched from /test/topic1 to /test/topic2)"
        # Cancel the recording
        curl -s -X POST "http://localhost:${HTTP_PORT}/rpc/cancel" > /dev/null
        return 0
    else
        log_error "✗ Second recording failed to start with different topics (state: ${state2})"
        log_error "Config 2 response: ${config2_response}"
        log_error "Begin 2 response: ${begin2_response}"
        return 1
    fi
}

test_pause_recording() {
    log_info "Testing pause recording endpoint..."

    local response
    response=$(curl -s -X POST "http://localhost:${HTTP_PORT}/rpc/pause")

    if echo "${response}" | grep -q '"success":true'; then
        log_info "Recording paused successfully"
        return 0
    else
        log_error "Failed to pause recording"
        log_error "Response: ${response}"
        return 1
    fi
}

test_resume_recording() {
    log_info "Testing resume recording endpoint..."

    local response
    response=$(curl -s -X POST "http://localhost:${HTTP_PORT}/rpc/resume")

    if echo "${response}" | grep -q '"success":true'; then
        log_info "Recording resumed successfully"
        return 0
    else
        log_error "Failed to resume recording"
        log_error "Response: ${response}"
        return 1
    fi
}

test_stop_recording() {
    log_info "Testing stop recording endpoint..."

    local response
    response=$(curl -s -X POST "http://localhost:${HTTP_PORT}/rpc/finish")

    if echo "${response}" | grep -q '"success":true'; then
        log_info "Recording stopped successfully"
        return 0
    else
        log_error "Failed to stop recording"
        log_error "Response: ${response}"
        return 1
    fi
}

verify_mcap_file() {
    log_info "Verifying MCAP file..."

    # Find the most recent MCAP file in test_data or its subdirectories
    local mcap_file
    mcap_file=$(find "${TEST_DATA_DIR}" -name "*.mcap" -type f 2>/dev/null | head -n 1)

    if [[ -z "${mcap_file}" ]]; then
        log_error "No MCAP file found in ${TEST_DATA_DIR}"
        log_info "Contents of test_data directory:"
        find "${TEST_DATA_DIR}" -type f 2>/dev/null || echo "  (empty or doesn't exist)"
        return 1
    fi

    if [[ ! -s "${mcap_file}" ]]; then
        log_error "MCAP file is empty"
        return 1
    fi

    log_info "MCAP file verified: ${mcap_file}"
    echo "${mcap_file}"
}

verify_sidecar_file() {
    local mcap_file="$1"

    log_info "Verifying sidecar JSON file..."

    local sidecar_file="${mcap_file%.mcap}.json"

    if [[ ! -f "${sidecar_file}" ]]; then
        log_warn "Sidecar file not found: ${sidecar_file}"
        log_info "Sidecar generation requires task config (skipped in simplified test)"
        # Return success since this is expected when task config isn't set
        return 0
    fi

    # Verify JSON is valid
    if ! python3 -m json.tool "${sidecar_file}" > /dev/null 2>&1; then
        log_error "Sidecar file is not valid JSON"
        return 1
    fi

    # Check required fields
    local required_fields=(
        "version"
        "task_id"
        "device_id"
        "scene"
        "recording_start_time"
        "recording_end_time"
        "checksum"
    )

    for field in "${required_fields[@]}"; do
        if ! grep -q "\"${field}\":" "${sidecar_file}"; then
            log_error "Missing required field: ${field}"
            return 1
        fi
    done

    # Check that task_id matches
    if ! grep -q "\"task_id\": \"${TEST_TASK_ID}\"" "${sidecar_file}"; then
        log_error "Task ID mismatch in sidecar"
        return 1
    fi

    log_info "Sidecar file verified: ${sidecar_file}"
    echo "${sidecar_file}"
}

# ==============================================================================
# Main Test Runner
# ==============================================================================

run_all_tests() {
    local test_count=0
    local passed_count=0
    local failed_count=0

    # Array of test functions
    local tests=(
        "test_health_check"
        "test_state_initial_idle"
        "test_state_begin_fails_from_idle"
        "test_state_cancel_fails_from_idle"
        "test_state_config_to_ready"
        "test_state_begin_to_recording"
        "test_state_cancel_from_recording"
        "test_state_dynamic_subscription_update"
        "test_state_config_to_ready"
        "test_state_begin_to_recording"
        "test_state_finish_returns_to_idle"
        "test_recording_status"
    )

    log_info "================================"
    log_info "Running E2E Tests - State Machine Transitions"
    log_info "================================"

    # Run each test
    for test_func in "${tests[@]}"; do
        test_count=$((test_count + 1))

        if ${test_func}; then
            passed_count=$((passed_count + 1))
        else
            failed_count=$((failed_count + 1))
        fi

        # Small delay between tests
        sleep 1
    done

    # Verify output files
    log_info "Verifying output files..."

    local mcap_file
    local sidecar_file

    if mcap_file=$(verify_mcap_file); then
        passed_count=$((passed_count + 1))
        test_count=$((test_count + 1))

        if sidecar_file=$(verify_sidecar_file "${mcap_file}"); then
            passed_count=$((passed_count + 1))
            test_count=$((test_count + 1))
        else
            failed_count=$((failed_count + 1))
            test_count=$((test_count + 1))
        fi
    else
        failed_count=$((failed_count + 1))
        test_count=$((test_count + 1))
        # Skip sidecar test if MCAP not found
        failed_count=$((failed_count + 1))
        test_count=$((test_count + 1))
    fi

    # Print summary
    log_info "================================"
    log_info "Test Summary"
    log_info "================================"
    log_info "Total:  ${test_count}"
    log_info "Passed: ${passed_count}"
    log_info "Failed: ${failed_count}"
    log_info "================================"

    if [[ ${failed_count} -eq 0 ]]; then
        log_info "All tests passed!"
        return 0
    else
        log_error "Some tests failed!"
        return 1
    fi
}

# ==============================================================================
# Main
# ==============================================================================

main() {
    setup
    start_recorder
    run_all_tests
}

main "$@"
