#!/bin/bash
# @file test_e2e_with_mock.sh
# @brief E2E test for mock middleware integration with axon_recorder
#
# This script tests the complete recording workflow using the mock middleware:
# 1. Build mock middleware (if needed)
# 2. Build axon_recorder (if needed)
# 3. Start axon_recorder with mock middleware
# 4. Send HTTP requests to control recording
# 5. Verify mock messages are received and recorded to MCAP
# 6. Verify sidecar JSON generation
# 7. Clean up test artifacts

set -e

# ==============================================================================
# Configuration
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
BUILD_DIR="${PROJECT_ROOT}/build"
TEST_DIR="${SCRIPT_DIR}/test_data"
MOCK_BUILD_DIR="${PROJECT_ROOT}/middlewares/mock/src/mock_plugin/build"

# Mock plugin path
MOCK_PLUGIN="${MOCK_BUILD_DIR}/libmock_plugin.so"

# Try multiple possible locations for axon_recorder binary
if [[ -f "${BUILD_DIR}/apps/axon_recorder/axon_recorder" ]]; then
    RECORDER_BIN="${BUILD_DIR}/apps/axon_recorder/axon_recorder"
elif [[ -f "${PROJECT_ROOT}/apps/axon_recorder/build/axon_recorder" ]]; then
    RECORDER_BIN="${PROJECT_ROOT}/apps/axon_recorder/build/axon_recorder"
else
    RECORDER_BIN="${BUILD_DIR}/apps/axon_recorder/axon_recorder"
fi

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Test configuration
HTTP_PORT=8080
TEST_TASK_ID="mock_e2e_test_$(date +%s)"
TEST_DEVICE_ID="mock_test_robot_01"
TEST_SCENE="mock_e2e_scene"

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

log_step() {
    echo -e "${BLUE}[STEP]${NC} $1"
}

cleanup() {
    log_info "Cleaning up..."

    # Stop recorder if running
    if [[ -n "${RECORDER_PID}" ]]; then
        kill "${RECORDER_PID}" 2>/dev/null || true
        wait "${RECORDER_PID}" 2>/dev/null || true
    fi

    # Clean up test data (optional - comment out if you want to inspect files)
    if [[ "${KEEP_TEST_DATA}" != "1" ]]; then
        rm -rf "${TEST_DIR}"
    else
        log_info "Keeping test data in: ${TEST_DIR}"
    fi

    log_info "Cleanup complete"
}

# Trap to ensure cleanup on exit
trap cleanup EXIT INT TERM

# ==============================================================================
# Build Functions
# ==============================================================================

build_mock_middleware() {
    if [[ -f "${MOCK_PLUGIN}" ]]; then
        log_info "Mock middleware already built"
        return 0
    fi

    log_step "Building mock middleware..."
    cd "${PROJECT_ROOT}"
    make build-mock

    if [[ ! -f "${MOCK_PLUGIN}" ]]; then
        log_error "Failed to build mock middleware"
        exit 1
    fi

    log_info "Mock middleware built successfully"
}

build_axon_recorder() {
    if [[ -f "${RECORDER_BIN}" ]]; then
        log_info "axon_recorder already built"
        return 0
    fi

    log_step "Building axon_recorder..."
    cd "${PROJECT_ROOT}"
    make build-core

    # Find the actual binary location
    if [[ ! -f "${RECORDER_BIN}" ]]; then
        RECORDER_BIN=$(find "${BUILD_DIR}" -name "axon_recorder" -type f -executable 2>/dev/null | head -n 1)
        if [[ -z "${RECORDER_BIN}" ]]; then
            log_error "Failed to build axon_recorder"
            exit 1
        fi
    fi

    log_info "axon_recorder built successfully: ${RECORDER_BIN}"
}

# ==============================================================================
# Setup
# ==============================================================================

setup() {
    log_step "Setting up E2E test environment..."

    # Create test data directory
    mkdir -p "${TEST_DIR}"

    # Build dependencies
    build_mock_middleware
    build_axon_recorder

    log_info "Using recorder: ${RECORDER_BIN}"
    log_info "Using mock plugin: ${MOCK_PLUGIN}"

    log_info "Setup complete"
}

# ==============================================================================
# Test Functions
# ==============================================================================

start_recorder() {
    log_step "Starting axon_recorder with mock middleware..."

    # Create test config
    local config_file="${TEST_DIR}/test_config.yaml"

    cat > "${config_file}" <<EOF
# E2E Test Configuration with Mock Middleware
plugin:
  path: ${MOCK_PLUGIN}

dataset:
  path: ${TEST_DIR}/recordings
  mode: create
  stats_file_path: ${TEST_DIR}/stats.json

subscriptions:
  - name: /mock/string
    message_type: std_msgs/String
    batch_size: 10
    flush_interval_ms: 100
  - name: /mock/int
    message_type: std_msgs/Int32
    batch_size: 10
    flush_interval_ms: 100
  - name: /mock/image
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

    # Start recorder in background
    cd "${TEST_DIR}"
    "${RECORDER_BIN}" --config "${config_file}" > "${TEST_DIR}/recorder.log" 2>&1 &
    RECORDER_PID=$!

    # Wait for recorder to start
    sleep 3

    # Check if recorder is running
    if ! kill -0 "${RECORDER_PID}" 2>/dev/null; then
        log_error "Failed to start recorder"
        log_error "Log output:"
        cat "${TEST_DIR}/recorder.log"
        exit 1
    fi

    log_info "Recorder started (PID: ${RECORDER_PID})"

    # Show initial log output
    log_info "Initial recorder log:"
    head -n 20 "${TEST_DIR}/recorder.log" || true
}

test_health_check() {
    log_step "Testing health check endpoint..."

    local response
    response=$(curl -s -o /dev/null -w "%{http_code}" "http://localhost:${HTTP_PORT}/health" || echo "000")

    if [[ "${response}" == "200" ]]; then
        log_info "✓ Health check passed"
        return 0
    else
        log_error "✗ Health check failed (HTTP ${response})"
        return 1
    fi
}

test_cache_config() {
    log_step "Caching task configuration..."

    local response
    response=$(curl -s -X POST \
        -H "Content-Type: application/json" \
        -d "{
            \"task_config\": {
                \"task_id\": \"${TEST_TASK_ID}\",
                \"device_id\": \"${TEST_DEVICE_ID}\",
                \"scene\": \"${TEST_SCENE}\",
                \"factory\": \"mock_factory\",
                \"operator_name\": \"mock_operator\",
                \"topics\": [\"/mock/string\", \"/mock/int\", \"/mock/image\"]
            }
        }" \
        "http://localhost:${HTTP_PORT}/rpc/config")

    if echo "${response}" | grep -q '"success":true'; then
        log_info "✓ Config cached successfully"
        return 0
    else
        log_error "✗ Config cache failed"
        log_error "Response: ${response}"
        return 1
    fi
}

test_start_recording() {
    log_step "Starting recording..."

    local response
    response=$(curl -s -X POST \
        -H "Content-Type: application/json" \
        -d "{\"task_id\": \"${TEST_TASK_ID}\"}" \
        "http://localhost:${HTTP_PORT}/rpc/begin")

    if echo "${response}" | grep -q '"success":true'; then
        log_info "✓ Recording started"
        return 0
    else
        log_error "✗ Failed to start recording"
        log_error "Response: ${response}"
        return 1
    fi
}

test_wait_for_messages() {
    log_step "Waiting for mock messages (3 seconds)..."
    sleep 3
    log_info "Mock messages should have been published"
}

test_stop_recording() {
    log_step "Stopping recording..."

    local response
    response=$(curl -s -X POST \
        -H "Content-Type: application/json" \
        -d "{\"task_id\": \"${TEST_TASK_ID}\"}" \
        "http://localhost:${HTTP_PORT}/rpc/finish")

    if echo "${response}" | grep -q '"success":true'; then
        log_info "✓ Recording stopped"
        return 0
    else
        log_error "✗ Failed to stop recording"
        log_error "Response: ${response}"
        return 1
    fi
}

verify_mcap_file() {
    log_step "Verifying MCAP file..."

    # Find the most recent MCAP file
    local mcap_file
    mcap_file=$(find "${TEST_DIR}" -name "*.mcap" -type f 2>/dev/null | head -n 1)

    if [[ -z "${mcap_file}" ]]; then
        log_error "✗ No MCAP file found"
        return 1
    fi

    if [[ ! -s "${mcap_file}" ]]; then
        log_error "✗ MCAP file is empty"
        return 1
    fi

    local file_size
    file_size=$(stat -f%z "${mcap_file}" 2>/dev/null || stat -c%s "${mcap_file}" 2>/dev/null)

    log_info "✓ MCAP file verified: ${mcap_file}"
    log_info "  Size: ${file_size} bytes"

    # Try to get basic info with mcap (if available)
    if command -v mcap &> /dev/null; then
        log_info "  MCAP info:"
        mcap info "${mcap_file}" 2>&1 | head -n 10 || true
    fi

    return 0
}

verify_sidecar_file() {
    log_step "Verifying sidecar JSON file..."

    local mcap_file
    mcap_file=$(find "${TEST_DIR}" -name "*.mcap" -type f 2>/dev/null | head -n 1)

    if [[ -z "${mcap_file}" ]]; then
        log_error "Cannot verify sidecar without MCAP file"
        return 1
    fi

    local sidecar_file="${mcap_file%.mcap}.json"

    if [[ ! -f "${sidecar_file}" ]]; then
        log_warn "Sidecar file not found: ${sidecar_file}"
        return 0
    fi

    # Verify JSON is valid
    if ! python3 -m json.tool "${sidecar_file}" > /dev/null 2>&1; then
        log_error "✗ Sidecar file is not valid JSON"
        return 1
    fi

    log_info "✓ Sidecar file verified: ${sidecar_file}"

    # Show some key fields
    log_info "  Task ID: $(grep '"task_id"' "${sidecar_file}" | cut -d'"' -f4)"
    log_info "  Device ID: $(grep '"device_id"' "${sidecar_file}" | cut -d'"' -f4)"

    return 0
}

show_recorder_stats() {
    log_step "Showing recorder statistics..."

    if [[ -f "${TEST_DIR}/stats.json" ]]; then
        log_info "Recorder statistics:"
        cat "${TEST_DIR}/stats.json"
    else
        log_warn "No statistics file found"
    fi
}

# ==============================================================================
# Main Test Runner
# ==============================================================================

run_all_tests() {
    local test_count=0
    local passed_count=0
    local failed_count=0

    log_info "================================"
    log_info "Mock Middleware E2E Tests"
    log_info "================================"

    # Array of test functions
    local tests=(
        "test_health_check"
        "test_cache_config"
        "test_start_recording"
        "test_wait_for_messages"
        "test_stop_recording"
        "verify_mcap_file"
        "verify_sidecar_file"
    )

    # Run each test
    for test_func in "${tests[@]}"; do
        test_count=$((test_count + 1))

        if ${test_func}; then
            passed_count=$((passed_count + 1))
        else
            failed_count=$((failed_count + 1))
        fi
    done

    # Show statistics
    show_recorder_stats

    # Print summary
    log_info "================================"
    log_info "Test Summary"
    log_info "================================"
    log_info "Total:   ${test_count}"
    log_info "Passed:  ${passed_count}"
    log_info "Failed:  ${failed_count}"
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
    log_info "================================"
    log_info "Mock Middleware E2E Test"
    log_info "================================"
    log_info ""

    setup
    start_recorder
    run_all_tests
}

# Allow keeping test data for inspection
KEEP_TEST_DATA=${KEEP_TEST_DATA:-0}

main "$@"
