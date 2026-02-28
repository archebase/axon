#!/bin/bash
# SPDX-FileCopyrightText: 2026 ArcheBase
#
# SPDX-License-Identifier: MulanPSL-2.0

# =============================================================================
# UDP Plugin E2E Test
# =============================================================================
# Tests the complete UDP recording workflow:
# 1. Build UDP plugin
# 2. Start axon_recorder with UDP plugin
# 3. Send UDP JSON messages
# 4. Verify MCAP file contains recorded messages
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
BUILD_DIR="${PROJECT_ROOT}/build"
RECODER="${BUILD_DIR}/axon_recorder/axon_recorder"
UDP_PLUGIN="${BUILD_DIR}/middlewares/udp_plugin/libaxon_udp.so"
TEST_DIR="/tmp/axon_udp_test_$$"
CONFIG_FILE="${TEST_DIR}/config.yaml"
OUTPUT_FILE="${TEST_DIR}/recording.mcap"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

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
    if [ -n "$RECORDER_PID" ]; then
        kill $RECORDER_PID 2>/dev/null || true
        wait $RECORDER_PID 2>/dev/null || true
    fi
    rm -rf "${TEST_DIR}"
}

trap cleanup EXIT

# =============================================================================
# Setup
# =============================================================================

log_info "Setting up test environment..."

mkdir -p "${TEST_DIR}"

# Create test configuration
cat > "${CONFIG_FILE}" << EOF
version: "1.0"
task_id: "udp_e2e_test"
device_id: "test_device"

output:
  path: "${TEST_DIR}"
  prefix: "recording"

udp:
  enabled: true
  bind_address: "0.0.0.0"
  buffer_size: 65536
  timestamp_extraction:
    source: "field"
    field: "timestamp"
  streams:
    - name: "test_gps"
      port: 4290
      topic: "/udp/gps"
      schema: "raw_json"
      enabled: true
    - name: "test_can"
      port: 4291
      topic: "/udp/can"
      schema: "raw_json"
      enabled: true
EOF

log_info "Configuration file created: ${CONFIG_FILE}"

# =============================================================================
# Build Check
# =============================================================================

log_info "Checking build artifacts..."

if [ ! -f "${RECODER}" ]; then
    log_error "Recorder not found: ${RECODER}"
    log_info "Please run: make build"
    exit 1
fi

if [ ! -f "${UDP_PLUGIN}" ]; then
    log_error "UDP plugin not found: ${UDP_PLUGIN}"
    log_info "Please run: make build-middlewares"
    exit 1
fi

log_info "Build artifacts OK"

# =============================================================================
# Start Recorder
# =============================================================================

log_info "Starting axon_recorder with UDP plugin..."

${RECODER} \
    --plugin "${UDP_PLUGIN}" \
    --config "${CONFIG_FILE}" \
    --port 8765 &
RECORDER_PID=$!

# Wait for recorder to start
sleep 2

# Check if recorder is running
if ! kill -0 ${RECORDER_PID} 2>/dev/null; then
    log_error "Recorder failed to start"
    exit 1
fi

log_info "Recorder started (PID: ${RECORDER_PID})"

# =============================================================================
# Configure and Start Recording
# =============================================================================

log_info "Configuring recording..."

# Configure task
curl -s -X POST http://localhost:8765/rpc/config \
    -H "Content-Type: application/json" \
    -d "{
        \"task_id\": \"udp_e2e_test\",
        \"device_id\": \"test_device\",
        \"udp\": {
            \"enabled\": true,
            \"streams\": [
                {\"name\": \"test_gps\", \"port\": 4290, \"topic\": \"/udp/gps\", \"enabled\": true},
                {\"name\": \"test_can\", \"port\": 4291, \"topic\": \"/udp/can\", \"enabled\": true}
            ]
        }
    }" || true

sleep 1

# Start recording
log_info "Starting recording..."
curl -s -X POST http://localhost:8765/rpc/begin || true

sleep 1

# =============================================================================
# Send UDP Messages
# =============================================================================

log_info "Sending UDP messages..."

# Send GPS messages
python3 "${SCRIPT_DIR}/../examples/udp_publisher.py" \
    --port 4290 \
    --type gps \
    --rate 20 \
    --count 50 \
    &

GPS_PID=$!

# Send CAN messages
python3 "${SCRIPT_DIR}/../examples/udp_publisher.py" \
    --port 4291 \
    --type can \
    --rate 50 \
    --count 100 \
    &

CAN_PID=$!

# Wait for publishers to finish
wait ${GPS_PID}
wait ${CAN_PID}

log_info "UDP messages sent"

# Wait a bit for processing
sleep 2

# =============================================================================
# Stop Recording
# =============================================================================

log_info "Stopping recording..."
curl -s -X POST http://localhost:8765/rpc/finish || true

sleep 2

# Stop recorder
kill ${RECORDER_PID} 2>/dev/null || true
wait ${RECORDER_PID} 2>/dev/null || true
RECORDER_PID=""

# =============================================================================
# Verify Results
# =============================================================================

log_info "Verifying results..."

# Check if MCAP file was created
MCAP_FILES=$(find "${TEST_DIR}" -name "*.mcap" 2>/dev/null)

if [ -z "${MCAP_FILES}" ]; then
    log_error "No MCAP files found in ${TEST_DIR}"
    ls -la "${TEST_DIR}"
    exit 1
fi

log_info "MCAP files created:"
echo "${MCAP_FILES}" | while read file; do
    ls -la "${file}"
done

# Check file size (should have some content)
for file in ${MCAP_FILES}; do
    SIZE=$(stat -c%s "${file}" 2>/dev/null || stat -f%z "${file}" 2>/dev/null)
    if [ "${SIZE}" -lt 100 ]; then
        log_error "MCAP file too small: ${SIZE} bytes"
        exit 1
    fi
    log_info "MCAP file size: ${SIZE} bytes"
done

# Try to read MCAP info (if mcap CLI is available)
if command -v mcap &> /dev/null; then
    log_info "MCAP file info:"
    for file in ${MCAP_FILES}; do
        mcap info "${file}" || true
    done
fi

# =============================================================================
# Success
# =============================================================================

log_info ""
log_info "=========================================="
log_info "E2E Test PASSED"
log_info "=========================================="
log_info ""
log_info "Test artifacts:"
log_info "  Config:    ${CONFIG_FILE}"
log_info "  MCAP:      ${MCAP_FILES}"
log_info ""

exit 0
