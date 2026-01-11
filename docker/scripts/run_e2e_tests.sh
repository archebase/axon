#!/bin/bash
# =============================================================================
# E2E Test Runner (Unified)
# =============================================================================
# This script builds the axon_recorder package, starts the axon_recorder_node,
# runs E2E tests via ROS service calls, and optionally collects coverage data.
# Used by both CI and local Docker testing.
#
# Usage:
#   ./run_e2e_tests.sh [OPTIONS] [source_path]
#
# Options:
#   --coverage              Build with coverage instrumentation and generate coverage report
#   --coverage-output DIR   Coverage output directory (default: /coverage_output)
#   --source-path PATH      Path to test source directory (default: auto-detect)
#
# Arguments:
#   source_path             Optional path to the source directory (legacy positional arg)
#                           Default: auto-detect from script location
#
# Environment:
#   ROS_VERSION             ROS version "1" or "2" (auto-detected from ROS_DISTRO if not set)
#   ROS_DISTRO              ROS distribution (e.g., "noetic", "humble")
#   COVERAGE_OUTPUT         Alternative way to specify coverage output directory
#
# Examples:
#   # Build and run tests (default, no coverage)
#   ./run_e2e_tests.sh
#
#   # Build with coverage, run tests, generate coverage report
#   ./run_e2e_tests.sh --coverage --coverage-output /coverage_output
# =============================================================================

set -eo pipefail

# =============================================================================
# Parse Arguments
# =============================================================================

ENABLE_COVERAGE=false
COVERAGE_OUTPUT_DIR=""
SOURCE_PATH=""

while [[ $# -gt 0 ]]; do
    case $1 in
        --coverage)
            ENABLE_COVERAGE=true
            shift
            ;;
        --coverage-output)
            COVERAGE_OUTPUT_DIR="$2"
            shift 2
            ;;
        --source-path)
            SOURCE_PATH="$2"
            shift 2
            ;;
        -*)
            echo "ERROR: Unknown option: $1" >&2
            exit 1
            ;;
        *)
            # Legacy positional argument for source_path
            SOURCE_PATH="$1"
            shift
            ;;
    esac
done

# =============================================================================
# Source Shared Libraries
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIB_DIR="${SCRIPT_DIR}/lib"

# Source libraries (with error handling for missing files)
# Note: We can't use library error functions here since libraries aren't loaded yet
if [ ! -f "${LIB_DIR}/ros_build_lib.sh" ]; then
    echo "ERROR: ros_build_lib.sh not found at ${LIB_DIR}/ros_build_lib.sh" >&2
    exit 1
fi
source "${LIB_DIR}/ros_build_lib.sh"

if [ ! -f "${LIB_DIR}/ros_workspace_lib.sh" ]; then
    echo "ERROR: ros_workspace_lib.sh not found at ${LIB_DIR}/ros_workspace_lib.sh" >&2
    exit 1
fi
source "${LIB_DIR}/ros_workspace_lib.sh"

if [ ! -f "${LIB_DIR}/ros_diagnostics_lib.sh" ]; then
    echo "ERROR: ros_diagnostics_lib.sh not found at ${LIB_DIR}/ros_diagnostics_lib.sh" >&2
    exit 1
fi
source "${LIB_DIR}/ros_diagnostics_lib.sh"

# Source coverage library only if coverage is enabled
if [ "$ENABLE_COVERAGE" = true ]; then
    if [ ! -f "${LIB_DIR}/ros_coverage_lib.sh" ]; then
        echo "ERROR: ros_coverage_lib.sh not found at ${LIB_DIR}/ros_coverage_lib.sh" >&2
        exit 1
    fi
    source "${LIB_DIR}/ros_coverage_lib.sh"
fi

# =============================================================================
# Set Defaults and Auto-detect ROS Version
# =============================================================================

# Set defaults
ROS_DISTRO="${ROS_DISTRO:-humble}"
WORKSPACE_ROOT="/workspace/axon"

# Set coverage output directory if coverage is enabled
if [ "$ENABLE_COVERAGE" = true ]; then
    COVERAGE_OUTPUT_DIR="${COVERAGE_OUTPUT_DIR:-${COVERAGE_OUTPUT:-/coverage_output}}"
fi

# Auto-detect ROS version from ROS_DISTRO if ROS_VERSION not set
if [ -z "$ROS_VERSION" ]; then
    if [ -z "$ROS_DISTRO" ]; then
        echo "ERROR: Neither ROS_VERSION nor ROS_DISTRO is set" >&2
        exit 1
    fi
    ROS_VERSION=$(ros_workspace_detect_ros_version)
    if [ -z "$ROS_VERSION" ]; then
        echo "ERROR: Failed to detect ROS_VERSION from ROS_DISTRO=$ROS_DISTRO" >&2
        exit 1
    fi
    echo "Auto-detected ROS_VERSION=$ROS_VERSION from ROS_DISTRO=$ROS_DISTRO"
fi

# Determine source path for test script
if [ -z "$SOURCE_PATH" ]; then
    # Try to find test directory based on ROS version
    if [ "$ROS_VERSION" = "1" ]; then
        TEST_DIR="${WORKSPACE_ROOT}/middlewares/ros1/src/axon_recorder/test/e2e"
    else
        TEST_DIR="${WORKSPACE_ROOT}/middlewares/ros2/src/axon_recorder/test/e2e"
    fi

    if [ -d "$TEST_DIR" ]; then
        SOURCE_PATH="$TEST_DIR"
    elif [ -d "${WORKSPACE_ROOT}/middlewares/src/axon_recorder/test/e2e" ]; then
        # Fallback to old directory structure (for backward compatibility)
        SOURCE_PATH="${WORKSPACE_ROOT}/middlewares/src/axon_recorder/test/e2e"
    else
        # Fallback to script directory (for backward compatibility)
        SOURCE_PATH="${SCRIPT_DIR}"
    fi
fi

# =============================================================================
# Display Header
# =============================================================================

echo "============================================"
if [ "$ENABLE_COVERAGE" = true ]; then
    echo "Running E2E Tests with Coverage (ROS $ROS_VERSION)"
    echo "Coverage output: ${COVERAGE_OUTPUT_DIR}"
else
    echo "Running E2E Tests (ROS $ROS_VERSION)"
fi
echo "============================================"

# =============================================================================
# Source ROS Base Environment
# =============================================================================

ros_workspace_source_base || {
    ros_workspace_error "Failed to source ROS base environment"
}

cd "${WORKSPACE_ROOT}"

# Create coverage output directory if coverage is enabled (done early for clarity)
[ "$ENABLE_COVERAGE" = true ] && mkdir -p "${COVERAGE_OUTPUT_DIR}"

# =============================================================================
# Part 1: Build Package
# =============================================================================

echo ""
echo "============================================"
if [ "$ENABLE_COVERAGE" = true ]; then
    echo "Part 1: Building with coverage instrumentation..."
else
    echo "Part 1: Building package..."
fi
echo "============================================"

# Build package (with or without coverage)
if [ "$ENABLE_COVERAGE" = true ]; then
    ros_build_package "${WORKSPACE_ROOT}" "true" "true" "Debug" || {
        ros_build_error "Failed to build package with coverage"
    }
    COVERAGE_DIR="${WORKSPACE_BUILD_DIR}"
else
    ros_build_package "${WORKSPACE_ROOT}" "true" "false" "Release" || {
        ros_build_error "Failed to build package"
    }
fi

# Re-source workspace after build
ros_workspace_source_workspace "${WORKSPACE_ROOT}" || {
    ros_workspace_error "Failed to source workspace after build"
}

# =============================================================================
# Part 2: Verify Package and Find Test Script
# =============================================================================

echo ""
echo "============================================"
echo "Part 2: Verifying package and test setup..."
echo "============================================"

# Verify package is available
ros_workspace_verify_package || {
    echo "WARNING: Package verification failed, but continuing..."
}

# Find test script
TEST_SCRIPT="${SOURCE_PATH}/test_ros_services.sh"
if [ ! -f "$TEST_SCRIPT" ]; then
    # Try alternative locations based on ROS version
    if [ "$ROS_VERSION" = "1" ] && [ -f "${WORKSPACE_ROOT}/middlewares/ros1/src/axon_recorder/test/e2e/test_ros_services.sh" ]; then
        TEST_SCRIPT="${WORKSPACE_ROOT}/middlewares/ros1/src/axon_recorder/test/e2e/test_ros_services.sh"
    elif [ "$ROS_VERSION" = "2" ] && [ -f "${WORKSPACE_ROOT}/middlewares/ros2/src/axon_recorder/test/e2e/test_ros_services.sh" ]; then
        TEST_SCRIPT="${WORKSPACE_ROOT}/middlewares/ros2/src/axon_recorder/test/e2e/test_ros_services.sh"
    elif [ -f "${WORKSPACE_ROOT}/middlewares/src/axon_recorder/test/e2e/test_ros_services.sh" ]; then
        # Fallback to old directory structure
        TEST_SCRIPT="${WORKSPACE_ROOT}/middlewares/src/axon_recorder/test/e2e/test_ros_services.sh"
    else
        echo "ERROR: test_ros_services.sh not found"
        echo "Searched: ${SOURCE_PATH}/test_ros_services.sh"
        echo "Searched: ${WORKSPACE_ROOT}/middlewares/ros1/src/axon_recorder/test/e2e/test_ros_services.sh (ROS1)"
        echo "Searched: ${WORKSPACE_ROOT}/middlewares/ros2/src/axon_recorder/test/e2e/test_ros_services.sh (ROS2)"
        echo "Searched: ${WORKSPACE_ROOT}/middlewares/src/axon_recorder/test/e2e/test_ros_services.sh (legacy)"
        exit 1
    fi
fi

chmod +x "$TEST_SCRIPT"

# =============================================================================
# Part 3: Start ROS Node
# =============================================================================

echo ""
echo "============================================"
echo "Part 3: Starting axon_recorder_node..."
echo "============================================"

# Start node based on ROS version
if [ "$ROS_VERSION" = "1" ]; then
    # ROS 1 - Need roscore
    roscore &
    ROSCORE_PID=$!
    sleep 2
    
    # Use __name:= to set node name so services appear at /axon_recorder/...
    rosrun axon_recorder axon_recorder_node __name:=axon_recorder &
    NODE_PID=$!
else
    # ROS 2 - Set node name to match ROS 1 (so services appear at /axon_recorder/...)
    ros2 run axon_recorder axon_recorder_node --ros-args -r __node:=axon_recorder &
    NODE_PID=$!
fi

sleep 3
echo "Node started (PID: $NODE_PID)"

# Run diagnostics if library is available (useful for both ROS1 and ROS2 issues)
echo ""
echo "Running diagnostics before tests..."
ros_diagnostics_full "axon_recorder" "axon_recorder/recording_control" || {
    echo "WARNING: Some diagnostic checks failed, but continuing with tests..."
}
echo ""

# =============================================================================
# Part 4: Run E2E Tests
# =============================================================================

echo ""
echo "============================================"
echo "Part 4: Running E2E tests..."
echo "============================================"

TEST_RESULT=0
if ! "$TEST_SCRIPT"; then
    TEST_RESULT=1
fi

# =============================================================================
# Part 5: Cleanup
# =============================================================================

echo ""
echo "Cleaning up..."
kill $NODE_PID 2>/dev/null || true

if [ "$ROS_VERSION" = "1" ]; then
    kill $ROSCORE_PID 2>/dev/null || true
    killall roscore 2>/dev/null || true
fi

# Wait for processes to terminate
sleep 1

# =============================================================================
# Part 6: Generate Coverage Report (if enabled and tests passed)
# =============================================================================

# Generate coverage report if enabled and tests passed
if [ "$ENABLE_COVERAGE" = true ] && [ $TEST_RESULT -eq 0 ]; then
    echo ""
    echo "============================================"
    echo "Part 6: Generating coverage report..."
    echo "============================================"
    
    # Generate coverage report using library function (COVERAGE_DIR is always set when coverage enabled)
    ros_coverage_generate "${COVERAGE_DIR}" "${COVERAGE_OUTPUT_DIR}" || {
        ros_coverage_error "Failed to generate coverage report"
    }
    
    # Display final summary
    echo ""
    echo "============================================"
    echo "E2E Test Coverage Summary"
    echo "============================================"
    lcov --list "${COVERAGE_OUTPUT_DIR}/coverage.info" || true
    
    echo ""
    echo "============================================"
    echo "E2E test coverage collection complete!"
    echo "============================================"
    echo "Coverage data: ${COVERAGE_OUTPUT_DIR}/coverage.info"
fi

# =============================================================================
# Final Status
# =============================================================================

echo ""
echo "============================================"
if [ $TEST_RESULT -eq 0 ]; then
    echo "✓ E2E tests passed"
else
    echo "✗ E2E tests failed"
fi
echo "============================================"

exit $TEST_RESULT
