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
#   --clean                 Clean build directory before building
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
CLEAN_BUILD=false
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
        --clean)
            CLEAN_BUILD=true
            shift
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
    # E2E tests are in apps/axon_recorder/test/e2e
    SOURCE_PATH="${WORKSPACE_ROOT}/apps/axon_recorder/test/e2e"
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
# Part 1: Build ROS Plugin and Axon Recorder
# =============================================================================

echo ""
echo "============================================"
if [ "$ENABLE_COVERAGE" = true ]; then
    echo "Part 1: Building with coverage instrumentation..."
else
    echo "Part 1: Building ROS plugin and axon_recorder..."
fi
echo "============================================"

# Clean build directory if requested or if coverage is enabled
# (coverage requires clean build to ensure flags are applied)
if [ "$CLEAN_BUILD" = true ] || [ "$ENABLE_COVERAGE" = true ]; then
    echo "Cleaning build directory..."
    rm -rf "${WORKSPACE_ROOT}/build"
fi

# Build ROS plugin
if [ "$ENABLE_COVERAGE" = true ]; then
    ros_build_package "${WORKSPACE_ROOT}" "true" "true" "Debug" "-DAXON_BUILD_MOCK_PLUGIN=ON" || {
        ros_build_error "Failed to build ROS plugin with coverage"
    }
    # Note: We don't use WORKSPACE_BUILD_DIR for E2E coverage
    # E2E tests collect coverage from axon_recorder and core libraries, not the plugin
else
    ros_build_package "${WORKSPACE_ROOT}" "true" "false" "Release" "-DAXON_BUILD_MOCK_PLUGIN=ON" || {
        ros_build_error "Failed to build ROS plugin"
    }
fi

# Re-source workspace after build
ros_workspace_source_workspace "${WORKSPACE_ROOT}" || {
    ros_workspace_error "Failed to source workspace after build"
}

# Verify build artifacts exist
echo ""
echo "Verifying build artifacts..."

# Check axon_recorder binary
RECORDER_BIN="${WORKSPACE_ROOT}/build/axon_recorder/axon_recorder"
if [ ! -f "$RECORDER_BIN" ]; then
    echo "ERROR: axon_recorder binary not found at ${RECORDER_BIN}"
    exit 1
fi
echo "✓ axon_recorder binary found at ${RECORDER_BIN}"

# Check mock plugin
MOCK_PLUGIN="${WORKSPACE_ROOT}/build/middlewares/axon_mock.so"
if [ ! -f "$MOCK_PLUGIN" ]; then
    echo "ERROR: Mock plugin not found at ${MOCK_PLUGIN}"
    echo "This should have been built with AXON_BUILD_MOCK_PLUGIN=ON"
    exit 1
fi
echo "✓ Mock plugin found at ${MOCK_PLUGIN}"

# =============================================================================
# Part 2: Run E2E Tests
# =============================================================================

echo ""
echo "============================================"
echo "Part 2: Running E2E tests..."
echo "============================================"

# Verify E2E test script exists
TEST_SCRIPT="${SOURCE_PATH}/run_e2e_tests.sh"
if [ ! -f "$TEST_SCRIPT" ]; then
    echo "ERROR: E2E test script not found at ${TEST_SCRIPT}"
    exit 1
fi

chmod +x "$TEST_SCRIPT"

# Run the E2E tests (they handle starting/stopping the recorder)
TEST_RESULT=0
if ! "$TEST_SCRIPT"; then
    TEST_RESULT=1
fi

# =============================================================================
# Part 3: Generate Coverage Report (if enabled and tests passed)
# =============================================================================

# Generate coverage report if enabled and tests passed
if [ "$ENABLE_COVERAGE" = true ] && [ $TEST_RESULT -eq 0 ]; then
    echo ""
    echo "============================================"
    echo "Part 3: Generating coverage report..."
    echo "============================================"

    # E2E tests collect coverage from axon_recorder and core libraries
    # NOT from the ROS plugin (plugin is used as a library but not tested in E2E)

    # Check lcov availability
    if ! command -v lcov &> /dev/null; then
        ros_coverage_log "WARN" "lcov not available, skipping coverage generation"
    else
        # Collect coverage data from multiple directories
        # Using a temporary file to collect from each directory
        raw_file="${COVERAGE_OUTPUT_DIR}/coverage_raw.info"
        filtered_file="${COVERAGE_OUTPUT_DIR}/coverage.info"

        # Find all gcda files in the target directories
        gcda_files=()
        while IFS= read -r -d '' file; do
            gcda_files+=("$file")
        done < <(find "${WORKSPACE_ROOT}/build/axon_mcap" \
                      "${WORKSPACE_ROOT}/build/axon_logging" \
                      "${WORKSPACE_ROOT}/build/apps/axon_recorder" \
                      -name "*.gcda" -type f -print0 2>/dev/null)

        if [ ${#gcda_files[@]} -eq 0 ]; then
            ros_coverage_log "WARN" "No .gcda files found in E2E test directories"
        else
            ros_coverage_log "INFO" "Found ${#gcda_files[@]} .gcda files"

            # Get the directory containing the gcda files for lcov
            gcda_dirs=()
            for file in "${gcda_files[@]}"; do
                dir=$(dirname "$file")
                # Add unique directories
                if [[ ! " ${gcda_dirs[*]} " =~ " ${dir} " ]]; then
                    gcda_dirs+=("$dir")
                fi
            done

            echo "Collecting coverage from directories:"
            printf '  - %s\n' "${gcda_dirs[@]}"

            # Capture coverage (using first directory as base)
            base_dir="${gcda_dirs[0]}"
            lcov_major=$(lcov --version 2>&1 | grep -oE '[0-9]+\.[0-9]+(\.[0-9]+)?' | head -1 | cut -d. -f1)

            lcov --capture \
                --directory "$base_dir" \
                --output-file "$raw_file" \
                --rc branch_coverage=1 \
                2>/dev/null || {
                ros_coverage_log "WARN" "lcov capture failed, trying alternative method..."
                # Fallback: collect from each directory separately
                for dir in "${gcda_dirs[@]}"; do
                    lcov --capture \
                        --directory "$dir" \
                        --output-file "$raw_file" \
                        --rc branch_coverage=1 2>/dev/null || true
                done
            }

            # Filter coverage data to remove external dependencies
            lcov --remove "$raw_file" \
                '/usr/*' \
                '/opt/*' \
                '*/_deps/*' \
                '*/generated/*' \
                '*/googletest/*' \
                '*/gtest/*' \
                '*/mcap-*/*' \
                '*/mcap/include/*' \
                '*/minio-cpp/*' \
                '*/test/*' \
                '*/rosidl_typesupport_cpp/*' \
                '*/rosidl_typesupport_introspection_cpp/*' \
                '*/rosidl_generator_cpp/*' \
                --output-file "$filtered_file" \
                2>/dev/null || cp "$raw_file" "$filtered_file"

            # Display final summary
            echo ""
            echo "============================================"
            echo "E2E Test Coverage Summary"
            echo "============================================"
            lcov --list "$filtered_file" || true

            echo ""
            echo "============================================"
            echo "E2E test coverage collection complete!"
            echo "============================================"
            echo "Coverage data: ${filtered_file}"
        fi
    fi
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
