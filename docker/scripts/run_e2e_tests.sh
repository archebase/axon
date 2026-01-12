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
    echo "Cleaning build directories..."
    if [ "$ROS_VERSION" = "1" ]; then
        rm -rf "${WORKSPACE_ROOT}/middlewares/ros1/build"
        rm -rf "${WORKSPACE_ROOT}/middlewares/ros1/devel"
        rm -rf "${WORKSPACE_ROOT}/middlewares/ros1/logs"
        rm -rf "${WORKSPACE_ROOT}/build"
    else
        rm -rf "${WORKSPACE_ROOT}/middlewares/ros2/build"
        rm -rf "${WORKSPACE_ROOT}/middlewares/ros2/install"
        rm -rf "${WORKSPACE_ROOT}/middlewares/ros2/log"
        rm -rf "${WORKSPACE_ROOT}/build"
    fi
fi

# Build ROS plugin
if [ "$ENABLE_COVERAGE" = true ]; then
    ros_build_package "${WORKSPACE_ROOT}" "true" "true" "Debug" || {
        ros_build_error "Failed to build ROS plugin with coverage"
    }
    COVERAGE_DIR="${WORKSPACE_BUILD_DIR}"
else
    ros_build_package "${WORKSPACE_ROOT}" "true" "false" "Release" || {
        ros_build_error "Failed to build ROS plugin"
    }
fi

# Re-source workspace after build
ros_workspace_source_workspace "${WORKSPACE_ROOT}" || {
    ros_workspace_error "Failed to source workspace after build"
}

# Build axon_recorder and mock plugin
echo ""
echo "Building axon_recorder..."
mkdir -p "${WORKSPACE_ROOT}/build"
cd "${WORKSPACE_ROOT}/build"

# Build core libraries first (required by axon_recorder)
# Note: Only axon_mcap and axon_logging are needed for E2E tests
# axon_uploader requires AWS SDK and is not needed for HTTP API tests
echo "Building core libraries..."
for lib in axon_mcap axon_logging; do
    echo "Building $lib..."
    mkdir -p "$lib"
    cd "$lib"
    if [ "$ENABLE_COVERAGE" = true ]; then
        cmake ../../core/$lib \
            -DCMAKE_BUILD_TYPE=Debug \
            -DAXON_ENABLE_COVERAGE=ON \
            -DAXON_BUILD_TESTS=OFF \
            -DAXON_REPO_ROOT="${WORKSPACE_ROOT}" || {
            echo "ERROR: Failed to configure $lib with coverage"
            exit 1
        }
    else
        cmake ../../core/$lib \
            -DCMAKE_BUILD_TYPE=Release \
            -DAXON_BUILD_TESTS=OFF \
            -DAXON_REPO_ROOT="${WORKSPACE_ROOT}" || {
            echo "ERROR: Failed to configure $lib"
            exit 1
        }
    fi
    make -j$(nproc) || {
        echo "ERROR: Failed to build $lib"
        exit 1
    }
    cd "${WORKSPACE_ROOT}/build"
done

# Configure and build axon_recorder
# Enable AXON_BUILD_WITH_CORE to link against the core libraries we just built
if [ "$ENABLE_COVERAGE" = true ]; then
    cmake ../apps/axon_recorder \
        -DCMAKE_BUILD_TYPE=Debug \
        -DAXON_ENABLE_COVERAGE=ON \
        -DAXON_BUILD_MOCK_PLUGIN=ON \
        -DAXON_BUILD_WITH_CORE=ON || {
        echo "ERROR: Failed to configure axon_recorder with coverage"
        exit 1
    }
else
    cmake ../apps/axon_recorder \
        -DCMAKE_BUILD_TYPE=Release \
        -DAXON_BUILD_MOCK_PLUGIN=ON \
        -DAXON_BUILD_WITH_CORE=ON || {
        echo "ERROR: Failed to configure axon_recorder"
        exit 1
    }
fi

make -j$(nproc) || {
    echo "ERROR: Failed to build axon_recorder"
    exit 1
}

# Ensure correct binary structure for E2E tests
# The E2E test script expects: ${BUILD_DIR}/apps/axon_recorder/axon_recorder
# Or it will search for: ${BUILD_DIR}/axon_recorder
if [ ! -f "${WORKSPACE_ROOT}/build/axon_recorder" ]; then
    # Copy the binary to the expected location
    cp "${WORKSPACE_ROOT}/build/apps/axon_recorder/axon_recorder" "${WORKSPACE_ROOT}/build/axon_recorder"
fi

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
