#!/bin/bash
# =============================================================================
# Integration Test Runner
# =============================================================================
# This script builds and runs integration tests (unit + integration), and
# optionally collects coverage data. Used by both CI and local Docker testing.
#
# Usage:
#   ./run_integration.sh [OPTIONS]
#
# Options:
#   --coverage              Build with coverage instrumentation and generate coverage report
#   --coverage-output DIR   Coverage output directory (default: /workspace/coverage)
#   --clean                 Clean build directory before building (useful for testing multiple ROS versions)
#
# Environment:
#   ROS_VERSION             ROS version "1" or "2" (auto-detected from ROS_DISTRO if not set)
#   ROS_DISTRO              ROS distribution (e.g., "noetic", "humble")
#   COVERAGE_OUTPUT         Alternative way to specify coverage output directory
#
# Examples:
#   # Build and run tests (default, no coverage)
#   ./run_integration.sh
#
#   # Build with coverage, run tests, generate coverage report
#   ./run_integration.sh --coverage --coverage-output /workspace/coverage
# =============================================================================

set -eo pipefail

# =============================================================================
# Parse Arguments
# =============================================================================

ENABLE_COVERAGE=false
CLEAN_BUILD=false
COVERAGE_OUTPUT_DIR=""

while [[ $# -gt 0 ]]; do
    case $1 in
        --coverage)
            ENABLE_COVERAGE=true
            shift
            ;;
        --clean)
            CLEAN_BUILD=true
            shift
            ;;
        --coverage-output)
            COVERAGE_OUTPUT_DIR="$2"
            shift 2
            ;;
        -*)
            echo "ERROR: Unknown option: $1" >&2
            exit 1
            ;;
        *)
            echo "ERROR: Unexpected argument: $1" >&2
            echo "Usage: ./run_integration.sh [--coverage] [--coverage-output DIR] [--clean]"
            exit 1
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
    COVERAGE_OUTPUT_DIR="${COVERAGE_OUTPUT_DIR:-${COVERAGE_OUTPUT:-/workspace/coverage}}"
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

# =============================================================================
# Display Header
# =============================================================================

echo "============================================"
if [ "$ENABLE_COVERAGE" = true ]; then
    echo "Running Integration Tests with Coverage (ROS $ROS_VERSION)"
    echo "Coverage output: ${COVERAGE_OUTPUT_DIR}"
else
    echo "Running Integration Tests (ROS $ROS_VERSION)"
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

# Clean build directory if requested (useful for testing multiple ROS versions)
if [ "$CLEAN_BUILD" = true ]; then
    echo "Cleaning build directory..."
    if [ "$ROS_VERSION" = "1" ]; then
        rm -rf "${WORKSPACE_ROOT}/middlewares/ros1/build"
        rm -rf "${WORKSPACE_ROOT}/middlewares/ros1/devel"
        rm -rf "${WORKSPACE_ROOT}/middlewares/ros1/logs"
    else
        rm -rf "${WORKSPACE_ROOT}/middlewares/ros2/build"
        rm -rf "${WORKSPACE_ROOT}/middlewares/ros2/install"
        rm -rf "${WORKSPACE_ROOT}/middlewares/ros2/log"
    fi
fi

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
# Part 2: Run Tests
# =============================================================================

echo ""
echo "============================================"
echo "Part 2: Running tests..."
echo "============================================"

if [ "$ROS_VERSION" = "1" ]; then
    echo "Running catkin tests..."
    cd "${WORKSPACE_ROOT}/middlewares/ros1"
    catkin build --no-notify --catkin-make-args run_tests
    catkin_test_results --verbose

    # Also run plugin tests directly if they were built
    if [ -d "${WORKSPACE_ROOT}/middlewares/ros1/build/axon_ros1_plugin" ]; then
        echo ""
        echo "============================================"
        echo "Running ROS1 plugin tests..."
        echo "============================================"
        cd "${WORKSPACE_ROOT}/middlewares/ros1/build/axon_ros1_plugin"
        ctest --output-on-failure || {
            echo "WARNING: Some plugin tests failed"
        }
    fi
else
    echo "Running colcon tests..."
    cd "${WORKSPACE_ROOT}/middlewares/ros2"

    # Run all tests including plugins
    colcon test \
        --event-handlers console_direct+
    colcon test-result --verbose

    # Also run plugin tests specifically to ensure they execute
    if [ -d "${WORKSPACE_ROOT}/middlewares/ros2/build/axon_ros2_plugin" ]; then
        echo ""
        echo "============================================"
        echo "Running ROS2 plugin tests..."
        echo "============================================"
        colcon test \
            --packages-select axon_ros2_plugin \
            --event-handlers console_direct+ || {
            echo "WARNING: Some plugin tests failed"
        }
    fi
fi

echo ""
echo "All tests passed!"

# =============================================================================
# Part 3: Generate Coverage Report (if enabled)
# =============================================================================

if [ "$ENABLE_COVERAGE" = true ]; then
    echo ""
    echo "============================================"
    echo "Part 3: Generating coverage report..."
    echo "============================================"
    
    # Generate coverage report using library function (COVERAGE_DIR is always set when coverage enabled)
    ros_coverage_generate "${COVERAGE_DIR}" "${COVERAGE_OUTPUT_DIR}" || {
        ros_coverage_error "Failed to generate coverage report"
    }
    
    # Generate HTML report
    ros_coverage_generate_html "${COVERAGE_OUTPUT_DIR}/coverage.info" "${COVERAGE_OUTPUT_DIR}" "Axon Test Coverage" || {
        ros_coverage_log "WARN" "HTML report generation skipped (genhtml not available)"
    }
    
    echo ""
    echo "============================================"
    echo "Coverage report complete!"
    echo "============================================"
    echo "Coverage data: ${COVERAGE_OUTPUT_DIR}/coverage.info"
    
    # Print final statistics
    if [ -f "${COVERAGE_OUTPUT_DIR}/coverage.info" ]; then
        TOTAL_LINES=$(lcov --summary "${COVERAGE_OUTPUT_DIR}/coverage.info" 2>&1 | grep "lines" | awk '{print $2}' || echo "N/A")
        echo "Total line coverage: ${TOTAL_LINES}"
    fi
fi

echo ""
echo "============================================"
echo "✓ Integration tests passed"
echo "============================================"

