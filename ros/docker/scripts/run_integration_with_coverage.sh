#!/bin/bash
set -eo pipefail

# =============================================================================
# Integration Test Runner with Coverage
# =============================================================================
# This script builds and runs integration tests with coverage instrumentation,
# then generates an lcov coverage report.
#
# Usage: ./run_integration_with_coverage.sh [output_dir]
#   output_dir: Directory to store coverage reports (default: /workspace/coverage)
#
# Environment variables:
#   ROS_DISTRO: ROS distribution (default: humble)
#   ROS_VERSION: ROS version 1 or 2 (default: 2)
# =============================================================================

# Source shared libraries
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIB_DIR="${SCRIPT_DIR}/lib"

# Source libraries (with error handling)
# Note: We can't use library error functions here since libraries aren't loaded yet
if [ ! -f "${LIB_DIR}/ros_build_lib.sh" ]; then
    echo "ERROR: ros_build_lib.sh not found at ${LIB_DIR}/ros_build_lib.sh" >&2
    exit 1
fi
source "${LIB_DIR}/ros_build_lib.sh"

if [ ! -f "${LIB_DIR}/ros_coverage_lib.sh" ]; then
    echo "ERROR: ros_coverage_lib.sh not found at ${LIB_DIR}/ros_coverage_lib.sh" >&2
    exit 1
fi
source "${LIB_DIR}/ros_coverage_lib.sh"

if [ ! -f "${LIB_DIR}/ros_workspace_lib.sh" ]; then
    echo "ERROR: ros_workspace_lib.sh not found at ${LIB_DIR}/ros_workspace_lib.sh" >&2
    exit 1
fi
source "${LIB_DIR}/ros_workspace_lib.sh"

# Support both argument and environment variable for output path
OUTPUT_DIR="${1:-${COVERAGE_OUTPUT:-/workspace/coverage}}"

echo "============================================"
echo "Running Coverage Tests for ROS ${ROS_VERSION:-2} (${ROS_DISTRO:-humble})"
echo "Output directory: ${OUTPUT_DIR}"
echo "============================================"

# Set defaults
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_VERSION="${ROS_VERSION:-2}"
WORKSPACE_ROOT="/workspace/axon"

# Source ROS base environment
ros_workspace_source_base || {
    ros_workspace_error "Failed to source ROS base environment"
}

cd "${WORKSPACE_ROOT}"

# Create output directory
mkdir -p "${OUTPUT_DIR}"

# =============================================================================
# Part 1: Build with Coverage Flags
# =============================================================================
echo ""
echo "============================================"
echo "Part 1: Building with coverage instrumentation..."
echo "============================================"

# Build package with coverage enabled
ros_build_package "${WORKSPACE_ROOT}" "true" "true" "Debug" || {
    ros_build_error "Failed to build package with coverage"
}

# Re-source workspace after build (build function already does this, but ensure it's set)
ros_workspace_source_workspace "${WORKSPACE_ROOT}" || {
    ros_workspace_error "Failed to source workspace after build"
}

# Set coverage directory from build output
COVERAGE_DIR="${WORKSPACE_BUILD_DIR}"

# =============================================================================
# Part 1.5: Run Tests (exits immediately on failure - no coverage for failed tests)
# =============================================================================
echo ""
echo "============================================"
echo "Running tests..."
echo "============================================"

if [ "${ROS_VERSION:-2}" = "1" ]; then
    echo "Running catkin tests..."
    catkin build --no-notify --catkin-make-args run_tests
    catkin_test_results --verbose
else
    echo "Running colcon tests..."
    colcon test \
        --packages-select axon_recorder \
        --event-handlers console_direct+
    colcon test-result --verbose
fi

echo ""
echo "All tests passed!"

# =============================================================================
# Part 2: Generate Coverage Report
# =============================================================================
echo ""
echo "============================================"
echo "Part 2: Generating coverage report..."
echo "============================================"

# Generate coverage report using library function
ros_coverage_generate "${COVERAGE_DIR}" "${OUTPUT_DIR}" || {
    ros_coverage_error "Failed to generate coverage report"
}

# Note: Path normalization is handled in CI workflow by normalize_coverage_paths.sh,
# not in this script (consistent with run_e2e_tests.sh --coverage)

# =============================================================================
# Part 3: Generate HTML Report (optional)
# =============================================================================
if command -v genhtml &> /dev/null; then
    echo ""
    echo "============================================"
    echo "Part 3: Generating HTML report..."
    echo "============================================"
    
    # Get lcov version for genhtml flags
    GENHTML_LCOV_VERSION=$(lcov --version 2>&1 | grep -oP 'LCOV version \K[0-9.]+' || echo "1.0")
    GENHTML_LCOV_MAJOR=$(echo "$GENHTML_LCOV_VERSION" | cut -d. -f1)
    
    # Build genhtml flags based on version
    GENHTML_IGNORE_FLAGS=""
    if [ "$GENHTML_LCOV_MAJOR" -ge 2 ]; then
        GENHTML_IGNORE_FLAGS="--ignore-errors source,unmapped"
    fi
    
    mkdir -p "${OUTPUT_DIR}/html"
    genhtml "${OUTPUT_DIR}/coverage.info" \
        --output-directory "${OUTPUT_DIR}/html" \
        --title "Axon Test Coverage" \
        --legend \
        --branch-coverage \
        ${GENHTML_IGNORE_FLAGS} || {
        echo "Warning: HTML generation had issues, trying with --ignore-errors source..."
        genhtml "${OUTPUT_DIR}/coverage.info" \
            --output-directory "${OUTPUT_DIR}/html" \
            --title "Axon Test Coverage" \
            --legend \
            --ignore-errors source 2>/dev/null || true
    }
    
    echo ""
    echo "HTML report generated at: ${OUTPUT_DIR}/html/index.html"
fi

echo ""
echo "============================================"
echo "Coverage report complete!"
echo "============================================"
echo "Coverage data: ${OUTPUT_DIR}/coverage.info"
echo ""

# Print final statistics
if [ -f "${OUTPUT_DIR}/coverage.info" ]; then
    TOTAL_LINES=$(lcov --summary "${OUTPUT_DIR}/coverage.info" 2>&1 | grep "lines" | awk '{print $2}' || echo "N/A")
    echo "Total line coverage: ${TOTAL_LINES}"
fi

# Script exits with 0 if we reach here (tests passed, coverage generated)

