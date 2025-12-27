#!/bin/bash
set -eo pipefail

# =============================================================================
# E2E Test Runner with Coverage
# =============================================================================
# This script builds with coverage instrumentation and runs ONLY E2E tests
# (not unit tests). Used by CI for E2E-specific coverage collection.
#
# Usage: ./run_e2e_with_coverage.sh [output_dir]
#   output_dir: Directory to store coverage reports (default: /coverage_output)
#
# Environment variables:
#   ROS_DISTRO: ROS distribution (default: humble)
#   ROS_VERSION: ROS version 1 or 2 (default: 2)
#   COVERAGE_OUTPUT: Alternative way to specify output directory
# =============================================================================

# Source shared libraries
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIB_DIR="${SCRIPT_DIR}/lib"

# Source libraries (with error handling for missing files)
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

if [ ! -f "${LIB_DIR}/ros_diagnostics_lib.sh" ]; then
    echo "ERROR: ros_diagnostics_lib.sh not found at ${LIB_DIR}/ros_diagnostics_lib.sh" >&2
    exit 1
fi
source "${LIB_DIR}/ros_diagnostics_lib.sh"

# Support both argument and environment variable for output path
OUTPUT_DIR="${1:-${COVERAGE_OUTPUT:-/coverage_output}}"

# Set defaults
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_VERSION="${ROS_VERSION:-2}"
WORKSPACE_ROOT="/workspace/axon"

echo "============================================"
echo "Running E2E Tests with Coverage"
echo "ROS ${ROS_VERSION} (${ROS_DISTRO})"
echo "Output directory: ${OUTPUT_DIR}"
echo "============================================"

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

# Set coverage directory based on build output
COVERAGE_DIR="${WORKSPACE_BUILD_DIR}"
E2E_RUNNER="${WORKSPACE_ROOT}/ros/src/axon_recorder/test/e2e/run_e2e_tests.sh"

# =============================================================================
# Part 2: Run E2E Tests ONLY (exits immediately on failure - no coverage for failed tests)
# =============================================================================
echo ""
echo "============================================"
echo "Part 2: Running E2E tests only..."
echo "============================================"

# CRITICAL: Re-source workspace setup files after build
# The build tool warns: "Workspace packages have changed, please re-source setup files"
# This ensures the test environment can find the newly built packages
ros_workspace_source_workspace "${WORKSPACE_ROOT}" || {
    ros_workspace_error "Failed to source workspace after build"
}

if [ ! -f "${E2E_RUNNER}" ]; then
    ros_workspace_error "E2E test runner not found at ${E2E_RUNNER}"
fi

chmod +x "${E2E_RUNNER}"
"${E2E_RUNNER}" || {
    ros_workspace_error "E2E tests failed"
}

echo "E2E tests passed!"

# =============================================================================
# Part 3: Generate Coverage Report
# =============================================================================
echo ""
echo "============================================"
echo "Part 3: Generating coverage report..."
echo "============================================"

# Generate coverage report using library function
ros_coverage_generate "${COVERAGE_DIR}" "${OUTPUT_DIR}" || {
    ros_coverage_error "Failed to generate coverage report"
}

# Display final summary
echo ""
echo "============================================"
echo "E2E Test Coverage Summary"
echo "============================================"
lcov --list "${OUTPUT_DIR}/coverage.info" || true

echo ""
echo "============================================"
echo "E2E test coverage collection complete!"
echo "============================================"
echo "Coverage data: ${OUTPUT_DIR}/coverage.info"

# Script exits with 0 if we reach here (tests passed, coverage generated)

