#!/bin/bash
set -eo pipefail

# =============================================================================
# ROS Test Runner
# =============================================================================
# This script runs tests using ROS's native test infrastructure:
#   - colcon test (ROS 2)
#   - catkin build --catkin-make-args run_tests (ROS 1)
#
# This matches how CI runs tests via industrial_ci.
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

if [ ! -f "${LIB_DIR}/ros_workspace_lib.sh" ]; then
    echo "ERROR: ros_workspace_lib.sh not found at ${LIB_DIR}/ros_workspace_lib.sh" >&2
    exit 1
fi
source "${LIB_DIR}/ros_workspace_lib.sh"

echo "============================================"
echo "Running ROS tests for ROS ${ROS_VERSION} (${ROS_DISTRO})"
echo "============================================"

# Source ROS base environment
ros_workspace_source_base || {
    ros_workspace_error "Failed to source ROS base environment"
}

WORKSPACE_ROOT="/workspace/axon"
cd "${WORKSPACE_ROOT}"

# =============================================================================
# Part 1: Build and Run ROS Tests
# =============================================================================
# This is equivalent to what industrial_ci does in ros-unit-tests job
# =============================================================================
echo ""
echo "============================================"
echo "Part 1: Building and running ROS tests..."
echo "============================================"

# Build package (Release, no coverage)
ros_build_package "${WORKSPACE_ROOT}" "true" "false" "Release" || {
    ros_build_error "Failed to build package"
}

# Re-source workspace after build
ros_workspace_source_workspace "${WORKSPACE_ROOT}" || {
    ros_workspace_error "Failed to source workspace after build"
}

# Run tests based on ROS version
if [ "${ROS_VERSION}" = "1" ]; then
    echo "Running catkin tests..."
    catkin build --no-notify --catkin-make-args run_tests
    catkin_test_results --verbose
    echo "✓ ROS 1 tests passed"
else
    echo "Running colcon tests..."
    colcon test \
        --packages-select axon_recorder \
        --event-handlers console_direct+
    colcon test-result --verbose
    echo "✓ ROS 2 tests passed"
fi

echo ""
echo "============================================"
echo "All unit/integration tests passed!"
echo "============================================"
echo ""
echo "NOTE: E2E tests (ROS service calls) run separately via run_e2e_tests.sh"
echo "      (with --coverage flag for coverage collection) via e2e-tests.yml workflow."
