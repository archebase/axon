#!/bin/bash
set -e

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

# Support both argument and environment variable for output path
OUTPUT_DIR="${1:-${COVERAGE_OUTPUT:-/coverage_output}}"

echo "============================================"
echo "Running E2E Tests with Coverage"
echo "ROS ${ROS_VERSION:-2} (${ROS_DISTRO:-humble})"
echo "Output directory: ${OUTPUT_DIR}"
echo "============================================"

# Source ROS environment
ROS_DISTRO="${ROS_DISTRO:-humble}"
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    source /opt/ros/${ROS_DISTRO}/setup.bash
    echo "Sourced ROS ${ROS_DISTRO} environment"
else
    echo "ERROR: ROS setup.bash not found at /opt/ros/${ROS_DISTRO}/setup.bash"
    exit 1
fi

cd /workspace/axon

# Create output directory
mkdir -p "${OUTPUT_DIR}"

# =============================================================================
# Part 1: Build with Coverage Flags
# =============================================================================
echo ""
echo "============================================"
echo "Part 1: Building with coverage instrumentation..."
echo "============================================"

# Clean previous build
rm -rf build install log

# Build with coverage flags
if [ "${ROS_VERSION:-2}" = "1" ]; then
    # ROS 1 - Use catkin with coverage
    echo "Building with catkin (ROS 1) + coverage..."
    
    rm -rf /workspace/catkin_ws
    mkdir -p /workspace/catkin_ws/src
    cd /workspace/catkin_ws
    # Symlink the package
    ln -sf /workspace/axon/ros/axon_recorder src/axon_recorder
    # Symlink cpp/ directory for dependencies (axon_mcap, axon_logging)
    # CMakeLists.txt uses ../../cpp/ relative paths
    ln -sf /workspace/axon/cpp cpp
    
    catkin_make \
        -DCMAKE_BUILD_TYPE=Debug \
        -DENABLE_COVERAGE=ON
    
    source devel/setup.bash
    
    COVERAGE_DIR="${PWD}/build"
    E2E_RUNNER="/workspace/axon/ros/axon_recorder/test/e2e/run_e2e_tests.sh"
else
    # ROS 2 - Use colcon with coverage
    echo "Building with colcon (ROS 2) + coverage..."
    
    cd /workspace/axon
    
    colcon build \
        --packages-select axon_recorder \
        --cmake-args \
            -DCMAKE_BUILD_TYPE=Debug \
            -DENABLE_COVERAGE=ON \
        --base-paths ros
    
    source install/setup.bash
    
    COVERAGE_DIR="${PWD}/build/axon_recorder"
    E2E_RUNNER="/workspace/axon/ros/axon_recorder/test/e2e/run_e2e_tests.sh"
fi

# =============================================================================
# Part 2: Run E2E Tests ONLY (no unit tests)
# =============================================================================
echo ""
echo "============================================"
echo "Part 2: Running E2E tests only..."
echo "============================================"

E2E_EXIT_CODE=0

if [ -f "${E2E_RUNNER}" ]; then
    chmod +x "${E2E_RUNNER}"
    "${E2E_RUNNER}" || E2E_EXIT_CODE=$?
else
    echo "ERROR: E2E test runner not found at ${E2E_RUNNER}"
    exit 1
fi

echo "E2E tests completed with exit code: ${E2E_EXIT_CODE}"

# =============================================================================
# Part 3: Generate Coverage Report
# =============================================================================
echo ""
echo "============================================"
echo "Part 3: Generating coverage report..."
echo "============================================"

# Check if lcov is available
if ! command -v lcov &> /dev/null; then
    echo "Installing lcov..."
    apt-get update && apt-get install -y lcov
fi

# Get lcov version to determine supported flags
LCOV_VERSION=$(lcov --version 2>&1 | grep -oP 'LCOV version \K[0-9.]+' || echo "1.0")
LCOV_MAJOR=$(echo "${LCOV_VERSION}" | cut -d. -f1)
echo "Detected lcov version: ${LCOV_VERSION} (major: ${LCOV_MAJOR})"

# Build lcov flags based on version
# Error types to ignore:
#   mismatch - mismatched exception tags (C++ std lib differences)
#   unused   - unused coverage data
#   negative - negative branch counts (compiler/gcov compatibility issue)
#   gcov     - unexecuted blocks on non-branch lines
LCOV_IGNORE_FLAGS=""
if [ "${LCOV_MAJOR}" -ge 2 ]; then
    LCOV_IGNORE_FLAGS="--ignore-errors mismatch,unused,negative,gcov"
fi

# Find all directories containing .gcda files
echo "Searching for coverage data in ${COVERAGE_DIR}..."
GCDA_DIRS=$(find "${COVERAGE_DIR}" -name "*.gcda" -type f -exec dirname {} \; 2>/dev/null | sort -u)

if [ -z "${GCDA_DIRS}" ]; then
    echo "WARNING: No .gcda files found in ${COVERAGE_DIR}!"
    echo "This may happen if E2E tests didn't execute any instrumented code."
    # Create empty coverage file and exit with test result
    touch "${OUTPUT_DIR}/coverage.info"
    exit ${E2E_EXIT_CODE}
fi

echo "Found .gcda files in directories:"
echo "${GCDA_DIRS}" | head -10

# Capture coverage data from the build directory
echo ""
echo "Capturing coverage data from ${COVERAGE_DIR}..."
lcov --capture \
    --directory "${COVERAGE_DIR}" \
    --output-file "${OUTPUT_DIR}/coverage_raw.info" \
    --rc lcov_branch_coverage=1 \
    ${LCOV_IGNORE_FLAGS} || {
    echo "Warning: lcov capture had issues, trying with more permissive ignore flags..."
    # Fallback: try with all common error types ignored (lcov 2.0+ only)
    if [ "${LCOV_MAJOR}" -ge 2 ]; then
        lcov --capture \
            --directory "${COVERAGE_DIR}" \
            --output-file "${OUTPUT_DIR}/coverage_raw.info" \
            --rc lcov_branch_coverage=1 \
            --ignore-errors mismatch,unused,negative,gcov,source || true
    else
        lcov --capture \
            --directory "${COVERAGE_DIR}" \
            --output-file "${OUTPUT_DIR}/coverage_raw.info" \
            --rc lcov_branch_coverage=1 || true
    fi
}

# Check if we got any coverage data
if [ ! -s "${OUTPUT_DIR}/coverage_raw.info" ]; then
    echo "WARNING: No coverage data captured!"
    touch "${OUTPUT_DIR}/coverage.info"
    exit ${E2E_EXIT_CODE}
fi

echo "Coverage data captured: $(wc -l < "${OUTPUT_DIR}/coverage_raw.info") lines"

# Remove external dependencies from coverage
echo "Filtering coverage data..."
lcov --remove "${OUTPUT_DIR}/coverage_raw.info" \
    '/usr/*' \
    '/opt/*' \
    '*/_deps/*' \
    '*/generated/*' \
    '*/googletest/*' \
    '*/gtest/*' \
    '*/mcap-*/*' \
    '*/mcap/include/*' \
    '*/minio-cpp/*' \
    '*/axon_recorder/test/*' \
    '*/axon_logging/test/*' \
    '*/axon_mcap/test/*' \
    '*/axon_uploader/test/*' \
    '*/rosidl_typesupport_cpp/*' \
    '*/rosidl_typesupport_introspection_cpp/*' \
    '*/rosidl_generator_cpp/*' \
    --output-file "${OUTPUT_DIR}/coverage.info" \
    --rc lcov_branch_coverage=1 \
    ${LCOV_IGNORE_FLAGS} || {
    echo "Warning: lcov filtering had issues, using raw coverage..."
    cp "${OUTPUT_DIR}/coverage_raw.info" "${OUTPUT_DIR}/coverage.info"
}

# Generate coverage summary
echo ""
echo "============================================"
echo "E2E Test Coverage Summary"
echo "============================================"
lcov --list "${OUTPUT_DIR}/coverage.info" || true

# Debug: show sample of paths
echo ""
echo "=== Source file paths in coverage.info ==="
grep "^SF:" "${OUTPUT_DIR}/coverage.info" | head -10 || true

# Verify file is not empty
if [ -s "${OUTPUT_DIR}/coverage.info" ]; then
    echo ""
    echo "Coverage file size: $(wc -c < "${OUTPUT_DIR}/coverage.info") bytes"
else
    echo "WARNING: coverage.info is empty!"
fi

echo ""
echo "============================================"
echo "E2E test coverage collection complete!"
echo "============================================"
echo "Coverage data: ${OUTPUT_DIR}/coverage.info"

# Exit with E2E test result
exit ${E2E_EXIT_CODE}

