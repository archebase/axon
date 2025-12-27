#!/bin/bash
set -e

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

# Support both argument and environment variable for output path
OUTPUT_DIR="${1:-${COVERAGE_OUTPUT:-/workspace/coverage}}"

echo "============================================"
echo "Running Coverage Tests for ROS ${ROS_VERSION:-2} (${ROS_DISTRO:-humble})"
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

if [ "${ROS_VERSION:-2}" = "1" ]; then
    # ROS 1 - Use catkin build
    echo "Building with catkin build (ROS 1) + coverage..."

    cd /workspace/axon/ros
    # Clean previous build (union of ROS1 and ROS2 build artifacts)
    rm -rf build devel install log logs

    source /opt/ros/${ROS_DISTRO}/setup.bash
    catkin build --no-notify \
        -DCMAKE_BUILD_TYPE=Debug \
        -DENABLE_COVERAGE=ON

    source devel/setup.bash

    COVERAGE_DIR="${PWD}/build/axon_recorder"
else
    # ROS 2 - Use colcon
    echo "Building with colcon (ROS 2) + coverage..."

    cd /workspace/axon/ros
    rm -rf build devel install log logs

    source /opt/ros/${ROS_DISTRO}/setup.bash
    colcon build \
        --packages-select axon_recorder \
        --cmake-args \
            -DCMAKE_BUILD_TYPE=Debug \
            -DENABLE_COVERAGE=ON \
        --base-paths ros

    source install/setup.bash

    COVERAGE_DIR="${PWD}/build/axon_recorder"
fi

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
        --event-handlers console_direct+ \
        --base-paths ros
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
# --ignore-errors mismatch is only available in lcov 2.0+
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
    echo "ERROR: No .gcda files found in ${COVERAGE_DIR}!"
    echo "This may happen if:"
    echo "  - Tests didn't run successfully"
    echo "  - Coverage flags weren't applied correctly"
    exit 1
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
    echo "ERROR: No coverage data captured!"
    echo ""
    echo "Searching for .gcda files in ${COVERAGE_DIR}..."
    find "${COVERAGE_DIR}" -name "*.gcda" -type f 2>/dev/null | head -20
    exit 1
fi

echo "Coverage data captured: $(wc -l < "${OUTPUT_DIR}/coverage_raw.info") lines"

# Remove external dependencies from coverage (keep test files for now to see what's tested)
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

# Normalize paths for Codecov compatibility (same as CI)
echo "Normalizing paths for Codecov compatibility..."
# Remove /workspace/axon/ prefix to get repo-relative paths
sed -i "s|/workspace/axon/||g" "${OUTPUT_DIR}/coverage.info" || true

# Debug: show sample of normalized paths
echo "Sample paths in coverage.info:"
grep "^SF:" "${OUTPUT_DIR}/coverage.info" | head -10 || true

# Generate coverage summary
echo ""
echo "============================================"
echo "Coverage Summary"
echo "============================================"
lcov --list "${OUTPUT_DIR}/coverage.info" || true

# =============================================================================
# Part 3: Generate HTML Report (optional)
# =============================================================================
if command -v genhtml &> /dev/null; then
    echo ""
    echo "============================================"
    echo "Part 3: Generating HTML report..."
    echo "============================================"
    
    # Build genhtml flags based on lcov version
    GENHTML_IGNORE_FLAGS=""
    if [ "${LCOV_MAJOR}" -ge 2 ]; then
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

