#!/bin/bash
set -e

# =============================================================================
# Coverage Test Runner
# =============================================================================
# This script builds and runs tests with coverage instrumentation,
# then generates an lcov coverage report.
#
# Usage: ./run_coverage.sh [output_dir]
#   output_dir: Directory to store coverage reports (default: /workspace/coverage)
#
# Environment variables:
#   ROS_DISTRO: ROS distribution (default: humble)
#   ROS_VERSION: ROS version 1 or 2 (default: 2)
# =============================================================================

OUTPUT_DIR="${1:-/workspace/coverage}"

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

# Clean previous build
rm -rf build install log

# Build with coverage flags
if [ "${ROS_VERSION:-2}" = "1" ]; then
    # ROS 1 - Use catkin with coverage
    echo "Building with catkin (ROS 1) + coverage..."
    
    rm -rf /workspace/catkin_ws
    mkdir -p /workspace/catkin_ws/src
    cd /workspace/catkin_ws
    ln -sf /workspace/axon/ros/axon_recorder src/axon_recorder
    
    catkin_make \
        -DCMAKE_BUILD_TYPE=Debug \
        -DENABLE_COVERAGE=ON
    
    source devel/setup.bash
    
    echo "Running catkin tests..."
    catkin_make run_tests || true
    
    # Capture coverage
    COVERAGE_DIR="${PWD}/build"
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
    
    echo "Running colcon tests..."
    colcon test \
        --packages-select axon_recorder \
        --event-handlers console_direct+ \
        --base-paths ros || true
    
    colcon test-result --verbose || true
    
    COVERAGE_DIR="${PWD}/build/axon_recorder"
fi

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
LCOV_VERSION=$(lcov --version 2>&1 | grep -oP 'lcov: LCOV version \K[0-9.]+' || echo "1.0")
LCOV_MAJOR=$(echo "${LCOV_VERSION}" | cut -d. -f1)
echo "Detected lcov version: ${LCOV_VERSION} (major: ${LCOV_MAJOR})"

# Build lcov flags based on version
# --ignore-errors mismatch is only available in lcov 2.0+
LCOV_IGNORE_FLAGS=""
if [ "${LCOV_MAJOR}" -ge 2 ]; then
    LCOV_IGNORE_FLAGS="--ignore-errors mismatch,unused"
fi

# Find all directories containing .gcda files
echo "Searching for coverage data..."
GCDA_DIRS=$(find /workspace/axon/build -name "*.gcda" -type f -exec dirname {} \; 2>/dev/null | sort -u)

if [ -z "${GCDA_DIRS}" ]; then
    echo "ERROR: No .gcda files found!"
    echo "This may happen if:"
    echo "  - Tests didn't run successfully"
    echo "  - Coverage flags weren't applied correctly"
    exit 1
fi

echo "Found .gcda files in directories:"
echo "${GCDA_DIRS}" | head -10

# Capture coverage data from the entire build directory
echo ""
echo "Capturing coverage data..."
lcov --capture \
    --directory /workspace/axon/build \
    --output-file "${OUTPUT_DIR}/coverage_raw.info" \
    --rc lcov_branch_coverage=1 \
    ${LCOV_IGNORE_FLAGS} || {
    echo "Warning: lcov capture had issues, trying without ignore flags..."
    lcov --capture \
        --directory /workspace/axon/build \
        --output-file "${OUTPUT_DIR}/coverage_raw.info" \
        --rc lcov_branch_coverage=1 || true
}

# Check if we got any coverage data
if [ ! -s "${OUTPUT_DIR}/coverage_raw.info" ]; then
    echo "ERROR: No coverage data captured!"
    echo ""
    echo "Searching for .gcda files..."
    find /workspace/axon/build -name "*.gcda" -type f 2>/dev/null | head -20
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
    --output-file "${OUTPUT_DIR}/coverage.info" \
    --rc lcov_branch_coverage=1 \
    ${LCOV_IGNORE_FLAGS} || {
    echo "Warning: lcov filtering had issues, using raw coverage..."
    cp "${OUTPUT_DIR}/coverage_raw.info" "${OUTPUT_DIR}/coverage.info"
}

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
    
    mkdir -p "${OUTPUT_DIR}/html"
    genhtml "${OUTPUT_DIR}/coverage.info" \
        --output-directory "${OUTPUT_DIR}/html" \
        --title "Axon Test Coverage" \
        --legend \
        --branch-coverage || {
        echo "Warning: HTML generation had issues"
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

