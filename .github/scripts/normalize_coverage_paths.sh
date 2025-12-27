#!/bin/bash
# =============================================================================
# Normalize Coverage Paths Script
# =============================================================================
# Normalizes paths in lcov coverage files to repo-relative format.
# This script can be used in individual workflows before uploading with flags.
#
# Usage: normalize_coverage_paths.sh <coverage.info>
# =============================================================================

set -e

if [ $# -lt 1 ]; then
  echo "Usage: $0 <coverage.info>"
  exit 1
fi

COVERAGE_FILE="$1"

if [ ! -f "$COVERAGE_FILE" ]; then
  echo "Error: Coverage file not found: $COVERAGE_FILE"
  exit 1
fi

echo "=== Normalizing paths in: $COVERAGE_FILE ==="

# -----------------------------------------------------------------
# Step 1: GitHub Actions workspace prefix
# -----------------------------------------------------------------
# Path: /home/runner/work/Axon/Axon/cpp/... → cpp/...
sed -i 's|^SF:/home/runner/work/Axon/Axon/|SF:|' "$COVERAGE_FILE"

# -----------------------------------------------------------------
# Step 2: Docker ROS2 workspace (/workspace/axon/...)
# -----------------------------------------------------------------
sed -i 's|^SF:/workspace/axon/|SF:|' "$COVERAGE_FILE"

# -----------------------------------------------------------------
# Step 3: Docker ROS1 catkin workspace
# -----------------------------------------------------------------
sed -i 's|^SF:/workspace/catkin_ws/src/axon_recorder/\.\./\.\./cpp/|SF:cpp/|' "$COVERAGE_FILE"
sed -i 's|^SF:/workspace/catkin_ws/src/axon_recorder/\.\./\.\./|SF:|' "$COVERAGE_FILE"
sed -i 's|^SF:/workspace/catkin_ws/cpp/|SF:cpp/|' "$COVERAGE_FILE"
sed -i 's|^SF:/workspace/catkin_ws/src/axon_recorder/|SF:ros/src/axon_recorder/|' "$COVERAGE_FILE"
sed -i 's|^SF:/workspace/catkin_ws/build/|SF:build/|' "$COVERAGE_FILE"

# -----------------------------------------------------------------
# Step 4: Cleanup remaining relative paths
# -----------------------------------------------------------------
sed -i 's|^SF:catkin_ws/cpp/|SF:cpp/|' "$COVERAGE_FILE"
sed -i 's|^SF:catkin_ws/src/axon_recorder/|SF:ros/src/axon_recorder/|' "$COVERAGE_FILE"
sed -i 's|^SF:Axon/|SF:|' "$COVERAGE_FILE"

# -----------------------------------------------------------------
# Step 5: Fix incorrect normalizations (ros/axon_recorder/ → ros/src/axon_recorder/)
# -----------------------------------------------------------------
# Some paths may have been normalized incorrectly to ros/axon_recorder/ (missing src/)
# This step fixes them to match the actual repo structure: ros/src/axon_recorder/
sed -i 's|^SF:ros/axon_recorder/|SF:ros/src/axon_recorder/|' "$COVERAGE_FILE"

# Note: Build directory paths (SF:build/...) are kept as-is so that
# Codecov's ignore rule "build/**/*" in codecov.yml can properly filter them.
# Earlier steps normalize absolute build paths to SF:build/... format,
# and they should remain in this format for proper exclusion.

echo "✓ Path normalization complete"
echo ""
echo "Sample normalized paths:"
grep "^SF:" "$COVERAGE_FILE" | head -5 || echo "  (none)"

