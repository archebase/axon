#!/bin/bash
# =============================================================================
# Normalize Coverage Paths Script
# =============================================================================
# Normalizes paths in lcov coverage files to repo-relative format.
# This script can be used in individual workflows before uploading with flags.
#
# Current directory structure:
#   - core/                    # Core libraries (axon_mcap, axon_logging, axon_uploader)
#   - middlewares/ros1/src/ros1_plugin/    # ROS1 plugin
#   - middlewares/ros2/src/ros2_plugin/    # ROS2 plugin
#   - apps/axon_recorder/      # Main recorder application
#   - build/                   # Build output (build/axon_mcap, build/axon_uploader, etc.)
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

# =============================================================================
# Step 1: GitHub Actions workspace prefix
# =============================================================================
# Path: /home/runner/work/Axon/Axon/core/... → core/...
sed -i 's|^SF:/home/runner/work/Axon/Axon/|SF:|' "$COVERAGE_FILE"

# =============================================================================
# Step 2: Docker workspace prefix (/workspace/axon/...)
# =============================================================================
sed -i 's|^SF:/workspace/axon/|SF:|' "$COVERAGE_FILE"

# =============================================================================
# Step 3: Build directory normalization
# =============================================================================
# Keep build/ prefix for Codecov filtering (codecov.yml has "build/**/*" ignore)
# Ensure build paths use forward slashes
sed -i 's|^SF:build\\|SF:build/|g' "$COVERAGE_FILE"

# =============================================================================
# Step 4: ROS1 catkin workspace paths
# =============================================================================
# middlewares/ros1/build/axon_ros1_plugin/src/... → middlewares/ros1/src/ros1_plugin/...
sed -i 's|^SF:middlewares/ros1/build/axon_ros1_plugin/src/|SF:middlewares/ros1/src/ros1_plugin/|' "$COVERAGE_FILE"
sed -i 's|^SF:middlewares/ros1/build/|SF:build/|' "$COVERAGE_FILE"

# =============================================================================
# Step 5: ROS2 colcon workspace paths
# =============================================================================
# middlewares/ros2/build/axon_ros2_plugin/src/... → middlewares/ros2/src/ros2_plugin/...
sed -i 's|^SF:middlewares/ros2/build/axon_ros2_plugin/src/|SF:middlewares/ros2/src/ros2_plugin/|' "$COVERAGE_FILE"
sed -i 's|^SF:middlewares/ros2/build/|SF:build/|' "$COVERAGE_FILE"
# Also handle install directory paths
sed -i 's|^SF:middlewares/ros2/install/axon_ros2_plugin/|SF:middlewares/ros2/src/ros2_plugin/|' "$COVERAGE_FILE"

# =============================================================================
# Step 6: Cleanup remaining incorrect paths
# =============================================================================
# Remove any remaining Axon/ prefix
sed -i 's|^SF:Axon/|SF:|' "$COVERAGE_FILE"

# =============================================================================
# Step 7: Fix old structure paths (if any exist)
# =============================================================================
# These are for backward compatibility with old CI runs
sed -i 's|^SF:middlewares/src/axon_recorder/|SF:apps/axon_recorder/|' "$COVERAGE_FILE"
sed -i 's|^SF:core/build_axon_uploader/|SF:build/axon_uploader/|' "$COVERAGE_FILE"

echo "✓ Path normalization complete"
echo ""
echo "Sample normalized paths:"
grep "^SF:" "$COVERAGE_FILE" | head -10 || echo "  (none)"

