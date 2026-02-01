#!/bin/bash
# =============================================================================
# Normalize Coverage Paths Script
# =============================================================================
# Normalizes paths in lcov coverage files to repo-relative format.
# This script can be used in individual workflows before uploading with flags.
#
# Current directory structure:
#   - core/                    # Core libraries (axon_mcap, axon_logging, axon_uploader)
#   - middlewares/             # Middleware plugins (ros1, ros2, zenoh)
#   - apps/                    # Applications (axon_recorder, plugin_example)
#   - build/                   # Unified build output for all components
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
# Step 4: Cleanup remaining incorrect paths
# =============================================================================
# Remove any remaining Axon/ prefix
sed -i 's|^SF:Axon/|SF:|' "$COVERAGE_FILE"

echo "✓ Path normalization complete"
echo ""
echo "Sample normalized paths:"
grep "^SF:" "$COVERAGE_FILE" | head -10 || echo "  (none)"


