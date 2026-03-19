#!/bin/bash
# SPDX-FileCopyrightText: 2026 ArcheBase
# SPDX-License-Identifier: MulanPSL-2.0

set -e

# =============================================================================
# Axon UDP Plugin Package Build Script
# =============================================================================
# Builds the UDP plugin Debian package.
# =============================================================================

# Detect if running in Docker
if [ -d "/axon" ] && [ -f "/axon/CMakeLists.txt" ]; then
    PROJECT_ROOT="/axon"
    SCRIPT_DIR="/axon/packaging/deb/scripts"
else
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    PROJECT_ROOT="$(cd "$(dirname "${SCRIPT_DIR}")/../.." && pwd)"
fi

PACKAGE_DIR="${PROJECT_ROOT}/packaging/deb"
OUTPUT_DIR="${PACKAGE_DIR}/output"
BUILD_DIR="${PACKAGE_DIR}/build"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Get version
if [ -f "${PROJECT_ROOT}/apps/axon_recorder/CMakeLists.txt" ]; then
    VERSION="$(grep 'project.*VERSION' "${PROJECT_ROOT}/apps/axon_recorder/CMakeLists.txt" | sed 's/.*VERSION \([0-9.]*\).*/\1/')"
fi
if [ -z "$VERSION" ]; then
    VERSION="0.3.0"
fi
DEBIAN_VERSION="${VERSION}-1"

log_info "Building axon-plugin-udp version ${DEBIAN_VERSION}..."
log_info "Build directory: ${BUILD_DIR}"
log_info "Output directory: ${OUTPUT_DIR}"

# Create output and build directories
mkdir -p "${OUTPUT_DIR}"
mkdir -p "${BUILD_DIR}"

# Save current directory
ORIGINAL_DIR="$(pwd)"

# Clean and create build area
BUILD_AREA="${BUILD_DIR}/axon-plugin-udp"
rm -rf "${BUILD_AREA}"
mkdir -p "${BUILD_AREA}"

# Copy source to build area (excluding build artifacts and .git)
mkdir -p "${BUILD_AREA}"
# Use tar instead of rsync for better compatibility
(cd "${PROJECT_ROOT}/middlewares/udp" && tar cf - \
    --exclude='.git*' \
    --exclude='build' \
    --exclude='install' \
    --exclude='log' \
    --exclude='*.pyc' \
    --exclude='__pycache__' \
    --exclude='debian' \
    --exclude='obj-*' \
    --exclude='.debian-build' \
    --exclude='CMakeCache.txt' \
    --exclude='CMakeFiles' \
    .) | (cd "${BUILD_AREA}" && tar xf -)

# Copy debian files to build area
cp -r "${PACKAGE_DIR}/plugin-udp/debian" "${BUILD_AREA}/debian"

# Build from build area
cd "${BUILD_AREA}"

# Pass AXON_REPO_ROOT to debian/rules
if AXON_REPO_ROOT="${PROJECT_ROOT}" dpkg-buildpackage -b -uc -us -j"$(nproc)" 2>&1; then
    log_info "Successfully built axon-plugin-udp"

    # Move built packages to output directory
    find "${BUILD_DIR}" -maxdepth 1 -name 'axon-plugin-udp_*.deb' -exec mv {} "${OUTPUT_DIR}/" \; 2>/dev/null || true

    cd "${ORIGINAL_DIR}"

    log_info "==================================================================="
    log_info "UDP plugin build complete!"
    log_info "==================================================================="
    ls -lh "${OUTPUT_DIR}"/axon-plugin-udp_*.deb 2>/dev/null || true
    log_info "==================================================================="
else
    log_error "Failed to build axon-plugin-udp"
    cd "${ORIGINAL_DIR}"
    exit 1
fi
