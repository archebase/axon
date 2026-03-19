#!/bin/bash
# SPDX-FileCopyrightText: 2026 ArcheBase
# SPDX-License-Identifier: MulanPSL-2.0

set -e

# =============================================================================
# Axon ROS1 Plugin Package Build Script
# =============================================================================
# Builds Debian packages for ROS1 Noetic middleware plugin.
# Requires a sourced ROS1 Noetic environment.
# =============================================================================

# Detect if running in Docker
if [ -d "/axon" ] && [ -f "/axon/CMakeLists.txt" ]; then
    PROJECT_ROOT="/axon"
else
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    PACKAGE_DIR="$(dirname "${SCRIPT_DIR}")"
    PROJECT_ROOT="$(cd "${PACKAGE_DIR}/../.." && pwd)"
fi

PACKAGE_DIR="${PROJECT_ROOT}/packaging/deb"
# ROS1 Noetic runs on Ubuntu 20.04 (Focal)
UBUNTU_DISTRO="focal"
OUTPUT_DIR="${PACKAGE_DIR}/output/${UBUNTU_DISTRO}"
BUILD_DIR="${PACKAGE_DIR}/build"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check for ROS1 Noetic
if [ -d "/opt/ros/noetic" ] && [ -f "/opt/ros/noetic/setup.bash" ]; then
    # Source ROS1 environment
    # shellcheck source=/dev/null
    source "/opt/ros/noetic/setup.bash"
else
    log_warn "ROS1 Noetic not found at /opt/ros/noetic"
    log_warn "Continuing anyway (Docker build may have pre-sourced environment)"
fi

# Create output and build directories
mkdir -p "${OUTPUT_DIR}"
mkdir -p "${BUILD_DIR}"

# Clean build directory for ROS1
rm -rf "${BUILD_DIR:?}"/ros1*
mkdir -p "${BUILD_DIR}"

# Get version from project
if [ -f "${PROJECT_ROOT}/apps/axon_recorder/CMakeLists.txt" ]; then
    VERSION="$(grep 'project.*VERSION' "${PROJECT_ROOT}/apps/axon_recorder/CMakeLists.txt" | sed 's/.*VERSION \([0-9.]*\).*/\1/')"
fi
if [ -z "$VERSION" ]; then
    VERSION="0.3.0"
fi
DEBIAN_VERSION="${VERSION}-1"

log_info "Building Axon ROS1 Noetic plugin packages version ${DEBIAN_VERSION}"
log_info "Build directory: ${BUILD_DIR}"
log_info "Output directory: ${OUTPUT_DIR}"

# Build function for plugins
build_plugin() {
    local pkg_name="$1"
    local source_dir="$2"
    local debian_dir="${PACKAGE_DIR}/plugin-${pkg_name#axon-plugin-}/debian"
    local build_area="${BUILD_DIR}/${pkg_name}"

    if [ ! -d "$debian_dir" ]; then
        log_error "Debian directory not found: $debian_dir"
        return 1
    fi

    if [ ! -d "$source_dir" ]; then
        log_error "Source directory not found: ${source_dir}"
        return 1
    fi

    log_info "Building ${pkg_name}..."

    # Save current directory
    local original_dir="$(pwd)"

    # Clean and create build area
    rm -rf "${build_area}"
    mkdir -p "${build_area}"

    # Copy source to build area (excluding build artifacts and .git)
    mkdir -p "${build_area}"
    # Use tar instead of rsync for better compatibility
    (cd "${source_dir}" && tar cf - \
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
        .) | (cd "${build_area}" && tar xf -)

    # Copy debian files to build area
    cp -r "${debian_dir}" "${build_area}/debian"

    # Update version in changelog
    if [ -f "${build_area}/debian/changelog" ]; then
        sed -i "s/([0-9]\+\.[0-9]\+\.[0-9]\+-[0-9]\+)/(${DEBIAN_VERSION})/" "${build_area}/debian/changelog"
    fi

    # Build from build area
    cd "${build_area}"
    if AXON_REPO_ROOT="${PROJECT_ROOT}" dpkg-buildpackage -b -uc -us -j"$(nproc)" 2>&1; then
        log_info "Successfully built ${pkg_name}"

        # Move built packages to output directory
        find "${BUILD_DIR}" -maxdepth 1 -name "${pkg_name}_*.deb" -exec mv {} "${OUTPUT_DIR}/" \; 2>/dev/null || true

        cd "${original_dir}"
        return 0
    else
        log_error "Failed to build ${pkg_name}"
        cd "${original_dir}"
        return 1
    fi
}

# Build ROS1 plugin
build_plugin "axon-plugin-ros1" "${PROJECT_ROOT}/middlewares/ros1" || exit 1

# Summary
log_info "==================================================================="
log_info "ROS1 plugin build complete!"
log_info "==================================================================="
ls -lh "${OUTPUT_DIR}"/axon-plugin-ros1_*.deb 2>/dev/null || log_warn "No ROS1 plugin package found"
log_info "==================================================================="
