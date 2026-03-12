#!/bin/bash
# SPDX-FileCopyrightText: 2026 ArcheBase
# SPDX-License-Identifier: MulanPSL-2.0

set -e

# =============================================================================
# Axon ROS2 Plugin Package Build Script
# =============================================================================
# Builds Debian packages for ROS2 middleware plugin.
# Requires a sourced ROS2 environment.
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
OUTPUT_DIR="${PACKAGE_DIR}/output"

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

# Detect ROS_DISTRO from environment
ROS_DISTRO="${ROS_DISTRO:-}"
if [ -z "$ROS_DISTRO" ]; then
    if [ -n "$AMENT_PREFIX_PATH" ]; then
        ROS_DISTRO=$(echo "$AMENT_PREFIX_PATH" | grep -oP '/opt/ros/\K[^:]+' | head -1)
    fi
fi

if [ -z "$ROS_DISTRO" ]; then
    # Default to humble if not set
    ROS_DISTRO="humble"
fi

log_info "Building for ROS2 distro: ${ROS_DISTRO}"

# Validate ROS2 installation
if [ -d "/opt/ros/${ROS_DISTRO}" ]; then
    # Source ROS2 environment
    # shellcheck source=/dev/null
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
elif [ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    log_warn "ROS2 ${ROS_DISTRO} not found at /opt/ros/${ROS_DISTRO}"
    log_warn "Continuing anyway (Docker build may have pre-sourced environment)"
fi

# Create output directory
mkdir -p "${OUTPUT_DIR}"

# Get version from project
if [ -f "${PROJECT_ROOT}/apps/axon_recorder/CMakeLists.txt" ]; then
    VERSION="$(grep 'project.*VERSION' "${PROJECT_ROOT}/apps/axon_recorder/CMakeLists.txt" | sed 's/.*VERSION \([0-9.]*\).*/\1/')"
fi
if [ -z "$VERSION" ]; then
    VERSION="0.2.1"
fi
DEBIAN_VERSION="${VERSION}-1"

log_info "Building Axon ROS2 plugin packages version ${DEBIAN_VERSION}"

# Build function for plugins
build_plugin() {
    local pkg_name="$1"
    local source_dir="$2"
    local debian_dir="${PACKAGE_DIR}/${pkg_name}/debian"

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

    # Create temp directory with debian files
    local temp_dir="${source_dir}/.debian-build"
    rm -rf "${temp_dir}"
    cp -r "${debian_dir}" "${temp_dir}"

    # Update rules with ROS_DISTRO
    sed -i "s/ROS_DISTRO ?= humble/ROS_DISTRO ?= ${ROS_DISTRO}/" "${temp_dir}/rules" || true

    # Move to actual debian directory
    rm -rf "${source_dir}/debian"
    mv "${temp_dir}" "${source_dir}/debian"

    # Build from source directory
    cd "${source_dir}"
    if dpkg-buildpackage -b -uc -us -j"$(nproc)" 2>&1; then
        log_info "Successfully built ${pkg_name}"

        # Move built packages to output directory
        find .. -maxdepth 1 -name "${pkg_name}_*.deb" -exec mv {} "${OUTPUT_DIR}/" \; 2>/dev/null || true

        # Clean up
        rm -rf "${source_dir}/debian"
        cd "${original_dir}"

        return 0
    else
        log_error "Failed to build ${pkg_name}"
        rm -rf "${source_dir}/debian"
        cd "${original_dir}"
        return 1
    fi
}

# Build ROS2 plugin
build_plugin "axon-plugin-ros2" "${PROJECT_ROOT}/middlewares/ros2" || exit 1

# Summary
log_info "==================================================================="
log_info "ROS2 plugin build complete!"
log_info "==================================================================="
ls -lh "${OUTPUT_DIR}"/axon-plugin-ros2_*.deb 2>/dev/null || log_warn "No ROS2 plugin package found"
log_info "==================================================================="
