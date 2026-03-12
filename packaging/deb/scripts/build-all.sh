#!/bin/bash
# SPDX-FileCopyrightText: 2026 ArcheBase
# SPDX-License-Identifier: MulanPSL-2.0

set -e

# =============================================================================
# Axon Complete Package Build Script
# =============================================================================
# Builds all Debian packages applicable to the current environment.
# Standalone packages are always built.
# ROS plugins are built if the corresponding ROS environment is available.
# =============================================================================

# Detect if running in Docker
if [ -d "/axon" ] && [ -f "/axon/CMakeLists.txt" ]; then
    PROJECT_ROOT="/axon"
    SCRIPT_DIR="/axon/packaging/deb/scripts"
else
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    PROJECT_ROOT="$(cd "$(dirname "${SCRIPT_DIR}")/../.." && pwd)"
fi

OUTPUT_DIR="${PROJECT_ROOT}/packaging/deb/output"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
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

log_section() {
    echo -e "${BLUE}=== $1 ===${NC}"
}

# =============================================================================
# Detect Available Environments
# =============================================================================

has_ros2_humble=0
has_ros2_jazzy=0
has_ros2_rolling=0
has_ros1_noetic=0

# Check ROS1 Noetic
if [ -d "/opt/ros/noetic" ]; then
    has_ros1_noetic=1
    log_info "Detected: ROS1 Noetic"
fi

# Check ROS2 distributions
for distro in humble jazzy rolling; do
    if [ -d "/opt/ros/$distro" ]; then
        eval "has_ros2_${distro}=1"
        log_info "Detected: ROS2 ${distro^}"
    fi
done

if [ "$has_ros2_humble" -eq 0 ] && [ "$has_ros2_jazzy" -eq 0 ] && [ "$has_ros2_rolling" -eq 0 ]; then
    # Check via environment variables
    if [ -n "$ROS_DISTRO" ]; then
        eval "has_ros2_${ROS_DISTRO}=1"
        log_info "Detected: ROS2 ${ROS_DISTRO^} (from environment)"
    fi
fi

# =============================================================================
# Build Standalone Packages
# =============================================================================

log_section "Building Standalone Packages"

if "${SCRIPT_DIR}/build-standalone.sh"; then
    log_info "Standalone packages built successfully"
else
    log_error "Failed to build standalone packages"
    exit 1
fi

# =============================================================================
# Build ROS Plugin Packages
# =============================================================================

# Build ROS1 Noetic plugin
if [ "$has_ros1_noetic" -eq 1 ]; then
    log_section "Building ROS1 Noetic Plugin"

    # Temporarily source ROS1 environment
    # shellcheck source=/dev/null
    if source /opt/ros/noetic/setup.bash 2>/dev/null; then
        if "${SCRIPT_DIR}/build-ros1.sh"; then
            log_info "ROS1 plugin built successfully"
        else
            log_warn "Failed to build ROS1 plugin (continuing)"
        fi
    else
        log_warn "Could not source ROS1 Noetic environment (skipping)"
    fi
else
    log_info "ROS1 Noetic not detected, skipping ROS1 plugin"
fi

# Build ROS2 plugins (only one at a time based on current environment)
if [ -n "$ROS_DISTRO" ] || [ "$has_ros2_humble" -eq 1 ] || [ "$has_ros2_jazzy" -eq 1 ] || [ "$has_ros2_rolling" -eq 1 ]; then
    # Use currently sourced ROS2 environment or default to humble
    active_ros_distro="${ROS_DISTRO:-humble}"

    # Verify the distro is available
    if [ -d "/opt/ros/$active_ros_distro" ]; then
        log_section "Building ROS2 ${active_ros_distro^} Plugin"

        # Source the environment if not already sourced
        if [ -z "$ROS_DISTRO" ]; then
            # shellcheck source=/dev/null
            source "/opt/ros/${active_ros_distro}/setup.bash"
        fi

        if ROS_DISTRO="$active_ros_distro" "${SCRIPT_DIR}/build-ros2.sh"; then
            log_info "ROS2 ${active_ros_distro^} plugin built successfully"
        else
            log_warn "Failed to build ROS2 ${active_ros_distro^} plugin (continuing)"
        fi
    else
        log_warn "ROS2 ${active_ros_distro} not available, skipping ROS2 plugin"
    fi
else
    log_info "ROS2 not detected, skipping ROS2 plugin"
    log_info "To build ROS2 packages, source a ROS2 environment first:"
    log_info "  source /opt/ros/humble/setup.bash"
fi

# =============================================================================
# Build Standalone Plugins
# =============================================================================

log_section "Building Standalone Plugins"

SCRIPT_DIR_SAVE="$(pwd)"

# Check if middlewares directory exists
if [ -d "${PROJECT_ROOT}/middlewares/udp" ]; then
    log_info "Building UDP plugin..."
    if "${SCRIPT_DIR}/build-plugin.sh" "axon-plugin-udp" "${PROJECT_ROOT}/middlewares/udp"; then
        log_info "UDP plugin built successfully"
    else
        log_warn "Failed to build UDP plugin (continuing)"
    fi
fi

if [ -d "${PROJECT_ROOT}/middlewares/zenoh" ]; then
    log_info "Building Zenoh plugin..."
    if pkg-config --exists libzenohc 2>/dev/null; then
        if "${SCRIPT_DIR}/build-plugin.sh" "axon-plugin-zenoh" "${PROJECT_ROOT}/middlewares/zenoh"; then
            log_info "Zenoh plugin built successfully"
        else
            log_warn "Failed to build Zenoh plugin (continuing)"
        fi
    else
        log_info "libzenohc not found, skipping Zenoh plugin"
    fi
fi

cd "$SCRIPT_DIR_SAVE"

# =============================================================================
# Summary
# =============================================================================

log_section "Build Complete"

if [ -d "$OUTPUT_DIR" ]; then
    log_info "Packages created:"
    ls -lh "${OUTPUT_DIR}"/*.deb 2>/dev/null || log_warn "No packages found"
fi

log_info "==================================================================="
log_info "All applicable packages have been built!"
log_info "==================================================================="
