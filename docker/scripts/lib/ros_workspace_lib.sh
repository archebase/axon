#!/bin/bash
# =============================================================================
# ROS Workspace Library
# =============================================================================
# Shared functions for sourcing ROS workspaces and managing ROS environment.
# Uses unified CMake build system for all components.
#
# Functions:
#   ros_workspace_detect_ros_version - Auto-detect ROS version from ROS_DISTRO
#   ros_workspace_source_base - Source ROS base distribution
#   ros_workspace_source_workspace - Source workspace setup files
#   ros_workspace_verify_package - Verify package is available
# =============================================================================

set -eo pipefail

# =============================================================================
# Helper Functions
# =============================================================================

ros_workspace_log() {
    local level="$1"
    shift
    echo "[ros_workspace] $level: $*" >&2
}

ros_workspace_error() {
    ros_workspace_log "ERROR" "$@"
    exit 1
}

# =============================================================================
# Auto-detect ROS Version from ROS_DISTRO
# =============================================================================
#
# Auto-detects ROS_VERSION from ROS_DISTRO environment variable.
# This is a shared helper function to avoid code duplication.
#
# Environment:
#   ROS_DISTRO - ROS distribution (e.g., noetic, humble, jazzy, rolling)
#
# Outputs:
#   Prints ROS version "1" or "2" to stdout
#   Returns 0 if detection successful, 1 if ROS_DISTRO not set or unknown
#
ros_workspace_detect_ros_version() {
    local ros_distro="${ROS_DISTRO:-}"
    
    if [ -z "$ros_distro" ]; then
        return 1
    fi
    
    case "$ros_distro" in
        noetic|melodic|kinetic)
            echo "1"
            return 0
            ;;
        *)
            echo "2"
            return 0
            ;;
    esac
}

# =============================================================================
# Source ROS Base Distribution
# =============================================================================
#
# Sources the ROS base distribution setup file.
#
# Arguments:
#   ROS_DISTRO - ROS distribution (e.g., noetic, humble)
#
# Returns:
#   0 if sourced successfully
#   1 if ROS distribution not found
#
ros_workspace_source_base() {
    local ros_distro="${ROS_DISTRO:-}"
    
    if [ -z "$ros_distro" ]; then
        ros_workspace_log "WARN" "ROS_DISTRO not set, skipping base ROS sourcing"
        return 1
    fi
    
    local setup_file="/opt/ros/${ros_distro}/setup.bash"
    if [ -f "$setup_file" ]; then
        # Suppress errors during sourcing (may already be sourced)
        set +e
        source "$setup_file" 2>/dev/null
        set -e
        ros_workspace_log "INFO" "Sourced ROS base: $setup_file"
        return 0
    else
        ros_workspace_log "WARN" "ROS base setup file not found: $setup_file"
        return 1
    fi
}

# =============================================================================
# Source Workspace Setup Files
# =============================================================================
#
# Sources the workspace setup file, checking the unified build directory.
#
# Arguments:
#   ROS_VERSION - ROS version (1 or 2, optional - auto-detected if not set)
#   WORKSPACE_ROOT - Root of the workspace (default: /workspace/axon)
#
# Environment:
#   Sets WORKSPACE_INSTALL_DIR - Directory where workspace is installed
#   For ROS 2: Sets AMENT_PREFIX_PATH appropriately
#
# Returns:
#   0 if workspace was sourced successfully
#   1 if workspace was not found
#
ros_workspace_source_workspace() {
    local ros_version="${ROS_VERSION:-}"
    local workspace_root="${1:-/workspace/axon}"

    # Auto-detect ROS version from ROS_DISTRO if not set
    if [ -z "$ros_version" ] && [ -n "${ROS_DISTRO:-}" ]; then
        ros_version=$(ros_workspace_detect_ros_version)
        if [ -n "$ros_version" ]; then
            ros_workspace_log "INFO" "Auto-detected ROS_VERSION=$ros_version from ROS_DISTRO=${ROS_DISTRO}"
        fi
    fi

    if [ -z "$ros_version" ]; then
        ros_workspace_error "ROS_VERSION must be set or ROS_DISTRO must be set for auto-detection"
    fi

    # Check unified build directory
    local build_dir="${workspace_root}/build"

    # Check if build directory exists
    if [ ! -d "$build_dir" ]; then
        ros_workspace_log "WARN" "Build directory not found: $build_dir"
        return 1
    fi

    # For unified CMake build, the build directory is the workspace
    export WORKSPACE_INSTALL_DIR="$build_dir"
    ros_workspace_log "INFO" "Using unified build directory: $build_dir"

    # For ROS 2: Set AMENT_PREFIX_PATH to include build directory
    if [ "$ros_version" = "2" ]; then
        export AMENT_PREFIX_PATH="${build_dir}:${AMENT_PREFIX_PATH:-}"
        ros_workspace_log "INFO" "Set AMENT_PREFIX_PATH to include $build_dir"
    fi

    return 0
}

# =============================================================================
# Verify Package Availability
# =============================================================================
#
# Verifies that the axon_ros2_plugin package is available in the ROS environment.
#
# Arguments:
#   ROS_VERSION - ROS version (1 or 2)
#
# Returns:
#   0 if package is available
#   1 if package is not available
#
ros_workspace_verify_package() {
    local ros_version="${ROS_VERSION:-}"

    if [ -z "$ros_version" ]; then
        ros_workspace_error "ROS_VERSION must be set"
    fi

    ros_workspace_log "INFO" "Verifying axon_ros2_plugin package availability..."

    if [ "$ros_version" = "2" ]; then
        # ROS 2 - Check via ros2 pkg list
        if ros2 pkg list 2>/dev/null | grep -q "^axon_ros2_plugin$"; then
            ros_workspace_log "INFO" "✓ axon_ros2_plugin found in ros2 pkg list"
            return 0
        else
            ros_workspace_log "WARN" "axon_ros2_plugin not found in ros2 pkg list"
            # Show first 30 packages for debugging
            ros_workspace_log "INFO" "First 30 packages:"
            ros2 pkg list 2>/dev/null | head -30 || true
            return 1
        fi
    else
        # ROS 1 - Check via rospack
        if rospack find axon_ros1_plugin &>/dev/null; then
            ros_workspace_log "INFO" "✓ axon_ros1_plugin found via rospack"
            return 0
        else
            ros_workspace_log "WARN" "axon_ros1_plugin not found via rospack"
            return 1
        fi
    fi
}

# Export functions for use in other scripts
export -f ros_workspace_detect_ros_version
export -f ros_workspace_source_base
export -f ros_workspace_source_workspace
export -f ros_workspace_verify_package
export -f ros_workspace_log
export -f ros_workspace_error

