#!/bin/bash
# =============================================================================
# ROS Workspace Library
# =============================================================================
# Shared functions for sourcing ROS workspaces and managing ROS environment.
# Handles both ROS 1 (catkin) and ROS 2 (colcon) workspace paths.
#
# Functions:
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
# Sources the workspace setup file, checking multiple possible locations.
# Handles both ROS 1 (catkin devel) and ROS 2 (colcon install) workspaces.
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
        case "${ROS_DISTRO}" in
            noetic|melodic|kinetic)
                ros_version=1
                ;;
            *)
                ros_version=2
                ;;
        esac
        ros_workspace_log "INFO" "Auto-detected ROS_VERSION=$ros_version from ROS_DISTRO=${ROS_DISTRO}"
    fi
    
    if [ -z "$ros_version" ]; then
        ros_workspace_error "ROS_VERSION must be set or ROS_DISTRO must be set for auto-detection"
    fi
    
    # List of workspace paths to check (in order of preference)
    local workspace_paths=()
    
    if [ "$ros_version" = "1" ]; then
        # ROS 1 - Check catkin devel directories
        workspace_paths=(
            "/root/target_ws/devel"
            "${workspace_root}/ros/devel"
            "${workspace_root}/devel"
            "/workspace/catkin_ws/devel"
        )
    else
        # ROS 2 - Check colcon install directories
        workspace_paths=(
            "/root/target_ws/install"
            "${workspace_root}/ros/install"
            "${workspace_root}/install"
        )
    fi
    
    # Try to source workspace setup file
    for ws_path in "${workspace_paths[@]}"; do
        local setup_file="${ws_path}/setup.bash"
        if [ -f "$setup_file" ]; then
            # Temporarily disable error handling for sourcing
            set +e
            source "$setup_file" 2>/dev/null
            local source_result=$?
            set -e
            
            if [ $source_result -eq 0 ]; then
                export WORKSPACE_INSTALL_DIR="$ws_path"
                ros_workspace_log "INFO" "Sourced workspace: $setup_file"
                
                # For ROS 2: Ensure workspace is in AMENT_PREFIX_PATH
                if [ "$ros_version" = "2" ]; then
                    local axon_recorder_path="${ws_path}/axon_recorder"
                    if [ -d "$axon_recorder_path" ]; then
                        export AMENT_PREFIX_PATH="${axon_recorder_path}:${ws_path}:${AMENT_PREFIX_PATH:-}"
                    else
                        export AMENT_PREFIX_PATH="${ws_path}:${AMENT_PREFIX_PATH:-}"
                    fi
                fi
                
                return 0
            fi
        fi
    done
    
    ros_workspace_log "WARN" "Workspace setup file not found in any expected location"
    ros_workspace_log "WARN" "Checked paths: ${workspace_paths[*]}"
    return 1
}

# =============================================================================
# Verify Package Availability
# =============================================================================
#
# Verifies that the axon_recorder package is available in the ROS environment.
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
    
    ros_workspace_log "INFO" "Verifying axon_recorder package availability..."
    
    if [ "$ros_version" = "2" ]; then
        # ROS 2 - Check via ros2 pkg list
        if ros2 pkg list 2>/dev/null | grep -q "^axon_recorder$"; then
            ros_workspace_log "INFO" "✓ axon_recorder found in ros2 pkg list"
            return 0
        else
            ros_workspace_log "WARN" "axon_recorder not found in ros2 pkg list"
            # Show first 30 packages for debugging
            ros_workspace_log "INFO" "First 30 packages:"
            ros2 pkg list 2>/dev/null | head -30 || true
            return 1
        fi
    else
        # ROS 1 - Check via rospack
        if rospack find axon_recorder &>/dev/null; then
            ros_workspace_log "INFO" "✓ axon_recorder found via rospack"
            return 0
        else
            ros_workspace_log "WARN" "axon_recorder not found via rospack"
            return 1
        fi
    fi
}

# Export functions for use in other scripts
export -f ros_workspace_source_base
export -f ros_workspace_source_workspace
export -f ros_workspace_verify_package
export -f ros_workspace_log
export -f ros_workspace_error

