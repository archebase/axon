#!/bin/bash
# =============================================================================
# ROS Diagnostics Library
# =============================================================================
# Shared functions for diagnosing ROS environment issues for both ROS1 and ROS2.
# Especially useful for ROS2 service discovery problems (e.g., missing FastDDS
# libraries in Rolling) and ROS1 roscore/node discovery issues.
#
# Functions:
#   ros_diagnostics_check_rmw - Check RMW implementation and libraries (ROS2 only)
#   ros_diagnostics_check_fastdds - Check FastDDS library availability (ROS2 only)
#   ros_diagnostics_check_services - Check ROS1/ROS2 service discovery
#   ros_diagnostics_check_node - Check if a node is running and discoverable (ROS1/ROS2)
#   ros_diagnostics_full - Run all diagnostic checks (ROS1/ROS2)
# =============================================================================

set -eo pipefail

# =============================================================================
# Helper Functions
# =============================================================================

ros_diagnostics_log() {
    local level="$1"
    shift
    echo "[ros_diagnostics] $level: $*" >&2
}

# =============================================================================
# Check RMW Implementation
# =============================================================================
#
# Checks which RMW implementation is configured and if required libraries exist.
#
# Arguments:
#   ROS_VERSION - ROS version (1 or 2, defaults to 2)
#
# Returns:
#   0 if RMW is properly configured
#   1 if there are issues
#
ros_diagnostics_check_rmw() {
    local ros_version="${ROS_VERSION:-2}"
    
    if [ "$ros_version" != "2" ]; then
        ros_diagnostics_log "INFO" "RMW check skipped (ROS 1)"
        return 0
    fi
    
    ros_diagnostics_log "INFO" "Checking RMW implementation..."
    
    # Check RMW implementation type
    local rmw_impl="${RMW_IMPLEMENTATION:-}"
    if [ -z "$rmw_impl" ]; then
        ros_diagnostics_log "WARN" "RMW_IMPLEMENTATION not set (will use default)"
        # Try to detect from environment
        if [ -n "${ROS_DISTRO:-}" ]; then
            case "${ROS_DISTRO}" in
                rolling)
                    rmw_impl="rmw_fastrtps_cpp"
                    ros_diagnostics_log "INFO" "Detected ROS Rolling, default RMW should be: $rmw_impl"
                    ;;
                jazzy|humble)
                    rmw_impl="rmw_fastrtps_cpp"
                    ros_diagnostics_log "INFO" "Detected ROS ${ROS_DISTRO}, default RMW should be: $rmw_impl"
                    ;;
                *)
                    ros_diagnostics_log "WARN" "Unknown ROS distro: ${ROS_DISTRO}"
                    ;;
            esac
        fi
    else
        ros_diagnostics_log "INFO" "RMW_IMPLEMENTATION is set to: $rmw_impl"
    fi
    
    # Check if RMW package is available
    if [ -n "$rmw_impl" ]; then
        ros_diagnostics_log "INFO" "Checking for RMW package: $rmw_impl"
        if ros2 pkg list 2>/dev/null | grep -q "^${rmw_impl}$"; then
            ros_diagnostics_log "INFO" "✓ RMW package found: $rmw_impl"
        else
            ros_diagnostics_log "WARN" "✗ RMW package not found: $rmw_impl"
            ros_diagnostics_log "INFO" "Available RMW packages:"
            ros2 pkg list 2>/dev/null | grep "^rmw_" || echo "  (none found)"
            return 1
        fi
    fi
    
    return 0
}

# =============================================================================
# Check FastDDS Libraries
# =============================================================================
#
# Checks if FastDDS libraries are available (required for rmw_fastrtps_cpp).
#
# Arguments:
#   None
#
# Returns:
#   0 if FastDDS libraries are found
#   1 if libraries are missing
#
ros_diagnostics_check_fastdds() {
    ros_diagnostics_log "INFO" "Checking FastDDS libraries..."
    
    local missing_libs=()
    local found_libs=()
    
    # Required FastDDS library names to check
    local libs_to_check=(
        "libfastdds"
        "libfastcdr"
    )
    
    # Check in common library paths (including ROS install directory)
    local lib_paths=(
        "/opt/ros/${ROS_DISTRO:-humble}/lib"
        "/usr/lib"
        "/usr/lib/x86_64-linux-gnu"
        "/usr/lib/aarch64-linux-gnu"
    )
    
    for lib_name in "${libs_to_check[@]}"; do
        local found=false
        for lib_path in "${lib_paths[@]}"; do
            if [ -d "$lib_path" ]; then
                if find "$lib_path" -name "${lib_name}*.so*" 2>/dev/null | head -1 | grep -q .; then
                    found_libs+=("$lib_name (in $lib_path)")
                    found=true
                    break
                fi
            fi
        done
        
        if [ "$found" = false ]; then
            missing_libs+=("$lib_name")
        fi
    done
    
    # Report results
    if [ ${#found_libs[@]} -gt 0 ]; then
        ros_diagnostics_log "INFO" "Found FastDDS libraries:"
        for lib in "${found_libs[@]}"; do
            ros_diagnostics_log "INFO" "  ✓ $lib"
        done
    fi
    
    if [ ${#missing_libs[@]} -gt 0 ]; then
        ros_diagnostics_log "WARN" "Missing FastDDS libraries:"
        for lib in "${missing_libs[@]}"; do
            ros_diagnostics_log "WARN" "  ✗ $lib"
        done
        
        # Check if we can find them via ldconfig
        ros_diagnostics_log "INFO" "Checking ldconfig cache..."
        for lib in "${missing_libs[@]}"; do
            if ldconfig -p 2>/dev/null | grep -q "$lib"; then
                ros_diagnostics_log "INFO" "  → $lib found in ldconfig cache"
                ldconfig -p 2>/dev/null | grep "$lib" | head -3
            else
                ros_diagnostics_log "WARN" "  → $lib not in ldconfig cache"
            fi
        done
        
        # Check RMW library for symbol issues
        ros_diagnostics_log "INFO" "Checking RMW library compatibility..."
        local rmw_lib="/opt/ros/${ROS_DISTRO:-humble}/lib/librmw_fastrtps_cpp.so"
        if [ -f "$rmw_lib" ]; then
            ros_diagnostics_log "INFO" "RMW library found: $rmw_lib"
            # Try to check if library can be loaded (basic sanity check)
            if ldd "$rmw_lib" 2>/dev/null | head -5; then
                ros_diagnostics_log "INFO" "RMW library dependencies:"
                ldd "$rmw_lib" 2>/dev/null | grep -E "(fastrtps|fastdds|fastcdr)" || true
            fi
        fi
        
        return 1
    fi
    
    ros_diagnostics_log "INFO" "✓ All FastDDS libraries found"
    return 0
}

# =============================================================================
# Check ROS Service Discovery
# =============================================================================
#
# Checks if ROS1 or ROS2 service discovery is working properly.
# For ROS1: Checks roscore, uses rosnode/rosservice
# For ROS2: Uses ros2 node/service commands
#
# Arguments:
#   node_name - Name of the node to check (optional)
#   service_name - Name of the service to check (optional)
#
# Returns:
#   0 if service discovery is working
#   1 if there are issues
#
ros_diagnostics_check_services() {
    local node_name="${1:-}"
    local service_name="${2:-}"
    
    local ros_version="${ROS_VERSION:-2}"
    
    if [ "$ros_version" = "1" ]; then
        ros_diagnostics_log "INFO" "Checking ROS1 service discovery..."
        
        # Check if roscore is running
        if ! pgrep -x roscore > /dev/null; then
            ros_diagnostics_log "WARN" "roscore is not running (required for ROS1)"
            return 1
        else
            ros_diagnostics_log "INFO" "✓ roscore is running"
        fi
        
        # Check if rosnode command is available
        if ! command -v rosnode &> /dev/null; then
            ros_diagnostics_log "ERROR" "rosnode command not found"
            return 1
        fi
        
        # List all nodes
        ros_diagnostics_log "INFO" "Discoverable nodes:"
        local nodes
        nodes=$(rosnode list 2>&1)
        if [ $? -eq 0 ] && [ -n "$nodes" ]; then
            echo "$nodes" | while read -r node; do
                ros_diagnostics_log "INFO" "  → $node"
            done
        else
            ros_diagnostics_log "WARN" "  (no nodes found or error listing nodes)"
            echo "$nodes" >&2
        fi
        
        # Check specific node if provided
        if [ -n "$node_name" ]; then
            ros_diagnostics_log "INFO" "Checking for node: $node_name"
            if echo "$nodes" | grep -q "/${node_name}$\|/${node_name}\["; then
                ros_diagnostics_log "INFO" "✓ Node found: $node_name"
            else
                ros_diagnostics_log "WARN" "✗ Node not found: $node_name"
                ros_diagnostics_log "INFO" "Available nodes:"
                echo "$nodes" | while read -r node; do
                    ros_diagnostics_log "INFO" "  $node"
                done
            fi
        fi
        
        # List all services
        ros_diagnostics_log "INFO" "Discoverable services:"
        local services
        services=$(rosservice list 2>&1)
        if [ $? -eq 0 ] && [ -n "$services" ]; then
            echo "$services" | while read -r service; do
                ros_diagnostics_log "INFO" "  → $service"
            done
        else
            ros_diagnostics_log "WARN" "  (no services found or error listing services)"
            echo "$services" >&2
        fi
        
        # Check specific service if provided
        if [ -n "$service_name" ]; then
            ros_diagnostics_log "INFO" "Checking for service: $service_name"
            if echo "$services" | grep -q "/${service_name}$\|/${service_name}\["; then
                ros_diagnostics_log "INFO" "✓ Service found: $service_name"
                return 0
            else
                ros_diagnostics_log "WARN" "✗ Service not found: $service_name"
                ros_diagnostics_log "INFO" "Available services:"
                echo "$services" | while read -r service; do
                    ros_diagnostics_log "INFO" "  $service"
                done
                return 1
            fi
        fi
        
        return 0
    else
        # ROS 2
        ros_diagnostics_log "INFO" "Checking ROS2 service discovery..."
        
        # Check if ros2 command is available
        if ! command -v ros2 &> /dev/null; then
            ros_diagnostics_log "ERROR" "ros2 command not found"
            return 1
        fi
        
        # List all nodes
        ros_diagnostics_log "INFO" "Discoverable nodes:"
        local nodes
        nodes=$(ros2 node list 2>&1)
        if [ $? -eq 0 ] && [ -n "$nodes" ]; then
            echo "$nodes" | while read -r node; do
                ros_diagnostics_log "INFO" "  → $node"
            done
        else
            ros_diagnostics_log "WARN" "  (no nodes found or error listing nodes)"
            echo "$nodes" >&2
        fi
        
        # Check specific node if provided
        if [ -n "$node_name" ]; then
            ros_diagnostics_log "INFO" "Checking for node: $node_name"
            if echo "$nodes" | grep -q "/${node_name}$\|/${node_name}\["; then
                ros_diagnostics_log "INFO" "✓ Node found: $node_name"
            else
                ros_diagnostics_log "WARN" "✗ Node not found: $node_name"
                ros_diagnostics_log "INFO" "Available nodes:"
                echo "$nodes" | while read -r node; do
                    ros_diagnostics_log "INFO" "  $node"
                done
            fi
        fi
        
        # List all services
        ros_diagnostics_log "INFO" "Discoverable services:"
        local services
        services=$(ros2 service list 2>&1)
        if [ $? -eq 0 ] && [ -n "$services" ]; then
            echo "$services" | while read -r service; do
                ros_diagnostics_log "INFO" "  → $service"
            done
        else
            ros_diagnostics_log "WARN" "  (no services found or error listing services)"
            echo "$services" >&2
        fi
        
        # Check specific service if provided
        if [ -n "$service_name" ]; then
            ros_diagnostics_log "INFO" "Checking for service: $service_name"
            if echo "$services" | grep -q "/${service_name}$\|/${service_name}\["; then
                ros_diagnostics_log "INFO" "✓ Service found: $service_name"
                return 0
            else
                ros_diagnostics_log "WARN" "✗ Service not found: $service_name"
                ros_diagnostics_log "INFO" "Available services:"
                echo "$services" | while read -r service; do
                    ros_diagnostics_log "INFO" "  $service"
                done
                return 1
            fi
        fi
        
        return 0
    fi
}

# =============================================================================
# Check Node Status
# =============================================================================
#
# Checks if a specific node is running and discoverable.
# For ROS1: Uses rosnode list/info (requires roscore)
# For ROS2: Uses ros2 node list/info
#
# Arguments:
#   node_name - Name of the node to check
#   timeout_sec - Timeout in seconds (default: 5)
#
# Returns:
#   0 if node is found
#   1 if node is not found
#
ros_diagnostics_check_node() {
    local node_name="$1"
    local timeout_sec="${2:-5}"
    
    local ros_version="${ROS_VERSION:-2}"
    
    if [ -z "$node_name" ]; then
        ros_diagnostics_log "ERROR" "Node name required"
        return 1
    fi
    
    ros_diagnostics_log "INFO" "Checking node status: $node_name (timeout: ${timeout_sec}s)"
    
    if [ "$ros_version" = "1" ]; then
        # ROS 1 - Check if roscore is running first
        if ! pgrep -x roscore > /dev/null; then
            ros_diagnostics_log "WARN" "roscore is not running (required for ROS1 node discovery)"
            return 1
        fi
        
        local found=false
        local elapsed=0
        while [ $elapsed -lt $timeout_sec ]; do
            if rosnode list 2>/dev/null | grep -q "/${node_name}$\|/${node_name}\["; then
                found=true
                break
            fi
            sleep 1
            elapsed=$((elapsed + 1))
        done
        
        if [ "$found" = true ]; then
            ros_diagnostics_log "INFO" "✓ Node is discoverable: $node_name"
            
            # Get node info
            ros_diagnostics_log "INFO" "Node info:"
            rosnode info "/${node_name}" 2>&1 | head -20 || true
            
            # Get node machine/uri
            ros_diagnostics_log "INFO" "Node machine/uri:"
            rosnode machine "/${node_name}" 2>&1 || true
            
            return 0
        else
            ros_diagnostics_log "WARN" "✗ Node not discoverable after ${timeout_sec}s: $node_name"
            ros_diagnostics_log "INFO" "Available nodes:"
            rosnode list 2>&1 | while read -r node; do
                ros_diagnostics_log "INFO" "  $node"
            done
            return 1
        fi
    else
        # ROS 2
        local found=false
        local elapsed=0
        while [ $elapsed -lt $timeout_sec ]; do
            if ros2 node list 2>/dev/null | grep -q "/${node_name}$\|/${node_name}\["; then
                found=true
                break
            fi
            sleep 1
            elapsed=$((elapsed + 1))
        done
        
        if [ "$found" = true ]; then
            ros_diagnostics_log "INFO" "✓ Node is discoverable: $node_name"
            
            # Get node info
            ros_diagnostics_log "INFO" "Node info:"
            ros2 node info "/${node_name}" 2>&1 | head -20 || true
            
            return 0
        else
            ros_diagnostics_log "WARN" "✗ Node not discoverable after ${timeout_sec}s: $node_name"
            return 1
        fi
    fi
}

# =============================================================================
# Full Diagnostic Check
# =============================================================================
#
# Runs all diagnostic checks for ROS environment.
#
# Arguments:
#   node_name - Optional node name to check
#   service_name - Optional service name to check
#
# Returns:
#   0 if all checks pass
#   1 if any check fails
#
ros_diagnostics_full() {
    local node_name="${1:-}"
    local service_name="${2:-}"
    
    local ros_version="${ROS_VERSION:-2}"
    
    echo ""
    echo "============================================"
    echo "ROS Diagnostics"
    echo "============================================"
    echo "ROS Version: ${ros_version}"
    echo "ROS Distro: ${ROS_DISTRO:-unknown}"
    echo "RMW Implementation: ${RMW_IMPLEMENTATION:-default}"
    echo ""
    
    local all_passed=true
    
    # Check RMW (only for ROS2)
    if [ "$ros_version" = "2" ]; then
        if ! ros_diagnostics_check_rmw; then
            all_passed=false
        fi
        echo ""
        
        # Check FastDDS (only for ROS2)
        if ! ros_diagnostics_check_fastdds; then
            all_passed=false
        fi
        echo ""
        
        # Network/Domain ID check (ROS2)
        ros_diagnostics_log "INFO" "ROS2 Domain ID: ${ROS_DOMAIN_ID:-0}"
        ros_diagnostics_log "INFO" "ROS2 Localhost only: ${ROS_LOCALHOST_ONLY:-false}"
        echo ""
    else
        # ROS1 specific checks
        ros_diagnostics_log "INFO" "Checking ROS1 environment..."
        
        # Check if roscore is running
        if pgrep -x roscore > /dev/null; then
            ros_diagnostics_log "INFO" "✓ roscore is running"
        else
            ros_diagnostics_log "WARN" "✗ roscore is not running (required for ROS1)"
            all_passed=false
        fi
        
        # Check ROS_MASTER_URI
        if [ -n "${ROS_MASTER_URI:-}" ]; then
            ros_diagnostics_log "INFO" "ROS_MASTER_URI: ${ROS_MASTER_URI}"
        else
            ros_diagnostics_log "WARN" "ROS_MASTER_URI not set"
            all_passed=false
        fi
        
        echo ""
    fi
    
    # Check service discovery (works for both ROS1 and ROS2)
    if ! ros_diagnostics_check_services "$node_name" "$service_name"; then
        all_passed=false
    fi
    echo ""
    
    # Check specific node if provided (works for both ROS1 and ROS2)
    if [ -n "$node_name" ]; then
        if ! ros_diagnostics_check_node "$node_name" 5; then
            all_passed=false
        fi
        echo ""
    fi
    
    echo "============================================"
    if [ "$all_passed" = true ]; then
        ros_diagnostics_log "INFO" "All diagnostic checks passed"
        return 0
    else
        ros_diagnostics_log "WARN" "Some diagnostic checks failed"
        return 1
    fi
}

# Export functions for use in other scripts
export -f ros_diagnostics_check_rmw
export -f ros_diagnostics_check_fastdds
export -f ros_diagnostics_check_services
export -f ros_diagnostics_check_node
export -f ros_diagnostics_full
export -f ros_diagnostics_log

