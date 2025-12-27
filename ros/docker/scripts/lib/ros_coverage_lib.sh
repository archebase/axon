#!/bin/bash
# =============================================================================
# ROS Coverage Library
# =============================================================================
# Shared functions for collecting and processing code coverage data.
# Handles lcov version differences and coverage report generation.
#
# Functions:
#   ros_coverage_check_lcov - Check if lcov is available, install if needed
#   ros_coverage_capture - Capture coverage data from build directory
#   ros_coverage_filter - Filter coverage data to remove external dependencies
#   ros_coverage_generate - Generate complete coverage report
# =============================================================================

set -eo pipefail

# =============================================================================
# Helper Functions
# =============================================================================

ros_coverage_log() {
    local level="$1"
    shift
    echo "[ros_coverage] $level: $*" >&2
}

ros_coverage_error() {
    ros_coverage_log "ERROR" "$@"
    exit 1
}

# =============================================================================
# Check and Install lcov
# =============================================================================

ros_coverage_check_lcov() {
    if command -v lcov &> /dev/null; then
        ros_coverage_log "INFO" "lcov is available"
        return 0
    fi
    
    ros_coverage_log "INFO" "Installing lcov..."
    apt-get update && apt-get install -y lcov || {
        ros_coverage_error "Failed to install lcov"
    }
    ros_coverage_log "INFO" "lcov installed successfully"
}

# =============================================================================
# Get lcov Version and Flags
# =============================================================================

ros_coverage_get_lcov_version() {
    local lcov_version
    # Try multiple patterns to match different lcov version output formats
    # Format 1: "LCOV version 2.0.0"
    # Format 2: "lcov 2.0.0"
    # Format 3: "lcov: LCOV version 2.0.0"
    lcov_version=$(lcov --version 2>&1 | grep -oE '[0-9]+\.[0-9]+(\.[0-9]+)?' | head -1 || echo "1.0")
    echo "$lcov_version"
}

ros_coverage_get_lcov_major() {
    local lcov_version
    lcov_version=$(ros_coverage_get_lcov_version)
    echo "$lcov_version" | cut -d. -f1
}

ros_coverage_get_branch_coverage_rc() {
    # Returns the appropriate RC option for branch coverage based on lcov version
    # Newer versions (2.0+) prefer 'branch_coverage', older versions use 'lcov_branch_coverage'
    # We use the new option for versions that support it to avoid deprecation warnings
    local lcov_major
    lcov_major=$(ros_coverage_get_lcov_major)
    
    # lcov 2.0+ supports the new 'branch_coverage' option
    # For older versions, use the deprecated but still supported 'lcov_branch_coverage'
    if [ "$lcov_major" -ge 2 ]; then
        echo "--rc branch_coverage=1"
    else
        echo "--rc lcov_branch_coverage=1"
    fi
}

ros_coverage_get_lcov_flags() {
    local lcov_version
    local lcov_major
    
    # Get lcov version
    lcov_version=$(ros_coverage_get_lcov_version)
    lcov_major=$(ros_coverage_get_lcov_major)
    
    ros_coverage_log "INFO" "Detected lcov version: $lcov_version (major: $lcov_major)"
    
    # Build ignore flags based on version
    # Error types to ignore:
    #   mismatch - mismatched exception tags (C++ std lib differences)
    #   unused   - unused coverage data (suppresses warnings about unused exclude patterns)
    #   negative - negative branch counts (compiler/gcov compatibility issue)
    #   gcov     - unexecuted blocks on non-branch lines
    local ignore_flags=""
    if [ "$lcov_major" -ge 2 ]; then
        ignore_flags="--ignore-errors mismatch,unused,negative,gcov"
        ros_coverage_log "INFO" "Using lcov 2.0+ ignore flags: $ignore_flags"
    else
        ros_coverage_log "WARN" "lcov version < 2.0, ignore flags not available"
    fi
    
    echo "$ignore_flags"
}

# =============================================================================
# Capture Coverage Data
# =============================================================================

ros_coverage_capture() {
    local coverage_dir="$1"
    local output_file="$2"
    
    if [ -z "$coverage_dir" ] || [ -z "$output_file" ]; then
        ros_coverage_error "Usage: ros_coverage_capture <coverage_dir> <output_file>"
    fi
    
    ros_coverage_log "INFO" "Searching for coverage data in $coverage_dir..."
    
    # Find all directories containing .gcda files
    local gcda_dirs
    gcda_dirs=$(find "$coverage_dir" -name "*.gcda" -type f -exec dirname {} \; 2>/dev/null | sort -u)
    
    if [ -z "$gcda_dirs" ]; then
        ros_coverage_error "No .gcda files found in $coverage_dir! This may happen if tests didn't execute any instrumented code."
    fi
    
    ros_coverage_log "INFO" "Found .gcda files in directories:"
    echo "$gcda_dirs" | head -10 | while read -r dir; do
        ros_coverage_log "INFO" "  $dir"
    done
    
    # Get lcov flags and branch coverage RC option
    local ignore_flags
    ignore_flags=$(ros_coverage_get_lcov_flags)
    local branch_coverage_rc
    branch_coverage_rc=$(ros_coverage_get_branch_coverage_rc)
    
    # Capture coverage data
    ros_coverage_log "INFO" "Capturing coverage data from $coverage_dir..."
    local capture_output
    local capture_exit_code
    capture_output=$(lcov --capture \
        --directory "$coverage_dir" \
        --output-file "$output_file" \
        $branch_coverage_rc \
        $ignore_flags 2>&1)
    capture_exit_code=$?
    
    # Filter out geninfo mismatch warnings (harmless, caused by exception tag differences in system headers)
    # The --ignore-errors mismatch flag should suppress them, but filter as fallback
    echo "$capture_output" | grep -v "geninfo: WARNING: ('mismatch')" || true
    
    if [ $capture_exit_code -ne 0 ]; then
        
        ros_coverage_log "WARN" "lcov capture had issues, trying with more permissive ignore flags..."
        
        # Fallback: try with all common error types ignored (lcov 2.0+ only)
        local lcov_major
        lcov_major=$(ros_coverage_get_lcov_major)
        
        if [ "$lcov_major" -ge 2 ]; then
            capture_output=$(lcov --capture \
                --directory "$coverage_dir" \
                --output-file "$output_file" \
                $branch_coverage_rc \
                --ignore-errors mismatch,unused,negative,gcov,source 2>&1 || true)
            echo "$capture_output" | grep -v "geninfo: WARNING: ('mismatch')" || true
        else
            capture_output=$(lcov --capture \
                --directory "$coverage_dir" \
                --output-file "$output_file" \
                $branch_coverage_rc 2>&1 || true)
            echo "$capture_output" | grep -v "geninfo: WARNING: ('mismatch')" || true
        fi
    fi
    
    # Check if we got any coverage data
    if [ ! -s "$output_file" ]; then
        ros_coverage_error "No coverage data captured!"
    fi
    
    local line_count
    line_count=$(wc -l < "$output_file")
    ros_coverage_log "INFO" "Coverage data captured: $line_count lines"
}

# =============================================================================
# Filter Coverage Data
# =============================================================================

ros_coverage_filter() {
    local input_file="$1"
    local output_file="$2"
    
    if [ -z "$input_file" ] || [ -z "$output_file" ]; then
        ros_coverage_error "Usage: ros_coverage_filter <input_file> <output_file>"
    fi
    
    if [ ! -f "$input_file" ]; then
        ros_coverage_error "Input coverage file not found: $input_file"
    fi
    
    ros_coverage_log "INFO" "Filtering coverage data..."
    
    # Get lcov flags and branch coverage RC option
    local ignore_flags
    ignore_flags=$(ros_coverage_get_lcov_flags)
    local branch_coverage_rc
    branch_coverage_rc=$(ros_coverage_get_branch_coverage_rc)
    
    # Remove external dependencies from coverage
    # Note: $ignore_flags and $branch_coverage_rc are intentionally unquoted to allow
    # proper expansion of multiple arguments (they may contain multiple space-separated flags)
    # The --ignore-errors unused flag (for lcov 2.0+) suppresses warnings about unused exclude patterns
    # If warnings still appear (e.g., if version detection failed), filter them while preserving exit code
    local lcov_output
    local lcov_exit_code
    lcov_output=$(lcov --remove "$input_file" \
        '/usr/*' \
        '/opt/*' \
        '*/_deps/*' \
        '*/generated/*' \
        '*/googletest/*' \
        '*/gtest/*' \
        '*/mcap-*/*' \
        '*/mcap/include/*' \
        '*/minio-cpp/*' \
        '*/axon_recorder/test/*' \
        '*/axon_logging/test/*' \
        '*/axon_mcap/test/*' \
        '*/axon_uploader/test/*' \
        '*/rosidl_typesupport_cpp/*' \
        '*/rosidl_typesupport_introspection_cpp/*' \
        '*/rosidl_generator_cpp/*' \
        --output-file "$output_file" \
        $branch_coverage_rc \
        $ignore_flags 2>&1)
    lcov_exit_code=$?
    
    # Filter out unused pattern warnings (they're harmless but noisy)
    # The --ignore-errors unused flag should suppress them, but filter as fallback
    echo "$lcov_output" | grep -v "WARNING: ('unused') 'exclude' pattern" || true
    
    if [ $lcov_exit_code -ne 0 ]; then
        
        ros_coverage_log "WARN" "lcov filtering had issues, using raw coverage..."
        cp "$input_file" "$output_file"
    fi
    
    ros_coverage_log "INFO" "Coverage data filtered: $output_file"
}

# =============================================================================
# Generate Coverage Report
# =============================================================================

ros_coverage_generate() {
    local coverage_dir="$1"
    local output_dir="$2"
    
    if [ -z "$coverage_dir" ] || [ -z "$output_dir" ]; then
        ros_coverage_error "Usage: ros_coverage_generate <coverage_dir> <output_dir>"
    fi
    
    # Create output directory
    mkdir -p "$output_dir"
    
    # Check lcov availability
    ros_coverage_check_lcov
    
    # Capture coverage
    local raw_file="$output_dir/coverage_raw.info"
    ros_coverage_capture "$coverage_dir" "$raw_file"
    
    # Filter coverage
    local filtered_file="$output_dir/coverage.info"
    ros_coverage_filter "$raw_file" "$filtered_file"
    
    # Generate summary
    ros_coverage_log "INFO" "Generating coverage summary..."
    echo ""
    echo "============================================"
    echo "Coverage Summary"
    echo "============================================"
    lcov --list "$filtered_file" || true
    
    # Debug: show sample of paths
    echo ""
    echo "=== Source file paths in coverage.info ==="
    grep "^SF:" "$filtered_file" | head -10 || true
    
    # Verify file is not empty
    if [ -s "$filtered_file" ]; then
        local file_size
        file_size=$(wc -c < "$filtered_file")
        ros_coverage_log "INFO" "Coverage file size: $file_size bytes"
    else
        ros_coverage_log "WARN" "coverage.info is empty!"
    fi
    
    ros_coverage_log "INFO" "Coverage report generated: $filtered_file"
    echo "$filtered_file"
}

# Export functions for use in other scripts
export -f ros_coverage_check_lcov
export -f ros_coverage_get_lcov_version
export -f ros_coverage_get_lcov_major
export -f ros_coverage_get_branch_coverage_rc
export -f ros_coverage_get_lcov_flags
export -f ros_coverage_capture
export -f ros_coverage_filter
export -f ros_coverage_generate
export -f ros_coverage_log
export -f ros_coverage_error

