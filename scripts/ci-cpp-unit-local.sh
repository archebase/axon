#!/bin/bash
# =============================================================================
# Local CI Script for C++ Unit Tests
# =============================================================================
# Mirrors .github/workflows/unit-tests-cpp.yml for local testing.
#
# Usage:
#   ./scripts/ci-cpp-unit-local.sh [library] [--coverage]
#
# Arguments:
#   library     Specific library to test: axon_mcap, axon_uploader, axon_logging
#               (default: all libraries)
#
# Options:
#   --coverage  Run with coverage instrumentation
#   --help      Show this help message
#
# Examples:
#   ./scripts/ci-cpp-unit-local.sh                  # Test all libraries
#   ./scripts/ci-cpp-unit-local.sh axon_mcap        # Test only axon_mcap
#   ./scripts/ci-cpp-unit-local.sh --coverage       # Test all with coverage
#   ./scripts/ci-cpp-unit-local.sh axon_uploader --coverage
# =============================================================================

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default settings
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
RUN_COVERAGE=false
LIBRARIES=("axon_mcap" "axon_uploader" "axon_logging")
SELECTED_LIBRARIES=()

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        axon_mcap|axon_uploader|axon_logging)
            SELECTED_LIBRARIES+=("$1")
            shift
            ;;
        --coverage)
            RUN_COVERAGE=true
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [library] [--coverage] [--help]"
            echo ""
            echo "Arguments:"
            echo "  library       Specific library to test: axon_mcap, axon_uploader, axon_logging"
            echo "                (default: all libraries)"
            echo ""
            echo "Options:"
            echo "  --coverage    Run with coverage instrumentation"
            echo "  --help        Show this help message"
            echo ""
            echo "This script mirrors the GitHub Actions workflow for C++ unit tests."
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            exit 1
            ;;
    esac
done

# If no specific library selected, test all
if [ ${#SELECTED_LIBRARIES[@]} -eq 0 ]; then
    SELECTED_LIBRARIES=("${LIBRARIES[@]}")
fi

echo -e "${BLUE}=============================================${NC}"
echo -e "${BLUE}Axon C++ Unit Tests Local CI${NC}"
echo -e "${BLUE}=============================================${NC}"
echo ""
echo -e "Libraries: ${YELLOW}${SELECTED_LIBRARIES[*]}${NC}"
echo -e "Coverage: ${YELLOW}${RUN_COVERAGE}${NC}"
echo ""

# Detect lcov version for compatibility flags
LCOV_VERSION=$(lcov --version 2>&1 | grep -oP 'LCOV version \K[0-9.]+' || echo "1.0")
LCOV_MAJOR=$(echo "${LCOV_VERSION}" | cut -d. -f1)
echo -e "lcov version: ${YELLOW}${LCOV_VERSION}${NC}"

LCOV_IGNORE_FLAGS=""
if [ "${LCOV_MAJOR}" -ge 2 ]; then
    LCOV_IGNORE_FLAGS="--ignore-errors mismatch,unused,negative,gcov"
fi

# =============================================================================
# Test function for each library
# =============================================================================
test_library() {
    local LIBRARY=$1
    local BUILD_TYPE="Release"
    local CMAKE_ARGS=""

    if [ "$RUN_COVERAGE" = true ]; then
        BUILD_TYPE="Debug"
        CMAKE_ARGS="-DAXON_ENABLE_COVERAGE=ON"
    fi

    echo ""
    echo -e "${YELLOW}=============================================${NC}"
    echo -e "${YELLOW}Testing: ${LIBRARY}${NC}"
    echo -e "${YELLOW}=============================================${NC}"

    cd "${PROJECT_ROOT}"

    case $LIBRARY in
        axon_mcap)
            make build-mcap BUILD_TYPE=${BUILD_TYPE} ${CMAKE_ARGS:+CMAKE_ARGS="${CMAKE_ARGS}"}
            cd build/axon_mcap
            ;;
        axon_uploader)
            make build-uploader BUILD_TYPE=${BUILD_TYPE} ${CMAKE_ARGS:+CMAKE_ARGS="${CMAKE_ARGS}"}
            cd build/axon_uploader
            ;;
        axon_logging)
            make build-logging BUILD_TYPE=${BUILD_TYPE} ${CMAKE_ARGS:+CMAKE_ARGS="${CMAKE_ARGS}"}
            cd build/axon_logging
            ;;
    esac

    echo ""
    echo -e "${BLUE}Running tests...${NC}"
    ctest --output-on-failure

    if [ "$RUN_COVERAGE" = true ]; then
        echo ""
        echo -e "${BLUE}Generating coverage report...${NC}"

        lcov --capture --directory . --output-file coverage_raw.info \
            --rc lcov_branch_coverage=1 ${LCOV_IGNORE_FLAGS} 2>/dev/null || true

        lcov --remove coverage_raw.info \
            '/usr/*' '/opt/*' '*/_deps/*' '*/test/*' '*/c++/*' \
            --output-file coverage.info --rc lcov_branch_coverage=1 ${LCOV_IGNORE_FLAGS} 2>/dev/null || {
            echo -e "${YELLOW}Warning: lcov filtering had issues, using raw coverage...${NC}"
            cp coverage_raw.info coverage.info
        }

        echo ""
        echo -e "${GREEN}Coverage Summary for ${LIBRARY}:${NC}"
        lcov --list coverage.info ${LCOV_IGNORE_FLAGS} 2>/dev/null || echo "No coverage data"
    fi

    cd "${PROJECT_ROOT}"
    echo -e "${GREEN}✓ ${LIBRARY} tests passed${NC}"
}

# =============================================================================
# Run tests for each selected library
# =============================================================================
FAILED_LIBRARIES=()

for LIBRARY in "${SELECTED_LIBRARIES[@]}"; do
    if ! test_library "$LIBRARY"; then
        FAILED_LIBRARIES+=("$LIBRARY")
    fi
done

# =============================================================================
# Summary
# =============================================================================
echo ""
echo -e "${BLUE}=============================================${NC}"
echo -e "${BLUE}Summary${NC}"
echo -e "${BLUE}=============================================${NC}"

if [ ${#FAILED_LIBRARIES[@]} -eq 0 ]; then
    echo -e "${GREEN}All tests passed!${NC}"
    echo ""
    echo -e "Libraries tested: ${GREEN}${SELECTED_LIBRARIES[*]}${NC}"
    exit 0
else
    echo -e "${RED}Some tests failed!${NC}"
    echo -e "Failed libraries: ${RED}${FAILED_LIBRARIES[*]}${NC}"
    exit 1
fi
