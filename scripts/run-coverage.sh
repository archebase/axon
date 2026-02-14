#!/bin/bash
# =============================================================================
# Coverage Report Generation
# =============================================================================
# Generates coverage reports for C++ libraries and ROS tests.
#
# Usage:
#   ./scripts/run-coverage.sh [target] [--html] [--clean]
#
# Arguments:
#   target      Coverage target: mcap, uploader, logging, ros2, all
#               (default: all)
#
# Options:
#   --html      Generate HTML report
#   --clean     Clean coverage data only
#   --help      Show this help message
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
BUILD_DIR="${PROJECT_ROOT}/build"
COVERAGE_DIR="${PROJECT_ROOT}/coverage"
GENERATE_HTML=false
CLEAN_ONLY=false
TARGET="all"

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        mcap|uploader|logging|ros2|all)
            TARGET=$1
            shift
            ;;
        --html)
            GENERATE_HTML=true
            shift
            ;;
        --clean)
            CLEAN_ONLY=true
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [target] [--html] [--clean] [--help]"
            echo ""
            echo "Arguments:"
            echo "  target      Coverage target: mcap, uploader, logging, ros2, all"
            echo "              (default: all)"
            echo ""
            echo "Options:"
            echo "  --html      Generate HTML report"
            echo "  --clean     Clean coverage data only"
            echo "  --help      Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                  # Generate all coverage reports"
            echo "  $0 mcap --html      # Generate mcap coverage with HTML"
            echo "  $0 ros2             # Generate ROS2 coverage"
            echo "  $0 --clean          # Clean coverage data"
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            exit 1
            ;;
    esac
done

# =============================================================================
# Clean coverage data
# =============================================================================
if [ "$CLEAN_ONLY" = true ]; then
    echo -e "${BLUE}=============================================${NC}"
    echo -e "${BLUE}Cleaning Coverage Data${NC}"
    echo -e "${BLUE}=============================================${NC}"
    echo ""

    echo -e "${YELLOW}Removing coverage directory...${NC}"
    rm -rf "${COVERAGE_DIR}"

    echo -e "${YELLOW}Removing .gcda files...${NC}"
    find "${BUILD_DIR}" -name "*.gcda" -delete 2>/dev/null || true

    echo -e "${YELLOW}Removing .gcno files...${NC}"
    find "${BUILD_DIR}" -name "*.gcno" -delete 2>/dev/null || true

    echo ""
    echo -e "${GREEN}✓ Coverage data cleaned${NC}"
    exit 0
fi

echo -e "${BLUE}=============================================${NC}"
echo -e "${BLUE}Coverage Report Generation${NC}"
echo -e "${BLUE}=============================================${NC}"
echo ""
echo -e "Target: ${YELLOW}${TARGET}${NC}"
echo -e "HTML: ${YELLOW}${GENERATE_HTML}${NC}"
echo ""

# Detect lcov version for compatibility flags
LCOV_VERSION=$(lcov --version 2>&1 | grep -oP 'LCOV version \K[0-9.]+' || echo "1.0")
LCOV_MAJOR=$(echo "${LCOV_VERSION}" | cut -d. -f1)
echo -e "lcov version: ${YELLOW}${LCOV_VERSION}${NC}"

LCOV_IGNORE_FLAGS=""
if [ "${LCOV_MAJOR}" -ge 2 ]; then
    LCOV_IGNORE_FLAGS="--ignore-errors mismatch,unused,negative,gcov"
fi

# Create coverage directory
mkdir -p "${COVERAGE_DIR}"

# =============================================================================
# Coverage generation function
# =============================================================================
generate_coverage() {
    local LIB_NAME=$1
    local LIB_DIR=$2

    echo ""
    echo -e "${YELLOW}Generating coverage for ${LIB_NAME}...${NC}"

    cd "${LIB_DIR}"

    # Capture coverage data
    lcov --capture --directory . --output-file coverage_raw.info \
        --rc lcov_branch_coverage=1 ${LCOV_IGNORE_FLAGS} 2>/dev/null || {
        echo -e "${YELLOW}Warning: lcov capture had issues for ${LIB_NAME}${NC}"
        return 1
    }

    # Filter coverage data
    lcov --remove coverage_raw.info \
        '/usr/*' '/opt/*' '*/_deps/*' '*/test/*' '*/c++/*' \
        --output-file "${COVERAGE_DIR}/${LIB_NAME}_coverage.info" \
        --rc lcov_branch_coverage=1 ${LCOV_IGNORE_FLAGS} 2>/dev/null || {
        echo -e "${YELLOW}Warning: lcov filtering had issues for ${LIB_NAME}${NC}"
        cp coverage_raw.info "${COVERAGE_DIR}/${LIB_NAME}_coverage.info"
    }

    # Show summary
    echo -e "${GREEN}Coverage summary for ${LIB_NAME}:${NC}"
    lcov --list "${COVERAGE_DIR}/${LIB_NAME}_coverage.info" ${LCOV_IGNORE_FLAGS} 2>/dev/null || true

    cd "${PROJECT_ROOT}"
}

# =============================================================================
# Build and test with coverage
# =============================================================================
build_with_coverage() {
    local LIB_NAME=$1

    echo ""
    echo -e "${YELLOW}Building ${LIB_NAME} with coverage...${NC}"

    rm -rf "${BUILD_DIR}"
    mkdir -p "${BUILD_DIR}"

    cd "${BUILD_DIR}"
    cmake .. \
        -DCMAKE_BUILD_TYPE=Debug \
        -DAXON_BUILD_TESTS=ON \
        -DAXON_ENABLE_COVERAGE=ON \
        -DAXON_BUILD_UPLOADER=ON

    cmake --build . -j$(nproc)

    echo ""
    echo -e "${YELLOW}Running ${LIB_NAME} tests...${NC}"
    cd "${BUILD_DIR}/${LIB_NAME}"
    ctest --output-on-failure
}

# =============================================================================
# Run coverage for selected target
# =============================================================================
case $TARGET in
    mcap)
        build_with_coverage "axon_mcap"
        generate_coverage "axon_mcap" "${BUILD_DIR}/axon_mcap"
        ;;
    uploader)
        build_with_coverage "axon_uploader"
        generate_coverage "axon_uploader" "${BUILD_DIR}/axon_uploader"
        ;;
    logging)
        build_with_coverage "axon_logging"
        generate_coverage "axon_logging" "${BUILD_DIR}/axon_logging"
        ;;
    ros2)
        if [ -z "$ROS_DISTRO" ]; then
            echo -e "${RED}Error: ROS_DISTRO not set. Source ROS setup.bash first.${NC}"
            exit 1
        fi

        echo -e "${YELLOW}Building ROS2 with coverage...${NC}"
        rm -rf "${BUILD_DIR}"
        mkdir -p "${BUILD_DIR}"

        source /opt/ros/${ROS_DISTRO}/setup.bash
        cd "${BUILD_DIR}"
        cmake .. \
            -DCMAKE_BUILD_TYPE=Debug \
            -DAXON_ENABLE_COVERAGE=ON \
            -DAXON_BUILD_TESTS=ON \
            -DAXON_BUILD_ROS2_PLUGIN=ON \
            -DAXON_BUILD_ROS1_PLUGIN=OFF \
            -DAXON_BUILD_ZENOH_PLUGIN=OFF
        cmake --build . -j$(nproc)
        ctest --output-on-failure || true

        generate_coverage "ros2" "${BUILD_DIR}"
        ;;
    all)
        build_with_coverage "all"
        for lib in axon_mcap axon_uploader axon_logging; do
            if [ -d "${BUILD_DIR}/${lib}" ]; then
                generate_coverage "${lib}" "${BUILD_DIR}/${lib}"
            fi
        done
        ;;
esac

# =============================================================================
# Generate HTML report
# =============================================================================
if [ "$GENERATE_HTML" = true ]; then
    echo ""
    echo -e "${YELLOW}Generating HTML coverage report...${NC}"

    mkdir -p "${COVERAGE_DIR}/html"

    # Find all coverage info files
    INFO_FILES=$(find "${COVERAGE_DIR}" -name "*_coverage.info" -type f)

    if [ -n "$INFO_FILES" ]; then
        genhtml $INFO_FILES \
            --output-directory "${COVERAGE_DIR}/html" \
            --title "Axon Test Coverage" \
            --legend \
            --branch-coverage || {
            echo -e "${YELLOW}Warning: HTML generation had issues${NC}"
        }
        echo -e "${GREEN}✓ HTML report: ${COVERAGE_DIR}/html/index.html${NC}"
    else
        echo -e "${YELLOW}No coverage info files found${NC}"
    fi
fi

# =============================================================================
# Summary
# =============================================================================
echo ""
echo -e "${GREEN}=============================================${NC}"
echo -e "${GREEN}Coverage Generation Complete${NC}"
echo -e "${GREEN}=============================================${NC}"
echo ""
echo -e "Coverage reports: ${YELLOW}${COVERAGE_DIR}/*_coverage.info${NC}"

if [ "$GENERATE_HTML" = true ] && [ -f "${COVERAGE_DIR}/html/index.html" ]; then
    echo -e "HTML report: ${YELLOW}${COVERAGE_DIR}/html/index.html${NC}"
fi
