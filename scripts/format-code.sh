#!/bin/bash
# =============================================================================
# Code Formatting Script
# =============================================================================
# Formats C/C++ and CMake code using clang-format and gersemi.
#
# Usage:
#   ./scripts/format-code.sh [--check] [--cmake-only] [--cpp-only]
#
# Options:
#   --check       Check formatting without modifying (CI mode)
#   --cmake-only  Only format CMake files
#   --cpp-only    Only format C/C++ files
#   --help        Show this help message
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
CHECK_ONLY=false
CMAKE_ONLY=false
CPP_ONLY=false

# clang-format version
CLANG_FORMAT_VERSION=21

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --check)
            CHECK_ONLY=true
            shift
            ;;
        --cmake-only)
            CMAKE_ONLY=true
            shift
            ;;
        --cpp-only)
            CPP_ONLY=true
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [--check] [--cmake-only] [--cpp-only] [--help]"
            echo ""
            echo "Options:"
            echo "  --check       Check formatting without modifying (CI mode)"
            echo "  --cmake-only  Only format CMake files"
            echo "  --cpp-only    Only format C/C++ files"
            echo "  --help        Show this help message"
            echo ""
            echo "This script formats C/C++ code with clang-format and CMake files with gersemi."
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            exit 1
            ;;
    esac
done

echo -e "${BLUE}=============================================${NC}"
echo -e "${BLUE}Code Formatting${NC}"
echo -e "${BLUE}=============================================${NC}"
echo ""
echo -e "Check only: ${YELLOW}${CHECK_ONLY}${NC}"
echo -e "CMake only: ${YELLOW}${CMAKE_ONLY}${NC}"
echo -e "C++ only: ${YELLOW}${CPP_ONLY}${NC}"
echo ""

cd "${PROJECT_ROOT}"

# =============================================================================
# Find clang-format
# =============================================================================
CLANG_FORMAT=""
for cmd in clang-format-${CLANG_FORMAT_VERSION} clang-format; do
    if command -v ${cmd} >/dev/null 2>&1; then
        VERSION=$(${cmd} --version 2>/dev/null | grep -oE 'version [0-9]+' | grep -oE '[0-9]+' | head -1)
        if [ "${VERSION}" = "${CLANG_FORMAT_VERSION}" ]; then
            CLANG_FORMAT=${cmd}
            break
        fi
    fi
done

# =============================================================================
# Format C/C++ code
# =============================================================================
format_cpp() {
    if [ "${CMAKE_ONLY}" = true ]; then
        return
    fi

    if [ -z "${CLANG_FORMAT}" ]; then
        echo -e "${YELLOW}⚠ clang-format version ${CLANG_FORMAT_VERSION} not found${NC}"
        echo -e "${YELLOW}  Linux: wget -qO- https://apt.llvm.org/llvm.sh | sudo bash -s ${CLANG_FORMAT_VERSION}${NC}"
        echo -e "${YELLOW}  macOS: brew install llvm@${CLANG_FORMAT_VERSION}${NC}"
        return 1
    fi

    echo -e "${YELLOW}Formatting C/C++ code...${NC}"
    echo -e "${YELLOW}  Using: ${CLANG_FORMAT}${NC}"

    # Build find command arguments
    FIND_ARGS=()
    FIND_ARGS+=("(")
    FIND_ARGS+=("-name" "*.cpp" "-o" "-name" "*.hpp" "-o" "-name" "*.h" "-o" "-name" "*.c")
    FIND_ARGS+=(")")
    FIND_ARGS+=("!")
    FIND_ARGS+=("-path" "*/build/*")
    FIND_ARGS+=("!")
    FIND_ARGS+=("-path" "*/build_*/*")
    FIND_ARGS+=("!")
    FIND_ARGS+=("-path" "*/install/*")
    FIND_ARGS+=("!")
    FIND_ARGS+=("-path" "*/devel/*")
    FIND_ARGS+=("!")
    FIND_ARGS+=("-path" "*/depthlitez/*")

    if [ "${CHECK_ONLY}" = true ]; then
        echo -e "${YELLOW}  Checking formatting (dry run)...${NC}"

        # Check core libraries
        echo -e "${YELLOW}  Checking core/...${NC}"
        find core "${FIND_ARGS[@]}" -print0 2>/dev/null | \
            xargs -0 ${CLANG_FORMAT} --dry-run --Werror > /dev/null 2>&1 || {
            echo -e "${RED}✗ Code formatting check failed in core/${NC}"
            return 1
        }

        # Check middlewares (excluding depthlitez)
        echo -e "${YELLOW}  Checking middlewares/...${NC}"
        find middlewares/ros1 middlewares/ros2 middlewares/filters middlewares/zenoh middlewares/mock \
            "${FIND_ARGS[@]}" -print0 2>/dev/null | \
            xargs -0 ${CLANG_FORMAT} --dry-run --Werror > /dev/null 2>&1 || {
            echo -e "${RED}✗ Code formatting check failed in middlewares/${NC}"
            return 1
        }

        # Check apps
        echo -e "${YELLOW}  Checking apps/...${NC}"
        find apps "${FIND_ARGS[@]}" -print0 2>/dev/null | \
            xargs -0 ${CLANG_FORMAT} --dry-run --Werror > /dev/null 2>&1 || {
            echo -e "${RED}✗ Code formatting check failed in apps/${NC}"
            return 1
        }

        echo -e "${GREEN}✓ C/C++ code formatting check passed${NC}"
    else
        # Format core libraries
        echo -e "${YELLOW}  Formatting core/...${NC}"
        find core "${FIND_ARGS[@]}" -print0 2>/dev/null | \
            xargs -0 ${CLANG_FORMAT} -i

        # Format middlewares (excluding depthlitez)
        echo -e "${YELLOW}  Formatting middlewares/...${NC}"
        find middlewares/ros1 middlewares/ros2 middlewares/filters middlewares/zenoh middlewares/mock \
            "${FIND_ARGS[@]}" -print0 2>/dev/null | \
            xargs -0 ${CLANG_FORMAT} -i

        # Format apps
        echo -e "${YELLOW}  Formatting apps/...${NC}"
        find apps "${FIND_ARGS[@]}" -print0 2>/dev/null | \
            xargs -0 ${CLANG_FORMAT} -i

        echo -e "${GREEN}✓ C/C++ code formatted${NC}"
    fi
}

# =============================================================================
# Format CMake files
# =============================================================================
format_cmake() {
    if [ "${CPP_ONLY}" = true ]; then
        return
    fi

    if ! command -v gersemi >/dev/null 2>&1; then
        echo -e "${YELLOW}⚠ gersemi not found, skipping CMake formatting${NC}"
        echo -e "${YELLOW}  Install with: pip install gersemi${NC}"
        return 0
    fi

    echo -e "${YELLOW}Formatting CMake files...${NC}"

    CMAKE_FILES=(
        "CMakeLists.txt"
        "apps/axon_recorder/CMakeLists.txt"
        "apps/axon_config/CMakeLists.txt"
        "apps/axon_panel/CMakeLists.txt"
        "core/axon_mcap/CMakeLists.txt"
        "core/axon_uploader/CMakeLists.txt"
        "core/axon_logging/CMakeLists.txt"
        "middlewares/ros1/CMakeLists.txt"
        "middlewares/ros2/CMakeLists.txt"
        "middlewares/zenoh/CMakeLists.txt"
        "middlewares/mock/CMakeLists.txt"
        "middlewares/filters/CMakeLists.txt"
        "cmake/Modules/AxonStdLib.cmake"
        "cmake/Modules/AxonCoverage.cmake"
    )

    EXISTING_FILES=()
    for f in "${CMAKE_FILES[@]}"; do
        if [ -f "${PROJECT_ROOT}/${f}" ]; then
            EXISTING_FILES+=("${f}")
        fi
    done

    if [ ${#EXISTING_FILES[@]} -eq 0 ]; then
        echo -e "${YELLOW}No CMake files found${NC}"
        return 0
    fi

    if [ "${CHECK_ONLY}" = true ]; then
        echo -e "${YELLOW}  Checking CMake formatting (dry run)...${NC}"
        gersemi --check "${EXISTING_FILES[@]}" 2>/dev/null || {
            echo -e "${RED}✗ CMake formatting check failed${NC}"
            echo -e "${YELLOW}Run without --check to fix${NC}"
            return 1
        }
        echo -e "${GREEN}✓ CMake formatting check passed${NC}"
    else
        gersemi --in-place "${EXISTING_FILES[@]}" 2>/dev/null
        echo -e "${GREEN}✓ CMake files formatted${NC}"
    fi
}

# =============================================================================
# Run formatting
# =============================================================================
FAILED=false

if ! format_cpp; then
    FAILED=true
fi

if ! format_cmake; then
    FAILED=true
fi

# =============================================================================
# Summary
# =============================================================================
echo ""
if [ "${FAILED}" = true ]; then
    echo -e "${RED}=============================================${NC}"
    echo -e "${RED}Formatting check failed${NC}"
    echo -e "${RED}=============================================${NC}"
    echo -e "${YELLOW}Run without --check to fix formatting issues${NC}"
    exit 1
else
    echo -e "${GREEN}=============================================${NC}"
    echo -e "${GREEN}Formatting complete${NC}"
    echo -e "${GREEN}=============================================${NC}"
fi
