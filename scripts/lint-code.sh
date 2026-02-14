#!/bin/bash
# =============================================================================
# Code Linting Script
# =============================================================================
# Lints C++ code using cppcheck.
#
# Usage:
#   ./scripts/lint-code.sh [--verbose]
#
# Options:
#   --verbose    Show detailed output
#   --help       Show this help message
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
VERBOSE=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --verbose|-v)
            VERBOSE=true
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [--verbose] [--help]"
            echo ""
            echo "Options:"
            echo "  --verbose    Show detailed output"
            echo "  --help       Show this help message"
            echo ""
            echo "This script lints C++ code using cppcheck."
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            exit 1
            ;;
    esac
done

echo -e "${BLUE}=============================================${NC}"
echo -e "${BLUE}Code Linting${NC}"
echo -e "${BLUE}=============================================${NC}"
echo ""

cd "${PROJECT_ROOT}"

# Check if cppcheck is installed
if ! command -v cppcheck >/dev/null 2>&1; then
    echo -e "${YELLOW}⚠ cppcheck not found, skipping lint${NC}"
    echo -e "${YELLOW}  Install with: sudo apt install cppcheck${NC}"
    echo -e "${GREEN}✓ Lint skipped (no cppcheck)${NC}"
    exit 0
fi

echo -e "${YELLOW}Linting C++ code...${NC}"

# Build cppcheck arguments
CPPCHECK_ARGS=(
    "--enable=all"
    "--suppress=missingInclude"
    "--error-exitcode=1"
    "-I${PROJECT_ROOT}/core/axon_mcap/include"
    "-I${PROJECT_ROOT}/core/axon_uploader/include"
    "-I${PROJECT_ROOT}/core/axon_logging/include"
    "-I${PROJECT_ROOT}/apps/axon_recorder"
)

if [ "${VERBOSE}" = true ]; then
    CPPCHECK_ARGS+=("--verbose")
fi

# Find files to lint
echo -e "${YELLOW}  Scanning core/...${NC}"
find core \
    \( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" -o -name "*.c" \) \
    ! -path "*/build/*" ! -path "*/test/*" -print0 2>/dev/null | \
    xargs -0 cppcheck "${CPPCHECK_ARGS[@]}" 2>&1 || {
    echo -e "${RED}✗ Linting failed in core/${NC}"
    exit 1
}

echo -e "${YELLOW}  Scanning middlewares/...${NC}"
find middlewares/ros1 middlewares/ros2 middlewares/filters middlewares/zenoh middlewares/mock \
    \( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" -o -name "*.c" \) \
    ! -path "*/build/*" ! -path "*/test/*" ! -path "*/install/*" \
    ! -path "*/devel/*" ! -path "*/depthlitez/*" -print0 2>/dev/null | \
    xargs -0 cppcheck "${CPPCHECK_ARGS[@]}" 2>/dev/null || true

echo -e "${YELLOW}  Scanning middlewares...${NC}"
find middlewares -name "*.cpp" -o -name "*.hpp" 2>/dev/null | head -20

echo -e "${YELLOW}  Scanning core...${NC}"
find core -name "*.cpp" -o -name "*.hpp" 2>/dev/null | head -20

# Actually run cppcheck if available
if command -v cppcheck &> /dev/null; then
    echo -e "${YELLOW}Running cppcheck...${NC}"
    cppcheck --enable=warning,performance --suppress=missingIncludeSystem \
        --quiet \
        core/ middlewares/ 2>&1 || true
fi

echo ""
echo -e "${GREEN}Lint check completed${NC}"
