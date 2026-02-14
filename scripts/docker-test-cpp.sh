#!/bin/bash
# =============================================================================
# Docker C++ Library Tests
# =============================================================================
# Runs C++ library tests (axon_mcap, axon_uploader, axon_logging) in Docker.
#
# Usage:
#   ./scripts/docker-test-cpp.sh [--coverage]
#
# Options:
#   --coverage    Run with coverage instrumentation
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
RUN_COVERAGE=false
IMAGE_NAME="axon:cpp-test"

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --coverage)
            RUN_COVERAGE=true
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [--coverage] [--help]"
            echo ""
            echo "Options:"
            echo "  --coverage    Run with coverage instrumentation"
            echo "  --help        Show this help message"
            echo ""
            echo "Runs C++ library tests in Docker container."
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            exit 1
            ;;
    esac
done

echo -e "${BLUE}=============================================${NC}"
echo -e "${BLUE}Docker C++ Library Tests${NC}"
echo -e "${BLUE}=============================================${NC}"
echo ""

# Build Docker image if not exists
if ! docker image inspect ${IMAGE_NAME} &>/dev/null; then
    echo -e "${YELLOW}Building C++ test Docker image...${NC}"
    DOCKER_BUILDKIT=1 docker build \
        -f "${PROJECT_ROOT}/docker/Dockerfile.cpp-test" \
        -t ${IMAGE_NAME} \
        --build-arg BUILDKIT_INLINE_CACHE=1 \
        "${PROJECT_ROOT}"
    echo -e "${GREEN}✓ C++ test image built${NC}"
else
    echo -e "${GREEN}Using existing image: ${IMAGE_NAME}${NC}"
fi

echo ""

# Build and test command
if [ "$RUN_COVERAGE" = true ]; then
    BUILD_TYPE="Debug"
    COVERAGE_ARG="-DAXON_ENABLE_COVERAGE=ON"
    echo -e "${YELLOW}Running C++ tests with coverage...${NC}"
else
    BUILD_TYPE="Release"
    COVERAGE_ARG=""
    echo -e "${YELLOW}Running C++ tests...${NC}"
fi

DOCKER_BUILDKIT=1 docker run --rm \
    -v "${PROJECT_ROOT}:/workspace/axon" \
    ${RUN_COVERAGE:+-v "${PROJECT_ROOT}/coverage:/workspace/coverage"} \
    ${IMAGE_NAME} \
    bash -c "
        set -e

        for lib in axon_mcap axon_uploader axon_logging; do
            echo ''
            echo '=========================================='
            echo \"Building \${lib}...\"
            echo '=========================================='
            cd /workspace/axon
            mkdir -p build/\${lib}
            cd build/\${lib}
            cmake ../../core/\${lib} \
                -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
                -DAXON_REPO_ROOT=/workspace/axon \
                -DAXON_BUILD_TESTS=ON \
                ${COVERAGE_ARG}
            make -j\$(nproc)

            echo ''
            echo '=========================================='
            echo \"Running \${lib} tests...\"
            echo '=========================================='
            ctest --output-on-failure

            ${RUN_COVERAGE:+lcov --capture --directory . \
                --output-file /workspace/coverage/\${lib}_coverage.info \
                --rc lcov_branch_coverage=1 || true}
        done

        echo ''
        echo '=========================================='
        echo 'All C++ tests passed!'
        echo '=========================================='
    "

echo ""
echo -e "${GREEN}=============================================${NC}"
echo -e "${GREEN}✓ C++ tests completed${NC}"
echo -e "${GREEN}=============================================${NC}"

if [ "$RUN_COVERAGE" = true ]; then
    echo -e "${YELLOW}Coverage reports: ${PROJECT_ROOT}/coverage/*_coverage.info${NC}"
fi
