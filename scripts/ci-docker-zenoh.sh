#!/bin/bash
# =============================================================================
# Docker CI Script for Zenoh Plugin Tests
# =============================================================================
# Runs Zenoh plugin tests in Docker, mirrors .github/workflows/tests-zenoh.yml.
#
# Usage:
#   ./scripts/ci-docker-zenoh.sh [--coverage] [--integration]
#
# Options:
#   --coverage     Run with coverage instrumentation
#   --integration  Run E2E integration tests with docker compose
#   --help         Show this help message
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
DOCKER_IMAGE="axon:zenoh"
RUN_COVERAGE=false
RUN_INTEGRATION=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --coverage)
            RUN_COVERAGE=true
            shift
            ;;
        --integration)
            RUN_INTEGRATION=true
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [--coverage] [--integration] [--help]"
            echo ""
            echo "Options:"
            echo "  --coverage     Run with coverage instrumentation"
            echo "  --integration  Run E2E integration tests with docker compose"
            echo "  --help         Show this help message"
            echo ""
            echo "This script mirrors the GitHub Actions workflow for Zenoh plugin tests."
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            exit 1
            ;;
    esac
done

echo -e "${BLUE}=============================================${NC}"
echo -e "${BLUE}Axon Zenoh Plugin Local CI${NC}"
echo -e "${BLUE}=============================================${NC}"
echo ""

# =============================================================================
# Step 1: Build Docker Image
# =============================================================================
echo -e "${YELLOW}Step 1: Building Zenoh Docker image...${NC}"
if ! docker image inspect ${DOCKER_IMAGE} &>/dev/null; then
    echo -e "${YELLOW}Building new image...${NC}"
    docker build -f "${PROJECT_ROOT}/docker/Dockerfile.zenoh" -t ${DOCKER_IMAGE} "${PROJECT_ROOT}"
else
    echo -e "${GREEN}Docker image already exists. Use 'docker rmi ${DOCKER_IMAGE}' to rebuild.${NC}"
fi
echo ""

# =============================================================================
# Step 2: Run Unit Tests
# =============================================================================
echo -e "${YELLOW}Step 2: Running Zenoh plugin unit tests...${NC}"

if [ "$RUN_COVERAGE" = true ]; then
    echo -e "${BLUE}Running with coverage instrumentation...${NC}"
    docker run --rm \
        -v "${PROJECT_ROOT}:/workspace/axon" \
        ${DOCKER_IMAGE} \
        bash -c "
            cd /workspace/axon && \
            rm -rf build && \
            mkdir -p build && cd build && \
            cmake .. \
                -DCMAKE_BUILD_TYPE=Debug \
                -DAXON_ENABLE_COVERAGE=ON \
                -DAXON_BUILD_ZENOH_PLUGIN=ON \
                -DAXON_BUILD_ROS1_PLUGIN=OFF \
                -DAXON_BUILD_ROS2_PLUGIN=OFF \
                -DAXON_BUILD_MOCK_PLUGIN=OFF \
                -DAXON_BUILD_TESTS=ON && \
            make -j\$(nproc) && \
            echo '' && \
            echo '========================================' && \
            echo 'Running Tests...' && \
            echo '========================================' && \
            ctest --output-on-failure -L zenoh_plugin && \
            echo '' && \
            echo '========================================' && \
            echo 'Generating Coverage Report...' && \
            echo '========================================' && \
            lcov --capture --directory . --output-file coverage_raw.info \
                --rc lcov_branch_coverage=1 2>/dev/null || true && \
            lcov --remove coverage_raw.info \
                '/usr/*' '/opt/*' '*/test/*' '*/_deps/*' '*/c++/*' \
                --output-file coverage.info --rc lcov_branch_coverage=1 2>/dev/null || true && \
            echo '' && \
            echo 'Coverage Summary:' && \
            lcov --list coverage.info 2>/dev/null || echo 'No coverage data generated'
        "
else
    echo -e "${BLUE}Running without coverage...${NC}"
    docker run --rm \
        -v "${PROJECT_ROOT}:/workspace/axon" \
        ${DOCKER_IMAGE} \
        bash -c "
            cd /workspace/axon && \
            rm -rf build && \
            mkdir -p build && cd build && \
            cmake .. \
                -DCMAKE_BUILD_TYPE=Release \
                -DAXON_BUILD_ZENOH_PLUGIN=ON \
                -DAXON_BUILD_ROS1_PLUGIN=OFF \
                -DAXON_BUILD_ROS2_PLUGIN=OFF \
                -DAXON_BUILD_MOCK_PLUGIN=OFF \
                -DAXON_BUILD_TESTS=ON && \
            make -j\$(nproc) && \
            echo '' && \
            echo '========================================' && \
            echo 'Running Tests...' && \
            echo '========================================' && \
            ctest --output-on-failure -V -L zenoh_plugin
        "
fi

echo ""

# =============================================================================
# Step 3: Run Integration Tests (optional)
# =============================================================================
if [ "$RUN_INTEGRATION" = true ]; then
    echo -e "${YELLOW}Step 3: Running Axon + Zenoh integration tests...${NC}"

    if [ -f "${PROJECT_ROOT}/docker/docker-compose.zenoh.yml" ]; then
        cd "${PROJECT_ROOT}/docker"
        docker compose -f docker-compose.zenoh.yml build
        docker compose -f docker-compose.zenoh.yml up --abort-on-container-exit --exit-code-from test-subscriber || true

        echo -e "${YELLOW}Cleaning up...${NC}"
        docker compose -f docker-compose.zenoh.yml down -v
    else
        echo -e "${RED}docker-compose.zenoh.yml not found, skipping integration tests${NC}"
    fi
fi

echo ""
echo -e "${GREEN}=============================================${NC}"
echo -e "${GREEN}Local CI completed successfully!${NC}"
echo -e "${GREEN}=============================================${NC}"
