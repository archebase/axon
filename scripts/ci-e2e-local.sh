#!/bin/bash
# =============================================================================
# Local CI Script for E2E Tests
# =============================================================================
# Mirrors .github/workflows/e2e-tests.yml for local testing.
# Runs end-to-end tests that start axon_recorder_node and make actual ROS
# service calls.
#
# Usage:
#   ./scripts/ci-e2e-local.sh [distro] [--coverage]
#
# Arguments:
#   distro      ROS distribution: noetic, humble, jazzy, rolling
#               (default: all distros sequentially)
#
# Options:
#   --coverage  Run with coverage instrumentation
#   --help      Show this help message
#
# Prerequisites:
#   - Docker
#   - Docker images built (run 'make docker-build-all' first)
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
DISTROS=("noetic" "humble" "jazzy" "rolling")
SELECTED_DISTROS=()

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        noetic|humble|jazzy|rolling)
            SELECTED_DISTROS+=("$1")
            shift
            ;;
        --coverage)
            RUN_COVERAGE=true
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [distro] [--coverage] [--help]"
            echo ""
            echo "Arguments:"
            echo "  distro      ROS distribution: noetic, humble, jazzy, rolling"
            echo "              (default: all distros sequentially)"
            echo ""
            echo "Options:"
            echo "  --coverage  Run with coverage instrumentation"
            echo "  --help      Show this help message"
            echo ""
            echo "This script mirrors the GitHub Actions workflow for E2E tests."
            echo "E2E tests start axon_recorder_node and make actual ROS service calls."
            echo ""
            echo "Prerequisites:"
            echo "  - Docker installed and running"
            echo "  - Docker images built (run 'make docker-build-all' first)"
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            exit 1
            ;;
    esac
done

# If no specific distro selected, test all
if [ ${#SELECTED_DISTROS[@]} -eq 0 ]; then
    SELECTED_DISTROS=("${DISTROS[@]}")
fi

echo -e "${BLUE}=============================================${NC}"
echo -e "${BLUE}Axon E2E Tests Local CI${NC}"
echo -e "${BLUE}=============================================${NC}"
echo ""
echo -e "Distributions: ${YELLOW}${SELECTED_DISTROS[*]}${NC}"
echo -e "Coverage: ${YELLOW}${RUN_COVERAGE}${NC}"
echo ""

# =============================================================================
# Check Docker
# =============================================================================
if ! command -v docker &> /dev/null; then
    echo -e "${RED}Error: Docker is not installed${NC}"
    exit 1
fi

if ! docker info &> /dev/null; then
    echo -e "${RED}Error: Docker daemon is not running${NC}"
    exit 1
fi

# =============================================================================
# Test function for each distribution
# =============================================================================
test_distro() {
    local DISTRO=$1
    local ROS_VERSION="2"
    local IMAGE_NAME="axon:test-${DISTRO}"

    if [ "$DISTRO" = "noetic" ]; then
        ROS_VERSION="1"
    fi

    echo ""
    echo -e "${YELLOW}=============================================${NC}"
    echo -e "${YELLOW}E2E Testing: ${DISTRO}${NC}"
    echo -e "${YELLOW}=============================================${NC}"

    # Check if image exists
    if ! docker image inspect ${IMAGE_NAME} &>/dev/null; then
        echo -e "${RED}Error: Docker image ${IMAGE_NAME} not found${NC}"
        echo "Please build the image first:"
        echo "  make docker-build-${DISTRO}"
        return 1
    fi

    # Create coverage output directory
    local COVERAGE_DIR="${PROJECT_ROOT}/coverage_output_e2e_${DISTRO}"
    mkdir -p "${COVERAGE_DIR}"

    # Build docker run command
    local DOCKER_CMD="docker run --rm \
        -v ${PROJECT_ROOT}:/workspace/axon \
        -v ${COVERAGE_DIR}:/coverage_output \
        -e ROS_DISTRO=${DISTRO} \
        -e ROS_VERSION=${ROS_VERSION} \
        -e COVERAGE_OUTPUT=/coverage_output \
        --network host \
        ${IMAGE_NAME}"

    if [ "$RUN_COVERAGE" = true ]; then
        DOCKER_CMD="${DOCKER_CMD} /workspace/axon/docker/scripts/run_e2e_tests.sh --coverage --coverage-output /coverage_output"
    else
        DOCKER_CMD="${DOCKER_CMD} /workspace/axon/docker/scripts/run_e2e_tests.sh"
    fi

    echo -e "${BLUE}Running E2E tests in Docker...${NC}"
    if eval ${DOCKER_CMD}; then
        echo -e "${GREEN}✓ ${DISTRO} E2E tests passed${NC}"

        if [ "$RUN_COVERAGE" = true ] && [ -f "${COVERAGE_DIR}/coverage.info" ]; then
            echo -e "${GREEN}Coverage report: ${COVERAGE_DIR}/coverage.info${NC}"
        fi
        return 0
    else
        echo -e "${RED}✗ ${DISTRO} E2E tests failed${NC}"
        return 1
    fi
}

# =============================================================================
# Run tests for each selected distribution
# =============================================================================
FAILED_DISTROS=()

for DISTRO in "${SELECTED_DISTROS[@]}"; do
    if ! test_distro "$DISTRO"; then
        FAILED_DISTROS+=("$DISTRO")
    fi
done

# =============================================================================
# Summary
# =============================================================================
echo ""
echo -e "${BLUE}=============================================${NC}"
echo -e "${BLUE}Summary${NC}"
echo -e "${BLUE}=============================================${NC}"

if [ ${#FAILED_DISTROS[@]} -eq 0 ]; then
    echo -e "${GREEN}All E2E tests passed!${NC}"
    echo -e "Distributions tested: ${GREEN}${SELECTED_DISTROS[*]}${NC}"

    if [ "$RUN_COVERAGE" = true ]; then
        echo ""
        echo -e "${YELLOW}Coverage reports:${NC}"
        for DISTRO in "${SELECTED_DISTROS[@]}"; do
            local COVERAGE_FILE="${PROJECT_ROOT}/coverage_output_e2e_${DISTRO}/coverage.info"
            if [ -f "${COVERAGE_FILE}" ]; then
                echo "  ${DISTRO}: ${COVERAGE_FILE}"
            fi
        done
    fi
    exit 0
else
    echo -e "${RED}Some E2E tests failed!${NC}"
    echo -e "Failed distributions: ${RED}${FAILED_DISTROS[*]}${NC}"
    exit 1
fi
