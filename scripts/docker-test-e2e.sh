#!/bin/bash
# =============================================================================
# Docker E2E Tests
# =============================================================================
# Runs E2E tests in Docker for specified ROS distribution.
#
# Usage:
#   ./scripts/docker-test-e2e.sh <distro> [--build]
#
# Arguments:
#   distro      ROS distribution: noetic, humble, jazzy, rolling
#
# Options:
#   --build     Build Docker image before running
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
BUILD_IMAGE=false
DISTRO=""

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        noetic|humble|jazzy|rolling)
            DISTRO=$1
            shift
            ;;
        --build)
            BUILD_IMAGE=true
            shift
            ;;
        --help|-h)
            echo "Usage: $0 <distro> [--build] [--help]"
            echo ""
            echo "Arguments:"
            echo "  distro      ROS distribution: noetic, humble, jazzy, rolling"
            echo ""
            echo "Options:"
            echo "  --build     Build Docker image before running"
            echo "  --help      Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0 humble             # Run E2E tests for ROS2 Humble"
            echo "  $0 noetic --build     # Build image and run E2E tests for ROS1"
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            exit 1
            ;;
    esac
done

if [ -z "$DISTRO" ]; then
    echo -e "${RED}Error: ROS distribution not specified${NC}"
    echo "Usage: $0 <distro> [--build]"
    echo "Distributions: noetic, humble, jazzy, rolling"
    exit 1
fi

# Set ROS version based on distro
if [ "$DISTRO" = "noetic" ]; then
    ROS_VERSION="1"
    DOCKERFILE="docker/Dockerfile.ros1"
else
    ROS_VERSION="2"
    DOCKERFILE="docker/Dockerfile.ros2.${DISTRO}"
fi

IMAGE_NAME="axon:ros${ROS_VERSION}-${DISTRO}"

echo -e "${BLUE}=============================================${NC}"
echo -e "${BLUE}Docker E2E Tests - ${DISTRO}${NC}"
echo -e "${BLUE}=============================================${NC}"
echo ""

# Build Docker image if requested or not exists
if [ "$BUILD_IMAGE" = true ] || ! docker image inspect ${IMAGE_NAME} &>/dev/null; then
    echo -e "${YELLOW}Building Docker image for ${DISTRO}...${NC}"
    DOCKER_BUILDKIT=1 docker build \
        -f "${PROJECT_ROOT}/${DOCKERFILE}" \
        -t ${IMAGE_NAME} \
        "${PROJECT_ROOT}" || {
        echo -e "${RED}Build failed. See docker/TROUBLESHOOTING.md${NC}"
        exit 1
    }
    echo -e "${GREEN}✓ Docker image built${NC}"
else
    echo -e "${GREEN}Using existing image: ${IMAGE_NAME}${NC}"
fi

echo ""
echo -e "${YELLOW}Running E2E tests in ${DISTRO} Docker container...${NC}"

DOCKER_BUILDKIT=1 docker run --rm \
    -v "${PROJECT_ROOT}:/workspace/axon" \
    -e ROS_DISTRO=${DISTRO} \
    -e ROS_VERSION=${ROS_VERSION} \
    --network host \
    ${IMAGE_NAME} \
    /usr/local/bin/run_e2e_tests.sh

echo ""
echo -e "${GREEN}=============================================${NC}"
echo -e "${GREEN}✓ ${DISTRO} E2E tests passed${NC}"
echo -e "${GREEN}=============================================${NC}"
