#!/bin/bash
# SPDX-FileCopyrightText: 2026 ArcheBase
# SPDX-License-Identifier: MulanPSL-2.0

set -e

# =============================================================================
# Axon Docker Package Build Script
# =============================================================================
# Builds Debian packages inside Docker containers for reproducible builds.
# Usage:
#   build-in-docker.sh [standalone|ros2|ros1|all]
#   build-in-docker.sh --distro <humble|jazzy|rolling|noetic>
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$(dirname "${SCRIPT_DIR}")/../.." && pwd)"
DOCKER_DIR="${PROJECT_ROOT}/docker"
OUTPUT_DIR="${PROJECT_ROOT}/packaging/deb/output"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

log_section() {
    echo -e "${BLUE}=== $1 ===${NC}"
}

# Check if Docker is available
if ! command -v docker &> /dev/null; then
    log_error "Docker not found. Please install Docker first."
    exit 1
fi

# Create output directory
mkdir -p "${OUTPUT_DIR}"

# Parse arguments
BUILD_TYPE="${1:-standalone}"
ROS_DISTRO="${ROS_DISTRO:-humble}"

case "$BUILD_TYPE" in
    standalone)
        DOCKERFILE="Dockerfile.package-standalone"
        BUILD_SCRIPT="build-standalone.sh"
        IMAGE_NAME="axon-package-standalone"
        ;;
    ros2)
        DOCKERFILE="Dockerfile.package-ros2.${ROS_DISTRO}"
        BUILD_SCRIPT="build-ros2.sh"
        IMAGE_NAME="axon-package-ros2-${ROS_DISTRO}"
        ;;
    ros1)
        DOCKERFILE="Dockerfile.package-ros1"
        BUILD_SCRIPT="build-ros1.sh"
        IMAGE_NAME="axon-package-ros1"
        ;;
    all)
        log_section "Building All Packages in Docker"

        # Build standalone packages
        "$0" standalone

        # Build ROS2 packages if Dockerfile exists
        for distro in humble jazzy rolling; do
            if [ -f "${DOCKER_DIR}/Dockerfile.package-ros2.${distro}" ]; then
                log_section "Building ROS2 ${distro^} Packages"
                "$0" --distro "$distro" || log_warn "Failed to build ROS2 ${distro}"
            fi
        done

        # Build ROS1 packages if Dockerfile exists
        if [ -f "${DOCKER_DIR}/Dockerfile.package-ros1" ]; then
            log_section "Building ROS1 Noetic Packages"
            "$0" ros1 || log_warn "Failed to build ROS1 packages"
        fi

        log_section "All Docker Builds Complete"
        exit 0
        ;;
    --distro)
        # Allow specifying distro directly
        ROS_DISTRO="$2"
        DOCKERFILE="Dockerfile.package-ros2.${ROS_DISTRO}"
        BUILD_SCRIPT="build-ros2.sh"
        IMAGE_NAME="axon-package-ros2-${ROS_DISTRO}"
        ;;
    *)
        log_error "Unknown build type: $BUILD_TYPE"
        echo "Usage: $0 [standalone|ros2|ros1|all|--distro <name>]"
        exit 1
        ;;
esac

# Verify Dockerfile exists
DOCKERFILE_PATH="${DOCKER_DIR}/${DOCKERFILE}"
if [ ! -f "$DOCKERFILE_PATH" ]; then
    log_error "Dockerfile not found: $DOCKERFILE_PATH"
    exit 1
fi

log_section "Building ${IMAGE_NAME} Image"

# Build Docker image
if docker build -t "$IMAGE_NAME" -f "$DOCKERFILE_PATH" "${PROJECT_ROOT}"; then
    log_info "Docker image built successfully"
else
    log_error "Failed to build Docker image"
    exit 1
fi

# Create a container to build the packages
log_section "Building Packages in Container"

# Run build in container
docker run --rm \
    -v "${PROJECT_ROOT}:/axon:rw" \
    -e ROS_DISTRO="${ROS_DISTRO}" \
    -e VERSION="${VERSION}" \
    -w /axon \
    "$IMAGE_NAME" \
    "/usr/local/bin/${BUILD_SCRIPT}" || {
        log_error "Build failed in container"
        exit 1
    }

# Ensure output directory has correct permissions
docker run --rm \
    -v "${PROJECT_ROOT}:/axon:rw" \
    -w /axon \
    "$IMAGE_NAME" \
    chown -R "$(id -u):$(id -g)" packaging/deb/output 2>/dev/null || true

log_section "Docker Build Complete"

# Show built packages
if [ -d "$OUTPUT_DIR" ]; then
    log_info "Packages created:"
    ls -lh "${OUTPUT_DIR}"/*.deb 2>/dev/null || log_warn "No packages found"
fi

log_info "==================================================================="
