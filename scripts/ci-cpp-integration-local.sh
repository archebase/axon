#!/bin/bash
# =============================================================================
# Local CI Script for C++ Integration Tests
# =============================================================================
# Mirrors .github/workflows/integration-tests-cpp.yml for local testing.
# Runs EdgeUploader integration tests with MinIO as S3 backend.
#
# Usage:
#   ./scripts/ci-cpp-integration-local.sh [--coverage] [--no-minio]
#
# Options:
#   --coverage    Run with coverage instrumentation
#   --no-minio    Don't start MinIO (use existing instance)
#   --help        Show this help message
#
# Prerequisites:
#   - Docker (for MinIO)
#   - AWS SDK for C++ installed
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
START_MINIO=true
MINIO_CONTAINER="minio-server-local"
MINIO_ACCESS_KEY="minioadmin"
MINIO_SECRET_KEY="minioadmin"
MINIO_PORT=9000
MINIO_CONSOLE_PORT=9001

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --coverage)
            RUN_COVERAGE=true
            shift
            ;;
        --no-minio)
            START_MINIO=false
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [--coverage] [--no-minio] [--help]"
            echo ""
            echo "Options:"
            echo "  --coverage    Run with coverage instrumentation"
            echo "  --no-minio    Don't start MinIO (use existing instance)"
            echo "  --help        Show this help message"
            echo ""
            echo "This script mirrors the GitHub Actions workflow for C++ integration tests."
            echo "Requires Docker for MinIO and AWS SDK for C++."
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            exit 1
            ;;
    esac
done

echo -e "${BLUE}=============================================${NC}"
echo -e "${BLUE}Axon C++ Integration Tests Local CI${NC}"
echo -e "${BLUE}=============================================${NC}"
echo ""
echo -e "Coverage: ${YELLOW}${RUN_COVERAGE}${NC}"
echo -e "Start MinIO: ${YELLOW}${START_MINIO}${NC}"
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
# Step 1: Start MinIO Server
# =============================================================================
if [ "$START_MINIO" = true ]; then
    echo -e "${YELLOW}Step 1: Starting MinIO server...${NC}"

    # Cleanup existing container
    docker rm -f ${MINIO_CONTAINER} 2>/dev/null || true

    # Start MinIO
    docker run -d --name ${MINIO_CONTAINER} \
        --network host \
        -e MINIO_ROOT_USER=${MINIO_ACCESS_KEY} \
        -e MINIO_ROOT_PASSWORD=${MINIO_SECRET_KEY} \
        quay.io/minio/minio server /data --console-address ":${MINIO_CONSOLE_PORT}"

    # Wait for MinIO to be ready
    echo -e "${BLUE}Waiting for MinIO to start...${NC}"
    MINIO_READY=false
    for i in {1..30}; do
        if curl -sf http://localhost:${MINIO_PORT}/minio/health/live 2>/dev/null; then
            echo -e "${GREEN}MinIO is ready!${NC}"
            MINIO_READY=true
            break
        fi
        echo "Waiting... ($i/30)"
        sleep 1
    done

    if [ "$MINIO_READY" != "true" ]; then
        echo -e "${RED}ERROR: MinIO failed to start within 30 seconds${NC}"
        echo "Docker container logs:"
        docker logs ${MINIO_CONTAINER} || true
        exit 1
    fi

    # Create test bucket
    echo -e "${BLUE}Creating test bucket...${NC}"
    docker run --rm --network host \
        -e MC_HOST_local=http://${MINIO_ACCESS_KEY}:${MINIO_SECRET_KEY}@localhost:${MINIO_PORT} \
        quay.io/minio/mc mb local/axon-raw-data --ignore-existing

    echo -e "${GREEN}Test bucket 'axon-raw-data' created${NC}"
else
    echo -e "${YELLOW}Skipping MinIO startup (--no-minio flag)${NC}"
fi

echo ""

# =============================================================================
# Step 2: Build axon_uploader
# =============================================================================
echo -e "${YELLOW}Step 2: Building axon_uploader...${NC}"

cd "${PROJECT_ROOT}"

BUILD_TYPE="Release"
if [ "$RUN_COVERAGE" = true ]; then
    BUILD_TYPE="Debug"
fi

make build-uploader BUILD_TYPE=${BUILD_TYPE} \
    ${RUN_COVERAGE:+AXON_ENABLE_COVERAGE=ON}

echo ""

# =============================================================================
# Step 3: Run Integration Tests
# =============================================================================
echo -e "${YELLOW}Step 3: Running integration tests...${NC}"

cd "${PROJECT_ROOT}/build/axon_uploader"

# Set environment for MinIO
export AWS_ACCESS_KEY_ID=${MINIO_ACCESS_KEY}
export AWS_SECRET_ACCESS_KEY=${MINIO_SECRET_KEY}

# Run the integration test
if [ -f "./test_edge_uploader" ]; then
    ./test_edge_uploader --gtest_output=xml:test_results.xml
    echo -e "${GREEN}Integration test completed successfully!${NC}"
else
    echo -e "${RED}Integration test binary not found: test_edge_uploader${NC}"
    echo "Available test binaries:"
    ls -la test_* 2>/dev/null || echo "No test binaries found"
    exit 1
fi

echo ""

# =============================================================================
# Step 4: Generate Coverage Report
# =============================================================================
if [ "$RUN_COVERAGE" = true ]; then
    echo -e "${YELLOW}Step 4: Generating coverage report...${NC}"

    lcov --capture --directory . --output-file coverage_raw.info \
        --rc lcov_branch_coverage=1 ${LCOV_IGNORE_FLAGS} 2>/dev/null || {
        echo -e "${YELLOW}Warning: lcov capture had issues, trying with more permissive flags...${NC}"
        if [ "${LCOV_MAJOR}" -ge 2 ]; then
            lcov --capture --directory . --output-file coverage_raw.info \
                --rc lcov_branch_coverage=1 --ignore-errors mismatch,unused,negative,gcov,source 2>/dev/null || true
        else
            lcov --capture --directory . --output-file coverage_raw.info \
                --rc lcov_branch_coverage=1 2>/dev/null || true
        fi
    }

    lcov --remove coverage_raw.info \
        '/usr/*' '/opt/*' '*/_deps/*' '*/test/*' '*/c++/*' \
        --output-file coverage.info --rc lcov_branch_coverage=1 ${LCOV_IGNORE_FLAGS} 2>/dev/null || {
        echo -e "${YELLOW}Warning: lcov filtering had issues, using raw coverage...${NC}"
        cp coverage_raw.info coverage.info
    }

    echo ""
    echo -e "${GREEN}Coverage Summary:${NC}"
    lcov --list coverage.info ${LCOV_IGNORE_FLAGS} 2>/dev/null || echo "No coverage data"
fi

# =============================================================================
# Cleanup
# =============================================================================
cleanup() {
    if [ "$START_MINIO" = true ]; then
        echo ""
        echo -e "${YELLOW}Cleaning up MinIO...${NC}"
        docker stop ${MINIO_CONTAINER} 2>/dev/null || true
        docker rm ${MINIO_CONTAINER} 2>/dev/null || true
    fi
}

trap cleanup EXIT

echo ""
echo -e "${GREEN}=============================================${NC}"
echo -e "${GREEN}Local CI completed successfully!${NC}"
echo -e "${GREEN}=============================================${NC}"
