#!/bin/bash
# Test runner script for Docker containers
# Runs all tests (Rust + C++)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="/workspace/edge_lance_recorder"

cd "$WORKSPACE_DIR"

echo "=========================================="
echo "Running Edge Lance Recorder Tests"
echo "ROS_VERSION: ${ROS_VERSION:-unknown}"
echo "ROS_DISTRO: ${ROS_DISTRO:-unknown}"
echo "=========================================="
echo ""

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NC='\033[0m'

TEST_FAILED=0

# Function to run test and capture result
run_test() {
    local test_name=$1
    shift
    echo -e "${YELLOW}Running: ${test_name}...${NC}"
    if "$@"; then
        echo -e "${GREEN}✓ ${test_name} passed${NC}"
        echo ""
        return 0
    else
        echo -e "${RED}✗ ${test_name} failed${NC}"
        echo ""
        TEST_FAILED=1
        return 1
    fi
}

# 1. Rust tests
echo "=== Rust Tests ==="
run_test "Rust Unit Tests" \
    bash -c "cd src/bridge && cargo test --release"

# 2. C++ tests
echo "=== C++ Tests ==="

# Build C++ tests
echo -e "${YELLOW}Building C++ tests...${NC}"
mkdir -p test/build
cd test/build

if [ "$ROS_VERSION" = "1" ]; then
    source /opt/ros/${ROS_DISTRO}/setup.bash
    cmake ../.. -DROS_VERSION=1
else
    source /opt/ros/${ROS_DISTRO}/setup.bash
    cmake ../.. -DROS_VERSION=2
fi

cmake --build . -j$(nproc 2>/dev/null || echo 4)
echo -e "${GREEN}✓ C++ tests built${NC}"
echo ""

# Run C++ tests
run_test "C++ Unit Tests" ctest --output-on-failure

cd "$WORKSPACE_DIR"

# 3. Integration tests (if available)
if [ -f "test/integration/test_rust_bridge.sh" ]; then
    echo "=== Integration Tests ==="
    run_test "Rust Bridge Integration" \
        bash test/integration/test_rust_bridge.sh
fi

# Summary
echo "=========================================="
if [ $TEST_FAILED -eq 0 ]; then
    echo -e "${GREEN}All tests passed!${NC}"
    echo "=========================================="
    exit 0
else
    echo -e "${RED}Some tests failed!${NC}"
    echo "=========================================="
    exit 1
fi



