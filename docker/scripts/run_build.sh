#!/bin/bash
# Build script for Docker containers

set -e

WORKSPACE_DIR="/workspace/edge_lance_recorder"

cd "$WORKSPACE_DIR"

echo "=========================================="
echo "Building Edge Lance Recorder"
echo "ROS_VERSION: ${ROS_VERSION:-unknown}"
echo "ROS_DISTRO: ${ROS_DISTRO:-unknown}"
echo "=========================================="
echo ""

# Colors
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NC='\033[0m'

# 1. Build Rust library
echo -e "${YELLOW}Building Rust bridge library...${NC}"
cd src/bridge
cargo build --release
cd "$WORKSPACE_DIR"
echo -e "${GREEN}✓ Rust library built${NC}"
echo ""

# 2. Build C++ code
echo -e "${YELLOW}Building C++ code...${NC}"
mkdir -p build
cd build

if [ "$ROS_VERSION" = "1" ]; then
    source /opt/ros/${ROS_DISTRO}/setup.bash
    cmake .. -DCMAKE_BUILD_TYPE=Release -DROS_VERSION=1
else
    source /opt/ros/${ROS_DISTRO}/setup.bash
    cmake .. -DCMAKE_BUILD_TYPE=Release -DROS_VERSION=2
fi

cmake --build . -j$(nproc 2>/dev/null || echo 4)
cd "$WORKSPACE_DIR"
echo -e "${GREEN}✓ C++ code built${NC}"
echo ""

echo -e "${GREEN}Build complete!${NC}"
echo "=========================================="



