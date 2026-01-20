#!/bin/bash
# Demo script showing how to test the ROS2 plugin with ros2 topic pub CLI tool

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

# Plugin path - use environment variable, command line argument, or default relative path
if [ -n "$AXON_ROS2_PLUGIN_PATH" ]; then
  PLUGIN_PATH="$AXON_ROS2_PLUGIN_PATH"
elif [ -n "$1" ]; then
  PLUGIN_PATH="$1"
else
  # Default relative path from project root
  PLUGIN_PATH="${PROJECT_ROOT}/middlewares/ros2/install/axon_ros2_plugin/lib/axon/plugins/libaxon_ros2_plugin.so"
fi

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================"
echo "  ROS2 Plugin CLI Tools Test"
echo "========================================${NC}"
echo ""
echo "This test will:"
echo "  1. Start the plugin subscriber in background"
echo "  2. Publish messages using 'ros2 topic pub'"
echo "  3. Verify the plugin receives the messages"
echo ""
echo "Plugin path: $PLUGIN_PATH"
echo ""

# Source ROS2
source /opt/ros/humble/setup.bash

cd "$SCRIPT_DIR"

# Check if plugin exists
if [ ! -f "$PLUGIN_PATH" ]; then
    echo -e "${RED}[ERROR] Plugin not found: $PLUGIN_PATH${NC}"
    exit 1
fi

# Start the subscriber in background
echo -e "${YELLOW}[INFO] Starting plugin subscriber...${NC}"
./build/test_with_ros2_tools "$PLUGIN_PATH" 20 > /tmp/plugin_test_output.txt 2>&1 &
SUBSCRIBER_PID=$!

# Wait for plugin to initialize
sleep 2

# Check if subscriber is still running
if ! kill -0 $SUBSCRIBER_PID 2>/dev/null; then
    echo -e "${RED}[ERROR] Subscriber failed to start${NC}"
    cat /tmp/plugin_test_output.txt
    exit 1
fi

echo -e "${GREEN}[OK] Subscriber started (PID: $SUBSCRIBER_PID)${NC}"
echo ""

# Publish some messages
echo -e "${YELLOW}========================================"
echo "  Publishing Messages"
echo "========================================${NC}"
echo ""

for i in {1..5}; do
    echo -e "${YELLOW}[PUBLISH] Sending message $i...${NC}"
    ros2 topic pub --once /chatter std_msgs/String "data: 'Hello from CLI $i'" >/dev/null 2>&1 &
    sleep 0.5
done

echo ""
echo -e "${GREEN}[INFO] Published 5 messages${NC}"
echo ""

# Wait for subscriber to process all messages
echo -e "${YELLOW}[INFO] Waiting for subscriber to finish (20 seconds)...${NC}"
wait $SUBSCRIBER_PID 2>/dev/null || true
EXIT_CODE=$?

# Display results
cat /tmp/plugin_test_output.txt

echo ""
echo -e "${GREEN}========================================"
echo "  Test Complete"
echo "========================================${NC}"
echo ""

# Check if any messages were received
if grep -q "MSG 1" /tmp/plugin_test_output.txt; then
    MESSAGE_COUNT=$(grep -c "^\\[MSG" /tmp/plugin_test_output.txt || echo "0")
    echo -e "${GREEN}[SUCCESS] Plugin received $MESSAGE_COUNT messages!${NC}"
    exit 0
else
    echo -e "${RED}[FAIL] No messages received${NC}"
    exit 1
fi
