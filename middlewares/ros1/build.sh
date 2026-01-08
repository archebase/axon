#!/bin/bash
# Build script for ROS1 plugin

set -e  # Exit on error

echo "================================"
echo "Building ROS1 Plugin"
echo "================================"

# Step 1: Source ROS1 environment
echo ""
echo "Step 1: Sourcing ROS1 environment..."
if [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
elif [ -f /opt/ros/melodic/setup.bash ]; then
    source /opt/ros/melodic/setup.bash
elif [ -f /opt/ros/kinetic/setup.bash ]; then
    source /opt/ros/kinetic/setup.bash
else
    echo "Error: ROS1 setup.bash not found. Please source it manually."
    echo "Expected locations: /opt/ros/noetic/setup.bash, /opt/ros/melodic/setup.bash, or /opt/ros/kinetic/setup.bash"
    exit 1
fi

# Step 2: Build the ROS1 plugin library
echo ""
echo "Step 2: Building ROS1 plugin library..."
cd ros1
catkin build --packages-select ros1_plugin
# Alternative: catkin_make --only-pkg-with-deps ros1_plugin

# Step 3: Copy the built library to examples directory
echo ""
echo "Step 3: Copying library to examples directory..."
cp devel/lib/libros1_plugin.so examples/ros1/

# Step 4: Build the example program
echo ""
echo "Step 4: Building example program..."
cd examples/ros1
mkdir -p build
cd build
cmake ..
make

echo ""
echo "================================"
echo "Build completed successfully!"
echo "================================"
echo ""
echo "To run the example:"
echo "  cd ros1/examples/ros1"
echo "  ./subscriber_test [topic_name] [message_type]"
echo ""
echo "Example:"
echo "  ./subscriber_test chatter std_msgs/String"
echo ""
echo "Make sure to run 'roscore' first in another terminal."
echo ""
