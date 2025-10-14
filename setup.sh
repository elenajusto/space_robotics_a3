#!/bin/bash

# Source ROS Humble
source /opt/ros/humble/setup.bash

# Change to ros_workspace directory
cd ros_workspace

# Source the workspace setup
source install/setup.bash

# Print success message
echo "ROS environment has been set up!"

# Start a new shell with the sourced environment
exec bash