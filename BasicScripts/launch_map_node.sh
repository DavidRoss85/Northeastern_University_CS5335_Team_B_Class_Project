#!/bin/bash
echo "Launching Mapping node..."
echo "Sourcing ROS2 files..."
source /opt/ros/jazzy/setup.bash
source ../install/setup.bash
echo "Running python script..."
ros2 run object_location map_node