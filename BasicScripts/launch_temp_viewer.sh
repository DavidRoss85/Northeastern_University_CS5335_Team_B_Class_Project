#!/bin/bash
echo "Launching Temp Viewer..."
echo "Sourcing ROS2 files..."
source /opt/ros/jazzy/setup.bash
source ../install/setup.bash
echo "Running python script..."
ros2 run object_location temp_viewer