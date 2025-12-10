#!/bin/bash
echo "Launching Nav2 Setup Node..."
source /opt/ros/jazzy/setup.bash
source ../install/setup.bash
ros2 run object_location navigator_node