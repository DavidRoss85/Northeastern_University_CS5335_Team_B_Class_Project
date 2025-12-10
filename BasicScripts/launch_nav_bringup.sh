#!/bin/bash
echo "Launching Nav2 Stack..."
source /opt/ros/jazzy/setup.bash

ros2 launch nav2_bringup navigation_launch.py slam:=true use_sim_time:=false