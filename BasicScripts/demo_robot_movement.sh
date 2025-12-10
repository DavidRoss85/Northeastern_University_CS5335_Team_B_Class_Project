#!/bin/bash
echo "Starting robot movement script..."
echo "Updating permissions..."
chmod +x launch_localizer.sh launch_nav2_navigator.sh launch_rviz.sh launch_nav2_slam_toolbox.sh
sleep 3
echo "Launching localization in separate terminal..."
gnome-terminal --working-directory="$PWD" -- bash -c "./launch_localizer.sh; exec bash"
echo "Waiting for localization to stabilize..."
sleep 5
echo "Launching Nav2 in separate terminal..."
gnome-terminal --working-directory="$PWD" -- bash -c "./launch_nav2_navigator.sh; exec bash"
sleep 5
echo "Launching RViz in separate terminal..."
gnome-terminal --working-directory="$PWD" -- bash -c "./launch_rviz.sh; exec bash"
sleep 5
echo "Sourcing ROS2 files..."
source /opt/ros/jazzy/setup.bash
source ../install/setup.bash
echo "Running python script..."
ros2 run pathfind predefined