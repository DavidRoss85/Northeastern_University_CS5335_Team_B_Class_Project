#!/bin/bash

export ROS_NAMESPACE="/TBOT_6"

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r cmd_vel:=${ROS_NAMESPACE}/cmd_vel