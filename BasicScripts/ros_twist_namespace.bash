#!/bin/bash

export ROS_NAMESPACE="/TBOT_6"

ros2 topic pub ${ROS_NAMESPACE}/cmd_vel geometry_msgs/msg/TwistStamped \
"twist:
  linear:
    x: 0.2
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0"