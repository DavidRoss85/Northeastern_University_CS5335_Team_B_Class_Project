#!/usr/bin/env python3

# Original tutorial version
# Copyright 2022 Clearpath Robotics, Inc.
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

# Modified version by Team B

# ROS2 imports
import rclpy
# TurtleBot4 Navigator imports (These are available via the turtlebot4 common and turtlebot4_navigation packages)
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator


# Predefined path coordinates
# These were found using RViz and the Publish Point tool
# Each point is defined as [x, y, z, yaw_direction]
#   x: -3.611196279525757
#   y: 1.0309427976608276
#   z: 0.002471923828125

test_path = [[-3.61,1.03,0.0,TurtleBot4Directions.SOUTH]]
predifined_path = [
    [-0.1924143135547638, -0.028697526082396507, 0.34039306640625, TurtleBot4Directions.SOUTH],
    [-2.03563785552978, -0.028697308152914047, -0.005340576171875, TurtleBot4Directions.SOUTH],
    [-3.7649571895599365, 0.9566565155982971, -0.001434326171875, TurtleBot4Directions.SOUTH],
    [-3.487504243850708, 6.843094825744629, -0.001434326171875, TurtleBot4Directions.WEST],
    [-0.39209800958633423, 6.831547260284424, -0.001434326171875, TurtleBot4Directions.NORTH],
    [-0.6808493137359619, 0.9428840279579163, -0.001434326171875, TurtleBot4Directions.EAST]
]

#------------------------------------------------------------------------------------
def main():
    print("/n**************** Starting Predefined Path Navigation *****************/n")

    initial_x = -1.0 #predifined_path[0][0]
    initial_y = 0.1 # predifined_path[0][1]
    initial_yaw = predifined_path[0][3]
    rclpy.init()

    navigator = TurtleBot4Navigator()
    # Start on dock
    if not navigator.getDockedStatus():
        navigator.info('Docking before intialising pose')
        navigator.dock()

    # Undock
    navigator.undock()
    
    
    # Set goal poses
    goal_pose = []
    for i,point in enumerate(predifined_path):
        if i>2:
            goal_pose.append(navigator.getPoseStamped([point[0], point[1]], point[3]))
            # goal_pose.append(navigator.getPoseStamped([PredefinedPath.x1, PredefinedPath.y1], TurtleBot4Directions.SOUTH))

    # Update the timestamp of each goal pose. Use this if there are delays between defining the poses and sending them
    for pose in goal_pose:
        pose.header.stamp = navigator.get_clock().now().to_msg()
        
    # Set initial pose. This must be done after undocking
    initial_pose = navigator.getPoseStamped([initial_x, initial_y], initial_yaw)
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2
    navigator.info('Waiting for Nav2 to become active')
    navigator.waitUntilNav2Active()    
    
    # Follow Waypoints
    navigator.startFollowWaypoints(goal_pose)

    # # Finished navigating, dock
    navigator.dock()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
