#!/usr/bin/env python3

# Copyright 2022 Clearpath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)


# header:
#   stamp:
#     sec: 1763078282
#     nanosec: 320479085
#   frame_id: map
# point:
#   x: -4.423905372619629
#   y: -1.2550456523895264
#   z: -0.001434326171875
# ---
# header:
#   stamp:
#     sec: 1763078294
#     nanosec: 724784038
#   frame_id: map
# point:
#   x: -6.432173252105713
#   y: 5.395950794219971
#   z: -0.005340576171875
# ---
# header:
#   stamp:
#     sec: 1763078303
#     nanosec: 394333113
#   frame_id: map
# point:
#   x: -3.7121074199676514
#   y: 6.206971168518066
#   z: -0.005340576171875
# ---
# header:
#   stamp:
#     sec: 1763078307
#     nanosec: 450214580
#   frame_id: map
# point:
#   x: -2.314723014831543
#   y: 0.6470600366592407
#   z: -0.0052490234375
# ---
# header:
#   stamp:
#     sec: 1763078321
#     nanosec: 833239134
#   frame_id: map
# point:
#   x: -7.920711040496826
#   y: -1.9573272466659546
#   z: -0.001434326171875
# ---
# header:
#   stamp:
#     sec: 1763078326
#     nanosec: 445965016
#   frame_id: map
# point:
#   x: -1.8310463428497314
#   y: -0.43005314469337463
#   z: -0.001434326171875
# ---

import rclpy

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator


def main():
    rclpy.init()

    navigator = TurtleBot4Navigator()

    # Start on dock
    if not navigator.getDockedStatus():
        navigator.info('Docking before intialising pose')
        navigator.dock()

    
    # Set goal poses
    goal_pose = []
    goal_pose.append(navigator.getPoseStamped([-1.255, -4.423905372619629], TurtleBot4Directions.SOUTH))
    goal_pose.append(navigator.getPoseStamped([5.395950794219971, -6.432173252105713], TurtleBot4Directions.WEST))
    goal_pose.append(navigator.getPoseStamped([6.206971168518066, -3.7121074199676514], TurtleBot4Directions.NORTH))
    # goal_pose.append(navigator.getPoseStamped([-2.31, 0.64], TurtleBot4Directions.EAST))

    # navigator.goToPose(goal_pose[0])
#   x: -4.423905372619629
#   y: -1.2550456523895264

#   x: -6.432173252105713
#   y: 5.395950794219971

#   x: -3.7121074199676514
#   y: 6.206971168518066

#   x: -2.314723014831543
#   y: 0.6470600366592407

#   x: -7.920711040496826
#   y: -1.9573272466659546

#   x: -1.8310463428497314
#   y: -0.43005314469337463



    # Undock
    navigator.undock()
    # Set initial pose
    # header:
    # stamp:
    #     sec: 1763526906
    #     nanosec: 510157548
    # frame_id: map
    # point:
    # x: -0.32711854577064514
    # y: 0.11737535893917084
    # z: 0.34820556640625
    # ---
    # header:
    # stamp:
    #     sec: 1763526916
    #     nanosec: 196155271
    # frame_id: map
    # point:
    # x: -1.2996826171875
    # y: -0.15014372766017914
    # z: 0.002471923828125
    # ---

    initial_pose = navigator.getPoseStamped([-1.11737535893917084, -0.32711854577064514], TurtleBot4Directions.SOUTH)
    navigator.setInitialPose(initial_pose)

    # # # Wait for Nav2
    print("Waiting for Nav2")
    navigator.waitUntilNav2Active()

    for pose in goal_pose:
        pose.header.stamp = navigator.get_clock().now().to_msg()
    # Follow Waypoints
    navigator.startFollowWaypoints(goal_pose)

    # # Finished navigating, dock
    navigator.dock()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
