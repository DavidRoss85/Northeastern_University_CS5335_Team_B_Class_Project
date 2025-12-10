#!/usr/bin/env python3
"""
Driving Node - Action server for executing precise distance and angle commands
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import TwistStamped
from node1pkg_interfaces.action import DriveCommand
import math

class PubNode(Node):
    def __init__(self):
        super().__init__('pub_node')
        self.publisher_ = self.create_publisher(TwistStamped, 'cmd_vel', 10)

    def publish(self,message):
        self.publisher_.publish(message)
       
class DrivingNode(Node):
    def __init__(self):
        super().__init__('driving_node')
        self._action_server = ActionServer(
            self,
            DriveCommand,
            'drive_command',
            self.execute_callback
        )
        self.publisher_ = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.get_logger().info('Driving Node started and ready to receive commands')
        self.action_progress = 0
        self.pub_node = PubNode()

    def execute_callback(self, goal_handle):
        """Execute the drive command"""
        self.get_logger().info('Executing drive command...')
        
        goal = goal_handle.request
        distance = goal.distance
        angle = goal.angle
        
        # Validate request
        if distance != 0.0 and angle != 0.0:
            self.get_logger().warn('Request rejected: both distance and angle are non-zero')
            goal_handle.abort()
            result = DriveCommand.Result()
            result.success = False
            result.message = "Invalid request: both distance and angle cannot be non-zero"
            return result
        
        # if distance < 0.0 or angle < 0.0:
        if distance < 0.0:            
            self.get_logger().warn('Request rejected: negative values not allowed')
            goal_handle.abort()
            result = DriveCommand.Result()
            result.success = False
            result.message = "Invalid request: negative values not allowed"
            return result
        
        feedback_msg = DriveCommand.Feedback()
        
        if distance != 0.0:
            # Handle distance movement
            self.get_logger().info(f'Moving distance: {distance} meters')
            success = self.execute_distance_move(distance, goal_handle, feedback_msg)
            message = f"Moved {distance} meters"
        elif angle != 0.0:
            # Handle angle turn
            self.get_logger().info(f'Turning angle: {angle} radians ({math.degrees(angle):.1f} degrees)')
            success = self.execute_angle_turn(angle, goal_handle, feedback_msg)
            message = f"Turned {angle} radians ({math.degrees(angle):.1f} degrees)"          
        else:
            # Both are zero - no movement needed
            self.get_logger().info('No movement requested (both distance and angle are 0)')
            success = True
            message = "No movement needed"
        
        # Send final result
        goal_handle.succeed()
        result = DriveCommand.Result()
        result.success = success
        result.message = message
        return result

    def execute_distance_move(self, distance, goal_handle, feedback_msg):
        """Execute a distance-based movement"""
        # Use linear velocity of 0.2 m/s
        linear_velocity = 0.2
        distance = distance / 1.03 + 0.08
        duration = distance / linear_velocity  # Calculate time needed
        
        # Create timer for movement
        steps = int(duration * 10)  # 10 Hz update rate
        step_duration = 0.1
        for step in range(steps):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.stop_robot()
                return False
            
            # Create and publish movement command
            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.header.frame_id = 'base_link'
            twist_msg.twist.linear.x = linear_velocity
            twist_msg.twist.linear.y = 0.0
            twist_msg.twist.linear.z = 0.0
            twist_msg.twist.angular.x = 0.0
            twist_msg.twist.angular.y = 0.0
            twist_msg.twist.angular.z = 0.0
            
            
            # self.publisher_.publish(twist_msg)
            self.pub_node.publish(twist_msg)
            # Send feedback
            feedback_msg.progress = float(step) / float(steps)
            goal_handle.publish_feedback(feedback_msg)
            
            # Wait for next step
            rclpy.spin_once(self.pub_node, timeout_sec=step_duration)         
        
        # Stop the robot
        self.stop_robot()
        return True

    def execute_angle_turn(self, angle, goal_handle, feedback_msg):
        """Execute an angle-based turn"""
        # Use angular velocity of 0.5 rad/s
        if angle < 0:
            angular_velocity = -0.5
        else:
            angular_velocity = 0.5

        duration = angle / angular_velocity  # Calculate time needed
        
        # Create timer for movement
        steps = int(duration * 10)  # 10 Hz update rate
        step_duration = 0.1
        for step in range(steps):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                # self.stop_robot()
                return False
            
            # Create and publish turn command
            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.header.frame_id = 'base_link'
            twist_msg.twist.linear.x = 0.0
            twist_msg.twist.linear.y = 0.0
            twist_msg.twist.linear.z = 0.0
            twist_msg.twist.angular.x = 0.0
            twist_msg.twist.angular.y = 0.0
            twist_msg.twist.angular.z = angular_velocity
            
            # self.publisher_.publish(twist_msg)
            self.pub_node.publish(twist_msg)
            
            # Send feedback
            feedback_msg.progress = float(step) / float(steps)
            goal_handle.publish_feedback(feedback_msg)
            
            # Wait for next step
            rclpy.spin_once(self.pub_node, timeout_sec=step_duration)
        
        # Stop the robot
        self.stop_robot()
        return True

    def stop_robot(self):
        """Send a stop command to the robot"""
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = 'base_link'
        twist_msg.twist.linear.x = 0.0
        twist_msg.twist.linear.y = 0.0
        twist_msg.twist.linear.z = 0.0
        twist_msg.twist.angular.x = 0.0
        twist_msg.twist.angular.y = 0.0
        twist_msg.twist.angular.z = 0.0
        
        self.pub_node.publish(twist_msg)
        self.get_logger().info('Robot stopped')

def main(args=None):
    rclpy.init(args=args)
    
    driving_node = DrivingNode()
    
    try:
        rclpy.spin(driving_node)
    except KeyboardInterrupt:
        pass
    finally:
        driving_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()