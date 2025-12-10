#!/usr/bin/env python3
"""
Pattern Driver Node - Drives TurtleBot in various geometric patterns
Based on the original node1 requirements with all driving patterns
"""


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import math


class PatternDriverNode(Node):
    def __init__(self):
        super().__init__('pattern_driver')
        self.publisher_ = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        # self.timer = self.create_timer(1.0, self.timer_callback)
        self.callback_count = 0
        self.get_logger().info('Pattern Driver Node started')


    def create_linear_twist(self, x):
        """Create a TwistStamped with linear x velocity"""
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = 'base_link'
        twist_stamped.twist.linear.x = x
        twist_stamped.twist.linear.y = 0.0
        twist_stamped.twist.linear.z = 0.0
        twist_stamped.twist.angular.x = 0.0
        twist_stamped.twist.angular.y = 0.0
        twist_stamped.twist.angular.z = 0.0
        return twist_stamped


    def create_angular_twist(self, z):
        """Create a TwistStamped with angular z velocity"""
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = 'base_link'
        twist_stamped.twist.linear.x = 0.0
        twist_stamped.twist.linear.y = 0.0
        twist_stamped.twist.linear.z = 0.0
        twist_stamped.twist.angular.x = 0.0
        twist_stamped.twist.angular.y = 0.0
        twist_stamped.twist.angular.z = z
        return twist_stamped


    def create_combined_twist(self, linear_x, angular_z):
        """Create a TwistStamped with both linear and angular velocities"""
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = 'base_link'
        twist_stamped.twist.linear.x = linear_x
        twist_stamped.twist.linear.y = 0.0
        twist_stamped.twist.linear.z = 0.0
        twist_stamped.twist.angular.x = 0.0
        twist_stamped.twist.angular.y = 0.0
        twist_stamped.twist.angular.z = angular_z
        return twist_stamped


    def timer_callback(self):
        """Default timer callback - 3-step demo sequence"""
        self.callback_count += 1
       
        if self.callback_count == 1:
            msg = self.create_linear_twist(0.2)
            self.get_logger().info(f'Step 1: Publishing linear twist: x={msg.twist.linear.x}')
        elif self.callback_count == 2:
            msg = self.create_angular_twist(0.5)
            self.get_logger().info(f'Step 2: Publishing angular twist: z={msg.twist.angular.z}')
        elif self.callback_count == 3:
            msg = self.create_combined_twist(0.1, 0.3)
            self.get_logger().info(f'Step 3: Publishing combined twist: x={msg.twist.linear.x}, z={msg.twist.angular.z}')
        else:
            msg = self.create_linear_twist(0.0)
            self.publisher_.publish(msg)
            self.get_logger().info('Demo completed - stopping robot and destroying node')
            self.destroy_node()
            return


        self.publisher_.publish(msg)


    # Pattern driving methods
    def drive_straight_50cm(self):
        """Drive the TurtleBot straight for 50 cm"""
        self.timer.cancel()
        self.timer = self.create_timer(0.1, self.straight_callback)
        self.start_time = self.get_clock().now()
        self.target_duration = 5  # 50cm at 0.1 m/s


    def straight_callback(self):
        """Callback for driving straight"""
        current_time = self.get_clock().now()
        elapsed = (current_time - self.start_time).nanoseconds / 1e9
       
        if elapsed < self.target_duration:
            msg = self.create_linear_twist(0.1)
            self.publisher_.publish(msg)
        else:
            msg = self.create_linear_twist(0.0)
            self.publisher_.publish(msg)
            self.get_logger().info('Completed 50cm straight drive')
            self.destroy_node()


    def drive_circle_50cm_radius(self):
        """Drive in a circle with 50cm radius"""
        self.timer.cancel()
        self.timer = self.create_timer(0.1, self.circle_callback)
        self.start_time = self.get_clock().now()
        self.target_duration = math.pi / 0.2  # Full circle


    def circle_callback(self):
        """Callback for driving in a circle"""
        current_time = self.get_clock().now()
        elapsed = (current_time - self.start_time).nanoseconds / 1e9
       
        if elapsed < self.target_duration:
            msg = self.create_combined_twist(0.1, 0.2)  # v=0.1, ω=0.2 for 50cm radius
            self.publisher_.publish(msg)
        else:
            msg = self.create_linear_twist(0.0)
            self.publisher_.publish(msg)
            self.get_logger().info('Completed 50m radius circle')
            self.destroy_node()


    def drive_90_degree_arc(self):
        """Drive 90 degrees of a circle with 50m radius"""
        self.timer.cancel()
        self.timer = self.create_timer(0.1, self.arc_callback)
        self.start_time = self.get_clock().now()
        self.target_duration = (math.pi / 4) / 0.2  # 90 degrees


    def arc_callback(self):
        """Callback for driving a 90-degree arc"""
        current_time = self.get_clock().now()
        elapsed = (current_time - self.start_time).nanoseconds / 1e9
       
        if elapsed < self.target_duration:
            msg = self.create_combined_twist(0.2, 0.2)
            self.publisher_.publish(msg)
        else:
            msg = self.create_linear_twist(0.0)
            self.publisher_.publish(msg)
            self.get_logger().info('Completed 90-degree arc')
            self.destroy_node()


    def drive_letter_p(self):
        """Drive a path like the letter P"""
        self.timer.cancel()
        self.timer = self.create_timer(0.1, self.letter_p_callback)
        self.start_time = self.get_clock().now()
        self.phase = 1


    def letter_p_callback(self):
        """Callback for driving letter P pattern"""
        current_time = self.get_clock().now()
        elapsed = (current_time - self.start_time).nanoseconds / 1e9
       
        if self.phase == 1:
            # Phase 1: Drive straight for 50cm
            if elapsed < 5.5:
                msg = self.create_linear_twist(0.1)
                self.publisher_.publish(msg)
            else:
                self.phase = 2
                self.start_time = self.get_clock().now()
        elif self.phase == 2:
            # Phase 2: Drive in circle with 50cm radius
            circle_duration = math.pi / 0.1  # Half circle
            if elapsed < circle_duration:
                msg = self.create_combined_twist(0.1, 0.2)
                self.publisher_.publish(msg)
            else:
                msg = self.create_linear_twist(0.0)
                self.publisher_.publish(msg)
                self.get_logger().info('Completed letter P pattern')
                self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PatternDriverNode()
   
    # Uncomment one of these to test different patterns:
    # node.drive_straight_50cm()
    # node.drive_circle_1m_radius()
    # node.drive_90_degree_arc()
    node.drive_letter_p()
   
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()

