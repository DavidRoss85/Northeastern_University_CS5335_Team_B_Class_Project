# Following the instructions in today's class powerpoint:
# Set up your GitHub Classroom folder to be a ROS Workspace
# Create a package called node1pkg
# In this package, create a node:
# Filename node1.py
# Executable name node1
# In this node, create a publisher for the same topic in our ros_twist script
# You will need a dependency on the geometry_msgs interface package
# To start with, just publish one TwistStamped message on the cmd_vel topic

# Now let's drive the Turtlebot a bit:
# Give your node a callback on a timer, once per second
# Let your callback execute just three times, then kill the node with node.destroy_node()
# Put your publish of the TwistStamped message into the callback

# Modify your code to run the callback more times, adjust the speed, etc.
# Make a function CreateLinearTwist(x) to create a TwistStamped with that value as Linear x
# Make a function CreateAngularTwist(z) to create a TwistStamped with that value as Angular z
# Now modify your callback so it changes the speed and/or angle as your Node executes

# What's the smallest diameter circle you can drive the Turtlebot?
# Make the Turtlebot drive in a circle twice that diameter

# Driving In Simple Paths

# Make your Node drive the Turtlebot 50 cm

# Make your Node drive the Turtlebot in a circle with a 1m radius

# Make your Node drive 90 degrees of a circle with a 1m radius

# Drive a path like the letter P - straight for 50 cm, then in a circle with a 50 cm radius

import math
import rclpy # import the client library
from rclpy.node import Node # import the Node class
from geometry_msgs.msg import TwistStamped

class Movement(Node):
    # LAG_TIME = 0.5
    def __init__(self, x, y, z, distance):
        super().__init__('speaker')
        self.time_elapsed = 0.0
        self.turn_time_elapsed = 0.0
        self.publishers_ = self.create_publisher(TwistStamped, 'cmd_vel', 10)

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.callback)

        self.x = x
        self.y = y
        self.z = z
        self.distance = distance
        self.forward_dtime = self.driving_time(self.distance, self.x)
        self.turn_dtime = self.turn_time(self.z)        
        # self.callback()
    
    def CreateLinearTwist(self, msg, x):
        msg.twist.linear.x = x

    def CreateAngularTwist(self, msg, z):        
        msg.twist.angular.z = z

    def driving_time(self, distance, x):
        dtime = distance / x
        return dtime
        # return dtime + self.LAG_TIME
    
    def turn_time(self, z):
        dtime = 2 * math.pi / z
        return dtime
    

    def callback(self):
        self.time_elapsed+=self.timer_period
        msg = TwistStamped()       
        if self.time_elapsed <= self.forward_dtime:
            self.CreateLinearTwist(msg, self.x)
            # msg.twist.linear.x = 0.1 #m/s
        elif self.time_elapsed <= self.turn_dtime + self.forward_dtime:
            self.CreateLinearTwist(msg, self.x)
            self.CreateAngularTwist(msg, self.z)
            print(self.turn_dtime)
            # msg.twist.angular.z = 0.1
            
            
        self.publishers_.publish(msg)
        # self.get_logger().info(f"Publishing: x:{msg.twist.linear.x}")        
        self.get_logger().info(f"Publishing: x:{msg.twist.linear.x} z:{msg.twist.angular.z}")
        # self.get_logger().info(f"Publishing: z:{msg.twist.angular.z}")

def main(args=None):
    rclpy.init(args=args)   
    p = Movement(0.1, 0.0, 0.2, 1.0)
    rclpy.spin(p)
    p.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


