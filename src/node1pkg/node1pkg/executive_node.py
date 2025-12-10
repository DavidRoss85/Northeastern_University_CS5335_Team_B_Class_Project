#!/usr/bin/env python3
"""
Executive Node - User interface for sending drive commands via Actions
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from node1pkg_interfaces.action import DriveCommand

class ExecutiveNode(Node):
    def __init__(self):
        super().__init__('executive_node')
        self._action_client = ActionClient(self, DriveCommand, 'drive_command')
        self.get_logger().info('Executive Node started. Waiting for action server...')

    def send_goal(self, distance, angle):
        """Send a drive command goal to the action server"""
        goal_msg = DriveCommand.Goal()
        goal_msg.distance = distance
        goal_msg.angle = angle

        self._action_client.wait_for_server()
        
        self.get_logger().info(f'Sending goal: distance={distance}, angle={angle}')
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle the response when goal is accepted/rejected"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.user_interface_loop()
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle the final result"""
        result = future.result().result
        if result.success:
            self.get_logger().info(f'Movement completed successfully! {result.message}')
        else:
            self.get_logger().info(f'Movement failed: {result.message}')
        
        # Go back to user interface loop
        self.user_interface_loop()

    def feedback_callback(self, feedback_msg):
        """Handle feedback during execution"""
        feedback = feedback_msg.feedback
        progress_percent = feedback.progress * 100
        self.get_logger().info(f'Progress: {progress_percent:.1f}%')

    def user_interface_loop(self):
        """Main user interface loop"""
        try:
            print("\n=== TurtleBot Drive Commands ===")
            print("Commands:")
            print("  d <distance>  - Move forward/backward (meters)")
            print("  a <angle>     - Turn left/right (radians)")
            print("  q             - Quit")
            print("Examples:")
            print("  d 1.0         - Move forward 1 meter")
            print("  d -0.5        - Move backward 0.5 meters")
            print("  a 1.57        - Turn left 90 degrees (π/2)")
            print("  a -1.57       - Turn right 90 degrees")
            print("================================")
            
            user_input = input("Command: ").strip().lower()
            
            if user_input == 'q' or user_input == 'quit':
                self.get_logger().info('Shutting down Executive Node')
                self.destroy_node()
                rclpy.shutdown()
                return
            
            parts = user_input.split()
            if len(parts) != 2:
                print("Invalid input format. Use 'd <distance>' or 'a <angle>'")
                self.user_interface_loop()
                return
            
            command = parts[0]
            try:
                value = float(parts[1])
            except ValueError:
                print("Invalid number format")
                self.user_interface_loop()
                return
            
            if command == 'd':
                # Distance command
                print(f"Sending distance command: {value}m")
                self.send_goal(value, 0.0)
            elif command == 'a':
                # Angle command  
                print(f"Sending turn command: {value} radians")
                self.send_goal(0.0, value)
            else:
                print("Invalid command. Use 'd' for distance or 'a' for angle")
                self.user_interface_loop()
                
        except KeyboardInterrupt:
            self.get_logger().info('Shutting down Executive Node')
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    executive = ExecutiveNode()
    
    # Start the user interface loop
    executive.user_interface_loop()
    
    try:
        rclpy.spin(executive)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            executive.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()