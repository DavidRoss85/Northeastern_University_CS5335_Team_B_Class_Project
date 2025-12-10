from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the driving node (action server)
        Node(
            package='node1pkg',
            executable='driving_node',
            name='driving_node',
            output='screen'
        ),
        
        # Optionally launch the executive node as well
        # Uncomment the following to launch both nodes together:
        Node(
            package='node1pkg',
            executable='executive_node',
            name='executive_node',
            output='screen'
        ),
    ])