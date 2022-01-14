from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='crazyswarm2',
            executable='teleop',
            name='teleop'
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node'
        ),
        Node(
            package='crazyswarm2',
            executable='crazyswarm2_server',
            name='crazyswarm2_server'
        )
    ])
