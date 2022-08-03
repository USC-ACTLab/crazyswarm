from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='crazyflie',
            executable='teleop',
            name='teleop'
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node'
        ),
        Node(
            package='crazyflie',
            executable='crazyflie_server',
            name='crazyflie_server'
        )
    ])
