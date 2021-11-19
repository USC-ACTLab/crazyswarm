import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    node_config = os.path.join(
        get_package_share_directory('crazyswarm2'),
        'config',
        'cfg.yaml')

    return LaunchDescription([
        # Node(
        #     package='crazyswarm2',
        #     executable='teleop',
        #     name='teleop',
        # ),
        Node(
            package='crazyswarm2',
            executable='crazyswarm2_server',
            name='crazyswarm2_server',
            output='screen',
            parameters=[node_config]
        ),
    ])
