import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # load crazyflies
    crazyflies_yaml = os.path.join(
        get_package_share_directory('crazyswarm2'),
        'config',
        'crazyflies.yaml')

    with open(crazyflies_yaml, 'r') as ymlfile:
        crazyflies = yaml.safe_load(ymlfile)

    # load crazyflie_types
    crazyflies_types_yaml = os.path.join(
        get_package_share_directory('crazyswarm2'),
        'config',
        'crazyflie_types.yaml')

    with open(crazyflies_types_yaml, 'r') as ymlfile:
        crazyflie_types = yaml.safe_load(ymlfile)

    # construct motion_capture_configuration
    motion_capture_yaml = os.path.join(
        get_package_share_directory('crazyswarm2'),
        'config',
        'motion_capture.yaml')

    with open(motion_capture_yaml, 'r') as ymlfile:
        motion_capture = yaml.safe_load(ymlfile)

    motion_capture_params = motion_capture["/motion_capture_tracking"]["ros__parameters"]
    motion_capture_params["rigid_bodies"] = dict()
    for key, value in crazyflies.items():
        type = crazyflie_types[value["type"]]
        
        motion_capture_params["rigid_bodies"] = {key: 
            {
                "initial_position": value["initial_position"],
                "marker": type["marker"],
                "dynamics": type["dynamics"],
            }
        }

    # construct crazyswarm2_server configuration
    server_params = dict()
    server_params["crazyflies"] = crazyflies
    server_params["crazyflie_types"] = crazyflie_types


    return LaunchDescription([
        Node(
            package='motion_capture_tracking',
            executable='motion_capture_tracking_node',
            name='motion_capture_tracking',
            output='screen',
            parameters=[motion_capture_params]
        ),
        # Node(
        #     package='crazyswarm2',
        #     executable='teleop',
        #     name='teleop',
        #     remappings=[
        #         ('takeoff', 'cf1/takeoff'),
        #         ('land', 'cf1/land'),
        #     ]
        # ),
        Node(
            package='crazyswarm2',
            executable='crazyswarm2_server',
            name='crazyswarm2_server',
            output='screen',
            parameters=[server_params]
        ),
    ])
