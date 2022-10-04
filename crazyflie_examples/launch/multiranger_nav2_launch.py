import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # load crazyflies
    crazyflies_yaml = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'crazyflies.yaml')

    with open(crazyflies_yaml, 'r') as ymlfile:
        crazyflies = yaml.safe_load(ymlfile)

    server_params = crazyflies

    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    mapfile='/home/knmcguire/development/collaborations/crazyswarm2/ros2_ws/map_1664875492.yaml'

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={'slam': 'False',
                          'use_sim_time': 'false',
                          'map': get_package_share_directory("crazyflie_examples") + '/data/map.yaml',
                          'params_file': os.path.join(get_package_share_directory("crazyflie_examples"), 'nav2_params.yaml'),
                          'autostart': 'true',
                          'use_composition': 'true',
                          'transform_publish_period': '0.02'
                          }.items()
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'rviz_launch.py')),
        launch_arguments={
                          'rviz_config': os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz')}.items())

    return LaunchDescription([
        Node(
            package='crazyflie',
            executable='crazyflie_server.py',
            name='crazyflie_server',
            output='screen',
            parameters=[{"world_tf_name": 'map'}, 
                        server_params],
        ),
        Node(
            package='crazyflie',
            executable='vel_mux.py',
            name='vel_mux',
            output='screen',
            parameters=[{"hover_height": 0.3},
                        {"incoming_twist_topic": "/cmd_vel"},
                        {"robot_prefix": "/cf1"}]
        ),
        Node(
        parameters=[
          {'odom_frame': 'odom'},
          {'map_frame': 'map'},
          {'base_frame': 'cf1'},
          {'scan_topic': '/cf1/scan'},
          {'use_scan_matching': False},
          {'max_laser_range': 3.5},
          {'resolution': 0.1},
          {'minimum_travel_distance': 0.01},
          {'minimum_travel_heading': 0.001},
          {'map_update_interval': 0.1},
          {'mode': 'localization'},
          {"map_file_name": get_package_share_directory("crazyflie_examples") + '/data/map'},
          {"map_start_pose": [0.0, 0.0, 0.0]} ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'),
        bringup_cmd,
        rviz_cmd
    ])
