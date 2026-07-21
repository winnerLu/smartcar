#!/usr/bin/env python3
"""比赛第一版自主探索：slam_toolbox + Nav2 + explore_lite。

硬件和安全链需另行启动：
  ros2 launch car_bringup bringup.launch.py use_safety:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    nav_share = get_package_share_directory('car_navigation')
    explore_share = get_package_share_directory('car_explore')

    default_slam_nav_launch = os.path.join(
        nav_share, 'launch', 'slam_navigation.launch.py')
    default_explore_launch = os.path.join(
        explore_share, 'launch', 'explore.launch.py')

    slam_params = LaunchConfiguration('slam_params_file')
    nav_params = LaunchConfiguration('nav_params_file')
    explore_params = LaunchConfiguration('explore_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_params_file', default_value=os.path.join(
                get_package_share_directory('car_slam'),
                'config', 'slam_toolbox.yaml')),
        DeclareLaunchArgument(
            'nav_params_file', default_value=os.path.join(
                nav_share, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument(
            'explore_params_file', default_value=os.path.join(
                explore_share, 'config', 'explore.yaml')),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(default_slam_nav_launch),
            launch_arguments={
                'slam_params_file': slam_params,
                'nav_params_file': nav_params,
                'use_sim_time': use_sim_time,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(default_explore_launch),
            launch_arguments={
                'params_file': explore_params,
                'use_sim_time': use_sim_time,
            }.items(),
        ),
    ])
