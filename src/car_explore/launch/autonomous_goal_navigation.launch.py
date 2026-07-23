#!/usr/bin/env python3
"""一键启动 slam_toolbox、Nav2 与目标偏置在线导航。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    explore_share = get_package_share_directory('car_explore')
    nav_share = get_package_share_directory('car_navigation')
    slam_share = get_package_share_directory('car_slam')

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(slam_share, 'config', 'slam_toolbox.yaml')),
        DeclareLaunchArgument(
            'nav_params_file',
            default_value=os.path.join(nav_share, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument(
            'explorer_params_file',
            default_value=os.path.join(
                explore_share, 'config', 'goal_directed_explorer.yaml')),
        DeclareLaunchArgument('goal_forward', default_value='3.0'),
        DeclareLaunchArgument('goal_left', default_value='0.0'),
        DeclareLaunchArgument('goal_radius', default_value='0.45'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                nav_share, 'launch', 'slam_navigation.launch.py')),
            launch_arguments={
                'slam_params_file': LaunchConfiguration('slam_params_file'),
                'nav_params_file': LaunchConfiguration('nav_params_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                explore_share, 'launch', 'goal_directed_navigation.launch.py')),
            launch_arguments={
                'params_file': LaunchConfiguration('explorer_params_file'),
                'goal_forward': LaunchConfiguration('goal_forward'),
                'goal_left': LaunchConfiguration('goal_left'),
                'goal_radius': LaunchConfiguration('goal_radius'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items()),
    ])
