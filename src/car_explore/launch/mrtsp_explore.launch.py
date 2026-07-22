#!/usr/bin/env python3
"""启动 WFD + Greedy MRTSP 探索器；要求 SLAM 和 Nav2 已经运行。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    explore_share = get_package_share_directory('car_explore')
    default_params = os.path.join(
        explore_share, 'config', 'mrtsp_explore.yaml')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='WFD + MRTSP 探索参数文件'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='仿真时间（实机为 false）'),
        Node(
            package='frontier_exploration_ros2',
            executable='frontier_explorer',
            name='frontier_explorer',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
    ])
