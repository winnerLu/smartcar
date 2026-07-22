#!/usr/bin/env python3
"""只启动目标偏置导航节点；要求 slam_toolbox 与 Nav2 已运行。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('car_explore')
    default_params = os.path.join(share, 'config', 'goal_directed_explorer.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument(
            'goal_forward', default_value='3.0',
            description='终点相对启动瞬间车头向前的距离(m)，后方为负'),
        DeclareLaunchArgument(
            'goal_left', default_value='0.0',
            description='终点相对启动瞬间车体向左的距离(m)，右侧为负'),
        DeclareLaunchArgument(
            'goal_radius', default_value='0.45',
            description='进入该半径即认为已到达大致终点(m)'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='car_explore',
            executable='goal_directed_explorer.py',
            name='goal_directed_explorer',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'goal_forward': ParameterValue(
                        LaunchConfiguration('goal_forward'), value_type=float),
                    'goal_left': ParameterValue(
                        LaunchConfiguration('goal_left'), value_type=float),
                    'goal_radius': ParameterValue(
                        LaunchConfiguration('goal_radius'), value_type=float),
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'), value_type=bool),
                },
            ],
        ),
    ])
