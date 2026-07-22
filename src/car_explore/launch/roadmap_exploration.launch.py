#!/usr/bin/env python3
"""启动适配智能车的 Roadmap Explorer 和探索完成返航管理器。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    car_explore_share = get_package_share_directory('car_explore')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return_to_start = LaunchConfiguration('return_to_start')
    start_delay = LaunchConfiguration('start_delay')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                car_explore_share, 'config', 'roadmap_explorer.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('return_to_start', default_value='true'),
        DeclareLaunchArgument('start_delay', default_value='5.0'),

        Node(
            package='roadmap_explorer',
            executable='roadmap_exploration_server',
            name='roadmap_explorer_node',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_roadmap_explorer',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['roadmap_explorer_node'],
                'use_sim_time': use_sim_time,
            }],
        ),
        Node(
            package='car_explore',
            executable='roadmap_explore_mission.py',
            name='roadmap_explore_mission',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'return_to_start': return_to_start,
                'start_delay': start_delay,
            }],
        ),
    ])
