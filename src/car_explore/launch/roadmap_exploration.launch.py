#!/usr/bin/env python3
"""启动目标导向 Roadmap Explorer 与最终 Nav2 直达管理器。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    car_explore_share = get_package_share_directory('car_explore')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return_to_start = LaunchConfiguration('return_to_start')
    start_delay = LaunchConfiguration('start_delay')
    goal_directed_mode = LaunchConfiguration('goal_directed_mode')
    goal_forward = LaunchConfiguration('goal_forward')
    goal_left = LaunchConfiguration('goal_left')
    goal_radius = LaunchConfiguration('goal_radius')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                car_explore_share, 'config', 'roadmap_explorer.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'goal_directed_mode', default_value='true',
            description='true=目标导向探索，false=原始完整探索'),
        DeclareLaunchArgument(
            'goal_forward', default_value='3.0',
            description='终点相对启动时车头前方距离(m)'),
        DeclareLaunchArgument(
            'goal_left', default_value='0.0',
            description='终点相对启动时左侧距离(m)，右侧为负'),
        DeclareLaunchArgument(
            'goal_radius', default_value='0.25',
            description='终点附近安全落点半径(m)，隔墙或未知区域不会触发完成'),
        DeclareLaunchArgument('return_to_start', default_value='false'),
        DeclareLaunchArgument('start_delay', default_value='5.0'),

        Node(
            package='roadmap_explorer',
            executable='roadmap_exploration_server',
            name='roadmap_explorer_node',
            output='screen',
            parameters=[
                params_file,
                {
                    'use_sim_time': ParameterValue(
                        use_sim_time, value_type=bool),
                    'goalDirected.enabled': ParameterValue(
                        goal_directed_mode, value_type=bool),
                    'goalDirected.goal_forward': ParameterValue(
                        goal_forward, value_type=float),
                    'goalDirected.goal_left': ParameterValue(
                        goal_left, value_type=float),
                },
            ],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_roadmap_explorer',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['roadmap_explorer_node'],
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }],
        ),
        Node(
            package='car_explore',
            executable='roadmap_explore_mission.py',
            name='roadmap_explore_mission',
            output='screen',
            parameters=[
                params_file,
                {
                    'use_sim_time': ParameterValue(
                        use_sim_time, value_type=bool),
                    'return_to_start': ParameterValue(
                        return_to_start, value_type=bool),
                    'start_delay': ParameterValue(
                        start_delay, value_type=float),
                    'goal_directed_mode': ParameterValue(
                        goal_directed_mode, value_type=bool),
                    'goal_forward': ParameterValue(
                        goal_forward, value_type=float),
                    'goal_left': ParameterValue(
                        goal_left, value_type=float),
                    'goal_radius': ParameterValue(
                        goal_radius, value_type=float),
                },
            ],
        ),
    ])
