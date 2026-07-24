#!/usr/bin/env python3
"""
Launch velocity arbitration with an optional collision monitor.

数据流:
    Nav2      -> /cmd_vel        ┐
    teleop    -> /cmd_vel_teleop ├-> twist_mux -> /cmd_vel_raw ───────────────> car_base
    泊车      -> /cmd_vel_dock   ┘                └-> collision_monitor
                                                     -> /cmd_vel_safe -> car_base

作用:
- twist_mux:多速度源按优先级仲裁(泊车>Nav2>遥控),防止多源抢底盘。
- collision_monitor:可选。启用时用 /scan 和当前速度方向预测 footprint 碰撞。

关闭碰撞监控时 car_base 应订阅 /cmd_vel_raw；开启时订阅 /cmd_vel_safe。

用法:
    ros2 launch car_navigation safety.launch.py use_collision_monitor:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory('car_navigation')
    twist_mux_params = os.path.join(nav_share, 'config', 'twist_mux.yaml')
    collision_params = os.path.join(nav_share, 'config', 'collision_monitor.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_collision_monitor = LaunchConfiguration('use_collision_monitor')
    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false')
    declare_collision_monitor = DeclareLaunchArgument(
        'use_collision_monitor',
        default_value='false',
        description='是否在twist_mux后启用Nav2 Collision Monitor')

    # twist_mux:仲裁后输出到 /cmd_vel_raw
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_params, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel_out', 'cmd_vel_raw')],
    )

    # collision_monitor(lifecycle 节点)
    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[collision_params, {'use_sim_time': use_sim_time}],
        condition=IfCondition(use_collision_monitor),
    )

    # 生命周期管理:自动配置+激活 collision_monitor
    lifecycle_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_safety',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['collision_monitor'],
        }],
        condition=IfCondition(use_collision_monitor),
    )

    return LaunchDescription([
        declare_sim_time,
        declare_collision_monitor,
        twist_mux,
        collision_monitor,
        lifecycle_mgr,
    ])
