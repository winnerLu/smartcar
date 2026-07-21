#!/usr/bin/env python3
"""速度安全链:twist_mux(速度源仲裁)+ collision_monitor(碰撞监控)。

数据流:
    Nav2      -> /cmd_vel_nav    ┐
    teleop    -> /cmd_vel_teleop ├-> twist_mux -> /cmd_vel_raw -> collision_monitor -> /cmd_vel_safe -> car_base
    泊车      -> /cmd_vel_dock   ┘   (优先级仲裁)                  (快撞减速/停车)

作用:
- twist_mux:多速度源按优先级仲裁(泊车>Nav2>遥控),防止多源抢底盘。
- collision_monitor:用 /scan 和当前速度方向预测 footprint 碰撞，仅在预计碰撞
  时间很短时限速，是雷达可见障碍的软件防撞兜底。

前置:需 car_base(订阅 /cmd_vel_safe)、/scan 和
     /local_costmap/published_footprint 在运行。
配合 slam_navigation 或 navigation 使用时,需把 Nav2 输出改到 /cmd_vel_nav(见 nav2_params 注释)。

用法:
    ros2 launch car_navigation safety.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory('car_navigation')
    twist_mux_params = os.path.join(nav_share, 'config', 'twist_mux.yaml')
    collision_params = os.path.join(nav_share, 'config', 'collision_monitor.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false')

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
    )

    return LaunchDescription([
        declare_sim_time,
        twist_mux,
        collision_monitor,
        lifecycle_mgr,
    ])
