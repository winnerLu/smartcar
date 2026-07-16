#!/usr/bin/env python3
"""smartcar 一键启动:底盘 car_base + 激光雷达 ldlidar。

前置(香橙派):需同时 source 两个工作空间,launch 才能找到 ldlidar 包:
    source ~/robot_ws/install/setup.bash
    source ~/sdk_ldrobotsensorteam_stl/ros2_app/install/setup.bash

用法:
    ros2 launch car_bringup bringup.launch.py
    # 只启底盘,不启雷达:
    ros2 launch car_bringup bringup.launch.py use_lidar:=false
    # 覆盖串口:
    ros2 launch car_bringup bringup.launch.py base_port:=/dev/ttyACM0

启动后:
    /wheel/odom  (car_base 里程计)  + TF odom->base_link
    /scan        (雷达)            + TF base_link->base_laser
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = get_package_share_directory('car_bringup')
    default_params = os.path.join(bringup_share, 'config', 'car_base.yaml')

    # ---- 启动参数 ----
    use_lidar = LaunchConfiguration('use_lidar')
    base_port = LaunchConfiguration('base_port')
    params_file = LaunchConfiguration('params_file')
    lidar_launch = LaunchConfiguration('lidar_launch')

    declare_use_lidar = DeclareLaunchArgument(
        'use_lidar', default_value='true',
        description='是否启动激光雷达')
    declare_base_port = DeclareLaunchArgument(
        'base_port', default_value='/dev/car_base',
        description='底盘串口设备(udev 固定软链接)')
    declare_params = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='car_base 参数文件')
    declare_lidar_launch = DeclareLaunchArgument(
        'lidar_launch', default_value='ld19.launch.py',
        description='雷达 launch 文件名(ldlidar 包内)')

    # ---- 底盘节点 ----
    car_base_node = Node(
        package='car_base',
        executable='car_base_node',
        name='car_base',
        output='screen',
        parameters=[params_file, {'port': base_port}],
    )

    # ---- 雷达(include ldlidar 包的 launch)----
    lidar_group = GroupAction(
        condition=IfCondition(use_lidar),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('ldlidar'), 'launch', lidar_launch,
                    ]),
                ]),
            ),
        ],
    )

    return LaunchDescription([
        declare_use_lidar,
        declare_base_port,
        declare_params,
        declare_lidar_launch,
        car_base_node,
        lidar_group,
    ])
