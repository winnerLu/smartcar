#!/usr/bin/env python3
"""Nav2 导航(第一版:静态图 + AMCL 定位)。

前置:先启动硬件 bringup(底盘 odom + 雷达 /scan + TF)。
配合:
    ros2 launch car_bringup bringup.launch.py                       # 终端1:硬件
    ros2 launch car_navigation navigation.launch.py map:=<地图yaml> # 终端2:导航
    # Foxglove/RViz 里用 "2D Pose Estimate" 给初始位姿,再用 "2D Goal Pose" 点目标

参数:
    map          静态地图 yaml 路径(必填,或用默认)
    params_file  Nav2 参数文件
    use_sim_time 仿真时间(实机 false;回放 bag 调参时 true)

说明:
- 使用 nav2_bringup 的 bringup_launch,自动起 map_server/amcl/planner/controller/
  bt_navigator/behavior/lifecycle_manager。
- 第一版 Nav2 直接发布 /cmd_vel(car_base 订阅)。后续插 twist_mux 时改话题重映射。
- 静态图模式需先给 AMCL 初始位姿(Foxglove/RViz 的 2D Pose Estimate),否则定位不收敛。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav_share = get_package_share_directory('car_navigation')
    default_params = os.path.join(nav_share, 'config', 'nav2_params.yaml')
    default_map = os.path.join(nav_share, 'maps', 'test1.yaml')

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_map = DeclareLaunchArgument(
        'map', default_value=default_map,
        description='静态地图 yaml 路径')
    declare_params = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Nav2 参数文件')
    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='仿真时间(实机 false)')

    # 复用 nav2_bringup 的整套启动(map_server + amcl + 导航 + 生命周期管理)
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py',
            ]),
        ]),
        launch_arguments={
            'map': map_yaml,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    return LaunchDescription([
        declare_map,
        declare_params,
        declare_sim_time,
        nav2_bringup_launch,
    ])
