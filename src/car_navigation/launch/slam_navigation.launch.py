#!/usr/bin/env python3
"""SLAM + Nav2 联动(比赛模式:边建图边导航,不预存地图)。

与 navigation.launch.py(静态图+AMCL)的区别:
- 定位/地图由 slam_toolbox 实时提供(发 /map 和 map->odom),不用 map_server/AMCL
- Nav2 只起导航栈(navigation_launch,不含 localization),在实时地图上规划避障
- 无需初始位姿、无需预存地图;SLAM 持续重定位,比静态图 AMCL 更抗里程计漂移

前置:先启动硬件 bringup(底盘 odom + 雷达 /scan + TF)。
配合:
    ros2 launch car_bringup bringup.launch.py                # 终端1:硬件
    ros2 launch car_navigation slam_navigation.launch.py     # 终端2:SLAM+Nav2
    ros2 run foxglove_bridge foxglove_bridge                 # 终端3:可视化
    # Foxglove/RViz 用 2D Nav Goal 直接点目标(不用初始位姿)

参数:
    slam_params_file  slam_toolbox 参数(默认用 car_slam 的)
    nav_params_file   Nav2 参数(默认用 car_navigation 的)
    use_sim_time      仿真时间(实机 false)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    car_slam_share = get_package_share_directory('car_slam')
    car_nav_share = get_package_share_directory('car_navigation')
    default_slam_params = os.path.join(car_slam_share, 'config', 'slam_toolbox.yaml')
    default_nav_params = os.path.join(car_nav_share, 'config', 'nav2_params.yaml')

    slam_params = LaunchConfiguration('slam_params_file')
    nav_params = LaunchConfiguration('nav_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_slam_params = DeclareLaunchArgument(
        'slam_params_file', default_value=default_slam_params,
        description='slam_toolbox 参数文件')
    declare_nav_params = DeclareLaunchArgument(
        'nav_params_file', default_value=default_nav_params,
        description='Nav2 参数文件')
    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='仿真时间(实机 false)')

    # ---- SLAM(实时建图 + map->odom)----
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': use_sim_time}],
    )

    # ---- Nav2 导航栈(不含 map_server/amcl)----
    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py',
            ]),
        ]),
        launch_arguments={
            'params_file': nav_params,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    return LaunchDescription([
        declare_slam_params,
        declare_nav_params,
        declare_sim_time,
        slam_node,
        nav2_navigation,
    ])
