#!/usr/bin/env python3
"""slam_toolbox 在线异步建图。

前置:需先启动 bringup(底盘 odom + 雷达 /scan + TF odom->base_link->base_laser)。
配合:
    ros2 launch car_bringup bringup.launch.py      # 终端1:硬件
    ros2 launch car_slam slam.launch.py            # 终端2:建图
    ros2 run car_base keyboard_teleop.py           # 终端3:遥控着建图

输出:/map (nav_msgs/OccupancyGrid) + TF map->odom。
用 rviz2 订阅 /map 查看建图效果(Fixed Frame 设 map)。

建完存图:
    ros2 run nav2_map_server map_saver_cli -f ~/robot_ws/maps/mymap
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    slam_share = get_package_share_directory('car_slam')
    default_params = os.path.join(slam_share, 'config', 'slam_toolbox.yaml')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_params = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='slam_toolbox 参数文件')
    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='仿真时间(实机为 false)')

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        declare_params,
        declare_sim_time,
        slam_node,
    ])
