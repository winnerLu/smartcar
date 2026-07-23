#!/usr/bin/env python3
"""smartcar 一键启动:底盘 car_base + 激光雷达 ldlidar + 雷达外参 TF。

前置(香橙派):需同时 source 两个工作空间,launch 才能找到 ldlidar 包:
    source ~/robot_ws/install/setup.bash
    source ~/sdk_ldrobotsensorteam_stl/ros2_app/install/setup.bash

用法:
    ros2 launch car_bringup bringup.launch.py
    # 只启底盘,不启雷达:
    ros2 launch car_bringup bringup.launch.py use_lidar:=false
    # rviz 实测微调雷达朝向(改 yaw,单位弧度):
    ros2 launch car_bringup bringup.launch.py laser_yaw:=-0.785
    # 临时关闭相机立柱盲区裁剪:
    ros2 launch car_bringup bringup.launch.py lidar_crop_enabled:=false

启动后:
    /wheel/odom (car_base 里程计) + TF odom->base_link
    /scan       (雷达)           + TF base_link->base_laser(本 launch 发布)
    /imu/data_raw (IMU)

重要:base_link->base_laser 的 static TF 由本 launch 发布。
     ldlidar 节点也由本 launch 直接启动,不再 include SDK 的 ld19.launch.py,
     因此不会重复发布 SDK 自带的错误外参 TF。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = get_package_share_directory('car_bringup')
    default_params = os.path.join(bringup_share, 'config', 'car_base.yaml')

    # ---- 启动参数 ----
    use_lidar = LaunchConfiguration('use_lidar')
    base_port = LaunchConfiguration('base_port')
    params_file = LaunchConfiguration('params_file')
    lidar_port = LaunchConfiguration('lidar_port')
    lidar_crop_enabled = LaunchConfiguration('lidar_crop_enabled')
    lidar_crop_min = LaunchConfiguration('lidar_crop_min')
    lidar_crop_max = LaunchConfiguration('lidar_crop_max')
    # 雷达相对 base_link 的外参(实测初值,rviz 可微调)
    laser_x = LaunchConfiguration('laser_x')
    laser_y = LaunchConfiguration('laser_y')
    laser_z = LaunchConfiguration('laser_z')
    laser_yaw = LaunchConfiguration('laser_yaw')
    use_safety = LaunchConfiguration('use_safety')
    # car_base 订阅话题:use_safety=true 时自动用 cmd_vel_safe,否则 cmd_vel
    cmd_vel_topic = PythonExpression(
        ["'cmd_vel_safe' if '", use_safety, "' == 'true' else 'cmd_vel'"])

    declare_use_lidar = DeclareLaunchArgument(
        'use_lidar', default_value='true', description='是否启动激光雷达')
    declare_use_safety = DeclareLaunchArgument(
        'use_safety', default_value='false',
        description='启用安全链(twist_mux+collision_monitor);启用时 car_base 自动订阅 cmd_vel_safe')
    declare_base_port = DeclareLaunchArgument(
        'base_port', default_value='/dev/car_base',
        description='底盘串口设备(udev 固定软链接)')
    declare_params = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='car_base 参数文件')
    declare_lidar_port = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/ldlidar',
        description='LD19 串口设备(udev 固定软链接)')
    # 2026-07-23 实车标定:
    # 200/200 帧检测到相机立柱，原始雷达角边界 294.48°~309.19°，
    # 取 293°~311° 给安装振动和扫描角分辨率留少量余量。
    declare_lidar_crop_enabled = DeclareLaunchArgument(
        'lidar_crop_enabled', default_value='true',
        description='屏蔽被相机立柱遮挡的雷达角段')
    declare_lidar_crop_min = DeclareLaunchArgument(
        'lidar_crop_min', default_value='293.0',
        description='雷达裁剪起始角，单位为度，使用雷达原始角坐标')
    declare_lidar_crop_max = DeclareLaunchArgument(
        'lidar_crop_max', default_value='311.0',
        description='雷达裁剪结束角，单位为度，使用雷达原始角坐标')
    # 实测标定:前 16cm、右偏 1cm、离地 11.5cm。
    # yaw=4.10rad(~235°):Foxglove 实测对齐值,含雷达上壳装错 180° 的补偿
    # (若上壳物理转正 180°,应改回约 0.96rad)。laser_scan_dir=True 无镜像。
    declare_laser_x = DeclareLaunchArgument('laser_x', default_value='0.16')
    declare_laser_y = DeclareLaunchArgument('laser_y', default_value='-0.01')
    declare_laser_z = DeclareLaunchArgument('laser_z', default_value='0.115')
    declare_laser_yaw = DeclareLaunchArgument('laser_yaw', default_value='4.10')

    # ---- 底盘节点 ----
    # 安全链开启时订阅 cmd_vel_safe,否则 cmd_vel(用 cmd_vel_topic 参数,默认随场景)
    car_base_node = Node(
        package='car_base',
        executable='car_base_node',
        name='car_base',
        output='screen',
        parameters=[params_file, {'port': base_port, 'cmd_vel_topic': cmd_vel_topic}],
    )

    # ---- 安全链(twist_mux + collision_monitor),use_safety:=true 时启动 ----
    safety_group = GroupAction(
        condition=IfCondition(use_safety),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('car_navigation'), 'launch', 'safety.launch.py',
                    ]),
                ]),
            ),
        ],
    )

    # ---- 雷达外参 static TF: base_link -> base_laser ----
    # static_transform_publisher 参数顺序: x y z yaw pitch roll parent child
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_laser',
        arguments=[
            laser_x, laser_y, laser_z,
            laser_yaw, '0', '0',
            'base_link', 'base_laser',
        ],
    )

    # ---- 雷达节点 ----
    # 直接启动驱动以便由本仓库管理立柱盲区裁剪，同时避免 SDK launch
    # 重复发布 base_link->base_laser。裁剪角内由驱动发布 NaN，SLAM/Nav2
    # 会忽略这些点，且不会把真实盲区错误清除为自由空间。
    ldlidar_node = Node(
        package='ldlidar',
        executable='ldlidar',
        name='LD19',
        output='screen',
        parameters=[{
            'product_name': 'LDLiDAR_LD19',
            'topic_name': 'scan',
            'frame_id': 'base_laser',
            'enable_serial_or_network_communication': True,
            'port_name': ParameterValue(lidar_port, value_type=str),
            'port_baudrate': 230400,
            'server_ip': '192.168.1.200',
            'server_port': '2000',
            'laser_scan_dir': True,
            'enable_angle_crop_func': ParameterValue(
                lidar_crop_enabled, value_type=bool),
            'angle_crop_min': ParameterValue(lidar_crop_min, value_type=float),
            'angle_crop_max': ParameterValue(lidar_crop_max, value_type=float),
            'measure_point_freq': 4500,
        }],
    )

    lidar_group = GroupAction(
        condition=IfCondition(use_lidar),
        actions=[ldlidar_node],
    )

    return LaunchDescription([
        declare_use_lidar,
        declare_use_safety,
        declare_base_port,
        declare_params,
        declare_lidar_port,
        declare_lidar_crop_enabled,
        declare_lidar_crop_min,
        declare_lidar_crop_max,
        declare_laser_x,
        declare_laser_y,
        declare_laser_z,
        declare_laser_yaw,
        car_base_node,
        laser_tf,
        lidar_group,
        safety_group,
    ])
