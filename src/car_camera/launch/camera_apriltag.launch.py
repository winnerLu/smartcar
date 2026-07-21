#!/usr/bin/env python3
"""摄像头 + Apriltag 检测 + 视觉伺服泊车联合启动。

用法:
    # 仅检测(不控制):
    ros2 launch car_camera camera_apriltag.launch.py video_device:=/dev/video0

    # 检测 + 自动泊车(距标签 5cm, 居中):
    ros2 launch car_camera camera_apriltag.launch.py \
        video_device:=/dev/video0 enable_follower:=true

    # 小车 + C270:
    ros2 launch car_camera camera_apriltag.launch.py \
        camera_name:=logitech_c270 \
        camera_info_url:=file:///path/to/c270_calibration.yaml \
        enable_follower:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('car_camera')
    default_params = os.path.join(pkg_share, 'config', 'camera.yaml')
    default_calib_url = 'package://car_camera/config/webcam_calibration.yaml'

    video_device = LaunchConfiguration('video_device')
    camera_name = LaunchConfiguration('camera_name')
    camera_info_url = LaunchConfiguration('camera_info_url')
    tag_size = LaunchConfiguration('tag_size')
    enable_follower = LaunchConfiguration('enable_follower')

    return LaunchDescription([
        DeclareLaunchArgument(
            'video_device', default_value='/dev/camera_c270',
            description='摄像头设备;电脑测试用 /dev/video0'),
        DeclareLaunchArgument(
            'camera_name', default_value='webcam',
            description='相机名称,须与标定文件一致'),
        DeclareLaunchArgument(
            'camera_info_url', default_value=default_calib_url,
            description='标定文件 URL(package:// 或 file://)'),
        DeclareLaunchArgument(
            'tag_size', default_value='0.120',
            description='AprilTag 黑色方块实测边长(米)'),
        DeclareLaunchArgument(
            'enable_follower', default_value='false',
            description='是否启动视觉伺服自动泊车'),

        # 摄像头采集
        Node(
            package='car_camera',
            executable='car_camera_node',
            name='camera',
            output='screen',
            parameters=[
                default_params,
                {'video_device': video_device,
                 'camera_name': camera_name,
                 'camera_info_url': camera_info_url},
            ],
        ),

        # Apriltag 检测
        Node(
            package='car_camera',
            executable='apriltag_detector_node',
            name='apriltag_detector',
            output='screen',
            parameters=[{
                'tag_size': tag_size,
                'tag_id': 0,
                'publish_tf': True,
                'child_frame_id': 'tag_0',
            }],
        ),

        # 视觉伺服泊车(默认关闭)
        Node(
            package='car_camera',
            executable='tag_follower.py',
            name='tag_follower',
            output='screen',
            condition=IfCondition(enable_follower),
            parameters=[{
                'target_z': 0.05,
                'kp_linear': 0.8,
                'kp_angular': 2.0,
                'max_linear': 0.08,
                'max_angular': 0.5,
                'dead_zone_x': 0.003,
                'dead_zone_y': 0.003,
                'dead_zone_z': 0.005,
            }],
        ),
    ])
