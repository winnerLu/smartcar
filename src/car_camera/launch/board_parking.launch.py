#!/usr/bin/env python3
"""Camera, eight-tag board detector, and four-way board parking controller."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('car_camera')
    camera_params = os.path.join(share, 'config', 'camera.yaml')
    parking_params = os.path.join(share, 'config', 'board_parking.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('video_device', default_value='/dev/camera_c270'),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value='package://car_camera/config/c270_calibration.yaml'),
        DeclareLaunchArgument('tag_size', default_value='0.050'),
        DeclareLaunchArgument('board_width', default_value='0.297'),
        DeclareLaunchArgument('board_height', default_value='0.300'),
        DeclareLaunchArgument('target_z', default_value='0.05'),
        DeclareLaunchArgument('target_x', default_value='0.0'),
        DeclareLaunchArgument(
            'cmd_topic', default_value='/cmd_vel_dock',
            description='Parking velocity input; use /cmd_vel_dock with twist_mux'),
        DeclareLaunchArgument(
            'parking_enabled', default_value='false',
            description='Start parking control immediately (unsafe for detection-only tests)'),
        DeclareLaunchArgument('max_linear', default_value='0.03'),
        DeclareLaunchArgument('max_angular', default_value='0.20'),
        Node(
            package='car_camera', executable='car_camera_node', name='camera',
            output='screen', parameters=[camera_params, {
                'video_device': LaunchConfiguration('video_device'),
                'camera_name': 'logitech_c270',
                'camera_info_url': LaunchConfiguration('camera_info_url'),
            }]),
        Node(
            package='car_camera', executable='apriltag_detector_node',
            name='apriltag_detector', output='screen', parameters=[{
                'tag_size': LaunchConfiguration('tag_size'),
                'tag_ids': '0,1,2,3,4,5,6,7',
                'board_width': LaunchConfiguration('board_width'),
                'board_height': LaunchConfiguration('board_height'),
                'publish_tf': True,
                'child_frame_id': 'parking_board',
            }]),
        Node(
            package='car_camera', executable='board_parker.py',
            name='board_parker', output='screen',
            parameters=[parking_params, {
                'target_z': ParameterValue(
                    LaunchConfiguration('target_z'), value_type=float),
                'target_x': ParameterValue(
                    LaunchConfiguration('target_x'), value_type=float),
                'cmd_topic': LaunchConfiguration('cmd_topic'),
                'enabled': ParameterValue(
                    LaunchConfiguration('parking_enabled'), value_type=bool),
                'max_linear': ParameterValue(
                    LaunchConfiguration('max_linear'), value_type=float),
                'max_angular': ParameterValue(
                    LaunchConfiguration('max_angular'), value_type=float),
            }]),
    ])
