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
        # Current print measured 48.5 mm instead of the intended 50 mm.  The
        # board dimensions use the same provisional 0.97 print scale.
        DeclareLaunchArgument('tag_size', default_value='0.0485'),
        DeclareLaunchArgument('board_width', default_value='0.2881'),
        DeclareLaunchArgument('board_height', default_value='0.2910'),
        DeclareLaunchArgument('target_forward', default_value='0.082'),
        DeclareLaunchArgument('target_left', default_value='0.0'),
        # base_link -> camera_optical_frame, calibrated with the chassis
        # geometric centre over the board centre on 2026-07-23.
        DeclareLaunchArgument('camera_x', default_value='0.082651338'),
        DeclareLaunchArgument('camera_y', default_value='-0.004695587'),
        DeclareLaunchArgument('camera_z', default_value='0.579014016'),
        DeclareLaunchArgument('camera_qx', default_value='0.706268778'),
        DeclareLaunchArgument('camera_qy', default_value='-0.707635714'),
        DeclareLaunchArgument('camera_qz', default_value='0.017286951'),
        DeclareLaunchArgument('camera_qw', default_value='0.011716286'),
        DeclareLaunchArgument(
            'cmd_topic', default_value='/cmd_vel_dock',
            description='Parking velocity input; use /cmd_vel_dock with twist_mux'),
        DeclareLaunchArgument(
            'parking_enabled', default_value='false',
            description='Start parking control immediately (unsafe for detection-only tests)'),
        DeclareLaunchArgument('max_linear', default_value='0.03'),
        DeclareLaunchArgument('max_angular', default_value='0.20'),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='base_link_to_camera_optical_frame',
            arguments=[
                '--x', LaunchConfiguration('camera_x'),
                '--y', LaunchConfiguration('camera_y'),
                '--z', LaunchConfiguration('camera_z'),
                '--qx', LaunchConfiguration('camera_qx'),
                '--qy', LaunchConfiguration('camera_qy'),
                '--qz', LaunchConfiguration('camera_qz'),
                '--qw', LaunchConfiguration('camera_qw'),
                '--frame-id', 'base_link',
                '--child-frame-id', 'camera_optical_frame',
            ]),
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
                'target_forward': ParameterValue(
                    LaunchConfiguration('target_forward'), value_type=float),
                'target_left': ParameterValue(
                    LaunchConfiguration('target_left'), value_type=float),
                'board_width': ParameterValue(
                    LaunchConfiguration('board_width'), value_type=float),
                'board_height': ParameterValue(
                    LaunchConfiguration('board_height'), value_type=float),
                'cmd_topic': LaunchConfiguration('cmd_topic'),
                'enabled': ParameterValue(
                    LaunchConfiguration('parking_enabled'), value_type=bool),
                'max_linear': ParameterValue(
                    LaunchConfiguration('max_linear'), value_type=float),
                'max_angular': ParameterValue(
                    LaunchConfiguration('max_angular'), value_type=float),
            }]),
    ])
