#!/usr/bin/env python3
"""car_camera 启动:USB 摄像头(罗技 C270)采集节点 + 相机外参 static TF。

用法:
    ros2 launch car_camera camera.launch.py
    # 调整相机外参(按实车安装位置):
    ros2 launch car_camera camera.launch.py cam_z:=0.25 cam_x:=0.12

启动后:
    /camera/image_raw    (sensor_msgs/Image, bgr8)
    /camera/camera_info  (sensor_msgs/CameraInfo)
    TF base_link -> camera_link(本 launch 发布)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('car_camera')
    default_params = os.path.join(pkg_share, 'config', 'camera.yaml')

    # ---- 启动参数 ----
    params_file = LaunchConfiguration('params_file')
    video_device = LaunchConfiguration('video_device')
    cam_x = LaunchConfiguration('cam_x')
    cam_y = LaunchConfiguration('cam_y')
    cam_z = LaunchConfiguration('cam_z')
    cam_yaw = LaunchConfiguration('cam_yaw')
    cam_pitch = LaunchConfiguration('cam_pitch')
    cam_roll = LaunchConfiguration('cam_roll')

    declare_params = DeclareLaunchArgument(
        'params_file', default_value=default_params, description='相机参数文件')
    declare_device = DeclareLaunchArgument(
        'video_device', default_value='/dev/camera_c270',
        description='摄像头设备(udev 软链接或 /dev/videoN);WSL 测试可用 /dev/video0')
    # 相机相对 base_link 的外参(实测初值,rviz 可微调)
    declare_cam_x = DeclareLaunchArgument('cam_x', default_value='0.15')
    declare_cam_y = DeclareLaunchArgument('cam_y', default_value='0.0')
    declare_cam_z = DeclareLaunchArgument('cam_z', default_value='0.20')
    declare_cam_yaw = DeclareLaunchArgument('cam_yaw', default_value='0.0')
    declare_cam_pitch = DeclareLaunchArgument('cam_pitch', default_value='0.0')
    declare_cam_roll = DeclareLaunchArgument('cam_roll', default_value='0.0')

    # ---- 采集节点 ----
    camera_node = Node(
        package='car_camera',
        executable='car_camera_node',
        name='camera',
        output='screen',
        parameters=[params_file, {'video_device': video_device}],
    )

    # ---- 相机外参 static TF: base_link -> camera_link ----
    # static_transform_publisher 参数顺序: x y z yaw pitch roll parent child
    # 按实测安装位置调整;光学坐标系约定 z 朝前、x 右、y 下
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera_link',
        arguments=[
            cam_x, cam_y, cam_z,
            cam_yaw, cam_pitch, cam_roll,
            'base_link', 'camera_link',
        ],
    )

    return LaunchDescription([
        declare_params,
        declare_device,
        declare_cam_x,
        declare_cam_y,
        declare_cam_z,
        declare_cam_yaw,
        declare_cam_pitch,
        declare_cam_roll,
        camera_node,
        camera_tf,
    ])
