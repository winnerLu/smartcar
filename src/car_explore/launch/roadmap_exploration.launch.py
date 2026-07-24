#!/usr/bin/env python3
"""启动目标导向 Roadmap、预泊车 Nav2 与视觉泊车管理器."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    car_explore_share = get_package_share_directory('car_explore')
    car_camera_share = get_package_share_directory('car_camera')
    car_navigation_share = get_package_share_directory('car_navigation')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return_to_start = LaunchConfiguration('return_to_start')
    start_delay = LaunchConfiguration('start_delay')
    goal_directed_mode = LaunchConfiguration('goal_directed_mode')
    goal_forward = LaunchConfiguration('goal_forward')
    goal_left = LaunchConfiguration('goal_left')
    goal_radius = LaunchConfiguration('goal_radius')
    visual_parking_enabled = LaunchConfiguration('visual_parking_enabled')
    launch_camera = LaunchConfiguration('launch_camera')
    video_device = LaunchConfiguration('video_device')
    preparking_distance = LaunchConfiguration('preparking_distance')
    position_arrival_tolerance = LaunchConfiguration(
        'position_arrival_tolerance')
    position_only_bt_xml = LaunchConfiguration('position_only_bt_xml')
    tag_acquire_timeout = LaunchConfiguration('tag_acquire_timeout')
    search_forward_step = LaunchConfiguration('search_forward_step')
    search_lateral_step = LaunchConfiguration('search_lateral_step')
    search_timeout = LaunchConfiguration('search_timeout')
    parking_timeout = LaunchConfiguration('parking_timeout')
    progressive_probe_enabled = LaunchConfiguration(
        'progressive_probe_enabled')
    exploration_stall_timeout = LaunchConfiguration(
        'exploration_stall_timeout')
    deadend_backtrack_enabled = LaunchConfiguration(
        'deadend_backtrack_enabled')
    deadend_backtrack_min_distance = LaunchConfiguration(
        'deadend_backtrack_min_distance')
    deadend_backtrack_max_distance = LaunchConfiguration(
        'deadend_backtrack_max_distance')
    progressive_probe_step = LaunchConfiguration(
        'progressive_probe_step')
    progressive_probe_min_step = LaunchConfiguration(
        'progressive_probe_min_step')
    progressive_probe_max_attempts = LaunchConfiguration(
        'progressive_probe_max_attempts')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                car_explore_share, 'config', 'roadmap_explorer.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'goal_directed_mode', default_value='true',
            description='true=目标导向探索，false=原始完整探索'),
        DeclareLaunchArgument(
            'goal_forward', default_value='3.0',
            description='终点相对启动时车头前方距离(m)'),
        DeclareLaunchArgument(
            'goal_left', default_value='0.0',
            description='终点相对启动时左侧距离(m)，右侧为负'),
        DeclareLaunchArgument(
            'goal_radius', default_value='0.25',
            description='标称终点不确定范围及视觉搜索上限(m)'),
        DeclareLaunchArgument(
            'visual_parking_enabled', default_value='true',
            description='到预泊车点后确认完整Tag并交给视觉泊车'),
        DeclareLaunchArgument(
            'launch_camera', default_value='true',
            description='由本启动文件启动相机、Tag检测和禁用状态的泊车控制器'),
        DeclareLaunchArgument(
            'video_device', default_value='/dev/camera_c270'),
        DeclareLaunchArgument(
            'preparking_distance', default_value='0.35',
            description='Nav2预泊车点位于标称终点前方的距离(m)'),
        DeclareLaunchArgument(
            'position_arrival_tolerance', default_value='0.06',
            description='Nav2位置到达容差(m)，不检查终点航向'),
        DeclareLaunchArgument(
            'position_only_bt_xml',
            default_value=os.path.join(
                car_navigation_share, 'behavior_trees',
                'navigate_to_pose_position_only.xml'),
            description='预泊车与Tag搜索使用的无航向约束Nav2行为树'),
        DeclareLaunchArgument(
            'tag_acquire_timeout', default_value='2.0',
            description='预泊车点静止等待完整Tag的时间(s)'),
        DeclareLaunchArgument(
            'search_forward_step', default_value='0.10',
            description='有限视觉搜索向目标前进的最大距离(m)'),
        DeclareLaunchArgument(
            'search_lateral_step', default_value='0.14',
            description='有限视觉搜索左右偏移距离(m)'),
        DeclareLaunchArgument(
            'search_timeout', default_value='25.0',
            description='有限视觉搜索总超时(s)'),
        DeclareLaunchArgument(
            'parking_timeout', default_value='45.0',
            description='视觉泊车接管后的超时(s)'),
        DeclareLaunchArgument(
            'progressive_probe_enabled', default_value='true',
            description='Roadmap停滞时启用目标方向渐进探测'),
        DeclareLaunchArgument(
            'exploration_stall_timeout', default_value='12.0',
            description='探索阶段无位置进展多久后触发渐进探测(s)'),
        DeclareLaunchArgument(
            'deadend_backtrack_enabled', default_value='true',
            description='停滞时优先沿已走安全轨迹退出死胡同'),
        DeclareLaunchArgument(
            'deadend_backtrack_min_distance', default_value='0.60',
            description='死胡同轨迹回退最短距离(m)'),
        DeclareLaunchArgument(
            'deadend_backtrack_max_distance', default_value='1.20',
            description='死胡同轨迹回退最远距离(m)'),
        DeclareLaunchArgument(
            'progressive_probe_step', default_value='0.35',
            description='目标方向渐进探测的最大步长(m)'),
        DeclareLaunchArgument(
            'progressive_probe_min_step', default_value='0.20',
            description='目标方向渐进探测的短步长(m)'),
        DeclareLaunchArgument(
            'progressive_probe_max_attempts', default_value='6',
            description='整次任务最多发送的渐进探测次数'),
        DeclareLaunchArgument('return_to_start', default_value='false'),
        DeclareLaunchArgument('start_delay', default_value='5.0'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                car_camera_share, 'launch', 'board_parking.launch.py')),
            condition=IfCondition(launch_camera),
            launch_arguments={
                'video_device': video_device,
                'parking_enabled': 'false',
                'cmd_topic': '/cmd_vel_dock',
            }.items(),
        ),

        Node(
            package='roadmap_explorer',
            executable='roadmap_exploration_server',
            name='roadmap_explorer_node',
            output='screen',
            parameters=[
                params_file,
                {
                    'use_sim_time': ParameterValue(
                        use_sim_time, value_type=bool),
                    'goalDirected.enabled': ParameterValue(
                        goal_directed_mode, value_type=bool),
                    'goalDirected.goal_forward': ParameterValue(
                        goal_forward, value_type=float),
                    'goalDirected.goal_left': ParameterValue(
                        goal_left, value_type=float),
                },
            ],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_roadmap_explorer',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['roadmap_explorer_node'],
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }],
        ),
        Node(
            package='car_explore',
            executable='roadmap_explore_mission.py',
            name='roadmap_explore_mission',
            output='screen',
            parameters=[
                params_file,
                {
                    'use_sim_time': ParameterValue(
                        use_sim_time, value_type=bool),
                    'return_to_start': ParameterValue(
                        return_to_start, value_type=bool),
                    'start_delay': ParameterValue(
                        start_delay, value_type=float),
                    'goal_directed_mode': ParameterValue(
                        goal_directed_mode, value_type=bool),
                    'goal_forward': ParameterValue(
                        goal_forward, value_type=float),
                    'goal_left': ParameterValue(
                        goal_left, value_type=float),
                    'goal_radius': ParameterValue(
                        goal_radius, value_type=float),
                    'visual_parking_enabled': ParameterValue(
                        visual_parking_enabled, value_type=bool),
                    'preparking_distance': ParameterValue(
                        preparking_distance, value_type=float),
                    'position_arrival_tolerance': ParameterValue(
                        position_arrival_tolerance, value_type=float),
                    'position_only_bt_xml': position_only_bt_xml,
                    'tag_acquire_timeout': ParameterValue(
                        tag_acquire_timeout, value_type=float),
                    'search_forward_step': ParameterValue(
                        search_forward_step, value_type=float),
                    'search_lateral_step': ParameterValue(
                        search_lateral_step, value_type=float),
                    'search_timeout': ParameterValue(
                        search_timeout, value_type=float),
                    'parking_timeout': ParameterValue(
                        parking_timeout, value_type=float),
                    'progressive_probe_enabled': ParameterValue(
                        progressive_probe_enabled, value_type=bool),
                    'exploration_stall_timeout': ParameterValue(
                        exploration_stall_timeout, value_type=float),
                    'deadend_backtrack_enabled': ParameterValue(
                        deadend_backtrack_enabled, value_type=bool),
                    'deadend_backtrack_min_distance': ParameterValue(
                        deadend_backtrack_min_distance, value_type=float),
                    'deadend_backtrack_max_distance': ParameterValue(
                        deadend_backtrack_max_distance, value_type=float),
                    'progressive_probe_step': ParameterValue(
                        progressive_probe_step, value_type=float),
                    'progressive_probe_min_step': ParameterValue(
                        progressive_probe_min_step, value_type=float),
                    'progressive_probe_max_attempts': ParameterValue(
                        progressive_probe_max_attempts, value_type=int),
                },
            ],
        ),
    ])
