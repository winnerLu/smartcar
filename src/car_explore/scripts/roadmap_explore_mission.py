#!/usr/bin/env python3
"""Run goal-directed Roadmap exploration, then hand off Nav2 to visual parking."""

import math
import os
from typing import List, Optional, Tuple

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from roadmap_explorer_msgs.action import Explore
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, UInt32
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener

from roadmap_handoff import (
    Point,
    append_breadcrumb,
    bounded_search_points,
    breadcrumb_backtrack_points,
    path_standoff_point,
    position_reached,
    progressive_probe_points,
    startup_escape_blocking_point,
    startup_escape_metrics,
    tag_observation_valid,
    target_reveal_points,
)


Cell = Tuple[int, int]


class RoadmapExploreMission(Node):
    """Coordinate Roadmap exploration and safe final-goal handoff."""

    def __init__(self):
        super().__init__('roadmap_explore_mission')
        self._declare_parameters()
        self._load_parameters()

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self._map_callback, map_qos)
        target_qos = QoSProfile(depth=1)
        target_qos.reliability = ReliabilityPolicy.RELIABLE
        target_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.target_pub = self.create_publisher(
            PoseStamped, '~/target_pose', target_qos)
        self.preparking_pub = self.create_publisher(
            PoseStamped, '~/preparking_pose', target_qos)
        self.startup_escape_pub = self.create_publisher(
            Twist, self.startup_escape_cmd_topic, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, self.startup_escape_scan_topic,
            self._scan_callback, 10)

        self.tag_pose_sub = self.create_subscription(
            PoseStamped, self.tag_pose_topic, self._tag_pose_callback, 10)
        self.tag_count_sub = self.create_subscription(
            UInt32, self.tag_count_topic, self._tag_count_callback, 10)
        self.tag_error_sub = self.create_subscription(
            Float32, self.tag_error_topic, self._tag_error_callback, 10)
        self.tag_inlier_sub = self.create_subscription(
            UInt32, self.tag_inlier_topic, self._tag_inlier_callback, 10)
        parking_qos = QoSProfile(depth=1)
        parking_qos.reliability = ReliabilityPolicy.RELIABLE
        parking_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.parking_complete_sub = self.create_subscription(
            Bool, self.parking_complete_topic,
            self._parking_complete_callback, parking_qos)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.explore_client = ActionClient(self, Explore, 'roadmap_explorer')
        self.plan_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.parking_client = self.create_client(
            SetBool, self.parking_enable_service)
        self.camera_client = self.create_client(
            SetBool, self.camera_capture_service)

        self.latest_map: Optional[OccupancyGrid] = None
        self.latest_scan: Optional[LaserScan] = None
        self.latest_scan_time = 0.0
        self.start_pose: Optional[PoseStamped] = None
        self.target_pose: Optional[PoseStamped] = None
        self.preparking_pose: Optional[PoseStamped] = None
        self.explore_goal_handle = None
        self.final_goal_handle = None
        self.final_candidate: Optional[PoseStamped] = None
        self.active_nav_target: Optional[PoseStamped] = None
        self.active_nav_purpose: Optional[str] = None
        self.initial_plan_candidate: Optional[PoseStamped] = None
        self.nav_cancel_reason: Optional[str] = None
        self.nav_sequence = 0
        self.plan_in_progress = False
        self.search_points: List[Point] = []
        self.search_index = 0
        self.search_start_time = 0.0
        self.search_wait_start_time = 0.0
        self.acquisition_start_time = 0.0
        self.acquisition_after_search = False
        self.parking_start_time = 0.0
        self.parking_complete = False
        self.parking_reset_requested = False
        self.parking_reset_complete = not self.visual_parking_enabled
        self.camera_reset_requested = False
        self.camera_reset_complete = not self.visual_parking_enabled
        self.camera_capture_enabled = False
        self.camera_activation_in_progress = False
        self.camera_handoff_pending = False
        self.pending_acquisition_after_search = False
        self.tag_visible_count = 0
        self.tag_reprojection_error = math.inf
        self.tag_inlier_count = 0
        self.last_tag_pose_time = 0.0
        self.last_tag_count_time = 0.0
        self.last_tag_error_time = 0.0
        self.last_tag_inlier_time = 0.0
        self.tag_stable_since: Optional[float] = None
        self.tag_confirmation_reported = False
        self.state = 'STARTING'
        self.node_start_time = self._now_seconds()
        self.mission_start_time = 0.0
        self.next_direct_check_time = 0.0
        self.last_wait_log_time = 0.0
        self.exploration_progress_pose: Optional[Point] = None
        self.exploration_progress_time = 0.0
        self.breadcrumbs: List[Point] = []
        self.backtrack_candidates: List[Point] = []
        self.backtrack_index = 0
        self.backtrack_plan_candidate: Optional[PoseStamped] = None
        self.backtrack_target: Optional[Point] = None
        self.backtrack_recoveries = 0
        self.recovery_cooldown_until = 0.0
        self.recovery_settle_kind = 'probe'
        self.exploration_request_mode = 'new'
        self.probe_candidates: List[Point] = []
        self.probe_index = 0
        self.probe_plan_candidate: Optional[PoseStamped] = None
        self.probe_start_target_distance = math.inf
        self.probe_attempts = 0
        self.probe_failures = 0
        self.probe_settle_start_time = 0.0
        self.pending_target_reveal = False
        self.target_reveal_candidates: List[Point] = []
        self.target_reveal_index = 0
        self.target_reveal_plan_candidate: Optional[PoseStamped] = None
        self.target_reveal_attempts = 0
        self.target_reveal_settle_start_time = 0.0
        self.startup_escape_start: Optional[Tuple[float, float, float]] = None
        self.startup_escape_start_time = 0.0
        self.startup_escape_settle_start_time = 0.0
        self.startup_escape_timer = self.create_timer(
            0.10, self._startup_escape_control_tick)
        self.timer = self.create_timer(0.5, self._tick)

    def _declare_parameters(self):
        defaults = {
            'map_frame': 'map',
            'base_frame': 'base_link',
            'map_topic': '/map',
            'planner_id': 'GridBased',
            'start_delay': 5.0,
            'startup_escape_enabled': True,
            'startup_escape_cmd_topic': '/cmd_vel_escape',
            'startup_escape_scan_topic': '/scan',
            'startup_escape_odom_frame': 'odom',
            'startup_escape_distance': 0.20,
            'startup_escape_speed': 0.08,
            'startup_escape_timeout': 4.0,
            'startup_escape_settle_time': 1.0,
            'startup_escape_max_yaw_error_deg': 8.0,
            'startup_escape_heading_kp': 1.5,
            'startup_escape_max_angular': 0.20,
            'startup_escape_scan_max_age': 0.50,
            'startup_escape_swept_half_width': 0.13,
            'startup_escape_footprint_front': 0.197,
            'startup_escape_braking_margin': 0.10,
            'return_to_start': False,
            'goal_directed_mode': True,
            'goal_forward': 3.0,
            'goal_left': 0.0,
            'goal_radius': 0.25,
            'visual_parking_enabled': True,
            'preparking_distance': 0.10,
            'position_arrival_tolerance': 0.06,
            'position_only_bt_xml': '',
            'tag_pose_topic': '/apriltag_detector/tag_pose',
            'tag_count_topic': '/apriltag_detector/visible_tag_count',
            'tag_error_topic': '/apriltag_detector/reprojection_error',
            'tag_inlier_topic': '/apriltag_detector/inlier_count',
            'parking_complete_topic': '/board_parker/parking_complete',
            'parking_enable_service': '/board_parker/set_enabled',
            'camera_capture_service': '/camera/set_enabled',
            'camera_warmup_target_distance': 1.0,
            'tag_max_age': 0.40,
            'tag_confirm_time': 0.50,
            'tag_max_reprojection_error': 3.0,
            'tag_min_inlier_points': 4,
            'tag_acquire_timeout': 2.0,
            'tag_handoff_max_target_distance': 0.75,
            'search_forward_step': 0.10,
            'search_lateral_step': 0.14,
            'search_dwell_time': 1.5,
            'search_timeout': 25.0,
            'parking_timeout': 45.0,
            'direct_path_known_ratio': 0.85,
            'direct_check_period': 1.0,
            'progressive_probe_enabled': True,
            'exploration_stall_timeout': 40.0,
            'exploration_progress_distance': 0.10,
            'deadend_backtrack_enabled': True,
            'breadcrumb_spacing': 0.25,
            'breadcrumb_max_points': 160,
            'deadend_backtrack_min_distance': 0.60,
            'deadend_backtrack_max_distance': 1.20,
            'deadend_backtrack_candidate_spacing': 0.20,
            'deadend_backtrack_max_recoveries': 8,
            'deadend_recovery_cooldown': 6.0,
            'progressive_probe_step': 0.35,
            'progressive_probe_min_step': 0.20,
            'progressive_probe_fan_angles_deg': [
                0.0, 15.0, -15.0, 30.0, -30.0, 45.0, -45.0],
            'progressive_probe_min_progress': 0.12,
            'progressive_probe_known_ratio': 1.0,
            'progressive_probe_settle_time': 1.0,
            'progressive_probe_max_attempts': 6,
            'progressive_probe_max_failures': 3,
            'target_local_reveal_enabled': True,
            'target_local_reveal_forward_offsets': [0.10, 0.15, 0.20],
            'target_local_reveal_lateral_offsets': [0.0, 0.08, -0.08],
            'target_local_reveal_min_standoff': 0.12,
            'target_local_reveal_known_ratio': 1.0,
            'target_local_reveal_settle_time': 1.5,
            'target_local_reveal_max_attempts': 3,
            'mission_timeout': 300.0,
            'free_threshold': 20,
            'occupied_threshold': 65,
            'endpoint_clearance': 0.14,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _load_parameters(self):
        for name in (
                'map_frame', 'base_frame', 'map_topic', 'planner_id',
                'start_delay',
                'startup_escape_enabled', 'startup_escape_cmd_topic',
                'startup_escape_scan_topic', 'startup_escape_odom_frame',
                'startup_escape_distance',
                'startup_escape_speed', 'startup_escape_timeout',
                'startup_escape_settle_time',
                'startup_escape_max_yaw_error_deg',
                'startup_escape_heading_kp',
                'startup_escape_max_angular',
                'startup_escape_scan_max_age',
                'startup_escape_swept_half_width',
                'startup_escape_footprint_front',
                'startup_escape_braking_margin',
                'return_to_start', 'goal_directed_mode',
                'goal_forward', 'goal_left', 'goal_radius',
                'visual_parking_enabled', 'preparking_distance',
                'position_arrival_tolerance',
                'position_only_bt_xml',
                'tag_pose_topic', 'tag_count_topic', 'tag_error_topic',
                'tag_inlier_topic', 'parking_complete_topic',
                'parking_enable_service', 'camera_capture_service',
                'camera_warmup_target_distance',
                'tag_max_age', 'tag_confirm_time',
                'tag_max_reprojection_error', 'tag_min_inlier_points',
                'tag_acquire_timeout', 'tag_handoff_max_target_distance',
                'search_forward_step', 'search_lateral_step',
                'search_dwell_time', 'search_timeout', 'parking_timeout',
                'direct_path_known_ratio', 'direct_check_period',
                'progressive_probe_enabled', 'exploration_stall_timeout',
                'exploration_progress_distance',
                'deadend_backtrack_enabled', 'breadcrumb_spacing',
                'breadcrumb_max_points',
                'deadend_backtrack_min_distance',
                'deadend_backtrack_max_distance',
                'deadend_backtrack_candidate_spacing',
                'deadend_backtrack_max_recoveries',
                'deadend_recovery_cooldown',
                'progressive_probe_step',
                'progressive_probe_min_step',
                'progressive_probe_fan_angles_deg',
                'progressive_probe_min_progress',
                'progressive_probe_known_ratio',
                'progressive_probe_settle_time',
                'progressive_probe_max_attempts',
                'progressive_probe_max_failures',
                'target_local_reveal_enabled',
                'target_local_reveal_forward_offsets',
                'target_local_reveal_lateral_offsets',
                'target_local_reveal_min_standoff',
                'target_local_reveal_known_ratio',
                'target_local_reveal_settle_time',
                'target_local_reveal_max_attempts',
                'mission_timeout', 'free_threshold', 'occupied_threshold',
                'endpoint_clearance'):
            setattr(self, name, self.get_parameter(name).value)

        self.start_delay = float(self.start_delay)
        self.startup_escape_enabled = bool(self.startup_escape_enabled)
        self.startup_escape_distance = max(
            0.0, float(self.startup_escape_distance))
        self.startup_escape_speed = max(
            0.0, float(self.startup_escape_speed))
        self.startup_escape_timeout = max(
            0.5, float(self.startup_escape_timeout))
        self.startup_escape_settle_time = max(
            0.5, float(self.startup_escape_settle_time))
        self.startup_escape_max_yaw_error = math.radians(max(
            1.0, float(self.startup_escape_max_yaw_error_deg)))
        self.startup_escape_heading_kp = max(
            0.0, float(self.startup_escape_heading_kp))
        self.startup_escape_max_angular = max(
            0.0, float(self.startup_escape_max_angular))
        self.startup_escape_scan_max_age = max(
            0.10, float(self.startup_escape_scan_max_age))
        self.startup_escape_swept_half_width = max(
            0.05, float(self.startup_escape_swept_half_width))
        self.startup_escape_footprint_front = max(
            0.0, float(self.startup_escape_footprint_front))
        self.startup_escape_braking_margin = max(
            0.0, float(self.startup_escape_braking_margin))
        self.return_to_start = bool(self.return_to_start)
        self.goal_directed_mode = bool(self.goal_directed_mode)
        self.goal_forward = float(self.goal_forward)
        self.goal_left = float(self.goal_left)
        self.goal_radius = max(0.0, float(self.goal_radius))
        self.visual_parking_enabled = bool(self.visual_parking_enabled)
        self.preparking_distance = max(
            0.0, float(self.preparking_distance))
        self.position_arrival_tolerance = max(
            0.01, float(self.position_arrival_tolerance))
        if not self.position_only_bt_xml:
            self.position_only_bt_xml = os.path.join(
                get_package_share_directory('car_navigation'),
                'behavior_trees', 'navigate_to_pose_position_only.xml')
        if not os.path.isfile(str(self.position_only_bt_xml)):
            raise RuntimeError(
                'Position-only Nav2 behavior tree does not exist: '
                f'{self.position_only_bt_xml}')
        self.tag_max_age = max(0.05, float(self.tag_max_age))
        self.tag_confirm_time = max(0.0, float(self.tag_confirm_time))
        self.tag_max_reprojection_error = max(
            0.0, float(self.tag_max_reprojection_error))
        self.tag_min_inlier_points = max(
            4, int(self.tag_min_inlier_points))
        self.tag_acquire_timeout = max(
            0.0, float(self.tag_acquire_timeout))
        self.tag_handoff_max_target_distance = max(
            0.05, float(self.tag_handoff_max_target_distance))
        self.camera_warmup_target_distance = max(
            self.tag_handoff_max_target_distance,
            float(self.camera_warmup_target_distance))
        self.search_forward_step = min(
            abs(float(self.search_forward_step)), self.goal_radius)
        self.search_lateral_step = min(
            abs(float(self.search_lateral_step)), self.goal_radius)
        self.search_dwell_time = max(0.0, float(self.search_dwell_time))
        self.search_timeout = max(0.0, float(self.search_timeout))
        self.parking_timeout = max(0.0, float(self.parking_timeout))
        self.direct_path_known_ratio = float(self.direct_path_known_ratio)
        self.direct_check_period = float(self.direct_check_period)
        self.progressive_probe_enabled = bool(
            self.progressive_probe_enabled)
        self.exploration_stall_timeout = max(
            1.0, float(self.exploration_stall_timeout))
        self.exploration_progress_distance = max(
            0.02, float(self.exploration_progress_distance))
        self.deadend_backtrack_enabled = bool(
            self.deadend_backtrack_enabled)
        self.breadcrumb_spacing = max(
            0.05, float(self.breadcrumb_spacing))
        self.breadcrumb_max_points = max(
            10, int(self.breadcrumb_max_points))
        self.deadend_backtrack_min_distance = max(
            self.breadcrumb_spacing,
            float(self.deadend_backtrack_min_distance))
        self.deadend_backtrack_max_distance = max(
            self.deadend_backtrack_min_distance,
            float(self.deadend_backtrack_max_distance))
        self.deadend_backtrack_candidate_spacing = max(
            self.breadcrumb_spacing,
            float(self.deadend_backtrack_candidate_spacing))
        self.deadend_backtrack_max_recoveries = max(
            1, int(self.deadend_backtrack_max_recoveries))
        self.deadend_recovery_cooldown = max(
            0.0, float(self.deadend_recovery_cooldown))
        self.progressive_probe_step = max(
            0.05, float(self.progressive_probe_step))
        self.progressive_probe_min_step = min(
            self.progressive_probe_step,
            max(0.05, float(self.progressive_probe_min_step)))
        self.progressive_probe_fan_angles_deg = tuple(
            float(value) for value in self.progressive_probe_fan_angles_deg)
        self.progressive_probe_min_progress = max(
            0.01, float(self.progressive_probe_min_progress))
        self.progressive_probe_known_ratio = min(
            1.0, max(0.0, float(self.progressive_probe_known_ratio)))
        self.progressive_probe_settle_time = max(
            0.0, float(self.progressive_probe_settle_time))
        self.progressive_probe_max_attempts = max(
            1, int(self.progressive_probe_max_attempts))
        self.progressive_probe_max_failures = max(
            1, int(self.progressive_probe_max_failures))
        self.target_local_reveal_enabled = bool(
            self.target_local_reveal_enabled)
        self.target_local_reveal_forward_offsets = tuple(
            max(0.0, float(value))
            for value in self.target_local_reveal_forward_offsets)
        self.target_local_reveal_lateral_offsets = tuple(
            float(value)
            for value in self.target_local_reveal_lateral_offsets)
        self.target_local_reveal_min_standoff = max(
            0.05, float(self.target_local_reveal_min_standoff))
        self.target_local_reveal_known_ratio = min(
            1.0, max(0.0, float(self.target_local_reveal_known_ratio)))
        self.target_local_reveal_settle_time = max(
            0.0, float(self.target_local_reveal_settle_time))
        self.target_local_reveal_max_attempts = max(
            1, int(self.target_local_reveal_max_attempts))
        self.mission_timeout = float(self.mission_timeout)
        self.free_threshold = int(self.free_threshold)
        self.occupied_threshold = int(self.occupied_threshold)
        self.endpoint_clearance = float(self.endpoint_clearance)

    def _map_callback(self, msg: OccupancyGrid):
        self.latest_map = msg

    def _scan_callback(self, msg: LaserScan):
        self.latest_scan = msg
        self.latest_scan_time = self._now_seconds()

    def _tag_pose_callback(self, _msg: PoseStamped):
        self.last_tag_pose_time = self._now_seconds()

    def _tag_count_callback(self, msg: UInt32):
        self.tag_visible_count = int(msg.data)
        self.last_tag_count_time = self._now_seconds()

    def _tag_error_callback(self, msg: Float32):
        self.tag_reprojection_error = float(msg.data)
        self.last_tag_error_time = self._now_seconds()

    def _tag_inlier_callback(self, msg: UInt32):
        self.tag_inlier_count = int(msg.data)
        self.last_tag_inlier_time = self._now_seconds()

    def _parking_complete_callback(self, msg: Bool):
        self.parking_complete = bool(msg.data)

    def _tick(self):
        now = self._now_seconds()
        if self.state == 'STARTING':
            self._try_start(now)
            return
        if self.state == 'STARTUP_ESCAPE':
            return
        if self.state == 'STARTUP_ESCAPE_SETTLE':
            if (now - self.startup_escape_settle_start_time >=
                    self.startup_escape_settle_time):
                self.get_logger().info(
                    'Startup escape settled; releasing direct velocity '
                    'ownership and starting normal planning')
                self._begin_planning_after_startup()
            return
        if self.state in ('COMPLETE', 'FAILED'):
            return
        if (self.mission_timeout > 0.0 and self.mission_start_time > 0.0 and
                now - self.mission_start_time >= self.mission_timeout):
            self._abort_mission('Mission timeout reached before the target became reachable')
            return

        self._maybe_warm_camera_near_target()

        if self.state == 'EXPLORING':
            # A complete Tag near the mission target is stronger arrival
            # evidence than the SLAM endpoint-clearance test. In particular,
            # the robot may already be over the parking board while nearby
            # walls make the nominal pre-parking endpoint fail clearance.
            # Hold stall recovery during the short confirmation window so it
            # cannot dispatch a breadcrumb backtrack just before handoff.
            if self._try_exploration_tag_handoff(now):
                return
            self._update_exploration_progress(now)
            if (self.goal_directed_mode and
                    self.progressive_probe_enabled and
                    not self.plan_in_progress and
                    self._exploration_stalled(now)):
                self._cancel_exploration_for_probe()
                return
            if (not self.goal_directed_mode or self.plan_in_progress or
                    now < self.next_direct_check_time):
                return
            self.next_direct_check_time = now + self.direct_check_period
            if self._target_is_known_free():
                self._request_direct_path()
            return

        if self.state in (
                'FINAL_NAVIGATION', 'SEARCH_NAVIGATION',
                'PROBE_NAVIGATION', 'BACKTRACK_NAVIGATION',
                'TARGET_REVEAL_NAVIGATION'):
            if (self.state not in (
                    'PROBE_NAVIGATION', 'BACKTRACK_NAVIGATION') and
                    self.visual_parking_enabled and
                    self._tag_confirmed(now) and
                    self._inside_tag_handoff_region()):
                self._cancel_active_nav('tag_confirmed')
                return
            if self._active_nav_position_reached():
                self._cancel_active_nav('position_reached')
                return
            if (self.state == 'SEARCH_NAVIGATION' and
                    self._search_timed_out(now)):
                self._cancel_active_nav('search_timeout')
            return

        if self.state == 'TARGET_REVEAL_SETTLE':
            if self._try_target_reveal_tag_handoff(now):
                return
            if (now - self.target_reveal_settle_start_time >=
                    self.target_local_reveal_settle_time):
                self._evaluate_target_local_reveal()
            return

        if self.state == 'PROBE_SETTLE':
            if (now - self.probe_settle_start_time >=
                    self.progressive_probe_settle_time):
                self._resume_roadmap_after_probe()
            return

        if self.state == 'TAG_ACQUISITION':
            if self._tag_confirmed(now):
                self._activate_visual_parking()
            elif now - self.acquisition_start_time >= self.tag_acquire_timeout:
                if self.acquisition_after_search:
                    self._send_next_search_goal()
                else:
                    self._begin_limited_search()
            return

        if self.state == 'SEARCH_WAIT':
            if self._tag_confirmed(now):
                self._activate_visual_parking()
            elif self._search_timed_out(now):
                self._abort_mission(
                    'Limited tag search timed out without one complete tag')
            elif now - self.search_wait_start_time >= self.search_dwell_time:
                self._send_next_search_goal()
            return

        if self.state == 'VISUAL_PARKING':
            if self.parking_complete:
                self._set_parking_enabled(False)
                self._set_camera_capture_enabled(False)
                self.state = 'COMPLETE'
                self.get_logger().info(
                    'Goal-directed mission complete: visual parking confirmed')
            elif (self.parking_timeout > 0.0 and
                    now - self.parking_start_time >= self.parking_timeout):
                self._set_parking_enabled(False)
                self._abort_mission('Visual parking timed out')

    def _try_start(self, now: float):
        if now - self.node_start_time < self.start_delay:
            return
        if self.latest_map is None:
            self._log_waiting('Waiting for /map from slam_toolbox')
            return
        if (not self.explore_client.server_is_ready() or
                not self.nav_client.server_is_ready() or
                (self.goal_directed_mode and not self.plan_client.server_is_ready()) or
                (self.goal_directed_mode and self.visual_parking_enabled and
                 (not self.parking_client.service_is_ready() or
                  not self.camera_client.service_is_ready()))):
            self._log_waiting(
                'Waiting for Roadmap Explorer, Nav2, camera, and visual parking interfaces')
            return
        if (self.goal_directed_mode and self.visual_parking_enabled and
                not self.parking_reset_complete):
            if not self.parking_reset_requested:
                self.parking_reset_requested = True
                request = SetBool.Request()
                request.data = False
                future = self.parking_client.call_async(request)
                future.add_done_callback(self._parking_reset_done)
            self._log_waiting(
                'Waiting for visual parking to release velocity ownership')
            return
        if (self.goal_directed_mode and self.visual_parking_enabled and
                not self.camera_reset_complete):
            if not self.camera_reset_requested:
                self.camera_reset_requested = True
                request = SetBool.Request()
                request.data = False
                future = self.camera_client.call_async(request)
                future.add_done_callback(self._camera_reset_done)
            self._log_waiting(
                'Waiting for camera capture to enter exploration sleep mode')
            return

        robot = self._robot_pose()
        if robot is None:
            return
        x, y, yaw = robot
        startup_escape_start = None
        if (
                self.startup_escape_enabled and
                self.startup_escape_distance > 0.0 and
                self.startup_escape_speed > 0.0):
            startup_escape_start = self._pose_in_frame(
                str(self.startup_escape_odom_frame))
            if startup_escape_start is None:
                return
        self.start_pose = self._make_pose(x, y, yaw)
        goal_x = x + math.cos(yaw) * self.goal_forward - math.sin(yaw) * self.goal_left
        goal_y = y + math.sin(yaw) * self.goal_forward + math.cos(yaw) * self.goal_left
        self.target_pose = self._make_pose(goal_x, goal_y, yaw)
        # The approach direction is not known at mission start.  It may be a
        # detour around walls, so derive the pre-parking pose later from the
        # tail of an actually reachable Nav2 path to the target.
        self.preparking_pose = None
        self.breadcrumbs = []
        append_breadcrumb(
            self.breadcrumbs, (x, y),
            self.breadcrumb_spacing, self.breadcrumb_max_points)
        self.target_pub.publish(self.target_pose)
        self.mission_start_time = now
        self.get_logger().info(
            f'Recorded start pose ({x:.2f}, {y:.2f}, yaw={math.degrees(yaw):.1f}deg); '
            f'Roadmap target=({goal_x:.2f}, {goal_y:.2f}), '
            f'relative=({self.goal_forward:.2f}m forward, {self.goal_left:.2f}m left). '
            f'The {self.preparking_distance:.2f}m pre-parking standoff will '
            'be measured backward along a reachable Nav2 path. '
            'Final Nav2 arrival uses XY position only; yaw is not a completion condition.')

        if (
                self.startup_escape_enabled and
                self.startup_escape_distance > 0.0 and
                self.startup_escape_speed > 0.0):
            self.startup_escape_start = startup_escape_start
            self.startup_escape_start_time = now
            self.state = 'STARTUP_ESCAPE'
            self._publish_startup_escape_stop()
            self.get_logger().warning(
                'Starting one-time trusted forward escape before Roadmap: '
                f'distance={self.startup_escape_distance:.2f}m, '
                f'speed={self.startup_escape_speed:.2f}m/s, '
                f'max_time={self.startup_escape_timeout:.1f}s. '
                'The absolute mission target is already locked to the '
                'original start pose.')
            return
        self._begin_planning_after_startup()

    def _startup_escape_control_tick(self):
        if self.state == 'STARTUP_ESCAPE':
            self._update_startup_escape(self._now_seconds())

    def _begin_planning_after_startup(self):
        if self.state in ('FAILED', 'COMPLETE'):
            return
        if self.goal_directed_mode:
            self._check_initial_direct_path()
        else:
            self._send_exploration_goal(new_session=True)

    def _publish_startup_escape_stop(self):
        self.startup_escape_pub.publish(Twist())

    def _startup_escape_scan_points(self) -> Optional[List[Point]]:
        if self.latest_scan is None:
            return None
        try:
            transform = self.tf_buffer.lookup_transform(
                str(self.base_frame), self.latest_scan.header.frame_id, Time(),
                timeout=Duration(seconds=0.2))
        except TransformException as exc:
            self._log_waiting(
                f'Waiting for startup scan transform into '
                f'{self.base_frame}: {exc}')
            return None

        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        translation_x = transform.transform.translation.x
        translation_y = transform.transform.translation.y
        minimum_range = max(0.02, float(self.latest_scan.range_min))
        maximum_range = float(self.latest_scan.range_max)
        points: List[Point] = []
        for index, distance in enumerate(self.latest_scan.ranges):
            distance = float(distance)
            if (
                    not math.isfinite(distance) or
                    distance < minimum_range or distance > maximum_range):
                continue
            angle = (
                float(self.latest_scan.angle_min) +
                index * float(self.latest_scan.angle_increment))
            scan_x = distance * math.cos(angle)
            scan_y = distance * math.sin(angle)
            points.append((
                translation_x + cos_yaw * scan_x - sin_yaw * scan_y,
                translation_y + sin_yaw * scan_x + cos_yaw * scan_y,
            ))
        return points

    def _update_startup_escape(self, now: float):
        if self.startup_escape_start is None:
            self._abort_mission('Startup escape has no recorded start pose')
            return
        if now - self.startup_escape_start_time > self.startup_escape_timeout:
            self._publish_startup_escape_stop()
            self._abort_mission(
                'Startup escape timed out before reaching '
                f'{self.startup_escape_distance:.2f}m')
            return

        robot = self._pose_in_frame(str(self.startup_escape_odom_frame))
        if robot is None:
            self._publish_startup_escape_stop()
            return
        progress, lateral_drift, heading_error = startup_escape_metrics(
            self.startup_escape_start, robot)
        if progress >= self.startup_escape_distance:
            self._finish_startup_escape(
                now, progress, lateral_drift, heading_error)
            return
        if progress < -0.02:
            self._publish_startup_escape_stop()
            self._abort_mission(
                f'Startup escape moved backward by {-progress:.3f}m')
            return
        if abs(heading_error) > self.startup_escape_max_yaw_error:
            self._publish_startup_escape_stop()
            self._abort_mission(
                f'Startup escape heading deviated by '
                f'{math.degrees(heading_error):.1f}deg')
            return
        if (
                self.latest_scan is None or
                now - self.latest_scan_time >
                self.startup_escape_scan_max_age):
            self._publish_startup_escape_stop()
            self._log_waiting(
                'Waiting for a fresh scan before startup forward motion')
            return

        scan_points = self._startup_escape_scan_points()
        if scan_points is None:
            self._publish_startup_escape_stop()
            return
        remaining = max(0.0, self.startup_escape_distance - progress)
        blocking = startup_escape_blocking_point(
            scan_points, remaining,
            self.startup_escape_footprint_front,
            self.startup_escape_braking_margin,
            self.startup_escape_swept_half_width)
        if blocking is not None:
            self._publish_startup_escape_stop()
            self._abort_mission(
                'Startup forward sweep is blocked at '
                f'base_link=({blocking[0]:.2f}, {blocking[1]:.2f})m')
            return

        cmd = Twist()
        cmd.linear.x = self.startup_escape_speed
        cmd.angular.z = max(
            -self.startup_escape_max_angular,
            min(
                self.startup_escape_max_angular,
                self.startup_escape_heading_kp * heading_error))
        self.startup_escape_pub.publish(cmd)
        self.get_logger().info(
            f'Startup escape: progress={progress:.3f}/'
            f'{self.startup_escape_distance:.3f}m, '
            f'lateral={lateral_drift:+.3f}m, '
            f'heading={math.degrees(heading_error):+.1f}deg, '
            f'cmd=(v={cmd.linear.x:.3f},w={cmd.angular.z:+.3f})',
            throttle_duration_sec=0.5)

    def _finish_startup_escape(
            self, now: float, progress: float,
            lateral_drift: float, heading_error: float):
        self._publish_startup_escape_stop()
        robot = self._robot_pose()
        if robot is not None:
            append_breadcrumb(
                self.breadcrumbs, (robot[0], robot[1]),
                min(self.breadcrumb_spacing, self.startup_escape_distance),
                self.breadcrumb_max_points)
        self.startup_escape_settle_start_time = now
        self.state = 'STARTUP_ESCAPE_SETTLE'
        self.get_logger().info(
            f'Startup escape complete: forward={progress:.3f}m, '
            f'lateral={lateral_drift:+.3f}m, '
            f'heading={math.degrees(heading_error):+.1f}deg. '
            f'Stopping for {self.startup_escape_settle_time:.1f}s so SLAM, '
            'costmaps, and twist_mux can settle.')

    def _check_initial_direct_path(self):
        """Skip Roadmap when a path-derived pre-parking pose is already safe."""
        if not self._target_is_known_free():
            self.get_logger().info(
                'Initial target cell is not known-free; starting Roadmap '
                'exploration before deriving a pre-parking pose')
            self._send_exploration_goal(new_session=True)
            return

        self.initial_plan_candidate = self.target_pose
        self.plan_in_progress = True
        self.state = 'CHECKING_INITIAL_PATH'
        goal = ComputePathToPose.Goal()
        goal.goal = self.target_pose
        goal.planner_id = str(self.planner_id)
        goal.use_start = False
        future = self.plan_client.send_goal_async(goal)
        future.add_done_callback(self._initial_plan_goal_response)
        self.get_logger().info(
            'Checking for an initial path to the target so pre-parking can '
            'be derived from its reachable approach')

    def _initial_plan_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().warning(
                f'Initial ComputePathToPose request failed: {exc}; '
                'starting Roadmap exploration')
            self._fallback_to_initial_exploration()
            return
        if self.state != 'CHECKING_INITIAL_PATH':
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        if not goal_handle.accepted:
            self.get_logger().info(
                'Nav2 rejected the initial direct-path check; starting '
                'Roadmap exploration')
            self._fallback_to_initial_exploration()
            return
        goal_handle.get_result_async().add_done_callback(
            self._initial_plan_result)

    def _initial_plan_result(self, future):
        self.plan_in_progress = False
        if self.state != 'CHECKING_INITIAL_PATH':
            return
        try:
            wrapped = future.result()
        except Exception as exc:
            self.get_logger().warning(
                f'Initial ComputePathToPose result failed: {exc}; '
                'starting Roadmap exploration')
            self._fallback_to_initial_exploration()
            return

        if (wrapped.status == GoalStatus.STATUS_SUCCEEDED and
                wrapped.result.path.poses):
            known_ratio = self._known_path_ratio(wrapped.result.path)
            candidate, reason = self._dynamic_preparking_from_path(
                wrapped.result.path)
            if (known_ratio >= self.direct_path_known_ratio and
                    candidate is not None):
                self.final_candidate = candidate
                self.initial_plan_candidate = None
                self.get_logger().info(
                    f'Initial target path is '
                    f'{known_ratio * 100.0:.0f}% known; skipping Roadmap '
                    'and navigating to its path-derived pre-parking pose')
                self._send_final_goal()
                return
            if known_ratio < self.direct_path_known_ratio:
                self.get_logger().info(
                    f'Initial target path is only '
                    f'{known_ratio * 100.0:.0f}% known; Roadmap exploration '
                    'is required')
            else:
                self.get_logger().info(
                    f'Initial target path cannot provide a safe dynamic '
                    f'pre-parking pose: {reason}; Roadmap exploration is '
                    'required')
        else:
            self.get_logger().info(
                'No initial Nav2 path to the target; Roadmap '
                'exploration is required')
        self._fallback_to_initial_exploration()

    def _fallback_to_initial_exploration(self):
        self.plan_in_progress = False
        self.initial_plan_candidate = None
        if self.state in ('FAILED', 'COMPLETE'):
            return
        self._send_exploration_goal(new_session=True)

    def _send_exploration_goal(self, new_session: bool):
        if self.explore_goal_handle is not None:
            self._abort_mission(
                'Cannot start Roadmap exploration while another goal is active')
            return
        goal = Explore.Goal()
        goal.exploration_bringup_mode = (
            Explore.Goal.NEW_EXPLORATION_SESSION
            if new_session else
            Explore.Goal.CONTINUE_FROM_TERMINATED_SESSION)
        goal.load_from_folder.data = ''
        goal.session_name.data = 'smartcar_goal_directed'
        self.exploration_request_mode = 'new' if new_session else 'continued'
        self.state = 'SENDING_EXPLORATION'
        future = self.explore_client.send_goal_async(goal)
        future.add_done_callback(self._explore_goal_response)

    def _parking_reset_done(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self._abort_mission(
                f'Could not disable visual parking before exploration: {exc}')
            return
        if not response.success:
            self._abort_mission(
                f'Visual parking did not release velocity ownership: '
                f'{response.message}')
            return
        self.parking_reset_complete = True
        self.get_logger().info(
            'Visual parking is inactive; Nav2 owns velocity during exploration')

    def _camera_reset_done(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self._abort_mission(
                f'Could not disable camera capture before exploration: {exc}')
            return
        if not response.success:
            self._abort_mission(
                f'Camera capture did not enter sleep mode: {response.message}')
            return
        self.camera_capture_enabled = False
        self.camera_reset_complete = True
        self.get_logger().info(
            'Camera capture is asleep during Roadmap exploration')

    def _maybe_warm_camera_near_target(self):
        if (not self.visual_parking_enabled or
                self.camera_capture_enabled or
                self.camera_activation_in_progress or
                self.target_pose is None or
                self.state in (
                    'STARTING', 'STARTUP_ESCAPE',
                    'STARTUP_ESCAPE_SETTLE', 'COMPLETE', 'FAILED',
                    'ACTIVATING_CAMERA', 'VISUAL_PARKING')):
            return
        robot = self._robot_pose()
        if robot is None:
            return
        target = self.target_pose.pose.position
        distance = math.hypot(robot[0] - target.x, robot[1] - target.y)
        if distance > self.camera_warmup_target_distance:
            return
        self.get_logger().info(
            f'Entered camera warm-up region ({distance:.2f}m <= '
            f'{self.camera_warmup_target_distance:.2f}m); '
            'opening camera while navigation continues')
        self._request_camera_activation()

    def _request_camera_activation(self):
        if self.camera_capture_enabled or self.camera_activation_in_progress:
            return
        if not self.camera_client.service_is_ready():
            self._abort_mission('Camera capture enable service is unavailable')
            return
        self.camera_activation_in_progress = True
        request = SetBool.Request()
        request.data = True
        future = self.camera_client.call_async(request)
        future.add_done_callback(self._camera_enable_done)

    def _explore_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._abort_mission(f'Roadmap Explorer request failed: {exc}')
            return
        if not goal_handle.accepted:
            self._abort_mission('Roadmap Explorer rejected the exploration goal')
            return
        if self.state in ('FAILED', 'COMPLETE'):
            goal_handle.cancel_goal_async()
            return
        self.explore_goal_handle = goal_handle
        self.state = 'EXPLORING'
        self._reset_tag_confirmation()
        self._reset_exploration_watchdog()
        self.next_direct_check_time = 0.0
        self.get_logger().info(
            f'Roadmap Explorer accepted the {self.exploration_request_mode} '
            'exploration goal')
        goal_handle.get_result_async().add_done_callback(self._explore_result)

    def _explore_result(self, future):
        try:
            wrapped = future.result()
        except Exception as exc:
            self._abort_mission(f'Roadmap Explorer result failed: {exc}')
            return
        self.explore_goal_handle = None

        if self.state == 'CANCELING_EXPLORATION':
            if wrapped.status != GoalStatus.STATUS_CANCELED:
                self._abort_mission(
                    'Roadmap exploration did not reach the canceled state '
                    f'before final navigation (status={wrapped.status})')
                return
            self.get_logger().info(
                'Roadmap exploration and its Nav2 goal are fully canceled; '
                'sending the final target goal')
            self._send_final_goal()
            return

        if self.state == 'CANCELING_EXPLORATION_FOR_TAG':
            if wrapped.status not in (
                    GoalStatus.STATUS_CANCELED,
                    GoalStatus.STATUS_SUCCEEDED):
                self._abort_mission(
                    'Roadmap exploration did not stop cleanly before visual '
                    f'parking handoff (status={wrapped.status})')
                return
            self.get_logger().info(
                'Roadmap exploration and its Nav2 goal are fully stopped; '
                'requiring a fresh complete Tag before visual parking takes '
                'velocity ownership')
            self._start_tag_acquisition(after_search=False)
            return

        if self.state == 'CANCELING_EXPLORATION_FOR_PROBE':
            if wrapped.status not in (
                    GoalStatus.STATUS_CANCELED,
                    GoalStatus.STATUS_SUCCEEDED):
                self._abort_mission(
                    'Roadmap exploration did not stop cleanly before a '
                    f'progressive probe (status={wrapped.status})')
                return
            self.get_logger().info(
                'Roadmap exploration is stopped; selecting a known-safe '
                'trajectory recovery')
            self._prepare_stall_recovery()
            return

        if self.state in ('FINAL_NAVIGATION', 'COMPLETE', 'FAILED'):
            return

        result = wrapped.result
        self.get_logger().info(
            f'Roadmap exploration finished: status={wrapped.status}, '
            f'success={result.success}, error_code={result.error_code}')
        if self.goal_directed_mode:
            self._abort_mission(
                'Roadmap exploration ended before a safe known path to the target was found')
            return
        if not result.success:
            self._abort_mission('Roadmap full exploration failed')
        elif self.return_to_start:
            self._send_return_goal()
        else:
            self.state = 'COMPLETE'
            self.get_logger().info('Roadmap full exploration complete')

    def _request_direct_path(self):
        if self.target_pose is None:
            return
        self.plan_in_progress = True
        goal = ComputePathToPose.Goal()
        goal.goal = self.target_pose
        goal.planner_id = str(self.planner_id)
        goal.use_start = False
        future = self.plan_client.send_goal_async(goal)
        future.add_done_callback(self._plan_goal_response)

    def _plan_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.plan_in_progress = False
            self.get_logger().warning(f'Final ComputePathToPose request failed: {exc}')
            return
        if self.state != 'EXPLORING':
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            self.plan_in_progress = False
            return
        if not goal_handle.accepted:
            self.plan_in_progress = False
            return
        goal_handle.get_result_async().add_done_callback(self._plan_result)

    def _plan_result(self, future):
        self.plan_in_progress = False
        if self.state != 'EXPLORING':
            return
        try:
            wrapped = future.result()
        except Exception as exc:
            self.get_logger().warning(f'Final ComputePathToPose result failed: {exc}')
            return
        if (wrapped.status != GoalStatus.STATUS_SUCCEEDED or
                not wrapped.result.path.poses):
            return
        known_ratio = self._known_path_ratio(wrapped.result.path)
        if known_ratio < self.direct_path_known_ratio:
            self.get_logger().info(
                f'Target path is only {known_ratio * 100.0:.0f}% known; '
                'Roadmap exploration continues')
            return

        candidate, reason = self._dynamic_preparking_from_path(
            wrapped.result.path)
        if candidate is None:
            self.get_logger().info(
                f'Target path cannot yet provide a safe dynamic pre-parking '
                f'pose: {reason}; Roadmap exploration continues')
            return

        self.final_candidate = candidate
        self.get_logger().info(
            f'Target path has {known_ratio * 100.0:.0f}% known cells and '
            f'a safe path-derived pre-parking pose; canceling Roadmap '
            'exploration and switching to position-only final Nav2 navigation')
        self._cancel_exploration_for_final_goal()

    def _cancel_exploration_for_final_goal(self):
        if self.explore_goal_handle is None:
            self._abort_mission('Cannot cancel Roadmap exploration: goal handle is missing')
            return
        self.state = 'CANCELING_EXPLORATION'
        future = self.explore_goal_handle.cancel_goal_async()
        future.add_done_callback(self._explore_cancel_done)

    def _explore_cancel_done(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self._abort_mission(f'Failed to cancel Roadmap exploration: {exc}')
            return
        if not response.goals_canceling:
            self._abort_mission('Roadmap exploration did not acknowledge cancellation')
            return
        # A successful cancellation response only means that the server accepted
        # the request. Roadmap Explorer still has to halt its custom Nav2 BT and
        # wait for that NavigateToPose goal to reach a terminal state. Sending
        # the final goal here causes Nav2 to treat it as a preemption; because
        # the two goals use different BT XML files, Nav2 rejects and aborts it.
        # _explore_result() sends the final goal after STATUS_CANCELED instead.
        if self.state == 'CANCELING_EXPLORATION_FOR_TAG':
            self.get_logger().info(
                'Roadmap exploration accepted cancellation; waiting for its '
                'Nav2 goal to terminate before visual parking handoff')
        elif self.state == 'CANCELING_EXPLORATION_FOR_PROBE':
            self.get_logger().info(
                'Roadmap exploration accepted cancellation; waiting for its '
                'Nav2 goal to terminate before progressive probing')
        else:
            self.get_logger().info(
                'Roadmap exploration accepted cancellation; waiting for its '
                'Nav2 goal to terminate before final navigation')

    def _reset_exploration_watchdog(self):
        now = self._now_seconds()
        robot = self._robot_pose()
        self.exploration_progress_pose = (
            (robot[0], robot[1]) if robot is not None else None)
        self.exploration_progress_time = now

    def _update_exploration_progress(self, now: float):
        robot = self._robot_pose()
        if robot is None:
            return
        current = (robot[0], robot[1])
        append_breadcrumb(
            self.breadcrumbs, current,
            self.breadcrumb_spacing, self.breadcrumb_max_points)
        if self.exploration_progress_pose is None:
            self.exploration_progress_pose = current
            self.exploration_progress_time = now
            return
        if math.hypot(
                current[0] - self.exploration_progress_pose[0],
                current[1] - self.exploration_progress_pose[1]
        ) >= self.exploration_progress_distance:
            self.exploration_progress_pose = current
            self.exploration_progress_time = now

    def _exploration_stalled(self, now: float) -> bool:
        return (
            now >= self.recovery_cooldown_until and
            self.exploration_progress_time > 0.0 and
            now - self.exploration_progress_time >=
            self.exploration_stall_timeout)

    def _cancel_exploration_for_probe(self):
        if self.explore_goal_handle is None:
            self._abort_mission(
                'Cannot start progressive probing: Roadmap goal handle is missing')
            return
        self.pending_target_reveal = self._target_local_reveal_applicable()
        self.state = 'CANCELING_EXPLORATION_FOR_PROBE'
        future = self.explore_goal_handle.cancel_goal_async()
        future.add_done_callback(self._explore_cancel_done)
        recovery = (
            'a bounded target-local reveal'
            if self.pending_target_reveal else
            'breadcrumb backtracking or a short target-directed probe')
        self.get_logger().warning(
            'Roadmap exploration made no positional progress for '
            f'{self.exploration_stall_timeout:.1f}s; canceling it for '
            f'{recovery}')

    def _try_exploration_tag_handoff(self, now: float) -> bool:
        """
        Prioritize a nearby complete Tag over map-only exploration recovery.

        ``True`` means the exploration tick has been consumed: either a fresh
        valid Tag is accumulating its stability time, or Roadmap cancellation
        has started. An invalid/out-of-region observation leaves normal
        exploration and its watchdog active.
        """
        if (not self.goal_directed_mode or
                not self.visual_parking_enabled or
                self.target_pose is None):
            self._reset_tag_confirmation()
            return False

        robot = self._robot_pose()
        if robot is None:
            self._reset_tag_confirmation()
            return False
        target = self.target_pose.pose.position
        target_distance = math.hypot(
            target.x - robot[0], target.y - robot[1])
        if target_distance > self.tag_handoff_max_target_distance:
            self._reset_tag_confirmation()
            return False

        confirmation_started = self.tag_stable_since is not None
        confirmed = self._tag_confirmed(now)
        if self.tag_stable_since is None:
            # The robot is near the nominal target, but there is no fresh,
            # complete, quality-filtered Tag. Map exploration must continue.
            return False
        if not confirmation_started:
            self.get_logger().info(
                'Complete Tag candidate detected during Roadmap exploration '
                f'{target_distance:.2f}m from the target; holding stall '
                f'recovery for {self.tag_confirm_time:.2f}s confirmation')
        if not confirmed:
            return True

        self._cancel_exploration_for_tag(robot, target_distance)
        return True

    def _cancel_exploration_for_tag(
            self, robot: Tuple[float, float, float],
            target_distance: float):
        if self.explore_goal_handle is None:
            self._abort_mission(
                'Cannot hand off to visual parking: Roadmap goal handle is '
                'missing')
            return

        # If the Tag disappears while Roadmap/Nav2 is stopping, bounded search
        # needs an anchor. The current pose is already physically reached and
        # therefore does not depend on the nominal pre-parking clearance test.
        self.final_candidate = self._make_pose(
            robot[0], robot[1], robot[2])
        self.state = 'CANCELING_EXPLORATION_FOR_TAG'
        future = self.explore_goal_handle.cancel_goal_async()
        future.add_done_callback(self._explore_cancel_done)
        self.get_logger().warning(
            'Stable complete Tag confirmed during Roadmap exploration at '
            f'target distance {target_distance:.2f}m; canceling Roadmap and '
            'its Nav2 goal before visual parking handoff')

    def _prepare_target_local_reveal(self):
        if self.preparking_pose is None or self.target_pose is None:
            self._prepare_stall_recovery(allow_target_reveal=False)
            return
        approach = self.preparking_pose.pose.position
        target = self.target_pose.pose.position
        generated = target_reveal_points(
            (approach.x, approach.y), (target.x, target.y),
            self.target_local_reveal_forward_offsets,
            self.target_local_reveal_lateral_offsets,
            self.target_local_reveal_min_standoff)
        self.target_reveal_candidates = [
            point for point in generated if self._safe_search_point(point)]
        self.target_reveal_index = 0
        self.target_reveal_plan_candidate = None
        self.target_reveal_attempts = 0
        self.get_logger().warning(
            'Target handoff is blocked only by unknown clearance cells; '
            f'target-local reveal generated {len(generated)} viewpoints and '
            f'{len(self.target_reveal_candidates)} have known-free endpoints')
        self._send_next_target_reveal_candidate()

    def _send_next_target_reveal_candidate(self):
        if (self.target_reveal_attempts >=
                self.target_local_reveal_max_attempts):
            self._finish_target_local_reveal(
                'Target-local reveal attempt limit reached')
            return
        while self.target_reveal_index < len(
                self.target_reveal_candidates):
            index = self.target_reveal_index
            point = self.target_reveal_candidates[index]
            self.target_reveal_index += 1
            robot = self._robot_pose()
            yaw = robot[2] if robot is not None else 0.0
            candidate = self._make_pose(point[0], point[1], yaw)
            self.get_logger().info(
                f'Checking target-local reveal viewpoint {index + 1}/'
                f'{len(self.target_reveal_candidates)}: '
                f'({point[0]:.2f}, {point[1]:.2f})')
            self._request_target_reveal_plan(candidate)
            return
        self._finish_target_local_reveal(
            'No target-local reveal viewpoint has a fully known Nav2 path')

    def _request_target_reveal_plan(self, candidate: PoseStamped):
        self.plan_in_progress = True
        self.target_reveal_plan_candidate = candidate
        self.state = 'PLANNING_TARGET_REVEAL'
        goal = ComputePathToPose.Goal()
        goal.goal = candidate
        goal.planner_id = str(self.planner_id)
        goal.use_start = False
        future = self.plan_client.send_goal_async(goal)
        future.add_done_callback(self._target_reveal_plan_goal_response)

    def _target_reveal_plan_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.plan_in_progress = False
            self.get_logger().warning(
                f'Target-local reveal planning request failed: {exc}')
            self._send_next_target_reveal_candidate()
            return
        if self.state != 'PLANNING_TARGET_REVEAL':
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            self.plan_in_progress = False
            return
        if not goal_handle.accepted:
            self.plan_in_progress = False
            self.get_logger().warning(
                'Nav2 rejected one target-local reveal plan request')
            self._send_next_target_reveal_candidate()
            return
        goal_handle.get_result_async().add_done_callback(
            self._target_reveal_plan_result)

    def _target_reveal_plan_result(self, future):
        self.plan_in_progress = False
        if self.state != 'PLANNING_TARGET_REVEAL':
            return
        try:
            wrapped = future.result()
        except Exception as exc:
            self.get_logger().warning(
                f'Target-local reveal planning result failed: {exc}')
            self._send_next_target_reveal_candidate()
            return
        if (wrapped.status != GoalStatus.STATUS_SUCCEEDED or
                not wrapped.result.path.poses):
            self.get_logger().warning(
                'Target-local reveal viewpoint has no Nav2 path')
            self._send_next_target_reveal_candidate()
            return
        known_ratio = self._known_path_ratio(
            wrapped.result.path, sample_stride=1)
        if known_ratio + 1e-9 < self.target_local_reveal_known_ratio:
            self.get_logger().warning(
                f'Target-local reveal path is only '
                f'{known_ratio * 100.0:.0f}% known; trying another viewpoint')
            self._send_next_target_reveal_candidate()
            return
        candidate = self.target_reveal_plan_candidate
        if candidate is None:
            self._abort_mission(
                'Target-local reveal candidate pose is missing')
            return
        self.target_reveal_attempts += 1
        self.get_logger().warning(
            f'Dispatching target-local reveal '
            f'{self.target_reveal_attempts}/'
            f'{self.target_local_reveal_max_attempts}: '
            f'({candidate.pose.position.x:.2f}, '
            f'{candidate.pose.position.y:.2f}), '
            f'path_known={known_ratio * 100.0:.0f}%')
        self._send_nav_goal(candidate, 'reveal')

    def _complete_target_local_reveal(self):
        robot = self._robot_pose()
        if robot is not None:
            append_breadcrumb(
                self.breadcrumbs, (robot[0], robot[1]),
                self.breadcrumb_spacing, self.breadcrumb_max_points)
        self.target_reveal_settle_start_time = self._now_seconds()
        self.state = 'TARGET_REVEAL_SETTLE'
        self.get_logger().info(
            f'Target-local reveal viewpoint reached; waiting '
            f'{self.target_local_reveal_settle_time:.1f}s for SLAM to '
            'integrate fresh laser observations')

    def _try_target_reveal_tag_handoff(self, now: float) -> bool:
        robot = self._robot_pose()
        if (robot is None or not self.visual_parking_enabled or
                not self._inside_tag_handoff_region() or
                not self._tag_confirmed(now)):
            return False
        self.final_candidate = self._make_pose(
            robot[0], robot[1], robot[2])
        self.get_logger().info(
            'Target-local reveal exposed a complete Tag; transferring '
            'to visual parking without another map handoff check')
        self._start_tag_acquisition(after_search=False)
        return True

    def _evaluate_target_local_reveal(self):
        if (self.visual_parking_enabled and
                self._inside_tag_handoff_region() and
                self.tag_stable_since is not None):
            # A Tag may have appeared at the end of the settle interval. Give
            # its configured confirmation window priority over moving again.
            return

        if not self._target_is_known_free():
            self.get_logger().info(
                'Fresh scan has not yet made the target cell known-free; '
                'trying the next bounded reveal viewpoint')
            self._send_next_target_reveal_candidate()
            return
        self._request_target_reveal_handoff_plan()

    def _request_target_reveal_handoff_plan(self):
        if self.target_pose is None:
            self._finish_target_local_reveal(
                'Cannot re-plan target-local handoff without a target pose')
            return
        self.plan_in_progress = True
        self.target_reveal_plan_candidate = self.target_pose
        self.state = 'PLANNING_TARGET_REVEAL_HANDOFF'
        goal = ComputePathToPose.Goal()
        goal.goal = self.target_pose
        goal.planner_id = str(self.planner_id)
        goal.use_start = False
        future = self.plan_client.send_goal_async(goal)
        future.add_done_callback(
            self._target_reveal_handoff_goal_response)

    def _target_reveal_handoff_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.plan_in_progress = False
            self.get_logger().warning(
                f'Post-reveal target path request failed: {exc}')
            self._send_next_target_reveal_candidate()
            return
        if self.state != 'PLANNING_TARGET_REVEAL_HANDOFF':
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            self.plan_in_progress = False
            return
        if not goal_handle.accepted:
            self.plan_in_progress = False
            self.get_logger().warning(
                'Nav2 rejected the post-reveal target path request')
            self._send_next_target_reveal_candidate()
            return
        goal_handle.get_result_async().add_done_callback(
            self._target_reveal_handoff_result)

    def _target_reveal_handoff_result(self, future):
        self.plan_in_progress = False
        if self.state != 'PLANNING_TARGET_REVEAL_HANDOFF':
            return
        try:
            wrapped = future.result()
        except Exception as exc:
            self.get_logger().warning(
                f'Post-reveal target path result failed: {exc}')
            self._send_next_target_reveal_candidate()
            return
        if (wrapped.status != GoalStatus.STATUS_SUCCEEDED or
                not wrapped.result.path.poses):
            self.get_logger().warning(
                'No Nav2 path to the target after local reveal')
            self._send_next_target_reveal_candidate()
            return
        known_ratio = self._known_path_ratio(wrapped.result.path)
        if known_ratio + 1e-9 < self.direct_path_known_ratio:
            self.get_logger().warning(
                f'Post-reveal target path is only '
                f'{known_ratio * 100.0:.0f}% known; trying another viewpoint')
            self._send_next_target_reveal_candidate()
            return
        candidate, reason = self._dynamic_preparking_from_path(
            wrapped.result.path)
        if candidate is None:
            self.get_logger().warning(
                f'Post-reveal target path still has no safe dynamic '
                f'pre-parking pose: {reason}')
            self._send_next_target_reveal_candidate()
            return
        self.final_candidate = candidate
        self.get_logger().info(
            f'Target-local reveal completed the handoff corridor '
            f'({known_ratio * 100.0:.0f}% known); sending position-only '
            'final Nav2 goal to the path-derived pre-parking pose')
        self._send_final_goal()

    def _finish_target_local_reveal(self, reason: str):
        self.target_reveal_plan_candidate = None
        self.get_logger().warning(
            f'{reason}; falling back to travelled-route recovery')
        self._prepare_stall_recovery(allow_target_reveal=False)

    def _prepare_stall_recovery(self, allow_target_reveal: bool = True):
        robot = self._robot_pose()
        if robot is None:
            self._abort_mission(
                'Cannot recover from exploration stall without robot pose')
            return

        if (allow_target_reveal and self.pending_target_reveal and
                self.target_local_reveal_enabled):
            self.pending_target_reveal = False
            self._prepare_target_local_reveal()
            return
        self.pending_target_reveal = False

        if (self.deadend_backtrack_enabled and
                self.backtrack_recoveries <
                self.deadend_backtrack_max_recoveries):
            generated = breadcrumb_backtrack_points(
                self.breadcrumbs, (robot[0], robot[1]),
                self.deadend_backtrack_min_distance,
                self.deadend_backtrack_max_distance,
                self.deadend_backtrack_candidate_spacing)
            self.backtrack_candidates = [
                point for point in generated
                if self._safe_search_point(point)]
            self.backtrack_index = 0
            self.backtrack_plan_candidate = None
            self.get_logger().info(
                f'Breadcrumb recovery found {len(generated)} travelled '
                f'candidates; {len(self.backtrack_candidates)} remain '
                'known-free with endpoint clearance')
            if self.backtrack_candidates:
                self._send_next_backtrack_candidate()
                return
        elif self.deadend_backtrack_enabled:
            self.get_logger().warning(
                'Breadcrumb recovery limit reached; using bounded '
                'target-directed probing as fallback')

        self.get_logger().warning(
            'No usable breadcrumb exit is available; falling back to a '
            'short known-safe target-directed probe')
        self._prepare_progressive_probe()

    def _send_next_backtrack_candidate(self):
        while self.backtrack_index < len(self.backtrack_candidates):
            index = self.backtrack_index
            point = self.backtrack_candidates[index]
            self.backtrack_index += 1
            robot = self._robot_pose()
            yaw = robot[2] if robot is not None else 0.0
            candidate = self._make_pose(point[0], point[1], yaw)
            self.get_logger().info(
                f'Checking breadcrumb recovery candidate {index + 1}/'
                f'{len(self.backtrack_candidates)}: '
                f'({point[0]:.2f}, {point[1]:.2f})')
            self._request_backtrack_plan(candidate)
            return

        self.get_logger().warning(
            'No breadcrumb candidate has a fully known Nav2 path; '
            'trying target-directed probing')
        self._prepare_progressive_probe()

    def _request_backtrack_plan(self, candidate: PoseStamped):
        self.plan_in_progress = True
        self.backtrack_plan_candidate = candidate
        self.state = 'PLANNING_BACKTRACK'
        goal = ComputePathToPose.Goal()
        goal.goal = candidate
        goal.planner_id = str(self.planner_id)
        goal.use_start = False
        future = self.plan_client.send_goal_async(goal)
        future.add_done_callback(self._backtrack_plan_goal_response)

    def _backtrack_plan_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.plan_in_progress = False
            self.get_logger().warning(
                f'Breadcrumb recovery planning request failed: {exc}')
            self._send_next_backtrack_candidate()
            return
        if self.state != 'PLANNING_BACKTRACK':
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            self.plan_in_progress = False
            return
        if not goal_handle.accepted:
            self.plan_in_progress = False
            self.get_logger().warning(
                'Nav2 rejected one breadcrumb recovery plan request')
            self._send_next_backtrack_candidate()
            return
        goal_handle.get_result_async().add_done_callback(
            self._backtrack_plan_result)

    def _backtrack_plan_result(self, future):
        self.plan_in_progress = False
        if self.state != 'PLANNING_BACKTRACK':
            return
        try:
            wrapped = future.result()
        except Exception as exc:
            self.get_logger().warning(
                f'Breadcrumb recovery planning result failed: {exc}')
            self._send_next_backtrack_candidate()
            return
        if (wrapped.status != GoalStatus.STATUS_SUCCEEDED or
                not wrapped.result.path.poses):
            self.get_logger().warning(
                'Breadcrumb recovery candidate has no Nav2 path')
            self._send_next_backtrack_candidate()
            return
        known_ratio = self._known_path_ratio(
            wrapped.result.path, sample_stride=1)
        if known_ratio + 1e-9 < self.progressive_probe_known_ratio:
            self.get_logger().warning(
                f'Breadcrumb recovery path is only '
                f'{known_ratio * 100.0:.0f}% known; trying an older point')
            self._send_next_backtrack_candidate()
            return
        candidate = self.backtrack_plan_candidate
        if candidate is None:
            self._abort_mission(
                'Breadcrumb recovery candidate pose is missing')
            return
        self.backtrack_target = (
            candidate.pose.position.x, candidate.pose.position.y)
        self.get_logger().warning(
            f'Dispatching breadcrumb recovery '
            f'{self.backtrack_recoveries + 1}/'
            f'{self.deadend_backtrack_max_recoveries}: '
            f'({self.backtrack_target[0]:.2f}, '
            f'{self.backtrack_target[1]:.2f}), '
            f'path_known={known_ratio * 100.0:.0f}%')
        self._send_nav_goal(candidate, 'backtrack')

    def _prepare_progressive_probe(self):
        robot = self._robot_pose()
        destination_pose = self.preparking_pose or self.target_pose
        if robot is None or destination_pose is None:
            self._abort_mission(
                'Cannot generate a progressive probe without robot and '
                'target poses')
            return
        destination = destination_pose.pose.position
        origin = (robot[0], robot[1])
        target = (destination.x, destination.y)
        self.probe_start_target_distance = math.hypot(
            target[0] - origin[0], target[1] - origin[1])
        generated = progressive_probe_points(
            origin, target,
            self.progressive_probe_step,
            self.progressive_probe_min_step,
            self.progressive_probe_fan_angles_deg,
            self.progressive_probe_min_progress)
        self.probe_candidates = [
            point for point in generated if self._safe_search_point(point)]
        self.probe_index = 0
        self.probe_plan_candidate = None
        self.get_logger().info(
            f'Progressive probe generated {len(generated)} geometric '
            f'candidates; {len(self.probe_candidates)} have known-free '
            'endpoints and required clearance')
        self._send_next_probe_candidate()

    def _send_next_probe_candidate(self):
        if self.probe_attempts >= self.progressive_probe_max_attempts:
            self.get_logger().warning(
                'Progressive probe attempt limit reached; preserving the '
                'mission and returning control to Roadmap')
            self.probe_attempts = 0
            self._settle_recovery('probe_limit')
            return
        while self.probe_index < len(self.probe_candidates):
            index = self.probe_index
            point = self.probe_candidates[index]
            self.probe_index += 1
            robot = self._robot_pose()
            yaw = robot[2] if robot is not None else 0.0
            candidate = self._make_pose(point[0], point[1], yaw)
            self.get_logger().info(
                f'Checking progressive probe candidate {index + 1}/'
                f'{len(self.probe_candidates)}: '
                f'({point[0]:.2f}, {point[1]:.2f})')
            self._request_probe_plan(candidate)
            return
        self._handle_probe_selection_exhausted()

    def _request_probe_plan(self, candidate: PoseStamped):
        self.plan_in_progress = True
        self.probe_plan_candidate = candidate
        self.state = 'PLANNING_PROBE'
        goal = ComputePathToPose.Goal()
        goal.goal = candidate
        goal.planner_id = str(self.planner_id)
        goal.use_start = False
        future = self.plan_client.send_goal_async(goal)
        future.add_done_callback(self._probe_plan_goal_response)

    def _probe_plan_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.plan_in_progress = False
            self.get_logger().warning(
                f'Progressive probe planning request failed: {exc}')
            self._send_next_probe_candidate()
            return
        if self.state != 'PLANNING_PROBE':
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            self.plan_in_progress = False
            return
        if not goal_handle.accepted:
            self.plan_in_progress = False
            self.get_logger().warning(
                'Nav2 rejected one progressive probe plan request')
            self._send_next_probe_candidate()
            return
        goal_handle.get_result_async().add_done_callback(
            self._probe_plan_result)

    def _probe_plan_result(self, future):
        self.plan_in_progress = False
        if self.state != 'PLANNING_PROBE':
            return
        try:
            wrapped = future.result()
        except Exception as exc:
            self.get_logger().warning(
                f'Progressive probe planning result failed: {exc}')
            self._send_next_probe_candidate()
            return
        if (wrapped.status != GoalStatus.STATUS_SUCCEEDED or
                not wrapped.result.path.poses):
            self.get_logger().warning(
                'Progressive probe candidate has no Nav2 path')
            self._send_next_probe_candidate()
            return
        # A short probe must remain entirely inside mapped free space. Unlike
        # the periodic final-path check, inspect every returned path pose.
        known_ratio = self._known_path_ratio(
            wrapped.result.path, sample_stride=1)
        if known_ratio + 1e-9 < self.progressive_probe_known_ratio:
            self.get_logger().warning(
                f'Progressive probe path is only '
                f'{known_ratio * 100.0:.0f}% known; trying another candidate')
            self._send_next_probe_candidate()
            return
        candidate = self.probe_plan_candidate
        if candidate is None:
            self._abort_mission('Progressive probe candidate pose is missing')
            return
        self.probe_attempts += 1
        self.get_logger().warning(
            f'Dispatching progressive probe {self.probe_attempts}/'
            f'{self.progressive_probe_max_attempts}: '
            f'({candidate.pose.position.x:.2f}, '
            f'{candidate.pose.position.y:.2f}), '
            f'path_known={known_ratio * 100.0:.0f}%')
        self._send_nav_goal(candidate, 'probe')

    def _handle_probe_selection_exhausted(self):
        self.probe_failures += 1
        if self.probe_failures >= self.progressive_probe_max_failures:
            self.get_logger().warning(
                'No known-safe target-directed probe remains after '
                f'{self.probe_failures} selection cycles; this may be a '
                'required topological detour, so Roadmap will continue')
            self.probe_failures = 0
            self._settle_recovery('probe_exhausted')
            return
        self.get_logger().warning(
            'No progressive probe candidate is currently safe and reachable; '
            'resuming Roadmap once before trying again')
        self._settle_recovery('probe_unavailable')

    def _complete_progressive_probe(self):
        robot = self._robot_pose()
        destination_pose = self.preparking_pose or self.target_pose
        if robot is None or destination_pose is None:
            self._abort_mission(
                'Cannot measure progress after target-directed probing')
            return
        target = destination_pose.pose.position
        remaining = math.hypot(target.x - robot[0], target.y - robot[1])
        progress = self.probe_start_target_distance - remaining
        if progress + 1e-9 < self.progressive_probe_min_progress:
            self.probe_failures += 1
            self.get_logger().warning(
                f'Progressive probe reduced target distance by only '
                f'{progress:.2f}m; failure '
                f'{self.probe_failures}/{self.progressive_probe_max_failures}')
            if self.probe_failures >= self.progressive_probe_max_failures:
                self.get_logger().warning(
                    'Progressive probing repeatedly failed to reduce target '
                    'distance; Roadmap will continue because a valid detour '
                    'may temporarily move away from the target')
                self.probe_failures = 0
                self.probe_attempts = 0
                robot_point = (robot[0], robot[1])
                append_breadcrumb(
                    self.breadcrumbs, robot_point,
                    self.breadcrumb_spacing, self.breadcrumb_max_points)
                self._settle_recovery('probe_no_progress')
                return
        else:
            self.probe_failures = 0
            self.probe_attempts = 0
            append_breadcrumb(
                self.breadcrumbs, (robot[0], robot[1]),
                self.breadcrumb_spacing, self.breadcrumb_max_points)
            self.get_logger().info(
                f'Progressive probe advanced {progress:.2f}m toward the '
                f'pre-parking point; remaining distance={remaining:.2f}m')
        self._settle_recovery('probe')

    def _resume_roadmap_after_probe(self):
        self.recovery_cooldown_until = (
            self._now_seconds() + self.deadend_recovery_cooldown)
        self.get_logger().info(
            f'{self.recovery_settle_kind} recovery settled; continuing the '
            'existing Roadmap session with the updated SLAM map and '
            f'{self.deadend_recovery_cooldown:.1f}s recovery cooldown')
        self._send_exploration_goal(new_session=False)

    def _settle_recovery(self, kind: str):
        self.recovery_settle_kind = kind
        self.probe_settle_start_time = self._now_seconds()
        self.state = 'PROBE_SETTLE'

    def _complete_backtrack(self):
        robot = self._robot_pose()
        if robot is None or self.backtrack_target is None:
            self._abort_mission(
                'Cannot finish breadcrumb recovery without robot and target')
            return

        if self.breadcrumbs:
            nearest_index = min(
                range(len(self.breadcrumbs)),
                key=lambda index: math.hypot(
                    self.breadcrumbs[index][0] - self.backtrack_target[0],
                    self.breadcrumbs[index][1] - self.backtrack_target[1]))
            self.breadcrumbs = self.breadcrumbs[:nearest_index + 1]
        append_breadcrumb(
            self.breadcrumbs, (robot[0], robot[1]),
            self.breadcrumb_spacing, self.breadcrumb_max_points)
        self.backtrack_recoveries += 1
        self.probe_failures = 0
        self.probe_attempts = 0
        self.get_logger().info(
            f'Breadcrumb recovery reached ({robot[0]:.2f}, '
            f'{robot[1]:.2f}); trimmed the dead-end tail and will resume '
            'Roadmap so it can choose another reachable frontier')
        self.backtrack_target = None
        self._settle_recovery('breadcrumb')

    def _send_final_goal(self):
        if self.final_candidate is None:
            self._abort_mission('Final goal pose is missing')
            return
        self._send_nav_goal(self.final_candidate, 'approach')

    def _send_nav_goal(self, pose: PoseStamped, purpose: str):
        """Send one bounded Nav2 motion whose completion is checked by XY only."""
        if self.final_goal_handle is not None:
            self._abort_mission('Cannot send Nav2 goal while another goal is active')
            return
        sending_states = {
            'approach': 'SENDING_FINAL_NAVIGATION',
            'search': 'SENDING_SEARCH_NAVIGATION',
            'probe': 'SENDING_PROBE_NAVIGATION',
            'backtrack': 'SENDING_BACKTRACK_NAVIGATION',
            'reveal': 'SENDING_TARGET_REVEAL_NAVIGATION',
        }
        if purpose not in sending_states:
            self._abort_mission(f'Unknown Nav2 mission purpose: {purpose}')
            return

        # A valid orientation is still required by NavigateToPose. The
        # mission-only behavior tree explicitly selects position_goal_checker,
        # so this orientation is never a completion condition. Keeping current
        # yaw also avoids requesting an arbitrary final turn during search.
        robot = self._robot_pose()
        yaw = robot[2] if robot is not None else 0.0
        nav_pose = self._make_pose(
            pose.pose.position.x, pose.pose.position.y, yaw)
        goal = NavigateToPose.Goal()
        goal.pose = nav_pose
        if self.position_only_bt_xml:
            goal.behavior_tree = str(self.position_only_bt_xml)
        self.active_nav_target = nav_pose
        self.active_nav_purpose = purpose
        self.nav_cancel_reason = None
        self.nav_sequence += 1
        sequence = self.nav_sequence
        self.state = sending_states[purpose]
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(
            lambda result, seq=sequence: self._nav_goal_response(result, seq))

    def _nav_goal_response(self, future, sequence: int):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._abort_mission(f'NavigateToPose request failed: {exc}')
            return
        if sequence != self.nav_sequence:
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        if not goal_handle.accepted:
            if self.active_nav_purpose == 'search':
                self.get_logger().warning(
                    'Nav2 rejected one limited-search waypoint; trying the next')
                self.final_goal_handle = None
                self._send_next_search_goal()
            elif self.active_nav_purpose == 'probe':
                self.get_logger().warning(
                    'Nav2 rejected one progressive probe; trying another')
                self.final_goal_handle = None
                self.active_nav_target = None
                self.active_nav_purpose = None
                self._send_next_probe_candidate()
            elif self.active_nav_purpose == 'backtrack':
                self.get_logger().warning(
                    'Nav2 rejected one breadcrumb recovery goal; trying an '
                    'older known-safe point')
                self.final_goal_handle = None
                self.active_nav_target = None
                self.active_nav_purpose = None
                self.backtrack_target = None
                self._send_next_backtrack_candidate()
            elif self.active_nav_purpose == 'reveal':
                self.get_logger().warning(
                    'Nav2 rejected one target-local reveal goal; trying '
                    'another known-safe viewpoint')
                self.final_goal_handle = None
                self.active_nav_target = None
                self.active_nav_purpose = None
                self._send_next_target_reveal_candidate()
            else:
                self._abort_mission('Nav2 rejected the pre-parking goal')
            return
        if self.state in ('FAILED', 'COMPLETE'):
            goal_handle.cancel_goal_async()
            return
        self.final_goal_handle = goal_handle
        purpose = self.active_nav_purpose
        active_states = {
            'approach': 'FINAL_NAVIGATION',
            'search': 'SEARCH_NAVIGATION',
            'probe': 'PROBE_NAVIGATION',
            'backtrack': 'BACKTRACK_NAVIGATION',
            'reveal': 'TARGET_REVEAL_NAVIGATION',
        }
        self.state = active_states[purpose]
        self.get_logger().info(
            f'Nav2 {purpose} goal accepted; arrival will use '
            f'XY tolerance {self.position_arrival_tolerance:.2f}m and ignore yaw')
        goal_handle.get_result_async().add_done_callback(
            lambda result, seq=sequence: self._nav_result(result, seq))

    def _cancel_active_nav(self, reason: str):
        if self.final_goal_handle is None or self.state == 'CANCELING_NAV':
            return
        self.nav_cancel_reason = reason
        self.state = 'CANCELING_NAV'
        future = self.final_goal_handle.cancel_goal_async()
        future.add_done_callback(self._nav_cancel_done)
        self.get_logger().info(
            f'Canceling Nav2 {self.active_nav_purpose} goal: {reason}')

    def _nav_cancel_done(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self._abort_mission(f'Failed to cancel Nav2 before handoff: {exc}')
            return
        if not response.goals_canceling:
            self.get_logger().warning(
                'Nav2 goal was already terminal while cancellation was requested')

    def _nav_result(self, future, sequence: int):
        if sequence != self.nav_sequence:
            return
        try:
            status = future.result().status
        except Exception as exc:
            self._abort_mission(f'NavigateToPose result failed: {exc}')
            return

        purpose = self.active_nav_purpose
        cancel_reason = self.nav_cancel_reason
        self.final_goal_handle = None
        self.active_nav_target = None
        self.active_nav_purpose = None
        self.nav_cancel_reason = None
        if self.state in ('FAILED', 'COMPLETE'):
            return

        if purpose == 'approach':
            reached_position = cancel_reason == 'position_reached'
            tag_seen = cancel_reason == 'tag_confirmed'
            if status == GoalStatus.STATUS_SUCCEEDED or reached_position or tag_seen:
                if self.visual_parking_enabled:
                    self._start_tag_acquisition(after_search=False)
                else:
                    self.state = 'COMPLETE'
                    self.get_logger().info(
                        'Goal-directed mission complete at pre-parking XY; '
                        'final yaw was intentionally ignored')
                return
            self._abort_mission(
                f'Pre-parking navigation failed with status {status}')
            return

        if purpose == 'search':
            if cancel_reason == 'search_timeout':
                self._abort_mission(
                    'Limited tag search timed out without one complete tag')
            elif cancel_reason == 'tag_confirmed':
                self._start_tag_acquisition(after_search=True)
            elif (status == GoalStatus.STATUS_SUCCEEDED or
                  cancel_reason == 'position_reached'):
                self._start_search_wait()
            else:
                self.get_logger().warning(
                    f'Limited-search Nav2 waypoint failed with status {status}; '
                    'trying the next waypoint')
                self._send_next_search_goal()
            return

        if purpose == 'probe':
            if (status == GoalStatus.STATUS_SUCCEEDED or
                    cancel_reason == 'position_reached'):
                self._complete_progressive_probe()
            else:
                self.get_logger().warning(
                    f'Progressive probe Nav2 goal failed with status {status}; '
                    'trying another known-safe candidate')
                self._send_next_probe_candidate()
            return

        if purpose == 'backtrack':
            if (status == GoalStatus.STATUS_SUCCEEDED or
                    cancel_reason == 'position_reached'):
                self._complete_backtrack()
            else:
                self.get_logger().warning(
                    f'Breadcrumb recovery Nav2 goal failed with status '
                    f'{status}; trying another older known-safe point')
                self.backtrack_target = None
                self._send_next_backtrack_candidate()
            return

        if purpose == 'reveal':
            if cancel_reason == 'tag_confirmed':
                robot = self._robot_pose()
                if robot is None:
                    self._abort_mission(
                        'Cannot anchor visual handoff after target reveal')
                    return
                self.final_candidate = self._make_pose(
                    robot[0], robot[1], robot[2])
                self._start_tag_acquisition(after_search=False)
            elif (status == GoalStatus.STATUS_SUCCEEDED or
                  cancel_reason == 'position_reached'):
                self._complete_target_local_reveal()
            else:
                self.get_logger().warning(
                    f'Target-local reveal Nav2 goal failed with status '
                    f'{status}; trying another known-safe viewpoint')
                self._send_next_target_reveal_candidate()
            return

        self._abort_mission('Received a Nav2 result with no mission purpose')

    def _active_nav_position_reached(self) -> bool:
        if self.active_nav_target is None:
            return False
        robot = self._robot_pose()
        if robot is None:
            return False
        goal = self.active_nav_target.pose.position
        return position_reached(
            (robot[0], robot[1]), (goal.x, goal.y),
            self.position_arrival_tolerance)

    def _reset_tag_confirmation(self):
        self.tag_stable_since = None
        self.tag_confirmation_reported = False

    def _tag_quality_valid(self, now: float) -> bool:
        return tag_observation_valid(
            self.tag_visible_count,
            self.tag_reprojection_error,
            self.tag_inlier_count,
            (
                now - self.last_tag_pose_time,
                now - self.last_tag_count_time,
                now - self.last_tag_error_time,
                now - self.last_tag_inlier_time,
            ),
            self.tag_max_age,
            self.tag_max_reprojection_error,
            self.tag_min_inlier_points)

    def _tag_confirmed(self, now: float) -> bool:
        if not self._tag_quality_valid(now):
            self._reset_tag_confirmation()
            return False
        if self.tag_stable_since is None:
            self.tag_stable_since = now
            return self.tag_confirm_time <= 0.0
        confirmed = now - self.tag_stable_since >= self.tag_confirm_time
        if confirmed and not self.tag_confirmation_reported:
            self.tag_confirmation_reported = True
            self.get_logger().info(
                'Complete AprilTag handoff confirmed: '
                f'tags={self.tag_visible_count}, '
                f'inliers={self.tag_inlier_count}, '
                f'rmse={self.tag_reprojection_error:.2f}px')
        return confirmed

    def _inside_tag_handoff_region(self) -> bool:
        if self.target_pose is None:
            return False
        robot = self._robot_pose()
        if robot is None:
            return False
        target = self.target_pose.pose.position
        return position_reached(
            (robot[0], robot[1]), (target.x, target.y),
            self.tag_handoff_max_target_distance)

    def _start_tag_acquisition(self, after_search: bool):
        self.pending_acquisition_after_search = after_search
        self.camera_handoff_pending = True
        if self.camera_capture_enabled:
            self.camera_handoff_pending = False
            self._begin_tag_acquisition(after_search)
            return
        self.state = 'ACTIVATING_CAMERA'
        if self.camera_activation_in_progress:
            self.get_logger().info(
                'Nav2 is stopped at visual handoff; waiting for camera warm-up to finish')
            return

        self.get_logger().info(
            'Nav2 is stopped at visual handoff; opening camera for Tag acquisition')
        self._request_camera_activation()

    def _camera_enable_done(self, future):
        self.camera_activation_in_progress = False
        try:
            response = future.result()
        except Exception as exc:
            self._abort_mission(f'Camera activation failed: {exc}')
            return
        if self.state == 'FAILED':
            if response.success:
                self._set_camera_capture_enabled(False)
            return
        if not response.success:
            self._abort_mission(
                f'Camera refused activation: {response.message}')
            return
        self.camera_capture_enabled = True
        if self.camera_handoff_pending:
            self.camera_handoff_pending = False
            self._begin_tag_acquisition(self.pending_acquisition_after_search)
        else:
            self.get_logger().info(
                'Camera warm-up complete; navigation continues while '
                'AprilTag handoff evidence is monitored')

    def _begin_tag_acquisition(self, after_search: bool):
        self.state = 'TAG_ACQUISITION'
        self.acquisition_start_time = self._now_seconds()
        self.acquisition_after_search = after_search
        self._reset_tag_confirmation()
        self.get_logger().info(
            'Nav2 is stopped; waiting for a fresh complete Tag before '
            'transferring velocity ownership to visual parking')

    def _begin_limited_search(self):
        if self.final_candidate is None or self.target_pose is None:
            self._abort_mission('Cannot search for Tag without a pre-parking pose')
            return
        origin = self.final_candidate.pose.position
        target = self.target_pose.pose.position
        approach_yaw = math.atan2(target.y - origin.y, target.x - origin.x)
        self.search_points = bounded_search_points(
            (origin.x, origin.y), approach_yaw,
            self.search_forward_step, self.search_lateral_step)
        self.search_index = 0
        self.search_start_time = self._now_seconds()
        self.get_logger().warning(
            'No complete Tag at the pre-parking point; starting bounded '
            f'Nav2 search ({len(self.search_points)} waypoints, '
            f'forward<={self.search_forward_step:.2f}m, '
            f'lateral<={self.search_lateral_step:.2f}m, '
            f'timeout={self.search_timeout:.1f}s)')
        self._send_next_search_goal()

    def _send_next_search_goal(self):
        now = self._now_seconds()
        if self._search_timed_out(now):
            self._abort_mission(
                'Limited tag search timed out without one complete tag')
            return
        while self.search_index < len(self.search_points):
            index = self.search_index
            point = self.search_points[index]
            self.search_index += 1
            if not self._safe_search_point(point):
                self.get_logger().warning(
                    f'Skipping unsafe/unknown search waypoint {index + 1}: '
                    f'({point[0]:.2f}, {point[1]:.2f})')
                continue
            robot = self._robot_pose()
            yaw = robot[2] if robot is not None else 0.0
            pose = self._make_pose(point[0], point[1], yaw)
            self._reset_tag_confirmation()
            self.get_logger().info(
                f'Sending limited-search waypoint {index + 1}/'
                f'{len(self.search_points)}: ({point[0]:.2f}, {point[1]:.2f})')
            self._send_nav_goal(pose, 'search')
            return
        self._abort_mission(
            'Limited tag search exhausted all safe waypoints without a complete Tag')

    def _start_search_wait(self):
        self.state = 'SEARCH_WAIT'
        self.search_wait_start_time = self._now_seconds()
        self._reset_tag_confirmation()
        self.get_logger().info(
            f'Position-only search waypoint reached; observing for '
            f'{self.search_dwell_time:.1f}s without requiring a final yaw')

    def _search_timed_out(self, now: float) -> bool:
        return (
            self.search_start_time > 0.0 and
            self.search_timeout > 0.0 and
            now - self.search_start_time >= self.search_timeout)

    def _safe_search_point(self, point: Point) -> bool:
        if self.latest_map is None:
            return False
        cell = self._world_to_map(self.latest_map, point[0], point[1])
        return (
            cell is not None and
            self._is_free(self.latest_map, cell[0], cell[1]) and
            self._safe_endpoint(self.latest_map, cell)
        )

    def _activate_visual_parking(self):
        if self.state in ('ACTIVATING_PARKING', 'VISUAL_PARKING'):
            return
        if self.final_goal_handle is not None:
            self._abort_mission(
                'Refusing visual handoff while a Nav2 goal is still active')
            return
        if not self.parking_client.service_is_ready():
            self._abort_mission('Visual parking enable service is unavailable')
            return
        self.state = 'ACTIVATING_PARKING'
        self.parking_complete = False
        request = SetBool.Request()
        request.data = True
        future = self.parking_client.call_async(request)
        future.add_done_callback(self._parking_enable_done)

    def _parking_enable_done(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self._abort_mission(f'Visual parking activation failed: {exc}')
            return
        if self.state == 'FAILED':
            if response.success:
                self._set_parking_enabled(False)
            return
        if not response.success:
            self._abort_mission(
                f'Visual parking refused handoff: {response.message}')
            return
        self.state = 'VISUAL_PARKING'
        self.parking_start_time = self._now_seconds()
        self.get_logger().info(
            'Nav2 velocity ownership released; visual parking is active: '
            f'{response.message}')

    def _set_parking_enabled(self, enabled: bool):
        if not self.parking_client.service_is_ready():
            return None
        request = SetBool.Request()
        request.data = enabled
        return self.parking_client.call_async(request)

    def _set_camera_capture_enabled(self, enabled: bool):
        if not self.camera_client.service_is_ready():
            return None
        if not enabled:
            self.camera_capture_enabled = False
            self.camera_activation_in_progress = False
            self.camera_handoff_pending = False
        request = SetBool.Request()
        request.data = enabled
        return self.camera_client.call_async(request)

    def _send_return_goal(self):
        if self.start_pose is None:
            self._abort_mission('Cannot return: start pose is missing')
            return
        goal = NavigateToPose.Goal()
        self.start_pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose = self.start_pose
        self.get_logger().info('Full exploration complete; returning to start pose')
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._return_goal_response)

    def _return_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._abort_mission('Nav2 rejected the return-to-start goal')
            return
        goal_handle.get_result_async().add_done_callback(self._return_result)

    def _return_result(self, future):
        if future.result().status == GoalStatus.STATUS_SUCCEEDED:
            self.state = 'COMPLETE'
            self.get_logger().info('Returned to the recorded start pose successfully')
        else:
            self._abort_mission('Return-to-start navigation failed')

    def _target_local_reveal_applicable(self) -> bool:
        """
        Identify the narrow case where local sensing can unlock handoff.

        The target and the final tail of a reachable path are already known
        free when ``preparking_pose`` is derived.  Local reveal is allowed
        only when that dynamic endpoint fails because its clearance disk
        contains unknown cells; an occupied cell keeps normal travelled-route
        recovery in control.
        """
        if (not self.target_local_reveal_enabled or
                self.latest_map is None or
                self.target_pose is None or
                self.preparking_pose is None):
            return False

        msg = self.latest_map
        target = self.target_pose.pose.position
        approach = self.preparking_pose.pose.position
        target_cell = self._world_to_map(msg, target.x, target.y)
        approach_cell = self._world_to_map(msg, approach.x, approach.y)
        if target_cell is None or approach_cell is None:
            return False
        if (not self._is_free(msg, target_cell[0], target_cell[1]) or
                not self._is_free(msg, approach_cell[0], approach_cell[1])):
            return False
        if self._clearance_status(msg, target_cell) != 'safe':
            return False
        if self._clearance_status(msg, approach_cell) != 'unknown':
            return False
        if not self._known_free_line(msg, approach_cell, target_cell):
            return False
        return True

    def _target_is_known_free(self) -> bool:
        """Return whether Nav2 may meaningfully plan to the mission target."""
        if self.latest_map is None or self.target_pose is None:
            return False
        msg = self.latest_map
        target = self.target_pose.pose.position
        target_cell = self._world_to_map(msg, target.x, target.y)
        return (
            target_cell is not None and
            self._is_free(msg, target_cell[0], target_cell[1]))

    def _dynamic_preparking_from_path(
            self, path: Path) -> Tuple[Optional[PoseStamped], str]:
        """
        Derive pre-parking from the reachable path's final approach.

        The standoff is accumulated backward along the Nav2 path.  This means
        a target reached from the side or after a wall detour no longer uses
        the unrelated mission-start-to-target direction.
        """
        if self.latest_map is None or self.target_pose is None:
            return None, 'map or target is unavailable'
        points = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in path.poses
        ]
        derived = path_standoff_point(points, self.preparking_distance)
        if derived is None:
            return None, 'Nav2 returned an empty path'

        approach_x, approach_y, approach_yaw = derived
        candidate = self._make_pose(
            approach_x, approach_y, approach_yaw)
        approach_cell = self._world_to_map(
            self.latest_map, approach_x, approach_y)
        if (approach_cell is None or
                not self._is_free(
                    self.latest_map, approach_cell[0], approach_cell[1])):
            return None, 'derived pre-parking cell is unknown or occupied'
        if not self._path_tail_known_free(path, candidate):
            return None, 'final path tail to the target is not fully known-free'

        # Publish even an unknown-clearance candidate so target-local reveal
        # can use the current reachable approach as its anchor.  It must not
        # be dispatched to Nav2 until the clearance disk becomes fully safe.
        self.preparking_pose = candidate
        self.preparking_pub.publish(candidate)
        clearance = self._clearance_status(
            self.latest_map, approach_cell)
        if clearance != 'safe':
            return None, (
                f'derived pre-parking clearance is {clearance}')
        return candidate, 'safe'

    def _path_tail_known_free(
            self, path: Path, approach: PoseStamped) -> bool:
        """Check every map cell along the final standoff-length path suffix."""
        if self.latest_map is None or not path.poses:
            return False
        points = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in path.poses
        ]
        tail_reversed: List[Point] = [points[-1]]
        remaining = self.preparking_distance
        for index in range(len(points) - 1, 0, -1):
            previous = points[index - 1]
            current = points[index]
            segment = math.hypot(
                current[0] - previous[0], current[1] - previous[1])
            if segment <= 1e-9:
                continue
            if remaining <= segment + 1e-9:
                tail_reversed.append((
                    approach.pose.position.x,
                    approach.pose.position.y))
                break
            remaining -= segment
            tail_reversed.append(previous)
        else:
            first = (
                approach.pose.position.x,
                approach.pose.position.y)
            if math.hypot(
                    tail_reversed[-1][0] - first[0],
                    tail_reversed[-1][1] - first[1]) > 1e-9:
                tail_reversed.append(first)

        tail = list(reversed(tail_reversed))
        cells: List[Cell] = []
        for point in tail:
            cell = self._world_to_map(
                self.latest_map, point[0], point[1])
            if cell is None or not self._is_free(
                    self.latest_map, cell[0], cell[1]):
                return False
            if not cells or cell != cells[-1]:
                cells.append(cell)
        return all(
            self._known_free_line(self.latest_map, start, end)
            for start, end in zip(cells, cells[1:]))

    def _known_free_line(
            self, msg: OccupancyGrid, start: Cell, end: Cell) -> bool:
        """Require a fully known, obstacle-free line from an approach pose to target."""
        x0, y0 = start
        x1, y1 = end
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        step_x = 1 if x0 < x1 else -1
        step_y = 1 if y0 < y1 else -1
        error = dx + dy
        while True:
            if not self._is_free(msg, x0, y0):
                return False
            if x0 == x1 and y0 == y1:
                return True
            twice_error = 2 * error
            if twice_error >= dy:
                error += dy
                x0 += step_x
            if twice_error <= dx:
                error += dx
                y0 += step_y

    def _safe_endpoint(self, msg: OccupancyGrid, cell: Cell) -> bool:
        return self._clearance_status(msg, cell) == 'safe'

    def _clearance_status(self, msg: OccupancyGrid, cell: Cell) -> str:
        """Return ``safe``, ``unknown`` or ``occupied`` for a clearance disk."""
        radius = max(1, math.ceil(self.endpoint_clearance / msg.info.resolution))
        unknown = False
        for mx in range(cell[0] - radius, cell[0] + radius + 1):
            for my in range(cell[1] - radius, cell[1] + radius + 1):
                if (mx - cell[0]) ** 2 + (my - cell[1]) ** 2 > radius ** 2:
                    continue
                value = self._cell_value(msg, mx, my)
                if value is not None and value >= self.occupied_threshold:
                    return 'occupied'
                if value is None or value < 0:
                    unknown = True
        return 'unknown' if unknown else 'safe'

    def _known_path_ratio(self, path: Path, sample_stride: int = 2) -> float:
        if self.latest_map is None or not path.poses:
            return 0.0
        sample_stride = max(1, int(sample_stride))
        known = 0
        total = 0
        for index, pose in enumerate(path.poses):
            if (index % sample_stride != 0 and
                    index != len(path.poses) - 1):
                continue
            cell = self._world_to_map(
                self.latest_map, pose.pose.position.x, pose.pose.position.y)
            total += 1
            if cell is not None and self._is_free(self.latest_map, cell[0], cell[1]):
                known += 1
        return known / total if total else 0.0

    def _robot_pose(self) -> Optional[Tuple[float, float, float]]:
        return self._pose_in_frame(str(self.map_frame))

    def _pose_in_frame(
            self, target_frame: str) -> Optional[Tuple[float, float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, str(self.base_frame), Time(),
                timeout=Duration(seconds=0.2))
        except TransformException as exc:
            self._log_waiting(
                f'Waiting for {target_frame}->{self.base_frame} '
                f'transform: {exc}')
            return None
        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
            yaw)

    def _make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = str(self.map_frame)
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        return pose

    @staticmethod
    def _world_to_map(
            msg: OccupancyGrid, wx: float, wy: float) -> Optional[Cell]:
        origin = msg.info.origin
        q = origin.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        dx = wx - origin.position.x
        dy = wy - origin.position.y
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        mx = math.floor(local_x / msg.info.resolution)
        my = math.floor(local_y / msg.info.resolution)
        if 0 <= mx < msg.info.width and 0 <= my < msg.info.height:
            return int(mx), int(my)
        return None

    @staticmethod
    def _map_to_world(
            msg: OccupancyGrid, mx: int, my: int) -> Optional[Tuple[float, float]]:
        if not (0 <= mx < msg.info.width and 0 <= my < msg.info.height):
            return None
        origin = msg.info.origin
        q = origin.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        local_x = (mx + 0.5) * msg.info.resolution
        local_y = (my + 0.5) * msg.info.resolution
        return (
            origin.position.x + math.cos(yaw) * local_x - math.sin(yaw) * local_y,
            origin.position.y + math.sin(yaw) * local_x + math.cos(yaw) * local_y)

    def _is_free(self, msg: OccupancyGrid, mx: int, my: int) -> bool:
        value = self._cell_value(msg, mx, my)
        return value is not None and 0 <= value <= self.free_threshold

    @staticmethod
    def _cell_value(msg: OccupancyGrid, mx: int, my: int) -> Optional[int]:
        if not (0 <= mx < msg.info.width and 0 <= my < msg.info.height):
            return None
        return msg.data[my * msg.info.width + mx]

    def _abort_mission(self, reason: str):
        if self.state in ('COMPLETE', 'FAILED'):
            return
        previous_state = self.state
        self.state = 'FAILED'
        self._publish_startup_escape_stop()
        if self.explore_goal_handle is not None:
            self.explore_goal_handle.cancel_goal_async()
        if self.final_goal_handle is not None:
            self.final_goal_handle.cancel_goal_async()
        if previous_state in ('ACTIVATING_PARKING', 'VISUAL_PARKING'):
            self._set_parking_enabled(False)
        if self.visual_parking_enabled:
            self._set_camera_capture_enabled(False)
        self.get_logger().error(f'Goal-directed Roadmap mission failed: {reason}')

    def _log_waiting(self, message: str):
        now = self._now_seconds()
        if now - self.last_wait_log_time >= 5.0:
            self.get_logger().info(message)
            self.last_wait_log_time = now

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None):
    rclpy.init(args=args)
    node = RoadmapExploreMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # A clean Ctrl-C must zero both direct velocity inputs before this
        # coordinator disappears.
        node._publish_startup_escape_stop()
        future = node._set_parking_enabled(False)
        if future is not None:
            rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
        future = node._set_camera_capture_enabled(False)
        if future is not None:
            rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
