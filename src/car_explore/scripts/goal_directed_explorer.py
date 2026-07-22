#!/usr/bin/env python3
"""目标偏置 Frontier 导航。

终点以节点启动时的车体坐标系给出（向前为 +X，向左为 +Y）。节点在
slam_toolbox 的实时 /map 上提取可达 Frontier，并用 Nav2 验证和执行路径。
当终点进入已知自由区后切换为直接导航；失败目标会被临时抑制，在无可用
Frontier 时退回最近的历史安全路标。
"""

from collections import deque
from dataclasses import dataclass
import math
from typing import List, Optional, Sequence, Set, Tuple

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


Cell = Tuple[int, int]


@dataclass
class Candidate:
    kind: str
    x: float
    y: float
    yaw: float
    score: float
    frontier_size: float = 0.0
    path: Optional[Path] = None
    breadcrumb_index: int = -1


class GoalDirectedExplorer(Node):
    """Select goal-directed reachable frontiers and execute them through Nav2."""

    def __init__(self):
        super().__init__('goal_directed_explorer')
        self._declare_parameters()
        self._load_parameters()

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self._map_callback, map_qos)
        target_qos = QoSProfile(depth=1)
        target_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.target_pub = self.create_publisher(
            PoseStamped, '~/target_pose', target_qos)
        self.waypoint_pub = self.create_publisher(
            PoseStamped, '~/selected_waypoint', target_qos)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.plan_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.latest_map: Optional[OccupancyGrid] = None
        self.start_pose: Optional[Tuple[float, float, float]] = None
        self.target: Optional[Tuple[float, float]] = None
        self.breadcrumbs: List[Tuple[float, float]] = []
        self.recent_targets: deque = deque(maxlen=12)
        self.blacklist: List[Tuple[float, float, float]] = []

        self.state = 'STARTING'
        self.node_start_time = self._now_seconds()
        self.mission_start_time: Optional[float] = None
        self.next_plan_time = 0.0
        self.candidate_queue: List[Candidate] = []
        self.active_candidate: Optional[Candidate] = None
        self.nav_goal_handle = None
        self.nav_sent_time = 0.0
        self.last_progress_time = 0.0
        self.best_distance_remaining = math.inf
        self.cancel_requested = False
        self.retreat_attempted = False
        self.no_candidate_log_time = 0.0

        self.timer = self.create_timer(0.5, self._tick)
        self.get_logger().info(
            'Goal-directed explorer waiting for SLAM, TF and Nav2; '
            f'relative target=({self.goal_forward:.2f}m forward, '
            f'{self.goal_left:.2f}m left)')

    def _declare_parameters(self):
        defaults = {
            'goal_forward': 3.0,
            'goal_left': 0.0,
            'goal_radius': 0.45,
            'map_topic': '/map',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'planner_id': 'GridBased',
            'start_delay': 4.0,
            'planning_period': 1.0,
            'mission_timeout': 300.0,
            'waypoint_progress_timeout': 25.0,
            'min_progress': 0.10,
            'free_threshold': 20,
            'occupied_threshold': 65,
            'min_frontier_size': 0.25,
            'frontier_goal_offset': 0.15,
            'endpoint_clearance': 0.14,
            'candidates_per_cluster': 3,
            'candidate_separation': 0.60,
            'max_candidates': 10,
            'primary_cone_deg': 60.0,
            'fallback_cone_deg': 120.0,
            'target_progress_weight': 5.0,
            'information_gain_weight': 1.2,
            'travel_cost_weight': 1.0,
            'heading_cost_weight': 0.7,
            'revisit_cost_weight': 2.0,
            'direct_path_known_ratio': 0.85,
            'blacklist_duration': 30.0,
            'blacklist_radius': 0.35,
            'breadcrumb_spacing': 0.60,
            'max_breadcrumbs': 30,
            'retry_wait': 2.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _load_parameters(self):
        for name in (
                'goal_forward', 'goal_left', 'goal_radius', 'map_topic',
                'map_frame', 'base_frame', 'planner_id', 'start_delay',
                'planning_period', 'mission_timeout', 'waypoint_progress_timeout',
                'min_progress', 'free_threshold', 'occupied_threshold',
                'min_frontier_size', 'frontier_goal_offset', 'endpoint_clearance',
                'candidates_per_cluster', 'candidate_separation', 'max_candidates',
                'primary_cone_deg', 'fallback_cone_deg',
                'target_progress_weight', 'information_gain_weight',
                'travel_cost_weight', 'heading_cost_weight',
                'revisit_cost_weight', 'direct_path_known_ratio',
                'blacklist_duration', 'blacklist_radius', 'breadcrumb_spacing',
                'max_breadcrumbs', 'retry_wait'):
            setattr(self, name, self.get_parameter(name).value)

        self.goal_forward = float(self.goal_forward)
        self.goal_left = float(self.goal_left)
        self.goal_radius = float(self.goal_radius)
        self.start_delay = float(self.start_delay)
        self.planning_period = float(self.planning_period)
        self.mission_timeout = float(self.mission_timeout)
        self.waypoint_progress_timeout = float(self.waypoint_progress_timeout)
        self.min_progress = float(self.min_progress)
        self.free_threshold = int(self.free_threshold)
        self.occupied_threshold = int(self.occupied_threshold)
        self.min_frontier_size = float(self.min_frontier_size)
        self.frontier_goal_offset = float(self.frontier_goal_offset)
        self.endpoint_clearance = float(self.endpoint_clearance)
        self.candidates_per_cluster = int(self.candidates_per_cluster)
        self.candidate_separation = float(self.candidate_separation)
        self.max_candidates = int(self.max_candidates)
        self.primary_cone = math.radians(float(self.primary_cone_deg))
        self.fallback_cone = math.radians(float(self.fallback_cone_deg))
        self.target_progress_weight = float(self.target_progress_weight)
        self.information_gain_weight = float(self.information_gain_weight)
        self.travel_cost_weight = float(self.travel_cost_weight)
        self.heading_cost_weight = float(self.heading_cost_weight)
        self.revisit_cost_weight = float(self.revisit_cost_weight)
        self.direct_path_known_ratio = float(self.direct_path_known_ratio)
        self.blacklist_duration = float(self.blacklist_duration)
        self.blacklist_radius = float(self.blacklist_radius)
        self.breadcrumb_spacing = float(self.breadcrumb_spacing)
        self.max_breadcrumbs = int(self.max_breadcrumbs)
        self.retry_wait = float(self.retry_wait)

    def _map_callback(self, msg: OccupancyGrid):
        self.latest_map = msg

    def _tick(self):
        now = self._now_seconds()
        if self.state == 'COMPLETE':
            return
        if self.state == 'STARTING':
            self._try_start(now)
            return
        if self.mission_timeout > 0.0 and self.mission_start_time is not None:
            if now - self.mission_start_time >= self.mission_timeout:
                self._finish('Mission timeout reached before the approximate target')
                return

        robot = self._robot_pose()
        if robot is None:
            return
        if self.target is not None and self._distance(robot[:2], self.target) <= self.goal_radius:
            self._finish(
                f'Approximate target reached; remaining distance '
                f'{self._distance(robot[:2], self.target):.2f}m')
            return

        if self.state == 'NAVIGATING':
            if (not self.cancel_requested and
                    now - self.last_progress_time > self.waypoint_progress_timeout):
                self.cancel_requested = True
                self.get_logger().warning(
                    f'No navigation progress for {self.waypoint_progress_timeout:.1f}s; '
                    'canceling this waypoint')
                if self.nav_goal_handle is not None:
                    self.nav_goal_handle.cancel_goal_async()
            return
        if self.state in ('PLANNING', 'SENDING', 'CANCELING') or now < self.next_plan_time:
            return

        self._begin_planning(robot)

    def _try_start(self, now: float):
        if now - self.node_start_time < self.start_delay:
            return
        if self.latest_map is None:
            self._log_waiting('Waiting for /map from slam_toolbox')
            return
        if not self.plan_client.server_is_ready() or not self.nav_client.server_is_ready():
            self._log_waiting('Waiting for Nav2 planning/navigation action servers')
            return
        robot = self._robot_pose()
        if robot is None:
            return

        self.start_pose = robot
        sx, sy, syaw = robot
        gx = sx + math.cos(syaw) * self.goal_forward - math.sin(syaw) * self.goal_left
        gy = sy + math.sin(syaw) * self.goal_forward + math.cos(syaw) * self.goal_left
        self.target = (gx, gy)
        self.breadcrumbs = [(sx, sy)]
        self.mission_start_time = now
        self.state = 'IDLE'
        target_pose = self._make_pose(gx, gy, syaw)
        self.target_pub.publish(target_pose)
        self.get_logger().info(
            f'Recorded start pose ({sx:.2f}, {sy:.2f}, yaw={math.degrees(syaw):.1f}deg); '
            f'absolute approximate target=({gx:.2f}, {gy:.2f}), '
            f'radius={self.goal_radius:.2f}m')

    def _begin_planning(self, robot: Tuple[float, float, float]):
        if self.latest_map is None or self.target is None:
            return
        self._prune_blacklist()
        candidates = self._build_candidates(robot)
        if not candidates:
            retreat = self._retreat_candidate(robot)
            if retreat is not None:
                candidates = [retreat]
                self.get_logger().warning(
                    'No usable frontier toward the target; retreating to the previous breadcrumb')
            else:
                now = self._now_seconds()
                if now - self.no_candidate_log_time > 5.0:
                    self.get_logger().warning(
                        'No reachable, non-blacklisted frontier is available; '
                        'waiting for map update')
                    self.no_candidate_log_time = now
                self.next_plan_time = now + self.retry_wait
                return

        self.candidate_queue = candidates[:self.max_candidates]
        self.retreat_attempted = any(
            candidate.kind == 'breadcrumb' for candidate in self.candidate_queue)
        self.state = 'PLANNING'
        self._try_next_candidate()

    def _build_candidates(
            self, robot: Tuple[float, float, float]) -> List[Candidate]:
        msg = self.latest_map
        assert msg is not None
        assert self.target is not None
        start_cell = self._world_to_map(msg, robot[0], robot[1])
        if start_cell is None:
            self.get_logger().warning('Robot pose is outside the current occupancy grid')
            return []

        reachable = self._reachable_free_cells(msg, start_cell)
        if not reachable:
            self.get_logger().warning('Robot is not located on a known free map cell yet')
            return []

        candidates: List[Candidate] = []
        target_cell = self._world_to_map(msg, self.target[0], self.target[1])
        if (not self._is_blacklisted(self.target[0], self.target[1]) and
                target_cell is not None and target_cell in reachable and
                self._safe_endpoint(msg, target_cell)):
            yaw = math.atan2(self.target[1] - robot[1], self.target[0] - robot[0])
            candidates.append(Candidate(
                'target', self.target[0], self.target[1], yaw, math.inf))

        clusters = self._frontier_clusters(msg, reachable)
        target_bearing = math.atan2(
            self.target[1] - robot[1], self.target[0] - robot[0])
        current_goal_distance = self._distance(robot[:2], self.target)
        scored: List[Tuple[int, Candidate]] = []
        for cluster in clusters:
            frontier_size = len(cluster) * msg.info.resolution
            dispatches = self._frontier_dispatches(
                msg, cluster, reachable, robot[:2])
            for dispatch, reference in dispatches:
                wx, wy = self._map_to_world(msg, dispatch[0], dispatch[1])
                if self._is_blacklisted(reference[0], reference[1]):
                    continue
                if self._distance((wx, wy), robot[:2]) < max(
                        0.20, self.goal_radius * 0.5):
                    continue

                travel = self._distance(robot[:2], (wx, wy))
                progress = current_goal_distance - self._distance((wx, wy), self.target)
                bearing = math.atan2(wy - robot[1], wx - robot[0])
                heading_error = abs(self._angle_difference(bearing, target_bearing))
                revisit = sum(
                    max(0.0, 1.0 - self._distance((wx, wy), point) / 0.75)
                    for point in self.recent_targets)
                score = (
                    self.target_progress_weight * progress +
                    self.information_gain_weight * frontier_size -
                    self.travel_cost_weight * travel -
                    self.heading_cost_weight * heading_error -
                    self.revisit_cost_weight * revisit)
                if heading_error <= self.primary_cone:
                    stage = 0
                elif heading_error <= self.fallback_cone:
                    stage = 1
                else:
                    stage = 2
                yaw = math.atan2(self.target[1] - wy, self.target[0] - wx)
                scored.append((stage, Candidate(
                    'frontier', wx, wy, yaw, score, frontier_size)))

        scored.sort(key=lambda item: (item[0], -item[1].score))
        candidates.extend(candidate for _, candidate in scored)
        if scored:
            best_stage, best = scored[0]
            self.get_logger().info(
                f'Found {len(clusters)} frontier clusters, {len(scored)} usable; '
                f'best stage={best_stage}, score={best.score:.2f}, '
                f'point=({best.x:.2f}, {best.y:.2f})')
        return candidates

    def _try_next_candidate(self):
        if not self.candidate_queue:
            robot = self._robot_pose()
            if not self.retreat_attempted and robot is not None:
                retreat = self._retreat_candidate(robot)
                if retreat is not None:
                    self.retreat_attempted = True
                    self.candidate_queue = [retreat]
                    self.get_logger().warning(
                        'All frontier paths were rejected by Nav2; trying the previous breadcrumb')
                    self._try_next_candidate()
                    return
            self.state = 'IDLE'
            self.next_plan_time = self._now_seconds() + self.retry_wait
            self.get_logger().warning(
                'Nav2 found no valid path for the current candidate set; waiting before retry')
            return
        candidate = self.candidate_queue.pop(0)
        self.active_candidate = candidate
        goal = ComputePathToPose.Goal()
        goal.goal = self._make_pose(candidate.x, candidate.y, candidate.yaw)
        goal.planner_id = str(self.planner_id)
        goal.use_start = False
        future = self.plan_client.send_goal_async(goal)
        future.add_done_callback(self._plan_goal_response)

    def _plan_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:  # rclpy future exceptions must not kill the node
            self.get_logger().error(f'ComputePathToPose request failed: {exc}')
            self._try_next_candidate()
            return
        if self.state == 'COMPLETE':
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        if not goal_handle.accepted:
            self._suppress_unplannable_candidate()
            self._try_next_candidate()
            return
        goal_handle.get_result_async().add_done_callback(self._plan_result)

    def _plan_result(self, future):
        if self.state == 'COMPLETE':
            return
        candidate = self.active_candidate
        try:
            wrapped = future.result()
        except Exception as exc:
            self.get_logger().warning(f'ComputePathToPose result failed: {exc}')
            self._try_next_candidate()
            return
        if (candidate is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED or
                not wrapped.result.path.poses):
            self._suppress_unplannable_candidate()
            self._try_next_candidate()
            return
        candidate.path = wrapped.result.path
        if candidate.kind == 'target':
            known_ratio = self._known_path_ratio(candidate.path)
            if known_ratio < self.direct_path_known_ratio:
                self.get_logger().info(
                    f'Direct target path is only {known_ratio * 100.0:.0f}% known '
                    f'(< {self.direct_path_known_ratio * 100.0:.0f}%); continuing frontier mode')
                self._try_next_candidate()
                return
        self._send_navigation(candidate)

    def _send_navigation(self, candidate: Candidate):
        if self.state == 'COMPLETE':
            return
        self.state = 'SENDING'
        pose = self._make_pose(candidate.x, candidate.y, candidate.yaw)
        self.waypoint_pub.publish(pose)
        goal = NavigateToPose.Goal()
        goal.pose = pose
        future = self.nav_client.send_goal_async(goal, feedback_callback=self._nav_feedback)
        future.add_done_callback(self._nav_goal_response)

    def _nav_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'NavigateToPose request failed: {exc}')
            self._navigation_finished(False)
            return
        if self.state == 'COMPLETE':
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        if not goal_handle.accepted:
            self.get_logger().warning('Nav2 rejected selected waypoint')
            self._navigation_finished(False)
            return
        self.nav_goal_handle = goal_handle
        self.nav_sent_time = self._now_seconds()
        self.last_progress_time = self.nav_sent_time
        self.best_distance_remaining = math.inf
        self.cancel_requested = False
        self.state = 'NAVIGATING'
        candidate = self.active_candidate
        if candidate is not None:
            self.get_logger().info(
                f'Navigating to {candidate.kind} ({candidate.x:.2f}, {candidate.y:.2f}); '
                f'score={candidate.score:.2f}, frontier_size={candidate.frontier_size:.2f}m')
        goal_handle.get_result_async().add_done_callback(self._nav_result)

    def _nav_feedback(self, message):
        remaining = float(message.feedback.distance_remaining)
        if remaining + self.min_progress < self.best_distance_remaining:
            self.best_distance_remaining = remaining
            self.last_progress_time = self._now_seconds()

    def _nav_result(self, future):
        try:
            status = future.result().status
        except Exception as exc:
            self.get_logger().error(f'NavigateToPose result failed: {exc}')
            self._navigation_finished(False)
            return
        self._navigation_finished(status == GoalStatus.STATUS_SUCCEEDED)

    def _navigation_finished(self, succeeded: bool):
        candidate = self.active_candidate
        if self.state == 'COMPLETE':
            self.nav_goal_handle = None
            self.active_candidate = None
            return
        robot = self._robot_pose()
        self.nav_goal_handle = None
        self.cancel_requested = False
        self.state = 'IDLE'
        self.next_plan_time = self._now_seconds() + self.planning_period
        if candidate is None:
            return
        if succeeded:
            self.get_logger().info(
                f'Reached {candidate.kind} waypoint ({candidate.x:.2f}, {candidate.y:.2f})')
            self.recent_targets.append((candidate.x, candidate.y))
            if candidate.kind == 'breadcrumb' and candidate.breadcrumb_index >= 0:
                self.breadcrumbs = self.breadcrumbs[:candidate.breadcrumb_index + 1]
            elif robot is not None:
                self._add_breadcrumb(robot[0], robot[1])
            if candidate.kind == 'target':
                self._finish('Nav2 reached the approximate target pose')
        else:
            if candidate.kind != 'breadcrumb':
                self.blacklist.append((
                    candidate.x, candidate.y,
                    self._now_seconds() + self.blacklist_duration))
            self.get_logger().warning(
                f'Failed to reach {candidate.kind} waypoint; suppressing nearby frontier '
                f'for {self.blacklist_duration:.0f}s')
        self.active_candidate = None

    def _finish(self, reason: str):
        if self.state == 'COMPLETE':
            return
        if self.nav_goal_handle is not None:
            self.nav_goal_handle.cancel_goal_async()
        self.state = 'COMPLETE'
        self.get_logger().info(f'Goal-directed mission complete: {reason}')

    def _frontier_clusters(
            self, msg: OccupancyGrid, reachable: Set[Cell]) -> List[List[Cell]]:
        width = msg.info.width
        height = msg.info.height
        data = msg.data
        frontier: Set[Cell] = set()
        for mx, my in reachable:
            for nx, ny in ((mx - 1, my), (mx + 1, my), (mx, my - 1), (mx, my + 1)):
                if 0 <= nx < width and 0 <= ny < height and data[ny * width + nx] < 0:
                    frontier.add((mx, my))
                    break

        min_cells = max(1, math.ceil(self.min_frontier_size / msg.info.resolution))
        clusters: List[List[Cell]] = []
        unvisited = set(frontier)
        while unvisited:
            seed = unvisited.pop()
            cluster = [seed]
            queue = deque([seed])
            while queue:
                cx, cy = queue.popleft()
                for nx in range(cx - 1, cx + 2):
                    for ny in range(cy - 1, cy + 2):
                        neighbor = (nx, ny)
                        if neighbor in unvisited:
                            unvisited.remove(neighbor)
                            cluster.append(neighbor)
                            queue.append(neighbor)
            if len(cluster) >= min_cells:
                clusters.append(cluster)
        return clusters

    def _reachable_free_cells(self, msg: OccupancyGrid, start: Cell) -> Set[Cell]:
        if not self._is_free(msg, start[0], start[1]):
            replacement = self._nearest_free_cell(msg, start, set(), 4)
            if replacement is None:
                return set()
            start = replacement
        reachable: Set[Cell] = {start}
        queue = deque([start])
        while queue:
            mx, my = queue.popleft()
            for nx, ny in ((mx - 1, my), (mx + 1, my), (mx, my - 1), (mx, my + 1)):
                cell = (nx, ny)
                if cell not in reachable and self._is_free(msg, nx, ny):
                    reachable.add(cell)
                    queue.append(cell)
        return reachable

    def _frontier_dispatches(
            self, msg: OccupancyGrid, cluster: Sequence[Cell],
            reachable: Set[Cell], robot: Tuple[float, float]
            ) -> List[Tuple[Cell, Tuple[float, float]]]:
        assert self.target is not None
        world_cells = [(cell, self._map_to_world(msg, cell[0], cell[1]))
                       for cell in cluster]
        if not world_cells:
            return []

        # 第一个锚点选最接近最终目标的 Frontier；其余使用最远点采样，避免一个
        # 大环形 Frontier 只生成中心附近的单一目标，保留绕开死路的替代入口。
        first = min(
            world_cells,
            key=lambda item: self._distance(item[1], self.target))
        anchors = [first]
        separation_cells = self.candidate_separation / msg.info.resolution
        while len(anchors) < self.candidates_per_cluster:
            remaining = [
                item for item in world_cells
                if min(self._distance(item[0], selected[0]) for selected in anchors)
                >= separation_cells]
            if not remaining:
                break
            anchors.append(max(
                remaining,
                key=lambda item: min(
                    self._distance(item[0], selected[0]) for selected in anchors)))

        radius_cells = max(2, math.ceil(
            (self.frontier_goal_offset + 0.15) / msg.info.resolution))
        results: List[Tuple[Cell, Tuple[float, float]]] = []
        used_dispatches: Set[Cell] = set()
        for _, reference in anchors:
            dx = robot[0] - reference[0]
            dy = robot[1] - reference[1]
            norm = math.hypot(dx, dy)
            if norm > 1e-6:
                desired_x = reference[0] + self.frontier_goal_offset * dx / norm
                desired_y = reference[1] + self.frontier_goal_offset * dy / norm
            else:
                desired_x, desired_y = reference
            desired = self._world_to_map(msg, desired_x, desired_y)
            if desired is None:
                continue
            dispatch = self._nearest_free_cell(
                msg, desired, reachable, radius_cells, safe=True)
            if dispatch is not None and dispatch not in used_dispatches:
                used_dispatches.add(dispatch)
                results.append((dispatch, reference))
        return results

    def _nearest_free_cell(
            self, msg: OccupancyGrid, center: Cell, allowed: Set[Cell],
            radius: int, safe: bool = False) -> Optional[Cell]:
        choices = []
        for mx in range(center[0] - radius, center[0] + radius + 1):
            for my in range(center[1] - radius, center[1] + radius + 1):
                cell = (mx, my)
                if allowed and cell not in allowed:
                    continue
                if not self._is_free(msg, mx, my):
                    continue
                if safe and not self._safe_endpoint(msg, cell):
                    continue
                choices.append(((mx - center[0]) ** 2 + (my - center[1]) ** 2, cell))
        if not choices:
            return None
        return min(choices, key=lambda item: item[0])[1]

    def _safe_endpoint(self, msg: OccupancyGrid, cell: Cell) -> bool:
        radius = max(1, math.ceil(self.endpoint_clearance / msg.info.resolution))
        for mx in range(cell[0] - radius, cell[0] + radius + 1):
            for my in range(cell[1] - radius, cell[1] + radius + 1):
                if (mx - cell[0]) ** 2 + (my - cell[1]) ** 2 > radius ** 2:
                    continue
                value = self._cell_value(msg, mx, my)
                if value is not None and value >= self.occupied_threshold:
                    return False
        return True

    def _known_path_ratio(self, path: Path) -> float:
        if self.latest_map is None or not path.poses:
            return 0.0
        known = 0
        total = 0
        for index, pose in enumerate(path.poses):
            if index % 2 != 0 and index != len(path.poses) - 1:
                continue
            cell = self._world_to_map(
                self.latest_map, pose.pose.position.x, pose.pose.position.y)
            total += 1
            if cell is not None:
                value = self._cell_value(self.latest_map, cell[0], cell[1])
                if value is not None and 0 <= value <= self.free_threshold:
                    known += 1
        return known / total if total else 0.0

    def _retreat_candidate(
            self, robot: Tuple[float, float, float]) -> Optional[Candidate]:
        for index in range(len(self.breadcrumbs) - 2, -1, -1):
            point = self.breadcrumbs[index]
            if self._distance(robot[:2], point) >= self.breadcrumb_spacing * 0.75:
                yaw = math.atan2(point[1] - robot[1], point[0] - robot[0])
                return Candidate(
                    'breadcrumb', point[0], point[1], yaw, -100.0,
                    breadcrumb_index=index)
        return None

    def _add_breadcrumb(self, x: float, y: float):
        if (not self.breadcrumbs or
                self._distance(self.breadcrumbs[-1], (x, y)) >= self.breadcrumb_spacing):
            self.breadcrumbs.append((x, y))
            if len(self.breadcrumbs) > self.max_breadcrumbs:
                self.breadcrumbs.pop(0)

    def _is_blacklisted(self, x: float, y: float) -> bool:
        return any(
            self._distance((x, y), (bx, by)) <= self.blacklist_radius
            for bx, by, _ in self.blacklist)

    def _suppress_unplannable_candidate(self):
        candidate = self.active_candidate
        if candidate is not None and candidate.kind in ('frontier', 'target'):
            self.blacklist.append((
                candidate.x, candidate.y,
                self._now_seconds() + self.blacklist_duration))

    def _prune_blacklist(self):
        now = self._now_seconds()
        self.blacklist = [entry for entry in self.blacklist if entry[2] > now]

    def _robot_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(
                str(self.map_frame), str(self.base_frame), Time(),
                timeout=Duration(seconds=0.2))
        except TransformException as exc:
            self._log_waiting(
                f'Waiting for {self.map_frame}->{self.base_frame} transform: {exc}')
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

    def _world_to_map(
            self, msg: OccupancyGrid, wx: float, wy: float) -> Optional[Cell]:
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

    def _map_to_world(self, msg: OccupancyGrid, mx: int, my: int) -> Tuple[float, float]:
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

    def _log_waiting(self, message: str):
        now = self._now_seconds()
        if now - self.no_candidate_log_time > 5.0:
            self.get_logger().info(message)
            self.no_candidate_log_time = now

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def _distance(a: Sequence[float], b: Sequence[float]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    @staticmethod
    def _angle_difference(a: float, b: float) -> float:
        return math.atan2(math.sin(a - b), math.cos(a - b))


def main(args=None):
    rclpy.init(args=args)
    node = GoalDirectedExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
