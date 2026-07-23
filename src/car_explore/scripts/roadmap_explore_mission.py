#!/usr/bin/env python3
"""Run Roadmap Explorer toward an approximate relative target, then hand off to Nav2."""

import math
from typing import Optional, Tuple

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
from roadmap_explorer_msgs.action import Explore
from tf2_ros import Buffer, TransformException, TransformListener


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
        target_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.target_pub = self.create_publisher(
            PoseStamped, '~/target_pose', target_qos)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.explore_client = ActionClient(self, Explore, 'roadmap_explorer')
        self.plan_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.latest_map: Optional[OccupancyGrid] = None
        self.start_pose: Optional[PoseStamped] = None
        self.target_pose: Optional[PoseStamped] = None
        self.explore_goal_handle = None
        self.final_goal_handle = None
        self.final_candidate: Optional[PoseStamped] = None
        self.plan_in_progress = False
        self.state = 'STARTING'
        self.node_start_time = self._now_seconds()
        self.mission_start_time = 0.0
        self.next_direct_check_time = 0.0
        self.last_wait_log_time = 0.0
        self.timer = self.create_timer(0.5, self._tick)

    def _declare_parameters(self):
        defaults = {
            'map_frame': 'map',
            'base_frame': 'base_link',
            'map_topic': '/map',
            'planner_id': 'GridBased',
            'start_delay': 5.0,
            'return_to_start': False,
            'goal_directed_mode': True,
            'goal_forward': 3.0,
            'goal_left': 0.0,
            'goal_radius': 0.25,
            'direct_path_known_ratio': 0.85,
            'direct_check_period': 1.0,
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
                'start_delay', 'return_to_start', 'goal_directed_mode',
                'goal_forward', 'goal_left', 'goal_radius',
                'direct_path_known_ratio', 'direct_check_period',
                'mission_timeout', 'free_threshold', 'occupied_threshold',
                'endpoint_clearance'):
            setattr(self, name, self.get_parameter(name).value)

        self.start_delay = float(self.start_delay)
        self.return_to_start = bool(self.return_to_start)
        self.goal_directed_mode = bool(self.goal_directed_mode)
        self.goal_forward = float(self.goal_forward)
        self.goal_left = float(self.goal_left)
        self.goal_radius = float(self.goal_radius)
        self.direct_path_known_ratio = float(self.direct_path_known_ratio)
        self.direct_check_period = float(self.direct_check_period)
        self.mission_timeout = float(self.mission_timeout)
        self.free_threshold = int(self.free_threshold)
        self.occupied_threshold = int(self.occupied_threshold)
        self.endpoint_clearance = float(self.endpoint_clearance)

    def _map_callback(self, msg: OccupancyGrid):
        self.latest_map = msg

    def _tick(self):
        now = self._now_seconds()
        if self.state == 'STARTING':
            self._try_start(now)
            return
        if self.state in ('COMPLETE', 'FAILED'):
            return
        if (self.mission_timeout > 0.0 and self.mission_start_time > 0.0 and
                now - self.mission_start_time >= self.mission_timeout):
            self._abort_mission('Mission timeout reached before the target became reachable')
            return
        if (self.state != 'EXPLORING' or not self.goal_directed_mode or
                self.plan_in_progress or now < self.next_direct_check_time):
            return

        self.next_direct_check_time = now + self.direct_check_period
        candidate = self._known_safe_target_pose()
        if candidate is not None:
            self._request_direct_path(candidate)

    def _try_start(self, now: float):
        if now - self.node_start_time < self.start_delay:
            return
        if self.latest_map is None:
            self._log_waiting('Waiting for /map from slam_toolbox')
            return
        if (not self.explore_client.server_is_ready() or
                not self.nav_client.server_is_ready() or
                (self.goal_directed_mode and not self.plan_client.server_is_ready())):
            self._log_waiting('Waiting for Roadmap Explorer and Nav2 action servers')
            return

        robot = self._robot_pose()
        if robot is None:
            return
        x, y, yaw = robot
        self.start_pose = self._make_pose(x, y, yaw)
        goal_x = x + math.cos(yaw) * self.goal_forward - math.sin(yaw) * self.goal_left
        goal_y = y + math.sin(yaw) * self.goal_forward + math.cos(yaw) * self.goal_left
        self.target_pose = self._make_pose(goal_x, goal_y, yaw)
        self.target_pub.publish(self.target_pose)
        self.mission_start_time = now
        self.state = 'SENDING_EXPLORATION'
        self.get_logger().info(
            f'Recorded start pose ({x:.2f}, {y:.2f}, yaw={math.degrees(yaw):.1f}deg); '
            f'Roadmap target=({goal_x:.2f}, {goal_y:.2f}), '
            f'relative=({self.goal_forward:.2f}m forward, {self.goal_left:.2f}m left)')

        goal = Explore.Goal()
        goal.exploration_bringup_mode = Explore.Goal.NEW_EXPLORATION_SESSION
        goal.load_from_folder.data = ''
        goal.session_name.data = 'smartcar_goal_directed'
        future = self.explore_client.send_goal_async(goal)
        future.add_done_callback(self._explore_goal_response)

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
        self.get_logger().info('Roadmap Explorer accepted the exploration goal')
        goal_handle.get_result_async().add_done_callback(self._explore_result)

    def _explore_result(self, future):
        try:
            wrapped = future.result()
        except Exception as exc:
            self._abort_mission(f'Roadmap Explorer result failed: {exc}')
            return

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

        if self.state in ('FINAL_NAVIGATION', 'COMPLETE'):
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

    def _request_direct_path(self, candidate: PoseStamped):
        self.plan_in_progress = True
        goal = ComputePathToPose.Goal()
        goal.goal = candidate
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

        self.final_candidate = wrapped.result.path.poses[-1]
        self.get_logger().info(
            f'Target has a safe path with {known_ratio * 100.0:.0f}% known cells; '
            'canceling Roadmap exploration and switching to final Nav2 navigation')
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
        self.get_logger().info(
            'Roadmap exploration accepted cancellation; waiting for its '
            'Nav2 goal to terminate before final navigation')

    def _send_final_goal(self):
        if self.final_candidate is None:
            self._abort_mission('Final goal pose is missing')
            return
        goal = NavigateToPose.Goal()
        self.final_candidate.header.stamp = self.get_clock().now().to_msg()
        goal.pose = self.final_candidate
        self.state = 'FINAL_NAVIGATION'
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._final_goal_response)

    def _final_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._abort_mission(f'Final NavigateToPose request failed: {exc}')
            return
        if not goal_handle.accepted:
            self._abort_mission('Nav2 rejected the final target goal')
            return
        if self.state in ('FAILED', 'COMPLETE'):
            goal_handle.cancel_goal_async()
            return
        self.final_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self._final_result)

    def _final_result(self, future):
        try:
            status = future.result().status
        except Exception as exc:
            self._abort_mission(f'Final NavigateToPose result failed: {exc}')
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.state = 'COMPLETE'
            self.get_logger().info(
                'Goal-directed Roadmap mission complete: Nav2 reached the target')
        else:
            self._abort_mission(f'Final target navigation failed with status {status}')

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

    def _known_safe_target_pose(self) -> Optional[PoseStamped]:
        if self.latest_map is None or self.target_pose is None:
            return None
        msg = self.latest_map
        target = self.target_pose.pose.position
        target_cell = self._world_to_map(msg, target.x, target.y)
        if target_cell is None or not self._is_free(
                msg, target_cell[0], target_cell[1]):
            return None
        robot = self._robot_pose()
        if robot is None:
            return None

        candidates = [(0.0, target_cell, target.x, target.y)]
        radius_cells = max(0, math.ceil(self.goal_radius / msg.info.resolution))
        for mx in range(target_cell[0] - radius_cells,
                        target_cell[0] + radius_cells + 1):
            for my in range(target_cell[1] - radius_cells,
                            target_cell[1] + radius_cells + 1):
                world = self._map_to_world(msg, mx, my)
                if world is None:
                    continue
                distance = math.hypot(world[0] - target.x, world[1] - target.y)
                if 1e-6 < distance <= self.goal_radius:
                    candidates.append((distance, (mx, my), world[0], world[1]))

        candidates.sort(key=lambda item: item[0])
        for _, cell, world_x, world_y in candidates:
            if (not self._is_free(msg, cell[0], cell[1]) or
                    not self._safe_endpoint(msg, cell) or
                    not self._known_free_line(msg, cell, target_cell)):
                continue
            yaw = math.atan2(target.y - world_y, target.x - world_x)
            if cell == target_cell:
                yaw = math.atan2(target.y - robot[1], target.x - robot[0])
            return self._make_pose(world_x, world_y, yaw)
        return None

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
        radius = max(1, math.ceil(self.endpoint_clearance / msg.info.resolution))
        for mx in range(cell[0] - radius, cell[0] + radius + 1):
            for my in range(cell[1] - radius, cell[1] + radius + 1):
                if (mx - cell[0]) ** 2 + (my - cell[1]) ** 2 > radius ** 2:
                    continue
                value = self._cell_value(msg, mx, my)
                if value is None or value < 0 or value >= self.occupied_threshold:
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
            if cell is not None and self._is_free(self.latest_map, cell[0], cell[1]):
                known += 1
        return known / total if total else 0.0

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
        if self.explore_goal_handle is not None and self.state == 'EXPLORING':
            self.explore_goal_handle.cancel_goal_async()
        if self.final_goal_handle is not None:
            self.final_goal_handle.cancel_goal_async()
        self.state = 'FAILED'
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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
