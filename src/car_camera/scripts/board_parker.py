#!/usr/bin/env python3
"""Park a downward-looking-camera robot on the centre of a four-way board.

The detector publishes the board pose in ``camera_optical_frame``.  This node
transforms it into ``base_link`` before controlling the robot, so camera height
or a small mounting tilt never masquerades as forward parking distance.
"""

import math
import time

from geometry_msgs.msg import PoseStamped, Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, Float32, UInt32
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener


def clamp(value, limit):
    return max(-limit, min(limit, value))


def wrap_period(angle, period):
    """Return the signed distance to the nearest multiple of period."""
    return (angle + period / 2.0) % period - period / 2.0


def quaternion_values(q):
    if isinstance(q, (tuple, list)):
        return tuple(float(value) for value in q)
    return float(q.x), float(q.y), float(q.z), float(q.w)


def normalize_quaternion(q):
    x, y, z, w = quaternion_values(q)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        raise ValueError('invalid zero-length quaternion')
    return x / norm, y / norm, z / norm, w / norm


def quaternion_multiply(left, right):
    lx, ly, lz, lw = normalize_quaternion(left)
    rx, ry, rz, rw = normalize_quaternion(right)
    return normalize_quaternion((
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ))


def quaternion_to_matrix(q):
    """Return a row-major 3x3 rotation matrix for a ROS quaternion."""
    x, y, z, w = normalize_quaternion(q)
    return (
        1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
        2 * (x * z + y * w), 2 * (x * y + z * w),
        1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
        2 * (x * z - y * w), 2 * (y * z + x * w),
        1 - 2 * (x * x + y * y),
    )


def rotate_vector(q, vector):
    rotation = quaternion_to_matrix(q)
    x, y, z = vector
    return (
        rotation[0] * x + rotation[1] * y + rotation[2] * z,
        rotation[3] * x + rotation[4] * y + rotation[5] * z,
        rotation[6] * x + rotation[7] * y + rotation[8] * z,
    )


def transform_pose(pose, transform):
    """Apply a geometry_msgs Transform to a geometry_msgs Pose."""
    rotated = rotate_vector(
        transform.rotation,
        (pose.position.x, pose.position.y, pose.position.z))
    position = (
        transform.translation.x + rotated[0],
        transform.translation.y + rotated[1],
        transform.translation.z + rotated[2],
    )
    orientation = quaternion_multiply(transform.rotation, pose.orientation)
    return position, orientation


def nearest_edge_error(q):
    """Board in-plane heading error, with all four edge headings equivalent."""
    rotation = quaternion_to_matrix(q)
    board_x_angle = math.atan2(rotation[3], rotation[0])
    return wrap_period(board_x_angle, math.pi / 2.0)


def footprint_inside_board(
        board_position, board_orientation, footprint,
        board_width, board_height, margin):
    """Return true when all base-frame footprint corners lie inside the board."""
    rotation = quaternion_to_matrix(board_orientation)
    half_width = board_width / 2.0 - margin
    half_height = board_height / 2.0 - margin
    if half_width <= 0.0 or half_height <= 0.0:
        return False

    for footprint_x, footprint_y in footprint:
        dx = footprint_x - board_position[0]
        dy = footprint_y - board_position[1]
        dz = -board_position[2]
        # board_point = R(base<-board)^T * (base_point - board_origin)
        board_x = rotation[0] * dx + rotation[3] * dy + rotation[6] * dz
        board_y = rotation[1] * dx + rotation[4] * dy + rotation[7] * dz
        if abs(board_x) > half_width or abs(board_y) > half_height:
            return False
    return True


def polygon_area(points):
    if len(points) < 3:
        return 0.0
    return abs(sum(
        points[i][0] * points[(i + 1) % len(points)][1] -
        points[(i + 1) % len(points)][0] * points[i][1]
        for i in range(len(points)))) * 0.5


def clip_polygon(points, axis, boundary, keep_less):
    """Clip a polygon against one axis-aligned half plane."""
    if not points:
        return []

    def inside(point):
        value = point[axis]
        return value <= boundary if keep_less else value >= boundary

    result = []
    previous = points[-1]
    previous_inside = inside(previous)
    for current in points:
        current_inside = inside(current)
        if current_inside != previous_inside:
            delta = current[axis] - previous[axis]
            if abs(delta) > 1e-12:
                ratio = (boundary - previous[axis]) / delta
                intersection = (
                    previous[0] + ratio * (current[0] - previous[0]),
                    previous[1] + ratio * (current[1] - previous[1]))
                result.append(intersection)
        if current_inside:
            result.append(current)
        previous = current
        previous_inside = current_inside
    return result


def footprint_overlap_ratio(
        board_position, board_orientation, footprint,
        board_width, board_height, margin):
    """Return the fraction of the chassis footprint lying on the board."""
    rotation = quaternion_to_matrix(board_orientation)
    board_polygon = []
    for footprint_x, footprint_y in footprint:
        dx = footprint_x - board_position[0]
        dy = footprint_y - board_position[1]
        dz = -board_position[2]
        board_polygon.append((
            rotation[0] * dx + rotation[3] * dy + rotation[6] * dz,
            rotation[1] * dx + rotation[4] * dy + rotation[7] * dz))

    original_area = polygon_area(board_polygon)
    if original_area < 1e-9:
        return 0.0
    half_width = board_width / 2.0 - margin
    half_height = board_height / 2.0 - margin
    if half_width <= 0.0 or half_height <= 0.0:
        return 0.0

    clipped = clip_polygon(board_polygon, 0, -half_width, False)
    clipped = clip_polygon(clipped, 0, half_width, True)
    clipped = clip_polygon(clipped, 1, -half_height, False)
    clipped = clip_polygon(clipped, 1, half_height, True)
    return min(1.0, polygon_area(clipped) / original_area)


def compute_parking_command(
        error_forward, error_left, error_edge,
        max_linear, max_angular, min_linear, slowdown_distance,
        kp_distance, kp_bearing, kp_edge, allow_reverse):
    """Smooth unicycle command for an arbitrary oblique board approach."""
    distance = math.hypot(error_forward, error_left)
    if distance < 1e-9:
        return 0.0, clamp(kp_edge * error_edge, max_angular)

    bearing = math.atan2(error_left, error_forward)
    direction = 1.0
    if allow_reverse and abs(bearing) > math.pi / 2.0:
        direction = -1.0
        bearing = wrap_period(bearing - math.copysign(math.pi, bearing), 2.0 * math.pi)

    angular = clamp(
        kp_bearing * bearing + kp_edge * error_edge, max_angular)
    # Turn on the spot when the destination lies mostly beside the robot.
    if abs(bearing) > math.radians(55.0):
        return 0.0, angular

    linear = min(max_linear, kp_distance * distance)
    if slowdown_distance > 1e-6 and distance < slowdown_distance:
        linear *= max(0.45, distance / slowdown_distance)
    linear *= max(0.20, math.cos(bearing))
    if 0.0 < linear < min_linear:
        linear = min_linear
    return direction * linear, angular


class BoardParker(Node):
    def __init__(self):
        super().__init__('board_parker')

        self.pose_topic = self.declare_parameter(
            'pose_topic', '/apriltag_detector/tag_pose').value
        self.visible_count_topic = self.declare_parameter(
            'visible_count_topic',
            '/apriltag_detector/visible_tag_count').value
        self.reprojection_error_topic = self.declare_parameter(
            'reprojection_error_topic',
            '/apriltag_detector/reprojection_error').value
        self.inlier_count_topic = self.declare_parameter(
            'inlier_count_topic',
            '/apriltag_detector/inlier_count').value
        self.cmd_topic = self.declare_parameter(
            'cmd_topic', '/cmd_vel_dock').value
        self.base_frame = self.declare_parameter(
            'base_frame', 'base_link').value

        # The measured footprint is asymmetric around base_link.  Its geometric
        # centre is 8.2 cm in front of the drive axle midpoint.
        self.target_forward = float(
            self.declare_parameter('target_forward', 0.082).value)
        self.target_left = float(
            self.declare_parameter('target_left', 0.0).value)
        self.board_width = float(
            self.declare_parameter('board_width', 0.2881).value)
        self.board_height = float(
            self.declare_parameter('board_height', 0.2910).value)
        footprint_x = [
            float(value) for value in
            self.declare_parameter(
                'footprint_x', [0.197, 0.197, -0.033, -0.033]).value]
        footprint_y = [
            float(value) for value in
            self.declare_parameter(
                'footprint_y', [0.093, -0.093, -0.093, 0.093]).value]
        if len(footprint_x) != len(footprint_y) or len(footprint_x) < 3:
            raise ValueError('footprint_x and footprint_y must describe >=3 corners')
        self.footprint = list(zip(footprint_x, footprint_y))
        self.board_margin = float(
            self.declare_parameter('board_margin', 0.005).value)

        self.kp_distance = float(
            self.declare_parameter('kp_distance', 0.8).value)
        self.kp_bearing = float(
            self.declare_parameter('kp_bearing', 1.8).value)
        self.kp_edge = float(self.declare_parameter('kp_edge', 0.8).value)
        self.max_linear = float(self.declare_parameter('max_linear', 0.05).value)
        self.max_angular = float(self.declare_parameter('max_angular', 0.30).value)
        self.single_tag_max_linear = float(
            self.declare_parameter('single_tag_max_linear', 0.025).value)
        self.single_tag_max_angular = float(
            self.declare_parameter('single_tag_max_angular', 0.18).value)
        self.min_linear = float(self.declare_parameter('min_linear', 0.018).value)
        self.position_tolerance = float(
            self.declare_parameter('position_tolerance', 0.025).value)
        self.edge_tolerance = math.radians(float(
            self.declare_parameter('edge_tolerance_deg', 15.0).value))
        self.min_footprint_overlap = float(
            self.declare_parameter('min_footprint_overlap', 0.90).value)
        self.slowdown_distance = float(
            self.declare_parameter('slowdown_distance', 0.10).value)
        self.loss_timeout = float(self.declare_parameter('loss_timeout', 0.35).value)
        self.max_pose_jump = float(
            self.declare_parameter('max_pose_jump', 0.05).value)
        self.max_base_pose_jump = float(
            self.declare_parameter('max_base_pose_jump', 0.03).value)
        self.max_edge_jump = math.radians(float(
            self.declare_parameter('max_edge_jump_deg', 15.0).value))
        self.max_reprojection_error = float(
            self.declare_parameter('max_reprojection_error', 3.0).value)
        self.min_inlier_points = int(
            self.declare_parameter('min_inlier_points', 4).value)
        self.tracking_stable_time = float(
            self.declare_parameter('tracking_stable_time', 0.25).value)
        self.filter_alpha = float(
            self.declare_parameter('filter_alpha', 0.35).value)
        self.max_linear_accel = float(
            self.declare_parameter('max_linear_accel', 0.08).value)
        self.max_angular_accel = float(
            self.declare_parameter('max_angular_accel', 0.50).value)
        self.min_visible_tags = int(
            self.declare_parameter('min_visible_tags', 1).value)
        self.stable_frames_required = int(
            self.declare_parameter('stable_frames', 6).value)
        self.single_tag_stable_frames = int(
            self.declare_parameter('single_tag_stable_frames', 12).value)
        self.allow_reverse = bool(self.declare_parameter('allow_reverse', True).value)
        self.enabled = bool(self.declare_parameter('enabled', False).value)

        self.latest_pose = None
        self.last_pose_time = None
        self.visible_tag_count = 0
        self.last_count_time = None
        self.reprojection_error = math.inf
        self.inlier_count = 0
        self.last_error_time = None
        self.last_inlier_time = None
        self.tracking_since = None
        self.filtered_position = None
        self.filtered_edge = None
        self.last_command_time = time.monotonic()
        self.last_linear_command = 0.0
        self.last_angular_command = 0.0
        self.stable_frames = 0
        self.parked = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.complete_pub = self.create_publisher(
            Bool, '~/parking_complete', state_qos)
        self.pose_sub = self.create_subscription(
            PoseStamped, self.pose_topic, self.on_pose, 10)
        self.visible_count_sub = self.create_subscription(
            UInt32, self.visible_count_topic, self.on_visible_count, 10)
        self.reprojection_error_sub = self.create_subscription(
            Float32, self.reprojection_error_topic,
            self.on_reprojection_error, 10)
        self.inlier_count_sub = self.create_subscription(
            UInt32, self.inlier_count_topic, self.on_inlier_count, 10)
        self.enable_service = self.create_service(
            SetBool, '~/set_enabled', self.on_set_enabled)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.publish_complete(False)
        self.get_logger().info(
            'Downward-camera board parking ready: active=%s, cmd_topic=%s, '
            'base_target=(forward=%.3f,left=%.3f), min_tags=%d, '
            'single-tag parking enabled' %
            (self.enabled, self.cmd_topic, self.target_forward,
             self.target_left, self.min_visible_tags))

        if self.enabled:
            self.cmd_pub.publish(Twist())
            self.get_logger().warn(
                'Parking control starts active because enabled=true; '
                'publishing a zero command before tracking')

    def on_visible_count(self, msg):
        self.visible_tag_count = int(msg.data)
        self.last_count_time = time.monotonic()

    def on_reprojection_error(self, msg):
        self.reprojection_error = float(msg.data)
        self.last_error_time = time.monotonic()

    def on_inlier_count(self, msg):
        self.inlier_count = int(msg.data)
        self.last_inlier_time = time.monotonic()

    def on_pose(self, msg):
        now = time.monotonic()
        if (self.latest_pose is not None and self.last_pose_time is not None and
                now - self.last_pose_time <= self.loss_timeout):
            previous = self.latest_pose.pose.position
            current = msg.pose.position
            jump = math.sqrt(
                (current.x - previous.x) ** 2 +
                (current.y - previous.y) ** 2 +
                (current.z - previous.z) ** 2)
            if jump > self.max_pose_jump:
                self.get_logger().warn(
                    'Rejected board pose jump of %.3f m' % jump,
                    throttle_duration_sec=1.0)
                return
        self.latest_pose = msg
        self.last_pose_time = now

    def publish_complete(self, complete):
        msg = Bool()
        msg.data = complete
        self.complete_pub.publish(msg)

    def on_set_enabled(self, request, response):
        requested = bool(request.data)
        if requested == self.enabled:
            response.success = True
            response.message = (
                'Parking control is already active' if requested
                else 'Parking control is already inactive')
            return response

        if requested:
            # Require fresh pose and count samples after takeover.
            self.latest_pose = None
            self.last_pose_time = None
            self.visible_tag_count = 0
            self.last_count_time = None
            self.reprojection_error = math.inf
            self.inlier_count = 0
            self.last_error_time = None
            self.last_inlier_time = None
            self.tracking_since = None
            self.filtered_position = None
            self.filtered_edge = None
            self.last_command_time = time.monotonic()
            self.last_linear_command = 0.0
            self.last_angular_command = 0.0
            self.stable_frames = 0
            self.parked = False
            self.enabled = True
            self.publish_complete(False)
            self.cmd_pub.publish(Twist())
            response.message = (
                'Parking control activated; waiting for fresh board data')
            self.get_logger().warn(response.message)
        else:
            # Send one final stop, then stop publishing.  twist_mux releases
            # /cmd_vel_dock after its timeout and Nav2 can regain control.
            self.cmd_pub.publish(Twist())
            self.enabled = False
            self.tracking_since = None
            self.filtered_position = None
            self.filtered_edge = None
            self.last_linear_command = 0.0
            self.last_angular_command = 0.0
            self.stable_frames = 0
            response.message = (
                'Parking control deactivated; velocity ownership released')
            self.get_logger().info(response.message)

        response.success = True
        return response

    def publish_stop(self, reset_tracking=False):
        self.cmd_pub.publish(Twist())
        self.last_linear_command = 0.0
        self.last_angular_command = 0.0
        self.last_command_time = time.monotonic()
        self.stable_frames = 0
        if reset_tracking:
            self.tracking_since = None
            self.filtered_position = None
            self.filtered_edge = None

    def control_loop(self):
        # Publishing zero while inactive would keep the high-priority dock
        # input alive in twist_mux and permanently block Nav2.
        if not self.enabled:
            return

        cmd = Twist()
        now = time.monotonic()
        if self.parked:
            self.cmd_pub.publish(cmd)
            return
        perception_fresh = (
            self.latest_pose is not None and
            self.last_pose_time is not None and
            now - self.last_pose_time <= self.loss_timeout and
            self.last_count_time is not None and
            now - self.last_count_time <= self.loss_timeout and
            self.last_error_time is not None and
            now - self.last_error_time <= self.loss_timeout and
            self.last_inlier_time is not None and
            now - self.last_inlier_time <= self.loss_timeout)
        quality_good = (
            self.visible_tag_count >= self.min_visible_tags and
            self.inlier_count >= self.min_inlier_points and
            math.isfinite(self.reprojection_error) and
            self.reprojection_error <= self.max_reprojection_error)
        if not perception_fresh or not quality_good:
            self.publish_stop(reset_tracking=True)
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, self.latest_pose.header.frame_id, Time())
            board_position, board_orientation = transform_pose(
                self.latest_pose.pose, transform.transform)
            error_edge = nearest_edge_error(board_orientation)
        except (TransformException, ValueError) as exc:
            self.get_logger().warn(
                'Cannot transform parking board into %s: %s' %
                (self.base_frame, str(exc)),
                throttle_duration_sec=2.0)
            self.publish_stop(reset_tracking=True)
            return

        if self.filtered_position is None or self.filtered_edge is None:
            self.filtered_position = board_position
            self.filtered_edge = error_edge
            self.tracking_since = now
            self.publish_stop()
            return

        position_jump = math.hypot(
            board_position[0] - self.filtered_position[0],
            board_position[1] - self.filtered_position[1])
        edge_jump = abs(wrap_period(
            error_edge - self.filtered_edge, math.pi / 2.0))
        if (position_jump > self.max_base_pose_jump or
                edge_jump > self.max_edge_jump):
            self.get_logger().warn(
                'Rejected unstable board pose: jump=%.3fm edge_jump=%.1fdeg' %
                (position_jump, math.degrees(edge_jump)),
                throttle_duration_sec=1.0)
            self.filtered_position = board_position
            self.filtered_edge = error_edge
            self.tracking_since = now
            self.publish_stop()
            return

        alpha = max(0.0, min(1.0, self.filter_alpha))
        self.filtered_position = (
            self.filtered_position[0] +
            alpha * (board_position[0] - self.filtered_position[0]),
            self.filtered_position[1] +
            alpha * (board_position[1] - self.filtered_position[1]),
            self.filtered_position[2] +
            alpha * (board_position[2] - self.filtered_position[2]))
        edge_delta = wrap_period(
            error_edge - self.filtered_edge, math.pi / 2.0)
        self.filtered_edge = wrap_period(
            self.filtered_edge + alpha * edge_delta, math.pi / 2.0)

        if (self.tracking_since is None or
                now - self.tracking_since < self.tracking_stable_time):
            self.publish_stop()
            return

        error_forward = self.filtered_position[0] - self.target_forward
        error_left = self.filtered_position[1] - self.target_left
        position_error = math.hypot(error_forward, error_left)
        overlap = footprint_overlap_ratio(
            self.filtered_position, board_orientation, self.footprint,
            self.board_width, self.board_height, self.board_margin)
        edge_aligned = abs(self.filtered_edge) <= self.edge_tolerance
        required_stable_frames = (
            self.single_tag_stable_frames
            if self.visible_tag_count == 1
            else self.stable_frames_required)

        if (position_error <= self.position_tolerance and
                overlap >= self.min_footprint_overlap and edge_aligned):
            self.stable_frames += 1
            if self.stable_frames >= required_stable_frames:
                self.parked = True
                self.publish_complete(True)
                self.get_logger().info(
                    'Parking complete: error=%.3fm edge=%.1fdeg '
                    'footprint_overlap=%.0f%% tags=%d' %
                    (position_error, math.degrees(self.filtered_edge),
                     overlap * 100.0, self.visible_tag_count))
            self.last_linear_command = 0.0
            self.last_angular_command = 0.0
            self.cmd_pub.publish(cmd)
            return
        self.stable_frames = 0

        max_linear = (
            self.single_tag_max_linear
            if self.visible_tag_count == 1 else self.max_linear)
        max_angular = (
            self.single_tag_max_angular
            if self.visible_tag_count == 1 else self.max_angular)
        target_linear, target_angular = compute_parking_command(
            error_forward, error_left, self.filtered_edge,
            max_linear, max_angular, self.min_linear,
            self.slowdown_distance, self.kp_distance,
            self.kp_bearing, self.kp_edge, self.allow_reverse)
        if position_error <= self.position_tolerance and not edge_aligned:
            target_linear = 0.0

        dt = max(0.01, min(0.20, now - self.last_command_time))
        linear_step = self.max_linear_accel * dt
        angular_step = self.max_angular_accel * dt
        cmd.linear.x = self.last_linear_command + clamp(
            target_linear - self.last_linear_command, linear_step)
        cmd.angular.z = self.last_angular_command + clamp(
            target_angular - self.last_angular_command, angular_step)
        self.last_linear_command = cmd.linear.x
        self.last_angular_command = cmd.angular.z
        self.last_command_time = now
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            'board_base(fwd=%+.3f,left=%+.3f) '
            'err(dist=%.3f,bearing=%+.1fdeg,edge=%+.1fdeg) '
            'tags=%d rmse=%.2fpx inliers=%d overlap=%.0f%% '
            'cmd(v=%+.3f,w=%+.3f)' % (
                self.filtered_position[0], self.filtered_position[1],
                position_error,
                math.degrees(math.atan2(error_left, error_forward)),
                math.degrees(self.filtered_edge),
                self.visible_tag_count, self.reprojection_error,
                self.inlier_count, overlap * 100.0,
                cmd.linear.x, cmd.angular.z),
            throttle_duration_sec=0.5)

    def stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = BoardParker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
