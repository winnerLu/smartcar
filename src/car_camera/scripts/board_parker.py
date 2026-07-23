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
from std_msgs.msg import Bool, UInt32
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


class BoardParker(Node):
    def __init__(self):
        super().__init__('board_parker')

        self.pose_topic = self.declare_parameter(
            'pose_topic', '/apriltag_detector/tag_pose').value
        self.visible_count_topic = self.declare_parameter(
            'visible_count_topic',
            '/apriltag_detector/visible_tag_count').value
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

        self.kp_linear = float(self.declare_parameter('kp_linear', 0.7).value)
        self.kp_lateral = float(
            self.declare_parameter('kp_lateral', 2.2).value)
        self.kp_edge = float(self.declare_parameter('kp_edge', 1.0).value)
        self.max_linear = float(self.declare_parameter('max_linear', 0.06).value)
        self.max_angular = float(self.declare_parameter('max_angular', 0.40).value)
        self.min_linear = float(self.declare_parameter('min_linear', 0.018).value)
        self.forward_tolerance = float(
            self.declare_parameter('forward_tolerance', 0.012).value)
        self.lateral_tolerance = float(
            self.declare_parameter('lateral_tolerance', 0.012).value)
        self.edge_tolerance = math.radians(float(
            self.declare_parameter('edge_tolerance_deg', 10.0).value))
        self.slowdown_distance = float(
            self.declare_parameter('slowdown_distance', 0.15).value)
        self.loss_timeout = float(self.declare_parameter('loss_timeout', 0.35).value)
        self.max_pose_jump = float(
            self.declare_parameter('max_pose_jump', 0.05).value)
        self.min_visible_tags = int(
            self.declare_parameter('min_visible_tags', 3).value)
        self.stable_frames_required = int(
            self.declare_parameter('stable_frames', 8).value)
        self.allow_reverse = bool(self.declare_parameter('allow_reverse', False).value)
        self.enabled = bool(self.declare_parameter('enabled', False).value)

        self.latest_pose = None
        self.last_pose_time = None
        self.visible_tag_count = 0
        self.last_count_time = None
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
        self.enable_service = self.create_service(
            SetBool, '~/set_enabled', self.on_set_enabled)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.publish_complete(False)
        self.get_logger().info(
            'Downward-camera board parking ready: active=%s, cmd_topic=%s, '
            'base_target=(forward=%.3f,left=%.3f), min_tags=%d' %
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
            self.stable_frames = 0
            response.message = (
                'Parking control deactivated; velocity ownership released')
            self.get_logger().info(response.message)

        response.success = True
        return response

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
        if (self.latest_pose is None or self.last_pose_time is None or
                now - self.last_pose_time > self.loss_timeout or
                self.last_count_time is None or
                now - self.last_count_time > self.loss_timeout or
                self.visible_tag_count < self.min_visible_tags):
            self.stable_frames = 0
            self.cmd_pub.publish(cmd)
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
            self.stable_frames = 0
            self.cmd_pub.publish(cmd)
            return

        error_forward = board_position[0] - self.target_forward
        error_left = board_position[1] - self.target_left
        contained = footprint_inside_board(
            board_position, board_orientation, self.footprint,
            self.board_width, self.board_height, self.board_margin)
        forward_ok = abs(error_forward) <= self.forward_tolerance
        lateral_ok = abs(error_left) <= self.lateral_tolerance
        edge_aligned = abs(error_edge) <= self.edge_tolerance

        if contained and forward_ok and lateral_ok and edge_aligned:
            self.stable_frames += 1
            if self.stable_frames >= self.stable_frames_required:
                self.parked = True
                self.publish_complete(True)
                self.get_logger().info(
                    'Parking complete: footprint is inside the board and '
                    'the base-frame target is stable; holding zero velocity')
            self.cmd_pub.publish(cmd)
            return
        self.stable_frames = 0

        linear = self.kp_linear * error_forward
        if not self.allow_reverse:
            linear = max(0.0, linear)
        linear = clamp(linear, self.max_linear)
        if (0.0 < linear < self.min_linear and
                error_forward > self.forward_tolerance):
            linear = self.min_linear

        # Reduce forward motion while lateral or edge alignment error is large.
        alignment_scale = max(
            0.20, 1.0 - abs(error_edge) / (math.pi / 4.0))
        centering_scale = max(
            0.20,
            1.0 - abs(error_left) / max(self.slowdown_distance, 1e-3))
        cmd.linear.x = linear * min(alignment_scale, centering_scale)
        cmd.angular.z = clamp(
            self.kp_lateral * error_left + self.kp_edge * error_edge,
            self.max_angular)
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            'board_base(fwd=%+.3f,left=%+.3f) '
            'err(fwd=%+.3f,left=%+.3f,edge=%+.1fdeg) '
            'tags=%d contained=%s cmd(v=%.3f,w=%+.3f)' % (
                board_position[0], board_position[1],
                error_forward, error_left, math.degrees(error_edge),
                self.visible_tag_count, contained,
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
