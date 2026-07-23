#!/usr/bin/env python3
"""Park on the centre of a four-way AprilTag board.

The board may be approached from any of its four edges.  Its in-plane angle is
therefore reduced modulo 90 degrees; no tag ID or absolute board heading is a
parking target.  The detector is responsible for publishing the physical board
centre on ``/apriltag_detector/tag_pose``.
"""

import math
import time

from geometry_msgs.msg import PoseStamped, Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


def clamp(value, limit):
    return max(-limit, min(limit, value))


def wrap_period(angle, period):
    """Return the signed distance to the nearest multiple of period."""
    return (angle + period / 2.0) % period - period / 2.0


def quaternion_to_matrix(q):
    """Return a row-major 3x3 rotation matrix for a ROS quaternion."""
    x, y, z, w = q.x, q.y, q.z, q.w
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        raise ValueError('invalid zero-length quaternion')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return (
        1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
        2 * (x * z + y * w), 2 * (x * y + z * w),
        1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
        2 * (x * z - y * w), 2 * (y * z + x * w),
        1 - 2 * (x * x + y * y),
    )


def nearest_edge_error(q):
    """Board in-plane heading error, with all four edge headings equivalent."""
    rotation = quaternion_to_matrix(q)
    board_x_angle = math.atan2(rotation[3], rotation[0])
    return wrap_period(board_x_angle, math.pi / 2.0)


class BoardParker(Node):
    def __init__(self):
        super().__init__('board_parker')

        self.pose_topic = self.declare_parameter(
            'pose_topic', '/apriltag_detector/tag_pose').value
        self.cmd_topic = self.declare_parameter(
            'cmd_topic', '/cmd_vel_dock').value
        self.target_x = float(self.declare_parameter('target_x', 0.0).value)
        self.target_z = float(self.declare_parameter('target_z', 0.05).value)
        self.kp_linear = float(self.declare_parameter('kp_linear', 0.7).value)
        self.kp_center = float(self.declare_parameter('kp_center', 2.2).value)
        self.kp_edge = float(self.declare_parameter('kp_edge', 1.0).value)
        self.max_linear = float(self.declare_parameter('max_linear', 0.06).value)
        self.max_angular = float(self.declare_parameter('max_angular', 0.40).value)
        self.min_linear = float(self.declare_parameter('min_linear', 0.018).value)
        self.center_tolerance = float(
            self.declare_parameter('center_tolerance', 0.008).value)
        self.distance_tolerance = float(
            self.declare_parameter('distance_tolerance', 0.008).value)
        self.edge_tolerance = math.radians(float(
            self.declare_parameter('edge_tolerance_deg', 5.0).value))
        self.slowdown_distance = float(
            self.declare_parameter('slowdown_distance', 0.15).value)
        self.loss_timeout = float(self.declare_parameter('loss_timeout', 0.35).value)
        self.stable_frames_required = int(
            self.declare_parameter('stable_frames', 8).value)
        self.allow_reverse = bool(self.declare_parameter('allow_reverse', False).value)
        self.enabled = bool(self.declare_parameter('enabled', False).value)

        self.latest_pose = None
        self.last_pose_time = None
        self.stable_frames = 0
        self.parked = False

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.complete_pub = self.create_publisher(
            Bool, '~/parking_complete', state_qos)
        self.pose_sub = self.create_subscription(
            PoseStamped, self.pose_topic, self.on_pose, 10)
        self.enable_service = self.create_service(
            SetBool, '~/set_enabled', self.on_set_enabled)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.publish_complete(False)
        self.get_logger().info(
            'Four-way board parking ready: active=%s, cmd_topic=%s, '
            'target=(x=%.3f,z=%.3f); use ~/set_enabled to take or release control' %
            (self.enabled, self.cmd_topic, self.target_x, self.target_z))

        if self.enabled:
            self.cmd_pub.publish(Twist())
            self.get_logger().warn(
                'Parking control starts active because enabled=true; '
                'publishing a zero command before tracking')

    def on_pose(self, msg):
        self.latest_pose = msg
        self.last_pose_time = time.monotonic()

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
            # Require a fresh detector sample after takeover.  This prevents a
            # cached pose from moving the vehicle immediately.
            self.latest_pose = None
            self.last_pose_time = None
            self.stable_frames = 0
            self.parked = False
            self.enabled = True
            self.publish_complete(False)
            self.cmd_pub.publish(Twist())
            response.message = (
                'Parking control activated; waiting for a fresh board pose')
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
        # Publishing zero while inactive would still keep the high-priority
        # dock input alive in twist_mux and permanently block Nav2.
        if not self.enabled:
            return

        cmd = Twist()
        now = time.monotonic()
        if self.parked:
            self.cmd_pub.publish(cmd)
            return
        if (self.latest_pose is None or self.last_pose_time is None or
                now - self.last_pose_time > self.loss_timeout):
            self.stable_frames = 0
            self.cmd_pub.publish(cmd)
            return

        pose = self.latest_pose.pose
        error_x = pose.position.x - self.target_x
        error_z = pose.position.z - self.target_z
        try:
            error_edge = nearest_edge_error(pose.orientation)
        except ValueError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=2.0)
            self.cmd_pub.publish(cmd)
            return

        centered = abs(error_x) <= self.center_tolerance
        at_distance = abs(error_z) <= self.distance_tolerance
        edge_aligned = abs(error_edge) <= self.edge_tolerance
        if centered and at_distance and edge_aligned:
            self.stable_frames += 1
            if self.stable_frames >= self.stable_frames_required:
                self.parked = True
                self.publish_complete(True)
                self.get_logger().info(
                    'Parking complete: board centre is under the calibrated '
                    'vehicle centre; holding zero velocity until deactivated')
            self.cmd_pub.publish(cmd)
            return
        self.stable_frames = 0

        linear = self.kp_linear * error_z
        if not self.allow_reverse:
            linear = max(0.0, linear)
        linear = clamp(linear, self.max_linear)
        if 0.0 < linear < self.min_linear and error_z > self.distance_tolerance:
            linear = self.min_linear

        # Do not drive quickly while the centre or nearest edge is poorly aligned.
        alignment_scale = max(0.20, 1.0 - abs(error_edge) / (math.pi / 4.0))
        centering_scale = max(0.20, 1.0 - abs(error_x) / max(self.slowdown_distance, 1e-3))
        cmd.linear.x = linear * min(alignment_scale, centering_scale)
        cmd.angular.z = clamp(
            -self.kp_center * error_x - self.kp_edge * error_edge,
            self.max_angular)
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            'centre(x=%+.3f,z=%.3f) err(x=%+.3f,z=%+.3f,edge=%+.1fdeg) '
            'cmd(v=%.3f,w=%+.3f)' % (
                pose.position.x, pose.position.z, error_x, error_z,
                math.degrees(error_edge), cmd.linear.x, cmd.angular.z),
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
