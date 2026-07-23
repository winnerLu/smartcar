#!/usr/bin/env python3
"""AprilTag 视觉伺服:自动泊入停车板正前方 target_z 距离,并保持板在画面正中。

支持多标签停车板(tag36h11 ID0-7):apriltag_detector_node 对所有可见标签
取均值作为停车板中心位姿;本节点订阅该中心位姿并执行 P 控制。

原理:
  1. 订阅 /apriltag_detector/tag_pose,获取标签/停车板在相机坐标系下的位姿
  2. 目标: x=0(水平居中), y=0(垂直居中), z=target_z(目标距离), yaw=0(正对板面)
  3. P 控制器: 横向误差 -> 角速度, 深度误差 -> 线速度, 朝向误差 -> 角速度修正
  4. 标签丢失时进入搜索模式:根据最后已知位置旋转找回标签
  5. 发布 cmd_topic(默认 /cmd_vel_dock)控制小车

用法:
    # 仅控制(配合已启动的 camera_apriltag):
    ros2 run car_camera tag_follower.py

    # 所有一起启动:
    ros2 launch car_camera camera_apriltag.launch.py \
        video_device:=/dev/video0 tag_size:=0.050 \
        enable_follower:=true

参数(tag_follower 节点):
    target_z           目标距离(米), 默认 0.05
    kp_linear          深度 P 增益, 默认 0.8
    kp_angular         横向 P 增益, 默认 2.0
    kp_yaw             朝向 P 增益, 默认 1.5
    enable_orientation 是否启用朝向对准, 默认 True
    max_linear         最大线速度(m/s), 默认 0.08
    max_angular        最大角速度(rad/s), 默认 0.5
    dead_zone_x        横向死区(米), 默认 0.003
    dead_zone_y        垂直死区(米), 默认 0.003
    dead_zone_z        深度死区(米), 默认 0.005
    loss_timeout       标签丢失超时(秒), 默认 0.5
    search_wz          搜索时旋转速度(rad/s), 默认 0.15
    search_timeout     搜索多久后放弃(秒), 默认 3.0
    search_enabled     是否在丢失标签后旋转搜索, 默认 False
    allow_reverse      是否允许倒车, 默认 False
    cmd_topic          速度输出话题, 默认 /cmd_vel_dock
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped


class TagFollower(Node):
    def __init__(self):
        super().__init__('tag_follower')

        # ---- 参数 ----
        self.target_z = self.declare_parameter('target_z', 0.05).value
        self.kp_linear = self.declare_parameter('kp_linear', 0.8).value
        self.kp_angular = self.declare_parameter('kp_angular', 2.0).value
        self.kp_yaw = self.declare_parameter('kp_yaw', 1.5).value
        self.enable_orientation = self.declare_parameter('enable_orientation', True).value
        self.max_linear = self.declare_parameter('max_linear', 0.03).value
        self.max_angular = self.declare_parameter('max_angular', 0.2).value
        self.dead_zone_x = self.declare_parameter('dead_zone_x', 0.003).value
        self.dead_zone_y = self.declare_parameter('dead_zone_y', 0.003).value
        self.dead_zone_z = self.declare_parameter('dead_zone_z', 0.005).value
        self.loss_timeout = self.declare_parameter('loss_timeout', 0.5).value
        self.search_wz = self.declare_parameter('search_wz', 0.15).value
        self.search_timeout = self.declare_parameter('search_timeout', 3.0).value
        self.search_enabled = self.declare_parameter(
            'search_enabled', False).value
        self.allow_reverse = self.declare_parameter(
            'allow_reverse', False).value
        self.enabled = self.declare_parameter('enabled', True).value
        self.cmd_topic = self.declare_parameter(
            'cmd_topic', '/cmd_vel_dock').value

        # ---- 状态 ----
        self.last_tag_time = None         # 最后一次检测到标签的时刻
        self.latest_tag = None            # 最新标签位姿
        self.arrived = False
        self.search_dir = 0.0             # 搜索旋转方向(+1左转, -1右转)
        self.search_start = None          # 搜索开始时刻
        self.last_seen_x = 0.0            # 标签丢失前最后的横向位置

        # ---- 发布/订阅 ----
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.tag_sub = self.create_subscription(
            PoseStamped,
            '/apriltag_detector/tag_pose',
            self.on_tag_pose,
            10,
        )

        # 50Hz 控制循环(比 200ms 看门狗快,确保不会超时)
        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info(
            f'TagFollower 已启动: target_z={self.target_z:.3f} '
            f'Kp=(linear={self.kp_linear}, angular={self.kp_angular}, yaw={self.kp_yaw}) '
            f'orientation={"ON" if self.enable_orientation else "OFF"} '
            f'max=(v={self.max_linear}, w={self.max_angular}) '
            f'search=(wz={self.search_wz}, timeout={self.search_timeout}s) '
            f'cmd_topic={self.cmd_topic}'
        )

    def on_tag_pose(self, msg: PoseStamped):
        """缓存最新标签位姿并更新时间戳。"""
        self.latest_tag = msg
        self.last_tag_time = time.monotonic()
        self.search_start = None  # 标签重新出现,退出搜索模式

    def control_loop(self):
        """50Hz 控制循环:跟踪 → 搜索 → 停车。"""
        if not self.enabled:
            return

        twist = Twist()
        now = time.monotonic()

        # ==================== 正常跟踪模式 ====================
        if self.last_tag_time is not None and (now - self.last_tag_time) < self.loss_timeout:
            self._track(twist)
        # ==================== 搜索模式 ====================
        elif (self.search_enabled and self.last_tag_time is not None and
              self.search_start is None):
            # 刚丢失,保存最后位置,进入搜索
            self.last_seen_x = self.latest_tag.pose.position.x
            self.search_start = now
            self.get_logger().info(
                f'标签丢失,进入搜索模式(最后位置 x={self.last_seen_x:.3f})')
            self._search(twist, now)
        elif (self.search_enabled and self.search_start is not None and
              (now - self.search_start) < self.search_timeout):
            self._search(twist, now)
        # ==================== 彻底停机 ====================
        else:
            if self.last_tag_time is not None:
                self.get_logger().warn(
                    '搜索超时,放弃搜索,停车',
                    throttle_duration_sec=3.0,
                )
            self.search_start = None
            self.arrived = False

        self.cmd_pub.publish(twist)

    # -----------------------------------------------------------
    def _extract_yaw(self, pose_msg):
        """从 PoseStamped 四元数提取停车板法向量在水平面的偏角(yaw)。

        原理: 四元数→旋转矩阵,取第三列(板法向量在相机系中的方向),
              atan2(法向量.x, 法向量.z) = 板偏离相机光轴的水平角度。

        Returns: yaw_err (rad), 正值=板向右偏, 小车需右转(wz<0)补偿。
        """
        qx = pose_msg.pose.orientation.x
        qy = pose_msg.pose.orientation.y
        qz = pose_msg.pose.orientation.z
        qw = pose_msg.pose.orientation.w

        # 旋转矩阵第三列: 标签 z 轴(法向量)在相机系中的方向
        nx = 2.0 * (qx * qz + qy * qw)   # r13
        nz = 1.0 - 2.0 * (qx * qx + qy * qy)  # r33

        # 法向量在水平面(x-z)的偏角
        yaw_err = math.atan2(nx, nz)
        return yaw_err

    # -----------------------------------------------------------
    def _track(self, twist):
        """正常 P 控制:根据标签位姿计算 cmd_vel。"""
        tag_x = self.latest_tag.pose.position.x
        tag_y = self.latest_tag.pose.position.y
        tag_z = self.latest_tag.pose.position.z

        err_x = 0.0 if abs(tag_x) < self.dead_zone_x else tag_x
        err_y = 0.0 if abs(tag_y) < self.dead_zone_y else tag_y
        err_z = 0.0 if abs(tag_z - self.target_z) < self.dead_zone_z else tag_z - self.target_z

        self.arrived = (abs(tag_x) < self.dead_zone_x and
                        abs(tag_y) < self.dead_zone_y and
                        abs(tag_z - self.target_z) < self.dead_zone_z)

        if self.arrived:
            self.get_logger().info(
                f'已到达! tag@{tag_z:.3f}m, 偏移(x={tag_x:.3f}, y={tag_y:.3f})',
                throttle_duration_sec=1.0,
            )
            return

        vx = max(-self.max_linear, min(self.max_linear, self.kp_linear * err_z))
        if not self.allow_reverse:
            vx = max(0.0, vx)
        wz_centering = -self.kp_angular * err_x

        yaw_err = 0.0
        if self.enable_orientation:
            yaw_err = self._extract_yaw(self.latest_tag)
            wz_orientation = -self.kp_yaw * yaw_err
        else:
            wz_orientation = 0.0

        wz = max(-self.max_angular, min(self.max_angular, wz_centering + wz_orientation))

        twist.linear.x = vx
        twist.angular.z = wz

        self.get_logger().info(
            f'tag(z={tag_z:.3f}, x={tag_x:.3f}, y={tag_y:.3f}) '
            f'err(z={err_z:.3f}, x={err_x:.3f}, y={err_y:.3f}) '
            f'yaw={math.degrees(yaw_err):.1f}° '
            f'cmd(v={vx:.3f}, w={wz:.3f})',
            throttle_duration_sec=0.5,
        )

    def _search(self, twist, now):
        """搜索模式:根据最后已知位置旋转,尝试找回标签。"""
        # 如果知道标签在哪个方向,就往那个方向转
        if abs(self.last_seen_x) > self.dead_zone_x:
            # 标签最后在右边(x>0) → 右转(wz<0)把它拉回画面中心
            direction = -1.0 if self.last_seen_x > 0 else 1.0
        else:
            # 不知道方向,缓慢左转扫一圈
            elapsed = now - self.search_start
            # 每 2 秒换一次方向,来回扫
            direction = 1.0 if (int(elapsed / 2.0) % 2 == 0) else -1.0

        twist.angular.z = direction * self.search_wz

        self.get_logger().info(
            f'搜索中(最后 x={self.last_seen_x:.3f}, '
            f'dir={"+左" if direction > 0 else "-右"}, '
            f'w={twist.angular.z:.3f})',
            throttle_duration_sec=1.0,
        )


def main():
    rclpy.init()
    node = TagFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
        print('已退出,已停车。')


if __name__ == '__main__':
    main()
