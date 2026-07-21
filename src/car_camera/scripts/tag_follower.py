#!/usr/bin/env python3
"""AprilTag 视觉伺服:自动泊入标签正前方 target_z 距离,并保持标签在画面正中。

原理:
  1. 订阅 /apriltag_detector/tag_pose,获取标签在相机坐标系下的位姿
  2. 目标: tag.x=0(水平居中), tag.y=0(垂直居中), tag.z=target_z(目标距离)
  3. P 控制器: 横向误差 -> 角速度,深度误差 -> 线速度
  4. 发布 /cmd_vel 控制小车
  5. 标签丢失超过 0.5 秒自动停车

用法:
    # 仅控制(配合已启动的 camera_apriltag):
    ros2 run car_camera tag_follower.py

    # 所有一起启动:
    ros2 launch car_camera camera_apriltag.launch.py \
        video_device:=/dev/video0 tag_size:=0.120 \
        enable_follower:=true

参数(tag_follower 节点):
    target_z         目标距离(米), 默认 0.05
    kp_linear        深度 P 增益, 默认 0.8
    kp_angular       横向 P 增益, 默认 2.0
    max_linear       最大线速度(m/s), 默认 0.08
    max_angular      最大角速度(rad/s), 默认 0.5
    dead_zone_x      横向死区(米), 默认 0.003
    dead_zone_y      垂直死区(米), 默认 0.003
    dead_zone_z      深度死区(米), 默认 0.005
    loss_timeout     标签丢失超时(秒), 默认 0.5
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
        self.max_linear = self.declare_parameter('max_linear', 0.08).value
        self.max_angular = self.declare_parameter('max_angular', 0.5).value
        self.dead_zone_x = self.declare_parameter('dead_zone_x', 0.003).value
        self.dead_zone_y = self.declare_parameter('dead_zone_y', 0.003).value
        self.dead_zone_z = self.declare_parameter('dead_zone_z', 0.005).value
        self.loss_timeout = self.declare_parameter('loss_timeout', 0.5).value
        self.enabled = self.declare_parameter('enabled', True).value

        # ---- 状态 ----
        self.last_tag_time = None
        self.latest_tag = None
        self.arrived = False

        # ---- 发布/订阅 ----
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
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
            f'Kp=(linear={self.kp_linear}, angular={self.kp_angular}) '
            f'max=(v={self.max_linear}, w={self.max_angular}) '
            f'dead_zone=(x={self.dead_zone_x}, y={self.dead_zone_y}, z={self.dead_zone_z})'
        )

    def on_tag_pose(self, msg: PoseStamped):
        """缓存最新标签位姿并更新时间戳。"""
        self.latest_tag = msg
        self.last_tag_time = time.monotonic()

    def control_loop(self):
        """50Hz 控制循环:计算误差 -> P 控制 -> 发布 /cmd_vel。"""
        twist = Twist()

        # 检查标签是否新鲜
        now = time.monotonic()
        lost = (self.last_tag_time is None or
                (now - self.last_tag_time) > self.loss_timeout)

        if lost or not self.enabled:
            if lost:
                self.get_logger().warn(
                    '标签丢失,停车',
                    throttle_duration_sec=2.0,
                )
            self.arrived = False
            self.cmd_pub.publish(twist)
            return

        # 提取标签位姿(相机坐标系)
        tag_x = self.latest_tag.pose.position.x   # 横向偏移(右正)
        tag_y = self.latest_tag.pose.position.y   # 垂直偏移(下正)
        tag_z = self.latest_tag.pose.position.z   # 深度距离(前正)

        # 误差
        err_x = tag_x           # 横向误差(目标 x=0)
        err_y = tag_y           # 垂直误差(目标 y=0)
        err_z = tag_z - self.target_z  # 深度误差(目标 z=0.05)

        # 死区
        if abs(err_x) < self.dead_zone_x:
            err_x = 0.0
        if abs(err_y) < self.dead_zone_y:
            err_y = 0.0
        if abs(err_z) < self.dead_zone_z:
            err_z = 0.0

        # 到达判定(水平、垂直、深度都在死区内)
        self.arrived = (abs(err_x) < self.dead_zone_x and
                        abs(err_y) < self.dead_zone_y and
                        abs(err_z) < self.dead_zone_z)

        if self.arrived:
            self.get_logger().info(
                f'已到达! tag@{tag_z:.3f}m, 偏移(x={tag_x:.3f}, y={tag_y:.3f})',
                throttle_duration_sec=1.0,
            )
            self.cmd_pub.publish(twist)
            return

        # ---- P 控制器 ----
        # 深度 -> 线速度 (正值前进,负值后退)
        vx = self.kp_linear * err_z
        vx = max(-self.max_linear, min(self.max_linear, vx))

        # 横向 -> 角速度
        # 标签在右(x>0) -> 右转(wz<0) -> 标签居中
        wz = -self.kp_angular * err_x
        wz = max(-self.max_angular, min(self.max_angular, wz))

        twist.linear.x = vx
        twist.angular.z = wz
        self.cmd_pub.publish(twist)

        self.get_logger().info(
            f'tag(z={tag_z:.3f}, x={tag_x:.3f}, y={tag_y:.3f}) '
            f'err(z={err_z:.3f}, x={err_x:.3f}, y={err_y:.3f}) '
            f'cmd(v={vx:.3f}, w={wz:.3f})',
            throttle_duration_sec=0.5,
        )


def main():
    rclpy.init()
    node = TagFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 停车
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
        print('已退出,已停车。')


if __name__ == '__main__':
    main()
