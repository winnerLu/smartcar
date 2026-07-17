#!/usr/bin/env python3
"""里程计标定辅助:实时打印 /wheel/odom 的累计位移和累计偏航角。

用于标定 odom_vx_scale / odom_wz_scale:
  1. 车放起点,运行本脚本,看到当前 x / yaw
  2. 记下起始读数(或按理解归零:重启节点使 odom 归零最干净)
  3. 遥控做已知运动(直行量距离 / 原地转已知角度)
  4. 记结束读数,算 里程计增量 vs 实际值,得出 scale

用法:
  python3 odom_monitor.py                 # 默认订阅 /wheel/odom
  python3 odom_monitor.py --topic /odometry/filtered
"""

import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def yaw_from_quat(q):
    # 仅绕 Z 的偏航角
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class OdomMonitor(Node):
    def __init__(self, topic):
        super().__init__('odom_monitor')
        self.start_x = None
        self.start_y = None
        self.start_yaw = None
        self.cum_yaw = 0.0        # 累计偏航(可超过 ±180,便于数整圈)
        self.last_yaw = None
        self.sub = self.create_subscription(Odometry, topic, self.cb, 20)
        self.get_logger().info(f'监听 {topic},实时打印累计位移/角度...')

    def cb(self, msg):
        p = msg.pose.pose.position
        yaw = yaw_from_quat(msg.pose.pose.orientation)

        if self.start_x is None:
            self.start_x, self.start_y = p.x, p.y
            self.start_yaw = yaw
            self.last_yaw = yaw
            self.cum_yaw = 0.0

        # 累计偏航:处理 ±pi 翻转,支持转多圈
        d = yaw - self.last_yaw
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        self.cum_yaw += d
        self.last_yaw = yaw

        dx = p.x - self.start_x
        dy = p.y - self.start_y
        dist = math.sqrt(dx * dx + dy * dy)
        print(f'\r累计: 直线位移={dist:6.3f} m (dx={dx:+.3f} dy={dy:+.3f})  '
              f'偏航={math.degrees(self.cum_yaw):+8.2f}°   ',
              end='', flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/wheel/odom')
    args = ap.parse_args(sys.argv[1:])
    rclpy.init()
    node = OdomMonitor(args.topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n退出')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
