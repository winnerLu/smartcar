#!/usr/bin/env python3
"""简单键盘遥控节点:发布 /cmd_vel 控制差速小车。

针对本项目定制,替代 teleop_twist_keyboard:
- 固定档位速度,带硬上限(调速不会超过 max,从根上防止速度溢出)
- "死人开关":一直按方向键才持续运动;松开超过 release_timeout 秒自动停车
- 只用 linear.x(前进/后退)和 angular.z(转向),契合差速车

按键:
  w / s : 前进 / 后退
  a / d : 左转 / 右转
  空格 或 x : 立即停止
  = / - : 增大 / 减小线速度档位(受 max_linear 限幅)
  ] / [ : 增大 / 减小角速度档位(受 max_angular 限幅)
  q / Ctrl-C : 退出(退出前发零速)

说明:终端收不到"松开"事件,故用超时判定——持续按键(键盘自动重复)会不断
刷新按键时间,车持续运动;一旦松开超过 release_timeout 秒,自动发停止。

参数:
  linear_speed / angular_speed  初始档位
  max_linear / max_angular      硬上限
  step                          调速增量
  release_timeout               松开多久后自动停(秒), 默认 1.0
"""

import sys
import termios
import tty
import select
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

HELP = """
============ 键盘遥控 ============
  w/s : 前进/后退      a/d : 左转/右转
  空格 或 x : 停止
  =/- : 线速度档位 增/减
  ]/[ : 角速度档位 增/减
  q   : 退出
  (松开超过 1 秒自动停车,需一直按才持续运动)
=================================="""


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.linear_speed = self.declare_parameter('linear_speed', 0.10).value
        self.angular_speed = self.declare_parameter('angular_speed', 0.4).value
        self.max_linear = self.declare_parameter('max_linear', 0.4).value
        self.max_angular = self.declare_parameter('max_angular', 1.5).value
        self.step = self.declare_parameter('step', 0.05).value
        self.release_timeout = self.declare_parameter('release_timeout', 0.4).value

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.vx = 0.0
        self.wz = 0.0
        self.last_move_key_time = None   # 最后一次收到方向键的时刻

    def clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    def publish(self):
        t = Twist()
        t.linear.x = self.vx
        t.angular.z = self.wz
        self.pub.publish(t)

    def stop(self):
        self.vx = 0.0
        self.wz = 0.0

    def handle_key(self, key, mono):
        """处理按键;返回 False 表示退出。mono 为单调时钟读数。"""
        if key in ('w', 'W'):
            self.vx = self.linear_speed
            self.last_move_key_time = mono
        elif key in ('s', 'S'):
            self.vx = -self.linear_speed
            self.last_move_key_time = mono
        elif key in ('a', 'A'):          # 左转 = 逆时针 = angular.z 正(REP-103 标准)
            self.wz = self.angular_speed
            self.last_move_key_time = mono
        elif key in ('d', 'D'):          # 右转 = 顺时针 = angular.z 负
            self.wz = -self.angular_speed
            self.last_move_key_time = mono
        elif key in (' ', 'x', 'X'):
            self.stop()
            self.last_move_key_time = None
        elif key == '=':
            self.linear_speed = self.clamp(
                self.linear_speed + self.step, 0.0, self.max_linear)
            self.print_speed()
        elif key == '-':
            self.linear_speed = self.clamp(
                self.linear_speed - self.step, 0.0, self.max_linear)
            self.print_speed()
        elif key == ']':
            self.angular_speed = self.clamp(
                self.angular_speed + self.step, 0.0, self.max_angular)
            self.print_speed()
        elif key == '[':
            self.angular_speed = self.clamp(
                self.angular_speed - self.step, 0.0, self.max_angular)
            self.print_speed()
        elif key in ('q', 'Q', '\x03'):
            return False
        return True

    def print_speed(self):
        print(f'\r档位: 线速度={self.linear_speed:.2f} m/s  '
              f'角速度={self.angular_speed:.2f} rad/s   ', end='', flush=True)


def get_key(settings, timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    rclpy.init()
    node = KeyboardTeleop()
    print(HELP)
    node.print_speed()
    settings = termios.tcgetattr(sys.stdin)
    try:
        while rclpy.ok():
            # 每 50ms 一个循环:读键 + 判定超时 + 发布(20Hz)
            key = get_key(settings, timeout=0.05)
            mono = time.monotonic()
            if key:
                if not node.handle_key(key, mono):
                    break
            # 死人开关:方向键松开超过 release_timeout 秒 -> 停车
            if node.last_move_key_time is not None:
                if (mono - node.last_move_key_time) > node.release_timeout:
                    node.stop()
                    node.last_move_key_time = None
            node.publish()
    except Exception as e:
        print(f'\n异常: {e}')
    finally:
        for _ in range(5):
            node.pub.publish(Twist())
            time.sleep(0.01)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
        print('\n已退出,已发送停车指令。')


if __name__ == '__main__':
    main()
