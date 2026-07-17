#!/usr/bin/env python3
"""简单键盘遥控节点:发布 /cmd_vel 控制差速小车。

针对本项目定制,替代 teleop_twist_keyboard:
- 固定档位速度,带硬上限(调速不会超过 max,从根上防止速度溢出)
- 按键即走,松手无输入自动停(依赖 car_base 的超时保护),空格立即停
- 只用 linear.x(前进/后退)和 angular.z(转向),契合差速车

按键:
  w / s : 前进 / 后退
  a / d : 左转 / 右转
  空格 或 x : 停止
  + / - : 增大 / 减小线速度档位(受 max_linear 限幅)
  [ / ] : 减小 / 增大角速度档位(受 max_angular 限幅)
  q / Ctrl-C : 退出(退出前发零速)

参数(ros2 run ... --ros-args -p linear_step:=0.05 形式覆盖):
  linear_speed   初始线速度档位 (m/s), 默认 0.15
  angular_speed  初始角速度档位 (rad/s), 默认 0.5
  max_linear     线速度硬上限 (m/s), 默认 0.4
  max_angular    角速度硬上限 (rad/s), 默认 1.5
  step           每次调速增量, 默认 0.05
"""

import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

HELP = """
============ 键盘遥控 ============
  w/s : 前进/后退      a/d : 左转/右转
  空格 或 x : 停止
  +/- : 线速度档位 增/减
  [/] : 角速度档位 减/增
  q   : 退出
=================================="""


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.linear_speed = self.declare_parameter('linear_speed', 0.15).value
        self.angular_speed = self.declare_parameter('angular_speed', 0.5).value
        self.max_linear = self.declare_parameter('max_linear', 0.4).value
        self.max_angular = self.declare_parameter('max_angular', 1.5).value
        self.step = self.declare_parameter('step', 0.05).value

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # 20Hz 持续发布当前指令(配合 car_base 超时保护)
        self.vx = 0.0
        self.wz = 0.0
        self.timer = self.create_timer(0.05, self._publish)

    def _clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    def _publish(self):
        t = Twist()
        t.linear.x = self.vx
        t.angular.z = self.wz
        self.pub.publish(t)

    def handle_key(self, key):
        """返回 False 表示要退出。"""
        if key in ('w', 'W'):
            self.vx = self.linear_speed
        elif key in ('s', 'S'):
            self.vx = -self.linear_speed
        elif key in ('a', 'A'):
            self.wz = self.angular_speed
        elif key in ('d', 'D'):
            self.wz = -self.angular_speed
        elif key in (' ', 'x', 'X'):
            self.vx = 0.0
            self.wz = 0.0
        elif key == '+':
            self.linear_speed = self._clamp(
                self.linear_speed + self.step, 0.0, self.max_linear)
            self._print_speed()
        elif key == '-':
            self.linear_speed = self._clamp(
                self.linear_speed - self.step, 0.0, self.max_linear)
            self._print_speed()
        elif key == ']':
            self.angular_speed = self._clamp(
                self.angular_speed + self.step, 0.0, self.max_angular)
            self._print_speed()
        elif key == '[':
            self.angular_speed = self._clamp(
                self.angular_speed - self.step, 0.0, self.max_angular)
            self._print_speed()
        elif key in ('q', 'Q', '\x03'):   # q 或 Ctrl-C
            return False
        return True

    def _print_speed(self):
        print(f'\r档位: 线速度={self.linear_speed:.2f} m/s  '
              f'角速度={self.angular_speed:.2f} rad/s   ', end='', flush=True)


def get_key(settings, timeout=0.1):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    rclpy.init()
    node = KeyboardTeleop()
    print(HELP)
    node._print_speed()
    settings = termios.tcgetattr(sys.stdin)
    try:
        while rclpy.ok():
            key = get_key(settings)
            if key:
                if not node.handle_key(key):
                    break
            # 处理定时器回调(发布)
            rclpy.spin_once(node, timeout_sec=0.0)
    except Exception as e:
        print(f'\n异常: {e}')
    finally:
        # 退出前发几帧零速,确保停车
        for _ in range(5):
            node.pub.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
        print('\n已退出,已发送停车指令。')


if __name__ == '__main__':
    main()
