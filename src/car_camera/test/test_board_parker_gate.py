"""Regression test for parking velocity ownership."""

import importlib.util
from pathlib import Path
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import SetBool


SCRIPT = Path(__file__).parents[1] / 'scripts' / 'board_parker.py'
SPEC = importlib.util.spec_from_file_location('board_parker', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def spin_for(executor, duration):
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)


def set_enabled(parker, enabled):
    request = SetBool.Request()
    request.data = enabled
    response = SetBool.Response()
    return parker.on_set_enabled(request, response)


def test_inactive_parker_does_not_hold_twist_mux_input():
    rclpy.init()
    parker = MODULE.BoardParker()
    observer = Node('board_parker_gate_test')
    received = []
    observer.create_subscription(
        Twist, '/cmd_vel_dock', lambda msg: received.append(msg), 10)

    executor = SingleThreadedExecutor()
    executor.add_node(parker)
    executor.add_node(observer)
    try:
        # Allow DDS discovery and several control timer periods.  An inactive
        # controller must remain completely silent on the dock velocity input.
        spin_for(executor, 0.20)
        assert received == []

        response = set_enabled(parker, True)
        assert response.success
        spin_for(executor, 0.12)
        assert received

        response = set_enabled(parker, False)
        assert response.success
        spin_for(executor, 0.10)
        received.clear()
        spin_for(executor, 0.15)
        assert received == []
    finally:
        executor.remove_node(observer)
        executor.remove_node(parker)
        observer.destroy_node()
        parker.destroy_node()
        rclpy.shutdown()
