"""Regression test for parking velocity ownership."""

import importlib.util
import math
from pathlib import Path
import time

from geometry_msgs.msg import Pose, Transform, Twist
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


def test_calibrated_camera_transform_recovers_base_frame_parking_target():
    camera_pose = Pose()
    camera_pose.position.x = -0.028429949941281923
    camera_pose.position.y = 0.005243128198284948
    camera_pose.position.z = 0.5783112928477445
    camera_pose.orientation.x = -0.003939055113442152
    camera_pose.orientation.y = -0.020508385656840416
    camera_pose.orientation.z = -0.0009665695387912702
    camera_pose.orientation.w = 0.9997814539717619

    base_from_camera = Transform()
    base_from_camera.translation.x = 0.082651338
    base_from_camera.translation.y = -0.004695587
    base_from_camera.translation.z = 0.579014016
    base_from_camera.rotation.x = 0.706268778
    base_from_camera.rotation.y = -0.707635714
    base_from_camera.rotation.z = 0.017286951
    base_from_camera.rotation.w = 0.011716286

    position, orientation = MODULE.transform_pose(
        camera_pose, base_from_camera)
    assert abs(position[0] - 0.082) < 1e-6
    assert abs(position[1]) < 1e-6
    assert abs(position[2]) < 1e-6
    assert abs(MODULE.nearest_edge_error(orientation)) < 1e-6


def test_measured_footprint_must_remain_inside_printed_board():
    board_orientation = (math.sqrt(0.5), -math.sqrt(0.5), 0.0, 0.0)
    footprint = [
        (0.197, 0.093), (0.197, -0.093),
        (-0.033, -0.093), (-0.033, 0.093),
    ]
    assert MODULE.footprint_inside_board(
        (0.082, 0.0, 0.0), board_orientation, footprint,
        0.2881, 0.2910, 0.005)
    assert not MODULE.footprint_inside_board(
        (0.122, 0.0, 0.0), board_orientation, footprint,
        0.2881, 0.2910, 0.005)


def test_relaxed_completion_uses_footprint_overlap_not_perfect_containment():
    board_orientation = (math.sqrt(0.5), -math.sqrt(0.5), 0.0, 0.0)
    footprint = [
        (0.197, 0.093), (0.197, -0.093),
        (-0.033, -0.093), (-0.033, 0.093),
    ]
    centred = MODULE.footprint_overlap_ratio(
        (0.082, 0.0, 0.0), board_orientation, footprint,
        0.2881, 0.2910, 0.005)
    slightly_offset = MODULE.footprint_overlap_ratio(
        (0.107, 0.015, 0.0), board_orientation, footprint,
        0.2881, 0.2910, 0.005)
    mostly_outside = MODULE.footprint_overlap_ratio(
        (0.25, 0.0, 0.0), board_orientation, footprint,
        0.2881, 0.2910, 0.005)
    assert centred > 0.99
    assert slightly_offset >= 0.90
    assert mostly_outside < 0.50


def test_oblique_board_generates_a_forward_arc():
    linear, angular = MODULE.compute_parking_command(
        0.15, 0.08, math.radians(8.0),
        0.05, 0.30, 0.018, 0.10,
        0.8, 1.8, 0.8, True)
    assert 0.018 <= linear <= 0.05
    assert 0.0 < angular <= 0.30


def test_side_board_turns_before_translating():
    linear, angular = MODULE.compute_parking_command(
        0.01, -0.15, 0.0,
        0.05, 0.30, 0.018, 0.10,
        0.8, 1.8, 0.8, True)
    assert linear == 0.0
    assert angular < 0.0


def test_board_behind_uses_bounded_reverse_recovery():
    linear, angular = MODULE.compute_parking_command(
        -0.10, 0.02, 0.0,
        0.05, 0.30, 0.018, 0.10,
        0.8, 1.8, 0.8, True)
    assert -0.05 <= linear < 0.0
    assert angular < 0.0


def test_visual_hold_is_bounded_by_time_and_odometry_distance():
    assert MODULE.visual_hold_allowed(2.0, 0.04, 6.0, 0.12)
    assert MODULE.visual_hold_allowed(6.0, 0.12, 6.0, 0.12)
    assert not MODULE.visual_hold_allowed(6.01, 0.04, 6.0, 0.12)
    assert not MODULE.visual_hold_allowed(2.0, 0.121, 6.0, 0.12)
    assert not MODULE.visual_hold_allowed(-0.1, 0.0, 6.0, 0.12)
