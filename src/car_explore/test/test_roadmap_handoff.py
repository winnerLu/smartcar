"""Regression tests for position-only Nav2 to visual-parking handoff."""

import importlib.util
import math
from pathlib import Path
import xml.etree.ElementTree as ET


HELPER_PATH = (
    Path(__file__).parents[1] / 'scripts' / 'roadmap_handoff.py')
SPEC = importlib.util.spec_from_file_location('roadmap_handoff', HELPER_PATH)
HANDOFF = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(HANDOFF)


def test_preparking_point_uses_start_to_target_direction():
    x, y, yaw = HANDOFF.preparking_point((1.0, 2.0), (1.0, 5.0), 0.35)

    assert math.isclose(x, 1.0, abs_tol=1e-9)
    assert math.isclose(y, 4.65, abs_tol=1e-9)
    assert math.isclose(yaw, math.pi / 2.0, abs_tol=1e-9)


def test_short_goal_never_puts_preparking_point_behind_start():
    x, y, _ = HANDOFF.preparking_point((0.0, 0.0), (0.20, 0.0), 0.35)

    assert math.isclose(x, 0.15, abs_tol=1e-9)
    assert math.isclose(y, 0.0, abs_tol=1e-9)


def test_bounded_search_is_a_five_point_snake():
    points = HANDOFF.bounded_search_points(
        (2.0, 3.0), 0.0, forward_step=0.08, lateral_step=0.12)

    assert points == [
        (2.0, 3.12),
        (2.08, 3.12),
        (2.08, 3.0),
        (2.08, 2.88),
        (2.0, 2.88),
    ]
    assert all(abs(x - 2.0) <= 0.080001 for x, _ in points)
    assert all(abs(y - 3.0) <= 0.120001 for _, y in points)


def test_position_arrival_has_no_heading_requirement():
    # The helper has no yaw argument by design: a car facing any direction at
    # this XY position is considered ready for Tag acquisition.
    assert HANDOFF.position_reached((1.08, 2.0), (1.0, 2.0), 0.10)
    assert not HANDOFF.position_reached((1.11, 2.0), (1.0, 2.0), 0.10)


def test_tag_handoff_requires_one_fresh_complete_quality_pose():
    valid = dict(
        visible_count=1,
        reprojection_error=0.8,
        inlier_count=4,
        sample_ages=(0.05, 0.04, 0.03, 0.02),
        max_age=0.40,
        max_reprojection_error=3.0,
        min_inlier_points=4,
    )

    assert HANDOFF.tag_observation_valid(**valid)
    assert not HANDOFF.tag_observation_valid(
        **{**valid, 'visible_count': 0})
    assert not HANDOFF.tag_observation_valid(
        **{**valid, 'reprojection_error': math.inf})
    assert not HANDOFF.tag_observation_valid(
        **{**valid, 'inlier_count': 3})
    assert not HANDOFF.tag_observation_valid(
        **{**valid, 'sample_ages': (0.05, 0.41, 0.03, 0.02)})


def test_mission_behavior_tree_selects_position_only_goal_checker():
    navigation_dir = Path(__file__).parents[2] / 'car_navigation'
    tree_path = (
        navigation_dir / 'behavior_trees' /
        'navigate_to_pose_position_only.xml')

    tree = ET.parse(tree_path)
    follow_path = tree.find('.//FollowPath')

    assert follow_path is not None
    assert follow_path.attrib['goal_checker_id'] == 'position_goal_checker'


def test_normal_navigation_behavior_tree_selects_normal_goal_checker():
    navigation_dir = Path(__file__).parents[2] / 'car_navigation'
    root = ET.parse(
        navigation_dir
        / 'behavior_trees'
        / 'navigate_to_pose_safe_recovery.xml'
    ).getroot()
    follow_path = root.find('.//FollowPath')
    assert follow_path is not None
    assert follow_path.attrib['goal_checker_id'] == 'goal_checker'


def test_position_goal_checker_keeps_normal_nav_checker_unchanged():
    navigation_dir = Path(__file__).parents[2] / 'car_navigation'
    params = (navigation_dir / 'config' / 'nav2_params.yaml').read_text()

    assert 'goal_checker_plugins: ["goal_checker", "position_goal_checker"]' in params
    assert 'position_goal_checker:' in params
    assert 'yaw_goal_tolerance: 6.283185' in params
    assert 'yaw_goal_tolerance: 0.20' in params
