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


def test_progressive_probe_prioritizes_target_direction_and_short_fallback():
    points = HANDOFF.progressive_probe_points(
        (0.0, 0.0), (2.0, 0.0),
        max_step=0.35, min_step=0.20,
        fan_angles_deg=(0.0, 15.0, -15.0),
        min_progress=0.12)

    assert len(points) == 6
    assert math.isclose(points[0][0], 0.35, abs_tol=1e-9)
    assert math.isclose(points[0][1], 0.0, abs_tol=1e-9)
    assert math.isclose(points[1][0], 0.20, abs_tol=1e-9)
    assert math.isclose(points[1][1], 0.0, abs_tol=1e-9)
    assert points[2][1] > 0.0
    assert points[4][1] < 0.0
    assert all(
        math.hypot(2.0 - x, -y) <= 2.0 - 0.12 + 1e-9
        for x, y in points)


def test_progressive_probe_never_overshoots_near_target():
    points = HANDOFF.progressive_probe_points(
        (0.0, 0.0), (0.18, 0.0),
        max_step=0.35, min_step=0.20,
        fan_angles_deg=(0.0,), min_progress=0.10)

    assert points == [(0.18, 0.0)]


def test_progressive_probe_rejects_sideways_points_without_progress():
    points = HANDOFF.progressive_probe_points(
        (0.0, 0.0), (2.0, 0.0),
        max_step=0.35, min_step=0.20,
        fan_angles_deg=(90.0, -90.0), min_progress=0.05)

    assert points == []


def test_progressive_probe_runtime_wiring_is_conservative():
    package_dir = Path(__file__).parents[1]
    params = (package_dir / 'config' / 'roadmap_explorer.yaml').read_text()
    mission = (
        package_dir / 'scripts' / 'roadmap_explore_mission.py').read_text()
    launch = (
        package_dir / 'launch' / 'roadmap_exploration.launch.py').read_text()

    assert 'progressive_probe_enabled: true' in params
    assert 'progressive_probe_known_ratio: 1.0' in params
    assert 'progressive_probe_max_attempts: 6' in params
    assert 'Explore.Goal.CONTINUE_FROM_TERMINATED_SESSION' in mission
    assert 'sample_stride=1' in mission
    assert "'progressive_probe_enabled'" in launch
    assert "'exploration_stall_timeout'" in launch


def test_position_arrival_has_no_heading_requirement():
    # The helper has no yaw argument by design: a car facing any direction at
    # this XY position is considered ready for Tag acquisition.
    assert HANDOFF.position_reached((1.08, 2.0), (1.0, 2.0), 0.10)
    assert not HANDOFF.position_reached((1.11, 2.0), (1.0, 2.0), 0.10)


def test_position_arrival_uses_euclidean_not_per_axis_distance():
    # Both axis errors are below 0.15 m, while hypot(0.11, 0.11) is 0.156 m.
    # A rectangular/per-axis test would incorrectly accept this pose.
    assert not HANDOFF.position_reached((0.11, 0.11), (0.0, 0.0), 0.15)
    assert HANDOFF.position_reached((0.10, 0.10), (0.0, 0.0), 0.15)


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


def test_roadmap_navigation_behavior_tree_selects_normal_goal_checker():
    roadmap_tree = (
        Path(__file__).parents[2]
        / 'roadmap-explorer'
        / 'roadmap_explorer'
        / 'xml'
        / 'explore_to_pose.xml'
    )
    root = ET.parse(roadmap_tree).getroot()
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


def test_dwb_rotate_to_goal_tolerance_matches_precise_position_checker():
    navigation_dir = Path(__file__).parents[2] / 'car_navigation'
    params = (navigation_dir / 'config' / 'nav2_params.yaml').read_text()
    follow_path = params.split('    FollowPath:', 1)[1].split(
        '\n# ==================== 全局代价地图', 1)[0]

    assert 'xy_goal_tolerance: 0.05' in follow_path
    assert 'trans_stopped_velocity: 0.02' in follow_path
