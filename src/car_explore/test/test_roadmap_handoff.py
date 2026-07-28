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


def test_startup_escape_progress_is_measured_along_original_heading():
    progress, lateral, heading_error = HANDOFF.startup_escape_metrics(
        (1.0, 2.0, math.pi / 2.0),
        (1.0, 2.20, math.pi / 2.0 + math.radians(2.0)))

    assert math.isclose(progress, 0.20, abs_tol=1e-9)
    assert math.isclose(lateral, 0.0, abs_tol=1e-9)
    assert math.isclose(
        heading_error, math.radians(-2.0), abs_tol=1e-9)


def test_startup_escape_guard_only_blocks_the_forward_swept_corridor():
    points = [
        (-0.05, 0.00),  # behind the base
        (0.20, 0.14),   # just outside the swept half-width
        (0.42, 0.00),   # beyond the current remaining sweep
    ]
    assert HANDOFF.startup_escape_blocking_point(
        points, remaining_distance=0.10, footprint_front=0.197,
        braking_margin=0.10, half_width=0.13) is None

    points.extend([(0.30, 0.08), (0.12, -0.05)])
    assert HANDOFF.startup_escape_blocking_point(
        points, remaining_distance=0.10, footprint_front=0.197,
        braking_margin=0.10, half_width=0.13) == (0.12, -0.05)


def test_startup_escape_is_one_time_and_has_dedicated_mux_ownership():
    package = Path(__file__).parents[1]
    mission = (
        package / 'scripts' / 'roadmap_explore_mission.py'
    ).read_text()
    launch = (
        package / 'launch' / 'roadmap_exploration.launch.py'
    ).read_text()
    params = (
        package / 'config' / 'roadmap_explorer.yaml'
    ).read_text()
    mux = (
        package.parent / 'car_navigation' / 'config' / 'twist_mux.yaml'
    ).read_text()

    start = mission.split(
        '    def _try_start', 1)[1].split(
        '    def _begin_planning_after_startup', 1)[0]
    assert start.index('self.target_pose = self._make_pose') < start.index(
        "self.state = 'STARTUP_ESCAPE'")
    assert "'startup_escape_cmd_topic': '/cmd_vel_escape'" in mission
    assert "'startup_escape_odom_frame': 'odom'" in mission
    assert 'self.create_timer(\n            0.10' in mission
    assert 'startup_escape_blocking_point(' in mission
    assert 'self._publish_startup_escape_stop()' in mission.split(
        '    def _abort_mission', 1)[1]
    assert "self.state = 'STARTUP_ESCAPE_SETTLE'" in mission
    assert "startup_escape_enabled', default_value='true'" in launch
    assert "startup_escape_distance', default_value='0.20'" in launch
    assert "startup_escape_speed', default_value='0.08'" in launch
    assert 'startup_escape_enabled: true' in params
    assert 'startup_escape_distance: 0.20' in params
    assert 'startup_escape_speed: 0.08' in params
    assert 'startup_escape:' in mux
    assert 'topic: cmd_vel_escape' in mux
    assert 'priority: 75' in mux


def test_breadcrumbs_sample_motion_and_bound_history():
    history = []

    assert HANDOFF.append_breadcrumb(history, (0.0, 0.0), 0.25, 3)
    assert not HANDOFF.append_breadcrumb(history, (0.10, 0.0), 0.25, 3)
    assert HANDOFF.append_breadcrumb(history, (0.25, 0.0), 0.25, 3)
    assert HANDOFF.append_breadcrumb(history, (0.50, 0.0), 0.25, 3)
    assert HANDOFF.append_breadcrumb(history, (0.75, 0.0), 0.25, 3)
    assert history == [(0.25, 0.0), (0.50, 0.0), (0.75, 0.0)]


def test_backtrack_uses_travelled_route_and_prefers_far_safe_exit():
    # The final two points enter a side pocket. Recovery must follow history
    # backwards, even though that temporarily moves away from the final goal.
    history = [
        (0.0, 0.0), (0.3, 0.0), (0.6, 0.0),
        (0.6, 0.3), (0.6, 0.6), (0.6, 0.9),
    ]

    points = HANDOFF.breadcrumb_backtrack_points(
        history, current=(0.6, 1.0),
        min_distance=0.55, max_distance=1.05,
        candidate_spacing=0.20)

    assert points
    assert points[0] == (0.6, 0.0)
    assert points[-1] == (0.6, 0.3)


def test_backtrack_does_not_invent_unknown_route_points():
    points = HANDOFF.breadcrumb_backtrack_points(
        [(0.0, 0.0), (0.25, 0.0)], current=(0.30, 0.0),
        min_distance=0.60, max_distance=1.20,
        candidate_spacing=0.20)

    assert points == []


def test_preparking_point_follows_reachable_path_tail_after_a_detour():
    # The start-target straight line points east, but the reachable path
    # approaches from the south.  Pre-parking must therefore be south of the
    # board rather than west of it.
    result = HANDOFF.path_standoff_point(
        [(0.0, 0.0), (0.5, 0.0), (0.5, -2.0),
         (3.55, -2.0), (3.55, 0.0)],
        0.35)

    assert result is not None
    x, y, yaw = result
    assert math.isclose(x, 3.55, abs_tol=1e-9)
    assert math.isclose(y, -0.35, abs_tol=1e-9)
    assert math.isclose(yaw, math.pi / 2.0, abs_tol=1e-9)


def test_short_path_uses_current_position_without_inventing_backtrack():
    result = HANDOFF.path_standoff_point(
        [(1.0, 2.0), (1.20, 2.0)], 0.35)

    assert result is not None
    x, y, yaw = result
    assert math.isclose(x, 1.0, abs_tol=1e-9)
    assert math.isclose(y, 2.0, abs_tol=1e-9)
    assert math.isclose(yaw, 0.0, abs_tol=1e-9)


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


def test_target_reveal_moves_toward_target_and_prioritizes_centerline():
    points = HANDOFF.target_reveal_points(
        preparking=(3.20, 0.0),
        target=(3.55, 0.0),
        forward_offsets=(0.10, 0.15, 0.20),
        lateral_offsets=(0.0, 0.08, -0.08),
        min_target_standoff=0.12)

    assert all(
        math.isclose(point[1], 0.0, abs_tol=1e-9)
        for point in points[:3])
    assert all(
        math.isclose(point[0], expected, abs_tol=1e-9)
        for point, expected in zip(points[:3], (3.30, 3.35, 3.40)))
    assert all(x > 3.20 for x, _ in points)
    assert all(x <= 3.43 + 1e-9 for x, _ in points)


def test_target_reveal_clamps_short_approach_without_overshoot():
    points = HANDOFF.target_reveal_points(
        preparking=(0.0, 0.0),
        target=(0.18, 0.0),
        forward_offsets=(0.10, 0.15, 0.20),
        lateral_offsets=(0.0,),
        min_target_standoff=0.12)

    assert len(points) == 1
    assert math.isclose(points[0][0], 0.06, abs_tol=1e-9)
    assert math.isclose(points[0][1], 0.0, abs_tol=1e-9)


def test_target_local_reveal_precedes_breadcrumb_and_stays_known_safe():
    package_dir = Path(__file__).parents[1]
    params = (package_dir / 'config' / 'roadmap_explorer.yaml').read_text()
    mission = (
        package_dir / 'scripts' / 'roadmap_explore_mission.py').read_text()
    launch = (
        package_dir / 'launch' / 'roadmap_exploration.launch.py').read_text()

    assert 'target_local_reveal_enabled: true' in params
    assert 'target_local_reveal_known_ratio: 1.0' in params
    assert 'target_local_reveal_max_attempts: 3' in params
    assert "'target_local_reveal_enabled': True" in mission
    assert "'reveal': 'SENDING_TARGET_REVEAL_NAVIGATION'" in mission
    assert 'self._clearance_status(msg, approach_cell) != \'unknown\'' in mission
    assert 'self._known_free_line(msg, approach_cell, target_cell)' in mission

    recovery = mission.split(
        '    def _prepare_stall_recovery', 1)[1].split(
        '    def _send_next_backtrack_candidate', 1)[0]
    assert (
        recovery.index('self._prepare_target_local_reveal()') <
        recovery.index('breadcrumb_backtrack_points(')
    )
    reveal_plan = mission.split(
        '    def _target_reveal_plan_result', 1)[1].split(
        '    def _complete_target_local_reveal', 1)[0]
    assert 'sample_stride=1' in reveal_plan
    assert 'self.target_local_reveal_known_ratio' in reveal_plan
    assert "'target_local_reveal_enabled'" in launch
    assert "'target_local_reveal_max_attempts'" in launch


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
    assert 'deadend_backtrack_enabled: true' in params
    assert 'deadend_backtrack_max_distance: 1.20' in params
    assert 'Explore.Goal.CONTINUE_FROM_TERMINATED_SESSION' in mission
    assert "'backtrack': 'SENDING_BACKTRACK_NAVIGATION'" in mission
    assert 'sample_stride=1' in mission
    assert "'progressive_probe_enabled'" in launch
    assert "'exploration_stall_timeout'" in launch


def test_roadmap_failure_reselection_has_single_owner():
    workspace_src = Path(__file__).parents[2]
    params = (
        workspace_src / 'car_explore' / 'config' /
        'roadmap_explorer.yaml'
    ).read_text()
    mission = (
        workspace_src / 'car_explore' / 'scripts' /
        'roadmap_explore_mission.py'
    ).read_text()
    launch = (
        workspace_src / 'car_explore' / 'launch' /
        'roadmap_exploration.launch.py'
    ).read_text()
    exploration_tree = ET.parse(
        workspace_src / 'roadmap-explorer' / 'roadmap_explorer' /
        'xml' / 'exploration.xml'
    ).getroot()
    navigation_tree = ET.parse(
        workspace_src / 'car_navigation' / 'behavior_trees' /
        'navigate_to_pose_roadmap.xml'
    ).getroot()

    retry = exploration_tree.find('.//RetryUntilSuccessful')
    assert retry is not None
    assert retry.attrib['num_attempts'] == '1'
    assert exploration_tree.find('.//BlacklistGoal') is not None

    recovery = navigation_tree.find('.//RecoveryNode')
    assert recovery is not None
    assert recovery.attrib['number_of_retries'] == '1'
    assert navigation_tree.find('.//BackUp') is None
    assert navigation_tree.find('.//Spin') is None
    assert navigation_tree.find('.//ClearEntireCostmap') is not None

    assert 'bt_sleep_ms: 250' in params
    assert 'exploration_stall_timeout: 40.0' in params
    assert "'exploration_stall_timeout': 40.0" in mission
    assert "'exploration_stall_timeout', default_value='40.0'" in launch
    assert "'explorationBT.nav2_bt_xml': os.path.join(" in launch
    assert "'navigate_to_pose_roadmap.xml'" in launch


def test_roadmap_and_nav2_share_narrow_passage_map():
    workspace_src = Path(__file__).parents[2]
    roadmap_params = (
        workspace_src / 'car_explore' / 'config' /
        'roadmap_explorer.yaml'
    ).read_text()
    nav_params = (
        workspace_src / 'car_navigation' / 'config' /
        'nav2_params.yaml'
    ).read_text()
    slam_launch = (
        workspace_src / 'car_navigation' / 'launch' /
        'slam_navigation.launch.py'
    ).read_text()

    assert 'map_topic: "/map_clearance"' in roadmap_params
    assert 'map_topic: /map_clearance' in nav_params
    assert "'clearance_radius', default_value='0.15'" in slam_launch
    assert 'startup_escape_radius' not in slam_launch
    assert '窄通道闭合半径' in slam_launch
    assert "'output_map_topic': '/map_clearance'" in slam_launch
    assert 'inflation_radius: 0.15' in nav_params
    assert 'cost_scaling_factor: 3.0' in nav_params
    assert 'ObstacleFootprint.scale: 0.05' in nav_params
    assert 'BaseObstacle.scale' not in nav_params
    assert 'footprint_padding: 0.04' in nav_params
    assert 'observation_persistence: 0.5' in nav_params
    assert 'inflation_radius: 0.08' in roadmap_params


def test_initial_known_path_skips_roadmap_before_nav2_handoff():
    mission = (
        Path(__file__).parents[1]
        / 'scripts'
        / 'roadmap_explore_mission.py'
    ).read_text()

    start_block = mission.split(
        '    def _check_initial_direct_path', 1)[0].rsplit(
        '    def _try_start', 1)[1]
    assert 'self._check_initial_direct_path()' in start_block
    assert 'self._send_exploration_goal(new_session=True)' in start_block
    assert "'CHECKING_INITIAL_PATH'" in mission
    assert 'skipping Roadmap' in mission
    assert 'self._send_final_goal()' in mission


def test_dynamic_preparking_is_derived_from_target_path_not_start_heading():
    package = Path(__file__).parents[1]
    mission = (
        package / 'scripts' / 'roadmap_explore_mission.py'
    ).read_text()
    helper = (
        package / 'scripts' / 'roadmap_handoff.py'
    ).read_text()
    params = (
        package / 'config' / 'roadmap_explorer.yaml'
    ).read_text()

    start = mission.split(
        '    def _try_start', 1)[1].split(
        '    def _check_initial_direct_path', 1)[0]
    initial = mission.split(
        '    def _check_initial_direct_path', 1)[1].split(
        '    def _initial_plan_goal_response', 1)[0]
    periodic = mission.split(
        '    def _request_direct_path', 1)[1].split(
        '    def _plan_goal_response', 1)[0]
    derive = mission.split(
        '    def _dynamic_preparking_from_path', 1)[1].split(
        '    def _path_tail_known_free', 1)[0]

    assert 'self.preparking_pose = None' in start
    assert 'preparking_point(' not in mission
    assert 'preparking_point(' not in helper
    assert 'goal.goal = self.initial_plan_candidate' in initial
    assert 'goal.goal = self.direct_plan_candidate' in periodic
    assert 'path_standoff_point(points, self.preparking_distance)' in derive
    assert 'self._path_tail_known_free(path, candidate)' in derive
    assert "clearance != 'safe'" in derive
    assert 'self.preparking_pub.publish(candidate)' in derive
    assert 'preparking_candidate_radius' not in params
    assert "'preparking_distance': 0.10" in mission
    assert "preparking_distance', default_value='0.10'" in (
        package / 'launch' / 'roadmap_exploration.launch.py').read_text()
    assert 'preparking_distance: 0.10' in params


def test_goal_radius_searches_known_safe_reachable_target_candidates():
    package = Path(__file__).parents[1]
    mission = (
        package / 'scripts' / 'roadmap_explore_mission.py'
    ).read_text()
    launch = (
        package / 'launch' / 'roadmap_exploration.launch.py'
    ).read_text()
    params = (
        package / 'config' / 'roadmap_explorer.yaml'
    ).read_text()
    launcher = (
        package.parents[1] / 'start_roadmap_mission.sh'
    ).read_text()

    candidate_search = mission.split(
        '    def _known_safe_target_candidates', 1)[1].split(
        '    def _target_candidate_description', 1)[0]
    periodic = mission.split(
        '    def _request_direct_path', 1)[1].split(
        '    def _cancel_exploration_for_final_goal', 1)[0]

    assert "'goal_radius': 0.15" in mission
    assert "goal_radius', default_value='0.15'" in launch
    assert 'goal_radius: 0.15' in params
    assert 'GOAL_RADIUS="${GOAL_RADIUS:-0.15}"' in launcher
    assert 'cell_radius = math.ceil(radius / msg.info.resolution)' in (
        candidate_search)
    assert 'distance > radius + 1e-9' in candidate_search
    assert 'self._is_free(msg, mx, my)' in candidate_search
    assert 'self._safe_endpoint(msg, cell)' in candidate_search
    assert 'self._is_free(msg, target_cell[0], target_cell[1])' in (
        candidate_search)
    assert 'safe_cells.sort(' in candidate_search
    assert 'goal.goal = self.initial_plan_candidate' in mission
    assert 'goal.goal = self.direct_plan_candidate' in periodic
    assert 'self._try_next_direct_plan()' in periodic
    assert 'self._target_candidate_description(selected)' in periodic


def test_exploration_tag_handoff_precedes_stall_backtracking():
    mission = (
        Path(__file__).parents[1]
        / 'scripts'
        / 'roadmap_explore_mission.py'
    ).read_text()

    exploring_block = mission.split(
        "        if self.state == 'EXPLORING':", 1)[1].split(
        "        if self.state in (", 1)[0]
    assert (
        exploring_block.index('self._try_exploration_tag_handoff(now)') <
        exploring_block.index('self._exploration_stalled(now)')
    )

    handoff_block = mission.split(
        '    def _try_exploration_tag_handoff', 1)[1].split(
        '    def _cancel_exploration_for_tag', 1)[0]
    assert 'self.tag_handoff_max_target_distance' in handoff_block
    assert 'self._tag_confirmed(now)' in handoff_block
    assert 'return True' in handoff_block
    assert 'self._known_safe_target_pose()' not in handoff_block


def test_exploration_tag_handoff_waits_for_roadmap_nav2_to_stop():
    mission = (
        Path(__file__).parents[1]
        / 'scripts'
        / 'roadmap_explore_mission.py'
    ).read_text()

    assert "'CANCELING_EXPLORATION_FOR_TAG'" in mission
    result_block = mission.split(
        '    def _explore_result', 1)[1].split(
        '    def _request_direct_path', 1)[0]
    tag_result = result_block.split(
        "if self.state == 'CANCELING_EXPLORATION_FOR_TAG':", 1)[1].split(
        "if self.state == 'CANCELING_EXPLORATION_FOR_PROBE':", 1)[0]
    assert 'GoalStatus.STATUS_CANCELED' in tag_result
    assert 'self._start_tag_acquisition(after_search=False)' in tag_result
    assert 'self._activate_visual_parking()' not in tag_result


def test_camera_capture_sleeps_during_far_exploration_and_warms_near_target():
    package = Path(__file__).parents[1]
    mission = (
        package / 'scripts' / 'roadmap_explore_mission.py'
    ).read_text()
    launch = (
        package / 'launch' / 'roadmap_exploration.launch.py'
    ).read_text()

    assert "'camera_capture_service': '/camera/set_enabled'" in mission
    assert "'camera_warmup_target_distance': 1.0" in mission
    warmup = mission.split(
        '    def _maybe_warm_camera_near_target', 1)[1].split(
        '    def _request_camera_activation', 1)[0]
    assert 'math.hypot(' in warmup
    assert 'distance > self.camera_warmup_target_distance' in warmup
    assert 'self._request_camera_activation()' in warmup
    assert "self.state = 'ACTIVATING_CAMERA'" in mission
    assert "request.data = True" in mission
    acquisition = mission.split(
        '    def _start_tag_acquisition', 1)[1].split(
        '    def _begin_limited_search', 1)[0]
    assert 'self._request_camera_activation()' in acquisition
    assert 'self.camera_client.call_async(request)' in mission
    callback = acquisition.split(
        '    def _camera_enable_done', 1)[1].split(
        '    def _begin_tag_acquisition', 1)[0]
    assert 'if not response.success:' in callback
    assert 'self._begin_tag_acquisition(' in callback
    assert "'camera_enabled': 'false'" in launch
    assert "'camera_warmup_target_distance', default_value='1.0'" in launch


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


def test_roadmap_navigation_ignores_arbitrary_frontier_heading():
    roadmap_tree = (
        Path(__file__).parents[2]
        / 'car_navigation'
        / 'behavior_trees'
        / 'navigate_to_pose_roadmap.xml'
    )
    root = ET.parse(roadmap_tree).getroot()
    follow_path = root.find('.//FollowPath')
    assert follow_path is not None
    assert follow_path.attrib['goal_checker_id'] == 'roadmap_goal_checker'


def test_position_goal_checker_keeps_normal_nav_checker_unchanged():
    navigation_dir = Path(__file__).parents[2] / 'car_navigation'
    params = (navigation_dir / 'config' / 'nav2_params.yaml').read_text()

    assert (
        'goal_checker_plugins: ["goal_checker", "roadmap_goal_checker", '
        '"position_goal_checker"]'
    ) in params
    assert 'roadmap_goal_checker:' in params
    assert 'position_goal_checker:' in params
    roadmap_checker = params.split(
        '    roadmap_goal_checker:', 1)[1].split(
        '    position_goal_checker:', 1)[0]
    assert 'xy_goal_tolerance: 0.15' in roadmap_checker
    assert 'yaw_goal_tolerance: 6.283185' in roadmap_checker
    assert 'yaw_goal_tolerance: 6.283185' in params
    assert 'yaw_goal_tolerance: 0.20' in params


def test_dwb_rotate_to_goal_tolerance_matches_precise_position_checker():
    navigation_dir = Path(__file__).parents[2] / 'car_navigation'
    params = (navigation_dir / 'config' / 'nav2_params.yaml').read_text()
    follow_path = params.split('    FollowPath:', 1)[1].split(
        '\n# ==================== 全局代价地图', 1)[0]

    assert 'xy_goal_tolerance: 0.05' in follow_path
    assert 'trans_stopped_velocity: 0.02' in follow_path


def test_dwb_and_velocity_smoother_share_022_forward_limit():
    navigation_dir = Path(__file__).parents[2] / 'car_navigation'
    params = (navigation_dir / 'config' / 'nav2_params.yaml').read_text()
    follow_path = params.split('    FollowPath:', 1)[1].split(
        '\n# ==================== 全局代价地图', 1)[0]
    smoother = params.split('velocity_smoother:', 1)[1]

    assert 'max_vel_x: 0.22' in follow_path
    assert 'max_speed_xy: 0.22' in follow_path
    assert 'max_velocity: [0.22, 0.0, 1.2]' in smoother
