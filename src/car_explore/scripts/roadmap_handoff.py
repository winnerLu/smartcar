"""Pure geometry and perception gates for Roadmap-to-parking handoff."""

import math
from typing import List, Optional, Sequence, Tuple


Point = Tuple[float, float]


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def startup_escape_metrics(
        start: Tuple[float, float, float],
        current: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """Return forward progress, lateral drift, and heading error from start."""
    dx = float(current[0]) - float(start[0])
    dy = float(current[1]) - float(start[1])
    yaw = float(start[2])
    return (
        math.cos(yaw) * dx + math.sin(yaw) * dy,
        -math.sin(yaw) * dx + math.cos(yaw) * dy,
        wrap_angle(float(start[2]) - float(current[2])),
    )


def startup_escape_blocking_point(
        points: Sequence[Point], remaining_distance: float,
        footprint_front: float, braking_margin: float,
        half_width: float) -> Optional[Point]:
    """Return the nearest scan point inside the remaining forward sweep."""
    maximum_x = (
        max(0.0, float(footprint_front)) +
        max(0.0, float(remaining_distance)) +
        max(0.0, float(braking_margin)))
    bounded_half_width = max(0.0, float(half_width))
    blocked = [
        (float(x), float(y))
        for x, y in points
        if 0.0 <= float(x) <= maximum_x and
        abs(float(y)) <= bounded_half_width
    ]
    if not blocked:
        return None
    return min(blocked, key=lambda point: point[0])


def append_breadcrumb(
        history: List[Point], point: Point, min_spacing: float,
        max_points: int) -> bool:
    """Append a travelled point when it is far enough from the last sample."""
    bounded_max = max(2, int(max_points))
    spacing = max(0.0, float(min_spacing))
    if history and math.hypot(
            point[0] - history[-1][0],
            point[1] - history[-1][1]) + 1e-9 < spacing:
        return False
    history.append((float(point[0]), float(point[1])))
    if len(history) > bounded_max:
        del history[:len(history) - bounded_max]
    return True


def breadcrumb_backtrack_points(
        history: Sequence[Point], current: Point,
        min_distance: float, max_distance: float,
        candidate_spacing: float) -> List[Point]:
    """
    Select previously travelled points for dead-end recovery.

    Distance is measured along the recorded trajectory rather than directly
    toward the mission target. Candidates are returned farthest first so the
    first attempt is likely to clear a pocket instead of stopping inside it.
    """
    if not history:
        return []

    minimum = max(0.0, float(min_distance))
    maximum = max(minimum, float(max_distance))
    spacing = max(0.01, float(candidate_spacing))
    walked = 0.0
    previous = (float(current[0]), float(current[1]))
    selected: List[Tuple[float, Point]] = []
    last_selected_distance = -math.inf

    for point in reversed(history):
        candidate = (float(point[0]), float(point[1]))
        walked += math.hypot(
            candidate[0] - previous[0], candidate[1] - previous[1])
        previous = candidate
        if walked + 1e-9 < minimum:
            continue
        if walked > maximum + 1e-9:
            break
        if walked - last_selected_distance + 1e-9 < spacing:
            continue
        selected.append((walked, candidate))
        last_selected_distance = walked

    selected.sort(key=lambda item: item[0], reverse=True)
    return [point for _, point in selected]


def path_standoff_point(
        path: Sequence[Point],
        standoff: float) -> Optional[Tuple[float, float, float]]:
    """
    Return a point ``standoff`` metres before the end of a reachable path.

    Distance is accumulated backwards along the path polyline, not along the
    start-to-target straight line.  The returned yaw follows the final path
    segment toward the target.  If the whole path is shorter than the requested
    standoff, its first point is returned instead of inventing a point behind
    the robot.
    """
    if not path:
        return None
    points = [(float(point[0]), float(point[1])) for point in path]
    if len(points) == 1:
        return points[0][0], points[0][1], 0.0

    remaining = max(0.0, float(standoff))
    last_yaw = 0.0
    for index in range(len(points) - 1, 0, -1):
        previous = points[index - 1]
        current = points[index]
        dx = current[0] - previous[0]
        dy = current[1] - previous[1]
        segment = math.hypot(dx, dy)
        if segment <= 1e-9:
            continue
        last_yaw = math.atan2(dy, dx)
        if remaining <= segment + 1e-9:
            fraction = min(1.0, remaining / segment)
            return (
                current[0] - fraction * dx,
                current[1] - fraction * dy,
                last_yaw,
            )
        remaining -= segment

    # The robot is already inside the requested visual-handoff distance.
    first_yaw = last_yaw
    for first, second in zip(points, points[1:]):
        dx = second[0] - first[0]
        dy = second[1] - first[1]
        if math.hypot(dx, dy) > 1e-9:
            first_yaw = math.atan2(dy, dx)
            break
    return points[0][0], points[0][1], first_yaw


def bounded_search_points(
        origin: Point, approach_yaw: float,
        forward_step: float, lateral_step: float) -> List[Point]:
    """
    Return a short snake around the pre-parking point.

    The order avoids a blind expanding spiral and keeps every candidate inside
    one forward and one lateral step from the original safe Nav2 endpoint.
    """
    forward = (math.cos(approach_yaw), math.sin(approach_yaw))
    left = (-forward[1], forward[0])
    offsets = (
        (0.0, lateral_step),
        (forward_step, lateral_step),
        (forward_step, 0.0),
        (forward_step, -lateral_step),
        (0.0, -lateral_step),
    )
    return [
        (
            origin[0] + along * forward[0] + lateral * left[0],
            origin[1] + along * forward[1] + lateral * left[1],
        )
        for along, lateral in offsets
    ]


def progressive_probe_points(
        origin: Point, target: Point, max_step: float, min_step: float,
        fan_angles_deg: Sequence[float],
        min_progress: float) -> List[Point]:
    """
    Generate short target-biased probe points without passing the target.

    Each fan direction tries the longer step first and then the shorter step.
    Candidates that do not reduce Euclidean distance to *target* by at least
    ``min_progress`` are omitted. Map safety and Nav2 reachability are checked
    by the mission node before any candidate is dispatched.
    """
    dx = target[0] - origin[0]
    dy = target[1] - origin[1]
    target_distance = math.hypot(dx, dy)
    if target_distance <= 1e-9:
        return []

    bounded_max = min(max(0.0, max_step), target_distance)
    bounded_min = min(max(0.0, min_step), bounded_max)
    distances = [bounded_max]
    if bounded_min > 1e-9 and abs(bounded_min - bounded_max) > 1e-9:
        distances.append(bounded_min)

    target_yaw = math.atan2(dy, dx)
    required_progress = max(0.0, min_progress)
    points: List[Point] = []
    for angle_deg in fan_angles_deg:
        yaw = target_yaw + math.radians(float(angle_deg))
        for distance in distances:
            if distance <= 1e-9:
                continue
            point = (
                origin[0] + distance * math.cos(yaw),
                origin[1] + distance * math.sin(yaw),
            )
            progress = target_distance - math.hypot(
                target[0] - point[0], target[1] - point[1])
            if progress + 1e-9 < required_progress:
                continue
            if not any(
                    math.hypot(point[0] - other[0], point[1] - other[1])
                    <= 1e-9 for other in points):
                points.append(point)
    return points


def target_reveal_points(
        preparking: Point, target: Point,
        forward_offsets: Sequence[float],
        lateral_offsets: Sequence[float],
        min_target_standoff: float) -> List[Point]:
    """
    Generate bounded viewpoints that can reveal cells near pre-parking.

    Candidates advance from the nominal pre-parking point toward the target,
    while retaining a minimum target standoff. All centerline advances are
    tried before lateral alternatives. Occupancy, clearance and Nav2 path
    checks remain the mission node's responsibility.
    """
    dx = target[0] - preparking[0]
    dy = target[1] - preparking[1]
    distance = math.hypot(dx, dy)
    if distance <= 1e-9:
        return []

    forward = (dx / distance, dy / distance)
    left = (-forward[1], forward[0])
    maximum_advance = max(
        0.0, distance - max(0.0, float(min_target_standoff)))
    offsets = sorted({
        min(max(0.0, float(value)), maximum_advance)
        for value in forward_offsets
        if float(value) > 1e-9
    })
    lateral = sorted(
        {float(value) for value in lateral_offsets},
        key=lambda value: (abs(value), value < 0.0))

    points: List[Point] = []
    for side in lateral:
        for advance in offsets:
            if advance <= 1e-9:
                continue
            point = (
                preparking[0] + advance * forward[0] + side * left[0],
                preparking[1] + advance * forward[1] + side * left[1],
            )
            if not any(
                    math.hypot(point[0] - other[0], point[1] - other[1])
                    <= 1e-9 for other in points):
                points.append(point)
    return points


def position_reached(robot: Point, goal: Point, tolerance: float) -> bool:
    """Position-only arrival check; intentionally has no heading input."""
    return math.hypot(robot[0] - goal[0], robot[1] - goal[1]) <= tolerance


def tag_observation_valid(
        visible_count: int, reprojection_error: float, inlier_count: int,
        sample_ages: Sequence[float], max_age: float,
        max_reprojection_error: float, min_inlier_points: int) -> bool:
    """Validate one complete decoded tag and a fresh accepted board pose."""
    return (
        visible_count >= 1 and
        inlier_count >= min_inlier_points and
        math.isfinite(reprojection_error) and
        reprojection_error <= max_reprojection_error and
        len(sample_ages) == 4 and
        all(0.0 <= age <= max_age for age in sample_ages)
    )
