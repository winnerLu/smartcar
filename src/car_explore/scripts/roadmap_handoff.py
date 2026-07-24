"""Pure geometry and perception gates for Roadmap-to-parking handoff."""

import math
from typing import List, Sequence, Tuple


Point = Tuple[float, float]


def preparking_point(
        start: Point, target: Point,
        standoff: float) -> Tuple[float, float, float]:
    """Return a point before *target* along the start-to-target direction."""
    dx = target[0] - start[0]
    dy = target[1] - start[1]
    distance = math.hypot(dx, dy)
    if distance <= 1e-9:
        return target[0], target[1], 0.0
    yaw = math.atan2(dy, dx)
    # Never put a pre-parking point behind the mission start for a short goal.
    bounded_standoff = min(max(0.0, standoff), max(0.0, distance - 0.15))
    return (
        target[0] - math.cos(yaw) * bounded_standoff,
        target[1] - math.sin(yaw) * bounded_standoff,
        yaw)


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
