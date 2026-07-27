#!/usr/bin/env python3
"""Publish a navigation-only map with a hard obstacle-clearance radius.

The SLAM map remains available on ``/map`` for mapping, target handoff, and
visualisation.  This node expands every occupied cell by a fixed metric radius
and publishes the result on ``/map_clearance``.  Global planners using the
expanded map cannot route the robot centre through a passage narrower than
twice that radius.
"""

import math
from typing import Iterable, List, Sequence, Tuple

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.qos import ReliabilityPolicy


Offset = Tuple[int, int]


def clearance_offsets(radius: float, resolution: float) -> List[Offset]:
    """Return grid offsets whose cell centres lie within ``radius``."""
    if radius <= 0.0 or resolution <= 0.0:
        return [(0, 0)]
    cell_radius = int(math.ceil(radius / resolution))
    radius_squared = radius * radius + 1e-12
    offsets = []
    for dy in range(-cell_radius, cell_radius + 1):
        for dx in range(-cell_radius, cell_radius + 1):
            if (dx * resolution) ** 2 + (dy * resolution) ** 2 <= radius_squared:
                offsets.append((dx, dy))
    return offsets


def inflate_grid(
        data: Sequence[int],
        width: int,
        height: int,
        offsets: Iterable[Offset],
        occupied_threshold: int = 65) -> List[int]:
    """Mark cells around occupied source cells as occupied.

    Expansion also covers unknown cells.  Otherwise a planner with
    ``allow_unknown`` enabled could cut through the unknown side of a mapped
    wall and bypass the hard-clearance guarantee.
    """
    if width <= 0 or height <= 0 or len(data) != width * height:
        raise ValueError('occupancy-grid dimensions do not match data length')

    result = list(data)
    occupied = [
        (index % width, index // width)
        for index, value in enumerate(data)
        if value >= occupied_threshold
    ]
    cached_offsets = tuple(offsets)
    for source_x, source_y in occupied:
        for dx, dy in cached_offsets:
            x = source_x + dx
            y = source_y + dy
            if 0 <= x < width and 0 <= y < height:
                result[y * width + x] = 100
    return result


def restore_startup_free_cells(
        expanded: Sequence[int],
        source: Sequence[int],
        width: int,
        height: int,
        centre: Tuple[int, int],
        offsets: Iterable[Offset],
        occupied_threshold: int = 65) -> List[int]:
    """Restore only originally free cells in a fixed startup escape region.

    A robot can legitimately start closer to a wall than the requested global
    clearance. Without this fixed exception, obstacle dilation can imprison
    the start pose. Original occupied and unknown cells are never cleared, and
    the exception does not follow the robot, so it cannot open later narrow
    passages.
    """
    if len(expanded) != width * height or len(source) != width * height:
        raise ValueError('occupancy-grid dimensions do not match data length')
    result = list(expanded)
    centre_x, centre_y = centre
    for dx, dy in offsets:
        x = centre_x + dx
        y = centre_y + dy
        if not (0 <= x < width and 0 <= y < height):
            continue
        index = y * width + x
        if 0 <= source[index] < occupied_threshold:
            result[index] = source[index]
    return result


class ClearanceMap(Node):
    """Expand the live SLAM/static map for global navigation only."""

    def __init__(self):
        super().__init__('clearance_map')
        self.declare_parameter('input_map_topic', '/map')
        self.declare_parameter('output_map_topic', '/map_clearance')
        self.declare_parameter('clearance_radius', 0.175)
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('startup_escape_radius', 0.0)
        self.declare_parameter('startup_x', 0.0)
        self.declare_parameter('startup_y', 0.0)

        self.input_topic = str(
            self.get_parameter('input_map_topic').value)
        self.output_topic = str(
            self.get_parameter('output_map_topic').value)
        self.clearance_radius = max(
            0.0, float(self.get_parameter('clearance_radius').value))
        self.occupied_threshold = int(
            self.get_parameter('occupied_threshold').value)
        self.startup_escape_radius = max(
            0.0, float(self.get_parameter('startup_escape_radius').value))
        self.startup_x = float(self.get_parameter('startup_x').value)
        self.startup_y = float(self.get_parameter('startup_y').value)
        self.cached_resolution = None
        self.cached_offsets: List[Offset] = [(0, 0)]
        self.cached_startup_offsets: List[Offset] = []
        self.last_geometry = None

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(
            OccupancyGrid, self.output_topic, map_qos)
        self.subscription = self.create_subscription(
            OccupancyGrid, self.input_topic, self._map_callback, map_qos)
        self.get_logger().info(
            f'Hard-clearance map ready: {self.input_topic} -> '
            f'{self.output_topic}, radius={self.clearance_radius:.3f}m, '
            f'minimum passage={2.0 * self.clearance_radius:.3f}m, '
            f'fixed startup escape={self.startup_escape_radius:.3f}m')

    def _map_callback(self, msg: OccupancyGrid):
        resolution = float(msg.info.resolution)
        if resolution <= 0.0:
            self.get_logger().error('Ignoring map with non-positive resolution')
            return
        if self.cached_resolution != resolution:
            self.cached_resolution = resolution
            self.cached_offsets = clearance_offsets(
                self.clearance_radius, resolution)
            self.cached_startup_offsets = (
                clearance_offsets(self.startup_escape_radius, resolution)
                if self.startup_escape_radius > 0.0 else [])

        try:
            expanded = inflate_grid(
                msg.data,
                int(msg.info.width),
                int(msg.info.height),
                self.cached_offsets,
                self.occupied_threshold)
            if self.cached_startup_offsets:
                startup_cell = self._world_to_map(
                    msg, self.startup_x, self.startup_y)
                if startup_cell is not None:
                    expanded = restore_startup_free_cells(
                        expanded,
                        msg.data,
                        int(msg.info.width),
                        int(msg.info.height),
                        startup_cell,
                        self.cached_startup_offsets,
                        self.occupied_threshold)
        except ValueError as exc:
            self.get_logger().error(f'Ignoring invalid map: {exc}')
            return

        output = OccupancyGrid()
        output.header = msg.header
        output.info = msg.info
        output.data = expanded
        self.publisher.publish(output)

        geometry = (
            int(msg.info.width), int(msg.info.height), resolution,
            len(self.cached_offsets))
        if geometry != self.last_geometry:
            self.last_geometry = geometry
            self.get_logger().info(
                f'Published clearance map {geometry[0]}x{geometry[1]} at '
                f'{resolution:.3f}m/pix using {geometry[3]} dilation cells')

    @staticmethod
    def _world_to_map(
            msg: OccupancyGrid, world_x: float, world_y: float):
        q = msg.info.origin.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        dx = world_x - msg.info.origin.position.x
        dy = world_y - msg.info.origin.position.y
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        x = int(math.floor(local_x / msg.info.resolution))
        y = int(math.floor(local_y / msg.info.resolution))
        if 0 <= x < msg.info.width and 0 <= y < msg.info.height:
            return x, y
        return None


def main(args=None):
    rclpy.init(args=args)
    node = ClearanceMap()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
