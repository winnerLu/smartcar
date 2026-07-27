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


class ClearanceMap(Node):
    """Expand the live SLAM/static map for global navigation only."""

    def __init__(self):
        super().__init__('clearance_map')
        self.declare_parameter('input_map_topic', '/map')
        self.declare_parameter('output_map_topic', '/map_clearance')
        self.declare_parameter('clearance_radius', 0.175)
        self.declare_parameter('occupied_threshold', 65)

        self.input_topic = str(
            self.get_parameter('input_map_topic').value)
        self.output_topic = str(
            self.get_parameter('output_map_topic').value)
        self.clearance_radius = max(
            0.0, float(self.get_parameter('clearance_radius').value))
        self.occupied_threshold = int(
            self.get_parameter('occupied_threshold').value)
        self.cached_resolution = None
        self.cached_offsets: List[Offset] = [(0, 0)]
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
            f'minimum passage={2.0 * self.clearance_radius:.3f}m')

    def _map_callback(self, msg: OccupancyGrid):
        resolution = float(msg.info.resolution)
        if resolution <= 0.0:
            self.get_logger().error('Ignoring map with non-positive resolution')
            return
        if self.cached_resolution != resolution:
            self.cached_resolution = resolution
            self.cached_offsets = clearance_offsets(
                self.clearance_radius, resolution)

        try:
            expanded = inflate_grid(
                msg.data,
                int(msg.info.width),
                int(msg.info.height),
                self.cached_offsets,
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
