#!/usr/bin/env python3
"""Publish a navigation-only map with narrow passages closed.

The SLAM map remains available on ``/map`` for mapping, target handoff, and
visualisation. This node applies a binary morphological closing operation to
the occupied cells and publishes the result on ``/map_clearance``. Passages
narrower than approximately twice the configured radius are filled, while the
outer boundary of isolated obstacles is restored by the erosion stage.

Only originally known-free cells can be added to the obstacle set. Unknown
cells remain unknown so the filtered map does not suppress unexplored
frontiers.
"""

import math
from typing import List, Sequence, Tuple

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


def close_narrow_passages(
        data: Sequence[int],
        width: int,
        height: int,
        offsets: Sequence[Offset],
        occupied_threshold: int = 65) -> List[int]:
    """Close narrow known-free gaps without thickening isolated obstacles.

    Binary closing is a dilation followed by an erosion using the same
    symmetric structuring element. The intermediate dilation joins obstacle
    boundaries separated by less than the requested passage width. Erosion
    then restores ordinary outer boundaries but leaves joined narrow gaps
    closed.

    The source obstacle set is explicitly preserved to avoid discretisation or
    map-edge erosion. New obstacle cells are written only over known-free
    source cells; unknown cells are never converted into obstacles.
    """
    if width <= 0 or height <= 0 or len(data) != width * height:
        raise ValueError('occupancy-grid dimensions do not match data length')

    cached_offsets = tuple(offsets)
    dilated = [False] * (width * height)
    occupied_indices = [
        index for index, value in enumerate(data)
        if value >= occupied_threshold
    ]

    # Temporary dilation. It may cover known-free and unknown cells, because
    # this intermediate mask is used only to decide whether closing joins two
    # obstacle sides; it is never published directly.
    for index in occupied_indices:
        source_x = index % width
        source_y = index // width
        for dx, dy in cached_offsets:
            x = source_x + dx
            y = source_y + dy
            if 0 <= x < width and 0 <= y < height:
                dilated[y * width + x] = True

    result = list(data)
    for index, value in enumerate(data):
        if value < 0 or value >= occupied_threshold or not dilated[index]:
            continue

        x = index % width
        y = index // width
        survives_erosion = True
        for dx, dy in cached_offsets:
            neighbour_x = x + dx
            neighbour_y = y + dy
            if (
                    neighbour_x < 0 or neighbour_x >= width or
                    neighbour_y < 0 or neighbour_y >= height or
                    not dilated[neighbour_y * width + neighbour_x]):
                survives_erosion = False
                break
        if survives_erosion:
            result[index] = 100

    return result


class ClearanceMap(Node):
    """Close narrow passages in the live SLAM/static navigation map."""

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
            f'Narrow-passage map ready: {self.input_topic} -> '
            f'{self.output_topic}, closing radius={self.clearance_radius:.3f}m, '
            f'minimum passage={2.0 * self.clearance_radius:.3f}m, '
            'isolated obstacle boundaries preserved')

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
            filtered = close_narrow_passages(
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
        output.data = filtered
        self.publisher.publish(output)

        added_cells = sum(
            1 for source, result in zip(msg.data, filtered)
            if 0 <= source < self.occupied_threshold and result == 100)
        geometry = (
            int(msg.info.width), int(msg.info.height), resolution,
            len(self.cached_offsets), added_cells)
        if geometry != self.last_geometry:
            self.last_geometry = geometry
            self.get_logger().info(
                f'Published narrow-passage map {geometry[0]}x{geometry[1]} at '
                f'{resolution:.3f}m/pix; added {geometry[4]} gate cells')


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
