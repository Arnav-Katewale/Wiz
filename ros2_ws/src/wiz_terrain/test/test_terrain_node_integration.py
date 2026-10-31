"""ROS integration test: camera_node feeding terrain_node.

The camera source is camera_node's SYNTHETIC generator, so this verifies
topic plumbing and message validity end to end -- not terrain-interpretation
quality on real imagery.
"""

import time

import pytest
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image

from wiz_terrain.terrain import COST_MAX, UNKNOWN
from wiz_terrain.terrain_node import TerrainNode
from wiz_vision.camera_node import CameraNode


def _spin_until(nodes, predicate, timeout_sec=8.0):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        for node in nodes:
            rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return True
    return False


def test_camera_and_terrain_nodes_publish_together():
    rclpy.init()
    camera = CameraNode()
    terrain = TerrainNode()
    listener = Node('test_terrain_listener')
    received = {}

    listener.create_subscription(
        OccupancyGrid, '/wiz/terrain/grid',
        lambda m: received.setdefault('grid', m), 10)
    listener.create_subscription(
        Image, '/wiz/terrain/overlay',
        lambda m: received.setdefault('overlay', m), 10)

    try:
        ok = _spin_until(
            [camera, terrain, listener],
            lambda: {'grid', 'overlay'} <= received.keys(),
            timeout_sec=8.0,
        )
        assert ok, 'did not receive terrain grid and overlay within timeout'

        grid = received['grid']
        # Defaults: 24 image rows -> width (forward), 32 cols -> height (lateral)
        assert grid.info.width == 24
        assert grid.info.height == 32
        assert len(grid.data) == 24 * 32
        assert grid.header.frame_id == 'base_link'
        assert grid.info.resolution > 0.0

        assert all(UNKNOWN <= v <= COST_MAX for v in grid.data), \
            'every cell must be UNKNOWN or within the cost range'

        # The grid is centred laterally and starts at the body origin.
        assert grid.info.origin.position.x == 0.0
        expected_y = -0.5 * grid.info.height * grid.info.resolution
        assert abs(grid.info.origin.position.y - expected_y) < 1e-6

        overlay = received['overlay']
        assert overlay.encoding == 'bgr8'
        assert overlay.width == 320  # default resize_width
    finally:
        camera.destroy_node()
        terrain.destroy_node()
        listener.destroy_node()
        rclpy.shutdown()


def test_terrain_node_refuses_invalid_parameters():
    """Bad parameters must fail loudly at construction, not degrade silently."""
    rclpy.init()
    try:
        with pytest.raises(ValueError, match='grid dimensions'):
            TerrainNode(parameter_overrides=[
                Parameter('grid_rows', Parameter.Type.INTEGER, 0),
            ])
    finally:
        rclpy.shutdown()


def test_terrain_node_honours_grid_parameter_overrides():
    """A custom grid size must reach the published message."""
    rclpy.init()
    camera = CameraNode()
    terrain = TerrainNode(parameter_overrides=[
        Parameter('grid_rows', Parameter.Type.INTEGER, 8),
        Parameter('grid_cols', Parameter.Type.INTEGER, 10),
        Parameter('publish_overlay', Parameter.Type.BOOL, False),
    ])
    listener = Node('test_terrain_param_listener')
    received = {}

    listener.create_subscription(
        OccupancyGrid, '/wiz/terrain/grid',
        lambda m: received.setdefault('grid', m), 10)

    try:
        ok = _spin_until(
            [camera, terrain, listener],
            lambda: 'grid' in received,
            timeout_sec=8.0,
        )
        assert ok, 'no grid received with overridden parameters'
        grid = received['grid']
        assert grid.info.width == 8
        assert grid.info.height == 10
        assert len(grid.data) == 80
    finally:
        camera.destroy_node()
        terrain.destroy_node()
        listener.destroy_node()
        rclpy.shutdown()
