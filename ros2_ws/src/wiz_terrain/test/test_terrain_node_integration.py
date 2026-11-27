"""ROS integration test: camera_node feeding terrain_node.

The camera source is camera_node's SYNTHETIC generator, so this verifies
topic plumbing and message validity end to end -- not terrain-interpretation
quality on real imagery.
"""

import math
import time

import pytest
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from wiz_interfaces.msg import Telemetry

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


def test_terrain_node_tracks_pose_from_telemetry():
    """A Telemetry message must update the node's latest pose (heading from quaternion)."""
    rclpy.init()
    terrain = TerrainNode()
    talker = Node('test_telemetry_talker')
    pub = talker.create_publisher(Telemetry, '/wiz/telemetry', 10)

    msg = Telemetry()
    msg.position.x, msg.position.y, msg.position.z = 2.0, -0.5, 3.0
    yaw = 0.6  # pure rotation about z
    msg.orientation.z = math.sin(yaw / 2)
    msg.orientation.w = math.cos(yaw / 2)

    def publish_and_check():
        pub.publish(msg)
        return terrain.latest_pose is not None

    try:
        assert terrain.latest_pose is None
        ok = _spin_until([talker, terrain], publish_and_check, timeout_sec=5.0)
        assert ok, 'terrain_node never received /wiz/telemetry'
        pose = terrain.latest_pose
        assert pose.x == pytest.approx(2.0)
        assert pose.y == pytest.approx(-0.5)
        assert pose.yaw == pytest.approx(yaw)
    finally:
        terrain.destroy_node()
        talker.destroy_node()
        rclpy.shutdown()


def _telemetry(x, y, yaw):
    msg = Telemetry()
    msg.position.x, msg.position.y, msg.position.z = x, y, 3.0
    msg.orientation.z = math.sin(yaw / 2)
    msg.orientation.w = math.cos(yaw / 2)
    return msg


def test_terrain_node_publishes_accumulated_map_at_pose():
    """Telemetry + camera frames must produce a map-frame grid filled around the pose."""
    rclpy.init()
    camera = CameraNode()
    terrain = TerrainNode()
    talker = Node('test_map_talker')
    pub = talker.create_publisher(Telemetry, '/wiz/telemetry', 10)
    received = []
    talker.create_subscription(OccupancyGrid, '/wiz/terrain/map', received.append, 10)
    pose_x, pose_y, yaw = 2.0, -1.0, math.pi / 2  # facing map +y

    def publish_and_check():
        pub.publish(_telemetry(pose_x, pose_y, yaw))
        return len(received) >= 2

    try:
        ok = _spin_until([camera, terrain, talker], publish_and_check, timeout_sec=10.0)
        assert ok, 'no /wiz/terrain/map received'
        m = received[-1]
        assert m.header.frame_id == 'map'
        assert (m.info.width, m.info.height) == (200, 200)
        assert m.info.resolution == pytest.approx(0.15)
        assert (m.info.origin.position.x, m.info.origin.position.y) == (-15.0, -15.0)
        assert len(m.data) == 200 * 200
        assert all(UNKNOWN <= v <= COST_MAX for v in m.data)

        known = [i for i, v in enumerate(m.data) if v != UNKNOWN]
        assert known, 'map has no observed cells'
        # Facing +y, everything seen lies ahead of the drone (y >= pose y)
        # and within the frame's footprint (a few metres).
        for i in known:
            iy, ix = divmod(i, m.info.width)
            cx = m.info.origin.position.x + (ix + 0.5) * m.info.resolution
            cy = m.info.origin.position.y + (iy + 0.5) * m.info.resolution
            assert cy >= pose_y - m.info.resolution
            assert math.hypot(cx - pose_x, cy - pose_y) < 5.0
        # Repeated frames from the same pose raise the observation count.
        assert terrain.accumulated_map.observations.max() >= 2
    finally:
        camera.destroy_node()
        terrain.destroy_node()
        talker.destroy_node()
        rclpy.shutdown()


def test_terrain_node_skips_map_until_pose_known():
    """Without telemetry, frames still give the local grid but nothing reaches the map."""
    rclpy.init()
    camera = CameraNode()
    terrain = TerrainNode()
    listener = Node('test_map_listener')
    grids, maps = [], []
    listener.create_subscription(OccupancyGrid, '/wiz/terrain/grid', grids.append, 10)
    listener.create_subscription(OccupancyGrid, '/wiz/terrain/map', maps.append, 10)
    try:
        ok = _spin_until([camera, terrain, listener], lambda: len(grids) >= 3, timeout_sec=8.0)
        assert ok, 'no local grid received'
        assert maps == []
        assert terrain.accumulated_map.observations.sum() == 0
    finally:
        camera.destroy_node()
        terrain.destroy_node()
        listener.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('name,ptype,value', [
    ('map_resolution_m', Parameter.Type.DOUBLE, 0.0),
    ('map_width_cells', Parameter.Type.INTEGER, 0),
    ('map_confidence_full_at', Parameter.Type.INTEGER, 0),
    ('map_max_observations', Parameter.Type.INTEGER, 0),
    ('map_max_observations', Parameter.Type.INTEGER, 5),  # below full_at 10
])
def test_terrain_node_refuses_invalid_map_parameters(name, ptype, value):
    rclpy.init()
    try:
        with pytest.raises(ValueError):
            TerrainNode(parameter_overrides=[Parameter(name, ptype, value)])
    finally:
        rclpy.shutdown()


def test_terrain_node_publishes_map_confidence():
    """Confidence grows as frames repeat from one pose; unseen cells stay 0."""
    rclpy.init()
    camera = CameraNode()
    terrain = TerrainNode(parameter_overrides=[
        Parameter('map_confidence_full_at', Parameter.Type.INTEGER, 100),
        Parameter('map_max_observations', Parameter.Type.INTEGER, 100),
    ])
    talker = Node('test_confidence_talker')
    pub = talker.create_publisher(Telemetry, '/wiz/telemetry', 10)
    received = []
    talker.create_subscription(
        OccupancyGrid, '/wiz/terrain/map_confidence', received.append, 10
    )

    def publish_and_check():
        pub.publish(_telemetry(0.0, 0.0, 0.0))
        return len(received) >= 5

    try:
        ok = _spin_until([camera, terrain, talker], publish_and_check, timeout_sec=12.0)
        assert ok, 'not enough /wiz/terrain/map_confidence messages'
        first, last = received[0], received[-1]
        assert last.header.frame_id == 'map'
        assert (last.info.width, last.info.height) == (200, 200)
        assert all(0 <= v <= 100 for v in last.data)
        # One frame can hit a map cell several times (map cells are larger
        # than frame cells), so compare totals, not single cells.
        assert sum(last.data) > sum(first.data)        # grows with repeat frames
        assert last.data.count(0) > 0                  # most of the map is unseen
        # Matches the observation counts the node holds.
        obs = terrain.accumulated_map.observations.flatten()
        for v, n in zip(last.data, obs):
            if n == 0:
                assert v == 0
    finally:
        camera.destroy_node()
        terrain.destroy_node()
        talker.destroy_node()
        rclpy.shutdown()
