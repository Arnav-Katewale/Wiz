"""Offline tests for 2-D pose transforms (no ROS, no imagery)."""

import math

import numpy as np
import pytest

from wiz_terrain.pose import (
    Pose2D,
    body_to_map,
    cell_centre_body,
    grid_cells_to_map,
    map_to_body,
    normalize_angle,
)
from wiz_terrain.terrain import to_occupancy_row_major

ORIGIN = Pose2D(0.0, 0.0, 0.0)


# --- body <-> map ---------------------------------------------------------

def test_identity_pose_leaves_points_unchanged():
    assert body_to_map(ORIGIN, 1.5, -0.25) == pytest.approx((1.5, -0.25))


def test_translation_only_shifts_points():
    pose = Pose2D(10.0, -3.0, 0.0)
    assert body_to_map(pose, 1.0, 2.0) == pytest.approx((11.0, -1.0))


def test_heading_left_turns_forward_into_map_plus_y():
    # Yaw +90 deg (CCW): the drone faces map +y, so "1 m ahead" is +y and
    # "1 m to the left" is -x.
    pose = Pose2D(0.0, 0.0, math.pi / 2)
    assert body_to_map(pose, 1.0, 0.0) == pytest.approx((0.0, 1.0), abs=1e-12)
    assert body_to_map(pose, 0.0, 1.0) == pytest.approx((-1.0, 0.0), abs=1e-12)


@pytest.mark.parametrize("yaw", [-3.0, -1.2, 0.0, 0.7, 2.9])
def test_map_to_body_inverts_body_to_map(yaw):
    pose = Pose2D(4.0, -2.5, yaw)
    mx, my = body_to_map(pose, 1.3, -0.8)
    assert map_to_body(pose, mx, my) == pytest.approx((1.3, -0.8))


def test_transform_preserves_distances():
    pose = Pose2D(1.0, 2.0, 0.9)
    a = np.array(body_to_map(pose, 0.0, 0.0))
    b = np.array(body_to_map(pose, 3.0, 4.0))
    assert np.linalg.norm(b - a) == pytest.approx(5.0)


@pytest.mark.parametrize("angle,expected", [
    (0.0, 0.0), (math.pi / 2, math.pi / 2), (3 * math.pi, -math.pi),
    (-3 * math.pi / 2, math.pi / 2), (2 * math.pi + 0.1, 0.1),
])
def test_normalize_angle(angle, expected):
    assert normalize_angle(angle) == pytest.approx(expected)


def test_non_finite_pose_rejected():
    with pytest.raises(ValueError, match="yaw"):
        Pose2D(0.0, 0.0, float("nan"))


# --- grid cells ---------------------------------------------------------

def test_top_left_cell_is_forward_left():
    bx, by = cell_centre_body(0, 0, rows=4, cols=6, cell_m=0.5)
    assert bx == pytest.approx(1.75)   # farthest forward row: (4 - 0.5) * 0.5
    assert by == pytest.approx(1.25)   # leftmost column is +y: (3 - 0.5) * 0.5


def test_bottom_right_cell_is_near_right():
    bx, by = cell_centre_body(3, 5, rows=4, cols=6, cell_m=0.5)
    assert bx == pytest.approx(0.25)
    assert by == pytest.approx(-1.25)


def test_cell_centres_match_occupancy_grid_layout():
    """Cross-check against to_occupancy_row_major plus terrain_node's origin.

    terrain_node places the OccupancyGrid origin at x=0, y=-cols/2*cell, so
    the centre of occupancy cell (ix, iy) is origin + (index + 0.5) * cell.
    Every image cell must land at the same point either way.
    """
    rows, cols, cell = 5, 8, 0.2
    ids = np.arange(rows * cols).reshape(rows, cols)
    data, width, height = to_occupancy_row_major(ids)
    origin_y = -0.5 * height * cell
    for iy in range(height):
        for ix in range(width):
            r, c = divmod(data[iy * width + ix], cols)
            expected = ((ix + 0.5) * cell, origin_y + (iy + 0.5) * cell)
            assert cell_centre_body(r, c, rows, cols, cell) == pytest.approx(expected)


def test_grid_cells_to_map_matches_single_cell_transform():
    pose = Pose2D(2.0, -1.0, 0.6)
    rows, cols, cell = 4, 6, 0.3
    out = grid_cells_to_map(pose, rows, cols, cell)
    assert out.shape == (rows, cols, 2)
    for r in range(rows):
        for c in range(cols):
            expected = body_to_map(pose, *cell_centre_body(r, c, rows, cols, cell))
            assert tuple(out[r, c]) == pytest.approx(expected)


def test_same_ground_seen_from_two_poses_lands_in_same_map_cell():
    # Drone at (0, 0) facing +x sees row 1 at 0.975 m ahead.
    # After flying 0.6 m forward, that ground is 0.375 m ahead: row 5.
    # (body x of row r is (rows - r - 0.5) * cell)
    rows, cols, cell = 8, 2, 0.15
    first = grid_cells_to_map(Pose2D(0.0, 0.0, 0.0), rows, cols, cell)
    second = grid_cells_to_map(Pose2D(0.6, 0.0, 0.0), rows, cols, cell)
    assert tuple(first[1, 0]) == pytest.approx(tuple(second[5, 0]))


@pytest.mark.parametrize("rows,cols,cell", [(0, 4, 0.1), (4, 0, 0.1), (4, 4, 0.0),
                                            (4, 4, -0.1), (4, 4, float("inf"))])
def test_invalid_grid_rejected(rows, cols, cell):
    with pytest.raises(ValueError):
        grid_cells_to_map(ORIGIN, rows, cols, cell)


def test_cell_outside_grid_rejected():
    with pytest.raises(ValueError, match="outside"):
        cell_centre_body(4, 0, rows=4, cols=4, cell_m=0.1)
