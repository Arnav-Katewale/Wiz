"""Offline tests for the accumulating map grid (no ROS, no imagery)."""

import math

import numpy as np
import pytest

from wiz_terrain.map_grid import AccumulatedMap
from wiz_terrain.pose import Pose2D
from wiz_terrain.terrain import UNKNOWN

ORIGIN = Pose2D(0.0, 0.0, 0.0)


def _map(**kwargs):
    # 10 x 10 cells of 1 m: map x in [0, 10), y in [-5, 5).
    args = dict(width=10, height=10, resolution_m=1.0, origin_x=0.0, origin_y=-5.0)
    args.update(kwargs)
    return AccumulatedMap(**args)


def test_new_map_is_all_unknown():
    m = _map()
    assert np.all(m.mean_cost() == UNKNOWN)
    assert np.all(m.observations == 0)


def test_one_frame_lands_in_the_right_cells():
    m = _map()
    # 2 x 2 frame of 1 m cells straight ahead of the origin. Row 0 is farthest
    # forward, column 0 is the drone's left (+y).
    grid = np.array([[10, 20],
                     [30, 40]], dtype=np.int16)
    assert m.integrate(grid, ORIGIN, cell_m=1.0) == 4
    cost = m.mean_cost()
    # Centres: row 0 -> x = 1.5, row 1 -> x = 0.5; col 0 -> y = +0.5, col 1 -> y = -0.5.
    iy_left, iy_right = 5, 4  # y = +0.5 and -0.5 with origin_y = -5
    assert cost[iy_left, 1] == 10 and cost[iy_right, 1] == 20
    assert cost[iy_left, 0] == 30 and cost[iy_right, 0] == 40
    assert m.observations.sum() == 4


def test_repeated_views_average_and_raise_confidence():
    m = _map()
    m.integrate(np.array([[20]], dtype=np.int16), ORIGIN, cell_m=1.0)
    m.integrate(np.array([[60]], dtype=np.int16), ORIGIN, cell_m=1.0)
    iy, ix = m.cell_index(0.5, 0.0)
    assert m.mean_cost()[iy, ix] == 40
    assert m.observations[iy, ix] == 2
    assert m.confidence(full_at=4)[iy, ix] == pytest.approx(0.5)
    assert m.confidence(full_at=1)[iy, ix] == 1.0


def test_unknown_cells_add_nothing():
    m = _map()
    grid = np.array([[UNKNOWN, 50]], dtype=np.int16)
    assert m.integrate(grid, ORIGIN, cell_m=1.0) == 1
    assert m.observations.sum() == 1


def test_rotated_pose_places_cells_by_heading():
    m = _map()
    # Drone at (5, 0) facing +y (yaw 90 deg), so "ahead" is map +y.
    m.integrate(np.array([[70]], dtype=np.int16), Pose2D(5.0, 0.0, math.pi / 2), cell_m=1.0)
    iy, ix = m.cell_index(5.0, 0.5)  # 0.5 m ahead of the drone along +y
    assert m.mean_cost()[iy, ix] == 70
    assert m.observations.sum() == 1


def test_cells_outside_the_map_are_dropped():
    m = _map()
    # A 3-cell column ahead of x = 9 has centres at x = 9.5, 10.5, 11.5; the map ends at 10.
    added = m.integrate(np.full((3, 1), 10, dtype=np.int16), Pose2D(9.0, 0.0, 0.0), cell_m=1.0)
    assert added == 1
    assert m.observations.sum() == 1


def test_occupancy_layout_is_row_major_from_origin():
    m = _map(width=3, height=2, origin_y=0.0)
    m.integrate(np.array([[5]], dtype=np.int16), Pose2D(1.0, 1.0, 0.0), cell_m=1.0)
    data, width, height = m.to_occupancy()
    assert (width, height) == (3, 2)
    # Centre lands at (1.5, 1.0): ix = 1, iy = 1, index 1 * 3 + 1.
    assert data[4] == 5
    assert data.count(UNKNOWN) == 5


@pytest.mark.parametrize("bad", [
    dict(width=0),
    dict(resolution_m=0.0),
    dict(resolution_m=float("nan")),
    dict(origin_x=float("inf")),
])
def test_invalid_map_rejected(bad):
    with pytest.raises(ValueError):
        _map(**bad)


def test_out_of_range_cost_rejected():
    with pytest.raises(ValueError):
        _map().integrate(np.array([[101]], dtype=np.int16), ORIGIN, cell_m=1.0)
