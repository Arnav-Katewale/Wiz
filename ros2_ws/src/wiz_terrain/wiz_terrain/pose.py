"""2-D pose and frame transforms for placing terrain grids in a map.

Deliberately free of ROS imports, like ``terrain.py``.

This is the first building block of Sprint 5 (pose-aware multi-frame terrain
mapping): given where the drone is, work out where each cell of a
single-frame, camera-relative grid lands in a fixed map frame. Nothing here
accumulates grids yet; that comes later.

Frames follow REP-103 / REP-105:

* ``base_link``: x forward, y left, z up (the drone body).
* ``map``: a fixed world frame. A pose ``(x, y, yaw)`` is the body origin in
  ``map`` and its heading, with yaw measured counter-clockwise from ``map``
  +x, in radians.

A body-frame point ``(bx, by)`` maps into ``map`` by rotating it by ``yaw``
and adding the pose position::

    mx = x + bx * cos(yaw) - by * sin(yaw)
    my = y + bx * sin(yaw) + by * cos(yaw)

Only the 2-D (flat ground, level drone) case is handled. Roll and pitch would
shift where a nadir camera looks; ignoring them is an explicit assumption,
the same one ``ground_sample_distance`` already makes.
"""

import math
from dataclasses import dataclass
from typing import Tuple

import numpy as np


def normalize_angle(angle: float) -> float:
    """Wrap an angle in radians to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class Pose2D:
    """Body origin position in ``map`` (metres) and heading (radians, CCW)."""

    x: float
    y: float
    yaw: float

    def __post_init__(self):
        for name in ("x", "y", "yaw"):
            value = getattr(self, name)
            if not math.isfinite(value):
                raise ValueError(f"pose {name} must be finite, got {value}")


def body_to_map(pose: Pose2D, bx, by):
    """Transform body-frame point(s) into ``map``. Works on floats or arrays."""
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return pose.x + bx * c - by * s, pose.y + bx * s + by * c


def map_to_body(pose: Pose2D, mx, my):
    """Inverse of ``body_to_map``."""
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    dx, dy = mx - pose.x, my - pose.y
    return dx * c + dy * s, -dx * s + dy * c


def cell_centre_body(row: int, col: int, rows: int, cols: int,
                     cell_m: float) -> Tuple[float, float]:
    """Centre of image-space grid cell ``(row, col)`` in ``base_link``.

    Uses the same placement as ``terrain_node``: the grid sits directly ahead
    of the body origin (x in [0, rows * cell_m]) and is centred laterally.
    Image row 0 is the farthest forward; image column 0 is the drone's left.
    """
    _check_grid(rows, cols, cell_m)
    if not (0 <= row < rows and 0 <= col < cols):
        raise ValueError(f"cell ({row}, {col}) outside a {rows}x{cols} grid")
    bx = (rows - row - 0.5) * cell_m
    by = (cols / 2.0 - col - 0.5) * cell_m
    return bx, by


def grid_cells_to_map(pose: Pose2D, rows: int, cols: int,
                      cell_m: float) -> np.ndarray:
    """``map`` coordinates of every cell centre, shape ``(rows, cols, 2)``.

    ``out[row, col]`` is ``(mx, my)`` for image-space cell ``(row, col)``.
    """
    _check_grid(rows, cols, cell_m)
    r = np.arange(rows, dtype=float)[:, None]
    c = np.arange(cols, dtype=float)[None, :]
    bx = np.broadcast_to((rows - r - 0.5) * cell_m, (rows, cols))
    by = np.broadcast_to((cols / 2.0 - c - 0.5) * cell_m, (rows, cols))
    mx, my = body_to_map(pose, bx, by)
    return np.stack([mx, my], axis=-1)


def _check_grid(rows: int, cols: int, cell_m: float) -> None:
    if rows < 1 or cols < 1:
        raise ValueError(f"grid must be at least 1x1, got {rows}x{cols}")
    if not (math.isfinite(cell_m) and cell_m > 0):
        raise ValueError(f"cell_m must be a positive finite size, got {cell_m}")
