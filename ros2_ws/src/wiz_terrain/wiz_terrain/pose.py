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

import bisect
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


def yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    """Heading (rotation about z) of a quaternion, in radians, in [-pi, pi).

    Uses the standard z-y-x (yaw-pitch-roll) decomposition::

        yaw = atan2(2 (qw qz + qx qy), 1 - 2 (qy^2 + qz^2))

    The quaternion is normalised first, so a slightly non-unit input (common
    after float round-trips) still gives the right heading.
    """
    values = (qx, qy, qz, qw)
    if not all(math.isfinite(v) for v in values):
        raise ValueError(f"quaternion must be finite, got {values}")
    norm = math.sqrt(sum(v * v for v in values))
    if norm < 1e-9:
        raise ValueError("quaternion has zero length")
    qx, qy, qz, qw = (v / norm for v in values)
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return normalize_angle(yaw)


def pose_from_telemetry(px: float, py: float,
                        qx: float, qy: float, qz: float, qw: float) -> Pose2D:
    """2-D pose from a ``Telemetry`` position and orientation.

    Takes plain numbers so this stays ROS-free; ``terrain_node`` unpacks the
    message. Altitude (``position.z``), roll and pitch are dropped: this is the
    flat-ground, level-drone case described at the top of this module.
    """
    return Pose2D(float(px), float(py), yaw_from_quaternion(qx, qy, qz, qw))


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


class PoseBuffer:
    """Recent timestamped poses, so a camera frame can be placed at its own time.

    Telemetry and camera frames arrive at different rates and with different
    delays, so "the latest pose" when a frame is processed is usually not the
    pose the drone had when the frame was taken. ``pose_at(t)`` answers with
    the pose at time ``t``:

    * between two stored poses: linear interpolation. Position blends as
      ``p0 + f * (p1 - p0)`` with ``f = (t - t0) / (t1 - t0)``; heading blends
      the same way but along the *shorter* way round the circle, so going
      from 179 deg to -179 deg passes through 180, not through 0.
    * slightly newer than the newest pose (at most ``max_hold_s``): the newest
      pose is held. Extrapolating forward would guess motion we have not seen.
    * older than the oldest stored pose, or too far past the newest: ``None``,
      because there is no honest answer.

    Poses older than ``max_history_s`` behind the newest are dropped. Times
    are plain seconds (floats); the ROS node converts message stamps.
    """

    def __init__(self, max_history_s: float = 2.0, max_hold_s: float = 0.2):
        if not (math.isfinite(max_history_s) and max_history_s > 0):
            raise ValueError(f"max_history_s must be positive, got {max_history_s}")
        if not (math.isfinite(max_hold_s) and max_hold_s >= 0):
            raise ValueError(f"max_hold_s must be >= 0, got {max_hold_s}")
        self.max_history_s = max_history_s
        self.max_hold_s = max_hold_s
        self._times = []
        self._poses = []

    def __len__(self) -> int:
        return len(self._times)

    def add(self, t_s: float, pose: Pose2D) -> bool:
        """Store a pose. Returns False (and ignores it) if not newer than the last one."""
        if not math.isfinite(t_s):
            raise ValueError(f"time must be finite, got {t_s}")
        if self._times and t_s <= self._times[-1]:
            return False
        self._times.append(t_s)
        self._poses.append(pose)
        cutoff = t_s - self.max_history_s
        drop = bisect.bisect_left(self._times, cutoff)
        if drop:
            del self._times[:drop]
            del self._poses[:drop]
        return True

    def pose_at(self, t_s: float):
        """Pose at time ``t_s``, or None if it cannot be known (see class docstring)."""
        if not self._times or not math.isfinite(t_s):
            return None
        if t_s < self._times[0]:
            return None
        if t_s >= self._times[-1]:
            return self._poses[-1] if t_s - self._times[-1] <= self.max_hold_s else None
        i = bisect.bisect_right(self._times, t_s)  # times[i-1] <= t < times[i]
        t0, t1 = self._times[i - 1], self._times[i]
        p0, p1 = self._poses[i - 1], self._poses[i]
        f = (t_s - t0) / (t1 - t0)
        yaw = normalize_angle(p0.yaw + f * normalize_angle(p1.yaw - p0.yaw))
        return Pose2D(p0.x + f * (p1.x - p0.x), p0.y + f * (p1.y - p0.y), yaw)
