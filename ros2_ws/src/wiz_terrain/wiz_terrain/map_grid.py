"""A fixed ``map``-frame terrain grid that accumulates many frames.

Deliberately free of ROS imports, like ``terrain.py`` and ``pose.py``.

Each camera frame gives a small, camera-relative cost grid (``terrain.py``).
``AccumulatedMap`` places every frame into one fixed grid in the ``map``
frame, using the drone pose at that frame (``pose.py``), and keeps for every
map cell:

* the **running average cost** of every observation that landed in it, and
* the **observation count**, used as the cell's confidence: a cell seen in
  ten frames is more trustworthy than one seen once.

Map layout matches ROS ``OccupancyGrid``: cell ``(iy, ix)`` covers
``x in [origin_x + ix * res, origin_x + (ix + 1) * res)`` and the same for
``y``, so the row-major data index is ``iy * width + ix``.

How a frame is merged: each frame cell's centre is transformed into ``map``
and its cost is added to the map cell containing that centre. Frame cells
marked ``UNKNOWN`` add nothing, and centres outside the map are dropped. This
nearest-cell scheme leaves gaps if the map is finer than the frame's cells,
so choose ``resolution_m`` at least as large as the frame cell size.
"""

import math

import numpy as np

from wiz_terrain.pose import Pose2D, grid_cells_to_map
from wiz_terrain.terrain import COST_MAX, COST_MIN, UNKNOWN


class AccumulatedMap:
    """Running-average cost and observation count for a fixed map grid."""

    def __init__(self, width: int, height: int, resolution_m: float,
                 origin_x: float = 0.0, origin_y: float = 0.0):
        if width < 1 or height < 1:
            raise ValueError(f"map must be at least 1x1 cells, got {width}x{height}")
        if not (math.isfinite(resolution_m) and resolution_m > 0):
            raise ValueError(f"resolution_m must be a positive finite size, got {resolution_m}")
        if not (math.isfinite(origin_x) and math.isfinite(origin_y)):
            raise ValueError("map origin must be finite")
        self.width = width
        self.height = height
        self.resolution_m = resolution_m
        self.origin_x = origin_x
        self.origin_y = origin_y
        self._cost_sum = np.zeros((height, width), dtype=np.float64)
        self.observations = np.zeros((height, width), dtype=np.int64)

    def cell_index(self, mx, my):
        """``(iy, ix)`` of the map cell containing map point(s) ``(mx, my)``."""
        ix = np.floor((np.asarray(mx) - self.origin_x) / self.resolution_m).astype(np.int64)
        iy = np.floor((np.asarray(my) - self.origin_y) / self.resolution_m).astype(np.int64)
        return iy, ix

    def integrate(self, grid: np.ndarray, pose: Pose2D, cell_m: float) -> int:
        """Merge one frame's image-space cost grid seen from ``pose``.

        Returns how many frame cells were added to the map.
        """
        grid = np.asarray(grid)
        if grid.ndim != 2:
            raise ValueError(f"grid must be 2-D, got shape {grid.shape}")
        known = grid != UNKNOWN
        if np.any((grid[known] < COST_MIN) | (grid[known] > COST_MAX)):
            raise ValueError(f"grid costs must be UNKNOWN or {COST_MIN}..{COST_MAX}")

        centres = grid_cells_to_map(pose, grid.shape[0], grid.shape[1], cell_m)
        iy, ix = self.cell_index(centres[..., 0], centres[..., 1])
        inside = (ix >= 0) & (ix < self.width) & (iy >= 0) & (iy < self.height)
        use = known & inside

        # np.add.at, not +=, so two frame cells landing in the same map cell
        # both count.
        np.add.at(self._cost_sum, (iy[use], ix[use]), grid[use].astype(np.float64))
        np.add.at(self.observations, (iy[use], ix[use]), 1)
        return int(use.sum())

    def mean_cost(self) -> np.ndarray:
        """Average cost per cell as ``int16``; ``UNKNOWN`` where never observed."""
        out = np.full((self.height, self.width), UNKNOWN, dtype=np.int16)
        seen = self.observations > 0
        out[seen] = np.rint(self._cost_sum[seen] / self.observations[seen]).astype(np.int16)
        return out

    def confidence(self, full_at: int) -> np.ndarray:
        """Observation count scaled to 0..1, reaching 1 at ``full_at`` observations."""
        if full_at < 1:
            raise ValueError(f"full_at must be >= 1, got {full_at}")
        return np.minimum(self.observations / float(full_at), 1.0)

    def to_occupancy(self):
        """``(data_list, width, height)`` in ``OccupancyGrid`` row-major order."""
        return self.mean_cost().astype(np.int8).flatten().tolist(), self.width, self.height
