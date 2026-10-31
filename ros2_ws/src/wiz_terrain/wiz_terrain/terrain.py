"""Terrain analysis algorithms for Wiz.

Deliberately free of ROS imports, so this whole layer is testable without a
ROS 2 installation (the same split as ``wiz_vision.processing`` versus
``wiz_vision.vision_node``).

What this produces is a **single-frame, camera-relative (local) traversability
grid**. It is explicitly NOT:

* a persistent world map -- there is no pose input and no accumulation across
  frames in this sprint; every frame yields an independent grid.
* an occupancy probability field -- the 0..100 range carries a *traversability
  cost*, not a probability that a cell is occupied. See ``UNKNOWN``/``COST_*``
  below and docs/ROS2_CONVENTIONS.md.

The grid produced here is in **image space**: ``grid[row][col]`` follows the
source image, row 0 being the top of the frame. Placing that grid into the
``base_link`` frame is the ROS wrapper's job (see ``to_occupancy_row_major``
and ``terrain_node.py``), which keeps the geometry assumptions in one place.
"""

import math
from dataclasses import dataclass, field
from typing import List, Tuple

import cv2
import numpy as np

#: Cell value reserved for "not known". Sprint 4 only emits this for the
#: configurable border; Sprint 5 will use it for genuinely unobserved map cells.
UNKNOWN = -1
#: Clear, low-cost terrain.
COST_MIN = 0
#: Impassable / highest-cost terrain.
COST_MAX = 100


@dataclass(frozen=True)
class TerrainConfig:
    """Tunable parameters for the terrain pipeline.

    Every value here is exposed as a ROS parameter by ``terrain_node``.
    """

    # --- framing ---
    resize_width: int = 320
    grid_rows: int = 24
    grid_cols: int = 32
    blur_kernel: int = 5

    # --- vegetation segmentation (HSV) ---
    veg_hsv_lower: Tuple[int, int, int] = (35, 40, 25)
    veg_hsv_upper: Tuple[int, int, int] = (90, 255, 255)
    veg_weight: float = 35.0

    # --- surface roughness (Sobel) ---
    roughness_threshold: float = 18.0
    roughness_gain: float = 2.0

    # --- discrete obstacles (Canny + contours) ---
    canny_low: int = 60
    canny_high: int = 160
    min_contour_area: int = 60
    obstacle_fraction_threshold: float = 0.25
    obstacle_cost: int = COST_MAX

    morph_kernel: int = 3

    # --- ground-scale ASSUMPTIONS (see ground_sample_distance) ---
    assumed_altitude_m: float = 3.0
    camera_hfov_deg: float = 70.0

    unknown_border_cells: int = 0

    def validate(self) -> None:
        """Raise ``ValueError`` on any parameter that cannot work."""
        if self.resize_width < 16:
            raise ValueError(f"resize_width must be >= 16, got {self.resize_width}")
        if self.grid_rows < 1 or self.grid_cols < 1:
            raise ValueError(
                f"grid dimensions must be >= 1, got {self.grid_rows}x{self.grid_cols}"
            )
        if self.blur_kernel < 1 or self.blur_kernel % 2 == 0:
            raise ValueError(
                f"blur_kernel must be a positive odd number, got {self.blur_kernel}"
            )
        if self.morph_kernel < 1:
            raise ValueError(f"morph_kernel must be >= 1, got {self.morph_kernel}")
        if self.roughness_gain < 0 or self.roughness_threshold < 0:
            raise ValueError("roughness_threshold and roughness_gain must be >= 0")
        if self.veg_weight < 0:
            raise ValueError(f"veg_weight must be >= 0, got {self.veg_weight}")
        if not 0.0 < self.obstacle_fraction_threshold <= 1.0:
            raise ValueError(
                "obstacle_fraction_threshold must be in (0, 1], got "
                f"{self.obstacle_fraction_threshold}"
            )
        if self.canny_low < 0 or self.canny_high <= self.canny_low:
            raise ValueError(
                f"require 0 <= canny_low < canny_high, got {self.canny_low}/{self.canny_high}"
            )
        if self.min_contour_area < 0:
            raise ValueError("min_contour_area must be >= 0")
        if not COST_MIN <= self.obstacle_cost <= COST_MAX:
            raise ValueError(
                f"obstacle_cost must be within {COST_MIN}..{COST_MAX}, got {self.obstacle_cost}"
            )
        if self.assumed_altitude_m <= 0:
            raise ValueError(
                f"assumed_altitude_m must be > 0, got {self.assumed_altitude_m}"
            )
        if not 0.0 < self.camera_hfov_deg < 180.0:
            raise ValueError(
                f"camera_hfov_deg must be in (0, 180), got {self.camera_hfov_deg}"
            )
        if self.unknown_border_cells < 0:
            raise ValueError("unknown_border_cells must be >= 0")
        lo, hi = self.veg_hsv_lower, self.veg_hsv_upper
        if len(lo) != 3 or len(hi) != 3:
            raise ValueError("veg_hsv_lower/upper must each have 3 components")
        if any(h <= l for l, h in zip(lo, hi)):
            raise ValueError(
                f"veg_hsv_upper must exceed veg_hsv_lower componentwise, got {lo} / {hi}"
            )


@dataclass
class TerrainResult:
    """Everything one frame's analysis produced.

    ``grid`` is image-space, ``int16``, with values ``UNKNOWN`` or
    ``COST_MIN..COST_MAX``.
    """

    grid: np.ndarray
    resized: np.ndarray
    gray: np.ndarray
    vegetation_mask: np.ndarray
    obstacle_mask: np.ndarray
    roughness: np.ndarray
    contours: List[np.ndarray] = field(default_factory=list)
    ground_sample_distance_m: float = 0.0
    cell_size_m: float = 0.0


def validate_frame(frame) -> None:
    """Raise a precise error for input the pipeline cannot process."""
    if frame is None:
        raise ValueError("frame is None")
    if not isinstance(frame, np.ndarray):
        raise TypeError(f"frame must be a numpy array, got {type(frame).__name__}")
    if frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError(f"frame must be HxWx3 BGR, got shape {frame.shape}")
    if frame.shape[0] < 2 or frame.shape[1] < 2:
        raise ValueError(f"frame is too small to analyse: {frame.shape}")
    if frame.dtype != np.uint8:
        raise ValueError(f"frame must be uint8, got {frame.dtype}")


def ground_sample_distance(config: TerrainConfig, width_px: int) -> float:
    """Metres of ground per pixel, from the declared altitude and FOV.

    ASSUMPTIONS, none of them measured in this sprint:

    1. the camera points straight down (nadir),
    2. the ground below is flat and horizontal,
    3. altitude is exactly ``assumed_altitude_m``,
    4. the lens is rectilinear with horizontal field of view
       ``camera_hfov_deg`` and no distortion.

    Sprint 5 replaces assumption 3 (and relaxes 1) with real pose input.
    """
    if width_px < 1:
        raise ValueError(f"width_px must be >= 1, got {width_px}")
    half = math.radians(config.camera_hfov_deg) / 2.0
    ground_width_m = 2.0 * config.assumed_altitude_m * math.tan(half)
    return ground_width_m / float(width_px)


def preprocess(frame: np.ndarray, config: TerrainConfig):
    """Resize, blur, and grey the frame.

    Returns ``(resized_bgr, blurred_bgr, gray)``.
    """
    validate_frame(frame)
    config.validate()

    h, w = frame.shape[:2]
    scale = config.resize_width / float(w)
    target = (config.resize_width, max(2, int(round(h * scale))))
    resized = cv2.resize(frame, target, interpolation=cv2.INTER_AREA)

    k = config.blur_kernel
    blurred = cv2.GaussianBlur(resized, (k, k), 0)
    gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
    return resized, blurred, gray


def _kernel(config: TerrainConfig) -> np.ndarray:
    k = config.morph_kernel
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))


def _morph_regions(mask: np.ndarray, config: TerrainConfig) -> np.ndarray:
    """Open then close: drops specks, then fills pinholes.

    For solid *region* masks only (e.g. vegetation), which are thick enough to
    survive an opening.
    """
    kernel = _kernel(config)
    opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)


def _morph_edges(edges: np.ndarray, config: TerrainConfig) -> np.ndarray:
    """Close only, to bridge small gaps along an edge.

    Canny edges are a single pixel wide, so an opening would delete them
    outright -- closing (dilate then erode) preserves the line and joins
    breaks in it.
    """
    return cv2.morphologyEx(edges, cv2.MORPH_CLOSE, _kernel(config))


def segment_vegetation(blurred_bgr: np.ndarray, config: TerrainConfig) -> np.ndarray:
    """Vegetation mask (0 or 255) by HSV thresholding.

    Hue is used rather than raw BGR because it stays comparatively stable as
    overall brightness changes, which matters for outdoor imagery.
    """
    config.validate()
    hsv = cv2.cvtColor(blurred_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        np.array(config.veg_hsv_lower, dtype=np.uint8),
        np.array(config.veg_hsv_upper, dtype=np.uint8),
    )
    return _morph_regions(mask, config)


def compute_roughness(gray: np.ndarray, config: TerrainConfig) -> np.ndarray:
    """Sobel gradient magnitude as a surface-roughness proxy.

    Flat, uniform ground produces small gradients; broken, textured or
    cluttered ground produces large ones. This is the primary traversability
    signal.

    Feed this the *unblurred* grey image. Gaussian smoothing is what Canny
    wants, but it removes exactly the high-frequency texture roughness is
    meant to measure -- blurring first makes coarse rubble read as clear
    ground. ``build_traversability_grid`` therefore passes the sharp grey
    here and the blurred grey to ``extract_obstacles``.
    """
    dx = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
    dy = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
    return cv2.magnitude(dx, dy)


def extract_obstacles(gray: np.ndarray, config: TerrainConfig):
    """Discrete obstacles via Canny edges and area-filtered contours.

    Returns ``(mask, contours)``. Contours below ``min_contour_area`` are
    discarded so that texture speckle is not mistaken for an object.
    """
    config.validate()
    edges = cv2.Canny(gray, config.canny_low, config.canny_high)
    edges = _morph_edges(edges, config)
    found, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    kept = [c for c in found if cv2.contourArea(c) >= config.min_contour_area]
    mask = np.zeros(gray.shape[:2], dtype=np.uint8)
    if kept:
        cv2.drawContours(mask, kept, -1, 255, thickness=cv2.FILLED)
    return mask, kept


def _cell_means(source: np.ndarray, config: TerrainConfig) -> np.ndarray:
    """Average ``source`` down to the grid with INTER_AREA.

    INTER_AREA averages over each source region, so this is exactly the
    per-cell mean -- and far faster than slicing cell by cell.
    """
    return cv2.resize(
        source.astype(np.float32),
        (config.grid_cols, config.grid_rows),
        interpolation=cv2.INTER_AREA,
    )


def build_traversability_grid(
    frame: np.ndarray, config: TerrainConfig = None
) -> TerrainResult:
    """Run the full single-frame pipeline.

    Cost for each cell, in order of precedence::

        rough_term = clip((mean_roughness - roughness_threshold) * roughness_gain, 0, 100)
        veg_term   = vegetation_fraction * veg_weight
        cost       = clip(rough_term + veg_term, 0, 100)

        if obstacle_fraction >= obstacle_fraction_threshold:
            cost = obstacle_cost          # a detected obstacle overrides the rest

    Border cells are set to ``UNKNOWN`` when ``unknown_border_cells > 0``.
    """
    config = config or TerrainConfig()
    config.validate()

    resized, blurred, gray = preprocess(frame, config)

    # Roughness reads the sharp grey so surface texture survives; Canny reads
    # the blurred grey, where smoothing suppresses spurious edges.
    gray_sharp = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)

    vegetation_mask = segment_vegetation(blurred, config)
    roughness = compute_roughness(gray_sharp, config)
    obstacle_mask, contours = extract_obstacles(gray, config)

    rough_cell = _cell_means(roughness, config)
    veg_cell = _cell_means(vegetation_mask / 255.0, config)
    obs_cell = _cell_means(obstacle_mask / 255.0, config)

    rough_term = np.clip(
        (rough_cell - config.roughness_threshold) * config.roughness_gain,
        0.0,
        float(COST_MAX),
    )
    veg_term = veg_cell * config.veg_weight
    cost = np.clip(rough_term + veg_term, float(COST_MIN), float(COST_MAX))
    cost[obs_cell >= config.obstacle_fraction_threshold] = float(config.obstacle_cost)

    grid = np.rint(cost).astype(np.int16)

    b = config.unknown_border_cells
    if b > 0:
        grid[:b, :] = UNKNOWN
        grid[-b:, :] = UNKNOWN
        grid[:, :b] = UNKNOWN
        grid[:, -b:] = UNKNOWN

    gsd = ground_sample_distance(config, resized.shape[1])
    cell_size = gsd * (resized.shape[1] / float(config.grid_cols))

    return TerrainResult(
        grid=grid,
        resized=resized,
        gray=gray,
        vegetation_mask=vegetation_mask,
        obstacle_mask=obstacle_mask,
        roughness=roughness,
        contours=contours,
        ground_sample_distance_m=gsd,
        cell_size_m=cell_size,
    )


def to_occupancy_row_major(grid: np.ndarray):
    """Convert an image-space grid to ROS ``OccupancyGrid`` layout.

    ``OccupancyGrid`` is row-major from its origin with ``+x`` along the row
    and ``+y`` up the columns, so ``data[iy * width + ix]``.

    The mapping, for a nadir camera whose image "up" is the drone's forward
    direction:

    * image row 0 (top) is the **farthest forward**, so it becomes the
      largest ``x`` index,
    * image column 0 (left) is the drone's **left**, which is ``+y`` in
      ``base_link`` (x-forward, y-left, z-up), so it becomes the largest
      ``y`` index.

    Hence ``width = grid_rows`` (forward extent) and ``height = grid_cols``
    (lateral extent).

    Returns ``(data_list, width, height)``.
    """
    if grid.ndim != 2:
        raise ValueError(f"grid must be 2-D, got shape {grid.shape}")
    flipped = np.flip(np.flip(grid, axis=0), axis=1)
    out = flipped.T  # -> (grid_cols, grid_rows) == (height, width)
    height, width = out.shape
    return out.astype(np.int8).flatten().tolist(), width, height


def cost_colour(cost: int) -> Tuple[int, int, int]:
    """BGR colour for a cost value: grey unknown, green clear, red impassable."""
    if cost == UNKNOWN:
        return (120, 120, 120)
    t = max(0.0, min(1.0, cost / float(COST_MAX)))
    if t < 0.5:  # green -> yellow
        f = t / 0.5
        return (0, 255, int(255 * f))
    f = (t - 0.5) / 0.5  # yellow -> red
    return (0, int(255 * (1.0 - f)), 255)


def render_overlay(result: TerrainResult, config: TerrainConfig = None) -> np.ndarray:
    """Draw the grid over the frame: cost-tinted cells plus obstacle outlines."""
    config = config or TerrainConfig()
    base = result.resized.copy()
    tint = base.copy()

    h, w = base.shape[:2]
    rows, cols = result.grid.shape
    for r in range(rows):
        y0 = int(round(r * h / rows))
        y1 = int(round((r + 1) * h / rows))
        for c in range(cols):
            x0 = int(round(c * w / cols))
            x1 = int(round((c + 1) * w / cols))
            cv2.rectangle(
                tint, (x0, y0), (x1, y1), cost_colour(int(result.grid[r, c])), -1
            )

    overlay = cv2.addWeighted(tint, 0.45, base, 0.55, 0)
    if result.contours:
        cv2.drawContours(overlay, result.contours, -1, (255, 120, 0), 1)
    return overlay
