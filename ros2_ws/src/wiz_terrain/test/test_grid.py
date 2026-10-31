"""Offline tests for traversability-grid generation and ROS grid layout.

All imagery is SYNTHETIC (see conftest.py).
"""

import numpy as np
import pytest

from wiz_terrain.terrain import (
    COST_MAX,
    COST_MIN,
    UNKNOWN,
    TerrainConfig,
    build_traversability_grid,
    cost_colour,
    render_overlay,
    to_occupancy_row_major,
)


# --- shape and value range ----------------------------------------------

def test_grid_has_configured_shape(flat_frame):
    cfg = TerrainConfig(grid_rows=12, grid_cols=16)
    result = build_traversability_grid(flat_frame, cfg)
    assert result.grid.shape == (12, 16)


def test_grid_values_stay_in_documented_range(textured_frame, vegetated_frame,
                                              obstacle_frame, flat_frame):
    cfg = TerrainConfig()
    for frame in (textured_frame, vegetated_frame, obstacle_frame, flat_frame):
        grid = build_traversability_grid(frame, cfg).grid
        assert grid.min() >= UNKNOWN
        assert grid.max() <= COST_MAX
        valid = grid[grid != UNKNOWN]
        assert valid.min() >= COST_MIN


def test_grid_uses_integer_cells(flat_frame):
    result = build_traversability_grid(flat_frame, TerrainConfig())
    assert np.issubdtype(result.grid.dtype, np.integer)


def test_build_rejects_invalid_frame():
    with pytest.raises(ValueError):
        build_traversability_grid(None, TerrainConfig())


def test_build_rejects_invalid_config(flat_frame):
    with pytest.raises(ValueError, match='grid dimensions'):
        build_traversability_grid(flat_frame, TerrainConfig(grid_rows=0))


# --- cost semantics -----------------------------------------------------

def test_flat_ground_is_low_cost(flat_frame):
    """A featureless surface must come out as clear, near COST_MIN."""
    result = build_traversability_grid(flat_frame, TerrainConfig())
    assert result.grid.max() <= 1, f'expected ~0 cost, got max {result.grid.max()}'


def test_vegetation_raises_cost_on_its_own_half(vegetated_frame):
    cfg = TerrainConfig()
    grid = build_traversability_grid(vegetated_frame, cfg).grid
    half = grid.shape[1] // 2
    # Compare interior columns to avoid the block's own boundary edge.
    grey_side = grid[:, : half - 2]
    green_side = grid[:, half + 2:]
    assert green_side.mean() > grey_side.mean() + 10, (
        f'vegetated side {green_side.mean():.1f} should exceed '
        f'bare side {grey_side.mean():.1f}'
    )


def test_textured_ground_costs_more_than_smooth_ground(textured_band_frame):
    """Regression: roughness must survive preprocessing.

    Computing Sobel on the Gaussian-blurred grey smoothed fine texture away
    entirely, so coarse rubble scored the same as flat ground. Roughness is
    now taken from the unblurred grey.
    """
    cfg = TerrainConfig()
    grid = build_traversability_grid(textured_band_frame, cfg).grid
    half = grid.shape[1] // 2
    smooth_side = grid[:, : half - 2]
    rough_side = grid[:, half + 2:]
    assert rough_side.mean() > smooth_side.mean() + 20, (
        f'textured side {rough_side.mean():.1f} should clearly exceed '
        f'smooth side {smooth_side.mean():.1f}'
    )
    assert smooth_side.max() <= 5, 'smooth ground should stay near clear'


def test_vegetation_cost_scales_with_weight(vegetated_frame):
    light = build_traversability_grid(
        vegetated_frame, TerrainConfig(veg_weight=10.0)
    ).grid
    heavy = build_traversability_grid(
        vegetated_frame, TerrainConfig(veg_weight=90.0)
    ).grid
    assert heavy.mean() > light.mean()


def test_obstacle_region_reaches_max_cost(obstacle_frame):
    cfg = TerrainConfig()
    grid = build_traversability_grid(obstacle_frame, cfg).grid
    assert grid.max() == COST_MAX, 'a solid block should saturate at impassable'


def test_roughness_threshold_suppresses_cost(textured_frame):
    """Raising the threshold above the observed roughness should zero it out."""
    low = build_traversability_grid(
        textured_frame, TerrainConfig(roughness_threshold=0.0)
    ).grid
    high = build_traversability_grid(
        textured_frame,
        TerrainConfig(roughness_threshold=10_000.0, canny_low=250, canny_high=255,
                      min_contour_area=10_000_000),
    ).grid
    assert low.mean() > high.mean()
    assert high.max() == 0


def test_cost_is_monotonic_in_roughness_gain(textured_frame):
    cfg_kwargs = dict(canny_low=250, canny_high=255, min_contour_area=10_000_000)
    means = []
    for gain in (0.1, 0.5, 2.0):
        grid = build_traversability_grid(
            textured_frame,
            TerrainConfig(roughness_gain=gain, roughness_threshold=0.0, **cfg_kwargs),
        ).grid
        means.append(grid.mean())
    assert means[0] <= means[1] <= means[2], f'not monotonic: {means}'


# --- unknown cells ------------------------------------------------------

def test_unknown_border_marks_edges_unknown(flat_frame):
    cfg = TerrainConfig(grid_rows=10, grid_cols=10, unknown_border_cells=2)
    grid = build_traversability_grid(flat_frame, cfg).grid
    assert (grid[:2, :] == UNKNOWN).all()
    assert (grid[-2:, :] == UNKNOWN).all()
    assert (grid[:, :2] == UNKNOWN).all()
    assert (grid[:, -2:] == UNKNOWN).all()
    assert (grid[2:-2, 2:-2] != UNKNOWN).all()


def test_no_unknown_cells_by_default(flat_frame):
    grid = build_traversability_grid(flat_frame, TerrainConfig()).grid
    assert (grid != UNKNOWN).all()


# --- ground scale carried on the result ---------------------------------

def test_result_reports_ground_scale(flat_frame):
    cfg = TerrainConfig(assumed_altitude_m=10.0, camera_hfov_deg=90.0,
                        resize_width=320, grid_cols=32)
    result = build_traversability_grid(flat_frame, cfg)
    # 90 deg at 10 m -> 20 m across 320 px -> 6.25 cm/px; 10 px cells -> 0.625 m
    assert result.ground_sample_distance_m == pytest.approx(20.0 / 320.0)
    assert result.cell_size_m == pytest.approx(0.625)


# --- OccupancyGrid layout conversion ------------------------------------

def test_occupancy_layout_dimensions_swap_rows_and_cols():
    """width is the forward extent (image rows), height the lateral one."""
    grid = np.zeros((4, 6), dtype=np.int16)
    data, width, height = to_occupancy_row_major(grid)
    assert (width, height) == (4, 6)
    assert len(data) == 24


def test_occupancy_layout_places_image_top_row_farthest_forward():
    """Image row 0 is farthest ahead, so it lands at the largest x index."""
    grid = np.zeros((3, 2), dtype=np.int16)
    grid[0, 0] = 77  # top-left of the image
    data, width, height = to_occupancy_row_major(grid)

    # top row -> largest x; left column -> largest y
    ix, iy = width - 1, height - 1
    assert data[iy * width + ix] == 77
    assert sum(1 for v in data if v == 77) == 1


def test_occupancy_layout_preserves_every_value():
    rng = np.random.default_rng(seed=7)
    grid = rng.integers(0, 101, (5, 8)).astype(np.int16)
    data, _, _ = to_occupancy_row_major(grid)
    assert sorted(data) == sorted(grid.flatten().tolist())


def test_occupancy_layout_keeps_unknown_as_minus_one():
    grid = np.full((2, 2), UNKNOWN, dtype=np.int16)
    data, _, _ = to_occupancy_row_major(grid)
    assert set(data) == {-1}


def test_occupancy_layout_rejects_non_2d():
    with pytest.raises(ValueError, match='2-D'):
        to_occupancy_row_major(np.zeros((2, 2, 2), dtype=np.int16))


def test_occupancy_data_fits_int8_range():
    """OccupancyGrid.data is int8; costs and UNKNOWN must fit without wrapping."""
    grid = np.array([[UNKNOWN, COST_MIN], [COST_MAX, 50]], dtype=np.int16)
    data, _, _ = to_occupancy_row_major(grid)
    assert sorted(data) == [-1, 0, 50, 100]


# --- visualization ------------------------------------------------------

def test_render_overlay_matches_frame_size(obstacle_frame):
    cfg = TerrainConfig()
    result = build_traversability_grid(obstacle_frame, cfg)
    overlay = render_overlay(result, cfg)
    assert overlay.shape == result.resized.shape
    assert overlay.dtype == np.uint8


def test_cost_colour_endpoints_are_green_and_red():
    assert cost_colour(COST_MIN) == (0, 255, 0)
    assert cost_colour(COST_MAX) == (0, 0, 255)
    assert cost_colour(UNKNOWN) == (120, 120, 120)
