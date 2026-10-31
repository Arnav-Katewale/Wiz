"""Offline tests for the ROS-independent terrain algorithms.

All imagery is SYNTHETIC (see conftest.py). These tests exercise the
algorithms, not real-world terrain performance.
"""

import math

import numpy as np
import pytest

from wiz_terrain.terrain import (
    COST_MAX,
    COST_MIN,
    TerrainConfig,
    compute_roughness,
    extract_obstacles,
    ground_sample_distance,
    preprocess,
    segment_vegetation,
    validate_frame,
)


# --- input validation ---------------------------------------------------

def test_validate_frame_rejects_none():
    with pytest.raises(ValueError, match='None'):
        validate_frame(None)


def test_validate_frame_rejects_non_array():
    with pytest.raises(TypeError, match='numpy array'):
        validate_frame([[1, 2, 3]])


def test_validate_frame_rejects_grayscale():
    with pytest.raises(ValueError, match='HxWx3'):
        validate_frame(np.zeros((10, 10), dtype=np.uint8))


def test_validate_frame_rejects_wrong_dtype():
    with pytest.raises(ValueError, match='uint8'):
        validate_frame(np.zeros((10, 10, 3), dtype=np.float32))


def test_validate_frame_rejects_empty():
    with pytest.raises(ValueError):
        validate_frame(np.zeros((0, 0, 3), dtype=np.uint8))


def test_validate_frame_accepts_valid_bgr(flat_frame):
    validate_frame(flat_frame)  # must not raise


# --- configuration validation -------------------------------------------

@pytest.mark.parametrize('kwargs, message', [
    ({'resize_width': 4}, 'resize_width'),
    ({'grid_rows': 0}, 'grid dimensions'),
    ({'grid_cols': -3}, 'grid dimensions'),
    ({'blur_kernel': 4}, 'odd'),
    ({'roughness_gain': -1.0}, 'roughness'),
    ({'veg_weight': -5.0}, 'veg_weight'),
    ({'obstacle_fraction_threshold': 0.0}, 'obstacle_fraction_threshold'),
    ({'obstacle_fraction_threshold': 1.5}, 'obstacle_fraction_threshold'),
    ({'canny_low': 200, 'canny_high': 100}, 'canny'),
    ({'assumed_altitude_m': 0.0}, 'assumed_altitude_m'),
    ({'camera_hfov_deg': 180.0}, 'camera_hfov_deg'),
    ({'obstacle_cost': 500}, 'obstacle_cost'),
])
def test_config_validation_rejects_bad_parameters(kwargs, message):
    with pytest.raises(ValueError, match=message):
        TerrainConfig(**kwargs).validate()


def test_default_config_is_valid():
    TerrainConfig().validate()  # must not raise


def test_default_config_yields_square_cells():
    """320x240 downscaled with a 24x32 grid gives 10x10 px cells."""
    cfg = TerrainConfig()
    assert cfg.resize_width / cfg.grid_cols == pytest.approx(10.0)
    assert (cfg.resize_width * 3 / 4) / cfg.grid_rows == pytest.approx(10.0)


# --- preprocessing ------------------------------------------------------

def test_preprocess_resizes_to_configured_width(flat_frame):
    cfg = TerrainConfig(resize_width=160)
    resized, blurred, gray = preprocess(flat_frame, cfg)
    assert resized.shape[1] == 160
    assert blurred.shape == resized.shape
    assert gray.ndim == 2
    assert gray.shape == resized.shape[:2]


def test_preprocess_preserves_aspect_ratio(flat_frame):
    cfg = TerrainConfig(resize_width=160)
    resized, _, _ = preprocess(flat_frame, cfg)
    source_aspect = flat_frame.shape[0] / flat_frame.shape[1]
    assert resized.shape[0] / resized.shape[1] == pytest.approx(source_aspect, abs=0.02)


def test_preprocess_propagates_invalid_input():
    with pytest.raises(ValueError):
        preprocess(None, TerrainConfig())


# --- vegetation segmentation --------------------------------------------

def test_segment_vegetation_finds_green_region(vegetated_frame):
    cfg = TerrainConfig()
    _, blurred, _ = preprocess(vegetated_frame, cfg)
    mask = segment_vegetation(blurred, cfg)

    width = mask.shape[1]
    left = mask[:, : width // 2]
    right = mask[:, width // 2:]
    # The green block occupies the right half of the synthetic frame.
    assert right.mean() > 200, 'green half should be strongly flagged'
    assert left.mean() < 20, 'grey half should be mostly unflagged'


def test_segment_vegetation_ignores_neutral_ground(flat_frame):
    cfg = TerrainConfig()
    _, blurred, _ = preprocess(flat_frame, cfg)
    mask = segment_vegetation(blurred, cfg)
    assert mask.max() == 0, 'uniform grey must not register as vegetation'


def test_segment_vegetation_mask_is_binary(vegetated_frame):
    cfg = TerrainConfig()
    _, blurred, _ = preprocess(vegetated_frame, cfg)
    mask = segment_vegetation(blurred, cfg)
    assert set(np.unique(mask)).issubset({0, 255})


# --- roughness ----------------------------------------------------------

def test_roughness_is_near_zero_on_flat_surface(flat_frame):
    cfg = TerrainConfig()
    _, _, gray = preprocess(flat_frame, cfg)
    assert compute_roughness(gray, cfg).mean() < 1.0


def test_roughness_is_higher_on_texture_than_flat(flat_frame, textured_frame):
    cfg = TerrainConfig()
    _, _, flat_gray = preprocess(flat_frame, cfg)
    _, _, rough_gray = preprocess(textured_frame, cfg)

    flat_rough = compute_roughness(flat_gray, cfg).mean()
    textured_rough = compute_roughness(rough_gray, cfg).mean()
    assert textured_rough > flat_rough * 10, (
        f'textured {textured_rough:.2f} should greatly exceed flat {flat_rough:.2f}'
    )


# --- obstacles ----------------------------------------------------------

def test_extract_obstacles_finds_the_block(obstacle_frame):
    cfg = TerrainConfig()
    _, _, gray = preprocess(obstacle_frame, cfg)
    mask, contours = extract_obstacles(gray, cfg)
    assert contours, 'the high-contrast block should produce a contour'
    assert mask.max() == 255


def test_extract_obstacles_finds_nothing_on_flat_ground(flat_frame):
    cfg = TerrainConfig()
    _, _, gray = preprocess(flat_frame, cfg)
    mask, contours = extract_obstacles(gray, cfg)
    assert contours == []
    assert mask.max() == 0


def test_min_contour_area_filters_small_detections(obstacle_frame):
    """A huge area floor should discard the block entirely."""
    cfg = TerrainConfig(min_contour_area=10_000_000)
    _, _, gray = preprocess(obstacle_frame, cfg)
    mask, contours = extract_obstacles(gray, cfg)
    assert contours == []
    assert mask.max() == 0


# --- ground sample distance (ASSUMPTION-based) --------------------------

def test_ground_sample_distance_matches_pinhole_geometry():
    cfg = TerrainConfig(assumed_altitude_m=10.0, camera_hfov_deg=90.0)
    # At 90 deg HFOV the ground footprint width equals 2 * altitude.
    expected_width = 2.0 * 10.0 * math.tan(math.radians(90.0) / 2.0)
    gsd = ground_sample_distance(cfg, 100)
    assert gsd == pytest.approx(expected_width / 100.0)
    assert gsd == pytest.approx(0.2)


def test_ground_sample_distance_scales_linearly_with_altitude():
    low = ground_sample_distance(TerrainConfig(assumed_altitude_m=2.0), 320)
    high = ground_sample_distance(TerrainConfig(assumed_altitude_m=4.0), 320)
    assert high == pytest.approx(2.0 * low)


def test_ground_sample_distance_rejects_bad_width():
    with pytest.raises(ValueError, match='width_px'):
        ground_sample_distance(TerrainConfig(), 0)


def test_cost_bounds_are_the_documented_range():
    assert (COST_MIN, COST_MAX) == (0, 100)
