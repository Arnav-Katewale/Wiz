"""Shared SYNTHETIC test imagery for the terrain pipeline.

Every frame produced here is procedurally generated and deterministic. None of
it is real terrain, aerial, or in-flight footage, and no test in this package
should be read as validating the pipeline against real-world imagery. Real
terrain validation is explicitly out of scope for Sprint 4 (see DEVLOG.md).
"""

import cv2
import numpy as np
import pytest

SYNTHETIC_HEIGHT = 240
SYNTHETIC_WIDTH = 320


def _blank(value=128):
    return np.full((SYNTHETIC_HEIGHT, SYNTHETIC_WIDTH, 3), value, dtype=np.uint8)


@pytest.fixture
def flat_frame():
    """SYNTHETIC: a featureless uniform surface. Zero gradient everywhere."""
    return _blank(128)


@pytest.fixture
def textured_frame():
    """SYNTHETIC: fine deterministic noise, standing in for broken ground."""
    rng = np.random.default_rng(seed=1234)
    noise = rng.integers(0, 256, (SYNTHETIC_HEIGHT, SYNTHETIC_WIDTH), dtype=np.uint16)
    gray = noise.astype(np.uint8)
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)


@pytest.fixture
def vegetated_frame():
    """SYNTHETIC: uniform ground with a solid green block on the right half.

    Pure green in BGR maps to hue 60 in OpenCV's 0..179 scale, inside the
    default vegetation band (35..90).
    """
    frame = _blank(128)
    frame[:, SYNTHETIC_WIDTH // 2:] = (0, 200, 0)
    return frame


@pytest.fixture
def textured_band_frame():
    """SYNTHETIC: smooth ground on the left, coarse rubble-like texture right.

    Guards against roughness being smoothed away before Sobel sees it -- the
    textured half must come out at a higher cost than the smooth half.
    """
    rng = np.random.default_rng(seed=99)
    frame = _blank(140)
    half = SYNTHETIC_WIDTH // 2
    band = frame[:, half:]
    noise = rng.integers(-70, 71, band.shape, dtype=np.int16)
    frame[:, half:] = np.clip(band.astype(np.int16) + noise, 0, 255).astype(np.uint8)
    return frame


@pytest.fixture
def obstacle_frame():
    """SYNTHETIC: uniform ground with one high-contrast solid block."""
    frame = _blank(170)
    cv2.rectangle(frame, (120, 80), (200, 160), (10, 10, 10), thickness=cv2.FILLED)
    return frame
