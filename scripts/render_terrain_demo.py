#!/usr/bin/env python3
"""Render a four-panel view of the terrain pipeline, without needing ROS.

    python3 scripts/render_terrain_demo.py                       # synthetic scene
    python3 scripts/render_terrain_demo.py --input photo.jpg      # your own image

Panels: source frame, preprocessed grey, cost overlay, resulting grid.

With no --input a procedurally generated scene is used, and the output is
labelled SYNTHETIC. That scene is invented test imagery -- it is not real
terrain, aerial, or in-flight footage, and rendering it validates the
pipeline's mechanics only.
"""

import argparse
import os
import sys

import cv2
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, '..', 'ros2_ws', 'src', 'wiz_terrain'))

from wiz_terrain.terrain import (  # noqa: E402
    COST_MAX,
    TerrainConfig,
    build_traversability_grid,
    cost_colour,
    render_overlay,
)


def synthetic_scene(width=640, height=480, seed=20261031):
    """Procedurally generate a SYNTHETIC overhead-looking terrain scene."""
    rng = np.random.default_rng(seed)

    # Base ground: mid-brown with fine gravel-like noise.
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    frame[:, :] = (90, 110, 135)
    grain = rng.integers(-12, 13, (height, width, 1), dtype=np.int16)
    frame = np.clip(frame.astype(np.int16) + grain, 0, 255).astype(np.uint8)

    # A smooth path down the middle: lower texture, easier going.
    cv2.rectangle(frame, (width // 2 - 55, 0), (width // 2 + 55, height),
                  (120, 140, 160), thickness=cv2.FILLED)

    # Vegetation patches (green -> inside the default HSV band).
    for cx, cy, r in ((90, 120, 70), (545, 330, 85), (120, 400, 55)):
        cv2.circle(frame, (cx, cy), r, (40, 150, 55), thickness=cv2.FILLED)

    # Coarse rubble: a band of strong high-frequency texture.
    band = frame[60:140, 360:620]
    rubble = rng.integers(-70, 71, band.shape, dtype=np.int16)
    frame[60:140, 360:620] = np.clip(
        band.astype(np.int16) + rubble, 0, 255).astype(np.uint8)

    # Two discrete obstacles: solid dark blocks.
    cv2.rectangle(frame, (250, 300), (330, 380), (25, 25, 30), thickness=cv2.FILLED)
    cv2.circle(frame, (430, 170), 28, (20, 20, 25), thickness=cv2.FILLED)

    return cv2.GaussianBlur(frame, (3, 3), 0)


def grid_panel(result, size):
    """Draw the traversability grid as a standalone colour-coded image."""
    rows, cols = result.grid.shape
    cell = np.zeros((rows, cols, 3), dtype=np.uint8)
    for r in range(rows):
        for c in range(cols):
            cell[r, c] = cost_colour(int(result.grid[r, c]))
    return cv2.resize(cell, size, interpolation=cv2.INTER_NEAREST)


def label(img, text):
    out = img.copy()
    cv2.rectangle(out, (0, 0), (out.shape[1], 22), (0, 0, 0), cv2.FILLED)
    cv2.putText(out, text, (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                (255, 255, 255), 1, cv2.LINE_AA)
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', help='image file; omit for a synthetic scene')
    parser.add_argument('--output', default='terrain_demo.png')
    parser.add_argument('--altitude', type=float, default=3.0,
                        help='ASSUMED altitude in metres')
    parser.add_argument('--hfov', type=float, default=70.0,
                        help='camera horizontal field of view in degrees')
    args = parser.parse_args()

    if args.input:
        frame = cv2.imread(args.input, cv2.IMREAD_COLOR)
        if frame is None:
            raise SystemExit(f'could not read image: {args.input}')
        origin = os.path.basename(args.input)
    else:
        frame = synthetic_scene()
        origin = 'SYNTHETIC generated scene'

    config = TerrainConfig(assumed_altitude_m=args.altitude,
                           camera_hfov_deg=args.hfov)
    result = build_traversability_grid(frame, config)

    h, w = result.resized.shape[:2]
    panels = [
        label(result.resized, f'1. source ({origin})'),
        label(cv2.cvtColor(result.gray, cv2.COLOR_GRAY2BGR), '2. preprocessed grey'),
        label(render_overlay(result, config), '3. traversability overlay'),
        label(grid_panel(result, (w, h)),
              f'4. grid {result.grid.shape[0]}x{result.grid.shape[1]} '
              f'(0=clear .. {COST_MAX}=impassable)'),
    ]
    montage = np.vstack([np.hstack(panels[:2]), np.hstack(panels[2:])])

    cv2.imwrite(args.output, montage)

    valid = result.grid[result.grid >= 0]
    print(f'wrote {args.output}  ({montage.shape[1]}x{montage.shape[0]})')
    print(f'source            : {origin}')
    print(f'grid              : {result.grid.shape[0]}x{result.grid.shape[1]} cells')
    print(f'cost min/mean/max : {valid.min()} / {valid.mean():.1f} / {valid.max()}')
    print(f'impassable cells  : {int((result.grid == COST_MAX).sum())}')
    print(f'obstacle contours : {len(result.contours)}')
    print('ASSUMED ground scale (not measured): '
          f'{result.ground_sample_distance_m * 100:.2f} cm/pixel, '
          f'{result.cell_size_m:.3f} m/cell '
          f'at {args.altitude:.1f} m altitude, {args.hfov:.0f} deg HFOV')


if __name__ == '__main__':
    main()
