# ROS 2 Conventions

## Workspace

All ROS 2 packages live in `ros2_ws/src/`. Build from `ros2_ws/`:

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Run everything together:

```bash
ros2 launch wiz_bringup wiz_bringup.launch.py
```

Run tests:

```bash
colcon test --packages-select wiz_status --event-handlers console_direct+
```

## Packages

| Package          | Build type    | Contents                                         |
|-------------------|---------------|---------------------------------------------------|
| `wiz_interfaces`  | ament_cmake   | Custom `.msg` definitions                         |
| `wiz_status`      | ament_python  | `status_publisher`, `status_subscriber` nodes + tests |
| `wiz_vision`      | ament_python  | `camera_node`, `vision_node` (OpenCV CV pipeline) + tests |
| `wiz_terrain`     | ament_python  | `terrain_node` (traversability grid) + tests      |
| `wiz_bringup`     | ament_cmake   | Launch files                                      |

### Vision pipeline setup

`wiz_vision` needs the MobileNet-SSD model weights, which are **not** committed
to git (23MB binary, third-party artifact). Fetch them once:

```bash
bash ~/Wiz/scripts/download_vision_models.sh
```

This downloads `deploy.prototxt` + `mobilenet_iter_73000.caffemodel` (MIT
licensed, from chuanqi305/MobileNet-SSD) into `~/Wiz/models/mobilenet_ssd/`.
Without this, `vision_node` still runs (camera + feature detection + topics
all work) but logs an error and skips object detection/tracking.

Run it:
```bash
ros2 launch wiz_bringup vision.launch.py
# or everything (status + vision) together:
ros2 launch wiz_bringup all.launch.py
```

Launch arguments for `vision.launch.py`: `source_type` (`synthetic` default |
`file` | `webcam`), `source_path` (file path or webcam index), `frame_rate`,
`model_dir`. Example with a real video file:
```bash
ros2 launch wiz_bringup vision.launch.py source_type:=file source_path:=/path/to/video.mp4
```
`webcam` is implemented but untested in this sprint — WSL2 has no USB camera
by default and would need `usbipd-win` passthrough (see docs/ARCHITECTURE.md
companion-computer notes).

### Visualizing detections

Both `rqt_image_view` and `rviz2` are already installed (ROS 2 Desktop) and
work through WSLg on this Windows 11 host:
```bash
ros2 run rqt_image_view rqt_image_view /wiz/camera/annotated
```
or add an Image display on `/wiz/camera/annotated` in `rviz2`.

## Topics

All Wiz topics are namespaced under `/wiz/`:

| Topic                | Message type                    | QoS                 |
|----------------------|-----------------------------------|----------------------|
| `/wiz/system_health` | `wiz_interfaces/SystemHealth`   | reliable, depth 10  |
| `/wiz/telemetry`     | `wiz_interfaces/Telemetry`      | reliable, depth 10  |
| `/wiz/sensor_data`   | `wiz_interfaces/SensorReading`  | reliable, depth 10  |
| `/wiz/camera/raw`        | `sensor_msgs/Image` (bgr8)          | reliable, depth 10 |
| `/wiz/camera/annotated`  | `sensor_msgs/Image` (bgr8)          | reliable, depth 10 |
| `/wiz/vision/detections` | `wiz_interfaces/DetectionArray`     | reliable, depth 10 |
| `/wiz/terrain/grid`      | `nav_msgs/OccupancyGrid`            | reliable, depth 10 |
| `/wiz/terrain/overlay`   | `sensor_msgs/Image` (bgr8)          | reliable, depth 10 |

QoS is kept at simple reliable defaults for now. Revisit once real sensors
are attached (e.g. best-effort + smaller depth for a high-rate raw sensor
stream, where losing an occasional sample is fine but backpressure isn't).

## Messages

- `SystemHealth`: `header`, `armed`, `flight_mode`, `battery_voltage`,
  `battery_percent`, `ok`, `message`.
- `Telemetry`: `header`, `position` (`geometry_msgs/Point`), `orientation`
  (`geometry_msgs/Quaternion`), `linear_velocity` (`geometry_msgs/Vector3`).
- `SensorReading`: `header`, `sensor_name`, `value`, `unit` — a generic
  single named reading, extensible to any future sensor without a schema
  change.
- `Detection`: `track_id`, `label`, `confidence`, `x`, `y`, `width`, `height`
  (pixel coordinates in the source image). `track_id` is assigned by a
  centroid tracker and persists across frames while the object keeps moving
  smoothly; it is re-issued if the object disappears for more than 10 frames.
- `DetectionArray`: `header` + `Detection[]` — one array per processed frame.

## Units and frames

Following REP-103 / REP-105:
- Units are SI: meters, meters/second, radians, radians/second.
- `header.frame_id` on every message is `base_link` (the drone body frame:
  x-forward, y-left, z-up) until a localization/SLAM sprint introduces
  `odom` and `map`.

## Logging and error handling

Nodes use `get_logger()` exclusively — no `print()`:
- `info` for normal state (startup, routine status).
- `warn` for a recoverable condition (e.g. simulated low battery).
- `error` for a lost/stale topic (the subscriber runs a 1s watchdog timer
  and flags any topic silent for more than 5s).

## Data source

All data published this sprint is **simulated** — there is no link to the
physical flight controller yet. See docs/ARCHITECTURE.md: the flight-control
firmware stack (Betaflight vs. ArduPilot/PX4) is still an open decision, so a
real hardware bridge is deliberately out of scope until that's resolved.

## Terrain grid semantics

`/wiz/terrain/grid` is a `nav_msgs/OccupancyGrid`, but **its values are a
traversability cost, not an occupancy probability**:

| Value   | Meaning                          |
|---------|----------------------------------|
| `0`     | clear / low-cost terrain         |
| `100`   | impassable / highest-cost terrain |
| `-1`    | unknown                          |

Nothing in Sprint 4 estimates the probability that a cell is occupied. The
number answers "how hard would this be to cross", derived from image
appearance alone.

### Cost formula

Per cell, after averaging each channel down to the grid:

```
rough_term = clip((mean_roughness - roughness_threshold) * roughness_gain, 0, 100)
veg_term   = vegetation_fraction * veg_weight
cost       = clip(rough_term + veg_term, 0, 100)

if obstacle_fraction >= obstacle_fraction_threshold:
    cost = obstacle_cost        # a detected obstacle overrides the rest
```

### What this grid is, and is not

It is a **single-frame, camera-relative (local) traversability grid**. Each
frame yields an independent grid describing only what that frame shows.

It is **not** a persistent world map: no pose is consumed and nothing is
accumulated between frames. `header.frame_id` stays `base_link`, which is
acceptable precisely because there is no localization yet. `odom`/`map`
frames, frame-to-map transformation and multi-frame accumulation arrive in
Sprint 5.

Grid placement in `base_link` (x-forward, y-left, z-up): image row 0 is
farthest forward and becomes the largest `x` index; image column 0 is the
drone's left and becomes the largest `y` index. So `info.width` is the
forward extent (image rows) and `info.height` the lateral one (image
columns). The grid spans `x` in `[0, forward_extent]` and `y` in
`[-lateral/2, +lateral/2]`.

### Parameters

`terrain_node` exposes every tunable: `resize_width`, `grid_rows`,
`grid_cols`, `blur_kernel`, `veg_hsv_lower`, `veg_hsv_upper`, `veg_weight`,
`roughness_threshold`, `roughness_gain`, `canny_low`, `canny_high`,
`min_contour_area`, `obstacle_fraction_threshold`, `obstacle_cost`,
`morph_kernel`, `assumed_altitude_m`, `camera_hfov_deg`,
`unknown_border_cells`, `publish_overlay`. Invalid values are rejected at
startup rather than silently clamped. Defaults are mirrored in
`config/default.yaml`.

`assumed_altitude_m` and `camera_hfov_deg` are **assumptions**, not
measurements — there is no altitude telemetry yet, and the node logs this at
startup.
