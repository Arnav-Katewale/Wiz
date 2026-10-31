# Wiz Development Log

## Sprint 1 — Repository & Foundation
**Date:** 2026-09-23

**Objective:** Initialize the Wiz repository and lay down a working project
foundation for all future sprints.

**Done:**
- Initialized git repo, WSL2 (Ubuntu 22.04) dev environment.
- Installed Python 3.10 venv tooling and GitHub CLI in WSL2.
- Created project structure: `src/wiz/` package (with `cv/`, `navigation/`,
  `simulation/`, `config/` placeholders), `config/`, `ros2_ws/`, `tests/`,
  `docs/`.
- Documented stack decisions in docs/ARCHITECTURE.md: WSL2/Ubuntu 22.04,
  ROS 2 Humble, Python 3.10+, Gazebo (paired with Humble). Flight-control
  stack explicitly left as an open decision (Betaflight can't do autonomous
  offboard control — flagged for a future sprint).
- Added a real entry point (`wiz` console script / `wiz.main:main`) and a
  passing pytest suite (`tests/test_main.py`).
- Wrote README, ARCHITECTURE, ROADMAP docs.

**Tested:**
- `pip install -e ".[dev]"` succeeds in a fresh venv.
- `wiz` console command runs and prints status.
- `pytest` passes (2/2 tests).

**Open issues / risks:**
- Flight-control stack undecided (Betaflight vs. ArduPilot/PX4) — must be
  resolved before any real offboard/autonomous flight sprint.
- No CI configured yet.
- No LICENSE chosen yet for the public repo.

**Next candidates:** see docs/ROADMAP.md. Waiting on go-ahead before starting
any further sprint.

## Sprint 2 — ROS 2 Integration & Drone Communication
**Date:** 2026-09-24
**Deliverable:** Wiz v0.2 — ROS 2 Communication Framework

**Objective:** Establish a working ROS 2 system so the drone's software
components can communicate — nodes, topics, messages, launch files, and
tests, all on simulated data (no physical FC link yet; see the open
flight-control decision below).

**Done:**
- Installed ROS 2 Humble Desktop (incl. RViz2) + ros-dev-tools in WSL2.
- Created `ros2_ws/` with three packages:
  - `wiz_interfaces` (ament_cmake): `SystemHealth.msg`, `Telemetry.msg`,
    `SensorReading.msg`.
  - `wiz_status` (ament_python): `status_publisher` and `status_subscriber`
    nodes.
  - `wiz_bringup` (ament_cmake): `wiz_bringup.launch.py`, starts both nodes.
- Topics: `/wiz/system_health`, `/wiz/telemetry`, `/wiz/sensor_data`
  (reliable QoS, depth 10).
- `status_publisher` simulates a slowly draining battery, position, and a
  fake rangefinder reading at 2 Hz; flips `SystemHealth.ok` to false and logs
  a warning below 20% battery.
- `status_subscriber` logs a consolidated status line per health update and
  runs a 1s watchdog that logs an error if any topic goes stale (>5s silent).
- Documented conventions in docs/ROS2_CONVENTIONS.md: topic namespace, SI
  units, `base_link` frame_id (REP-103/105), QoS choice, logging levels.
- Updated docs/ARCHITECTURE.md and README.md to reflect ROS 2 now installed.

**Tested:**
- `colcon build` — all 3 packages build cleanly.
- `colcon test --packages-select wiz_status` — 2/2 pytest cases pass
  (all three topics deliver valid fields; sustained delivery over multiple
  ticks).
- `ros2 launch wiz_bringup wiz_bringup.launch.py` — manually verified both
  nodes start together and the subscriber logs live status from the
  publisher.
- `ros2 topic echo` on `/wiz/sensor_data` and `/wiz/telemetry` — confirmed
  message fields and `frame_id` match the documented schema.

**Open issues / risks:**
- Flight-control stack still undecided (carried over from Sprint 1) — real
  hardware data will need this resolved first.
- QoS is default "reliable" everywhere; not yet tuned for real sensor rates.
- No CI configured yet.
- No LICENSE chosen yet for the public repo.

**Next candidates:** see docs/ROADMAP.md. Waiting on go-ahead before starting
any further sprint.

## Sprint 3 — Computer Vision & Camera Processing
**Date:** 2026-09-24
**Deliverable:** Wiz v0.3 — Computer Vision Pipeline

**Objective:** Enable Wiz to process camera images and extract useful visual
information, published through ROS 2.

**Key decision:** detection method was left up to hardware capability. This
machine (13th-gen i9, 24C/32T, 32GB RAM) comfortably handles CPU inference of
a lightweight pretrained detector, so the pipeline uses a real pretrained
model (MobileNet-SSD, VOC classes) instead of pure classical/motion-based
detection, while still keeping classical ORB feature detection as a separate
stage per the task breakdown.

**Done:**
- New `wiz_vision` package (`ros2_ws/src/wiz_vision/`):
  - `camera_node`: publishes `/wiz/camera/raw`. Source is configurable
    (`source_type`: `synthetic` default | `file` | `webcam`); synthetic mode
    needs no external file and always works.
  - `vision_node`: subscribes to raw frames, runs preprocessing (grayscale +
    blur), ORB feature detection, MobileNet-SSD object detection (Caffe,
    OpenCV DNN module, VOC0712 classes), and a centroid tracker that assigns
    persistent IDs across frames. Publishes annotated frames
    (`/wiz/camera/annotated`) and structured results
    (`/wiz/vision/detections`, new `Detection`/`DetectionArray` messages in
    `wiz_interfaces`).
- Model weights (23MB, MIT licensed, chuanqi305/MobileNet-SSD) are fetched by
  `scripts/download_vision_models.sh` rather than committed to git;
  `vision_node` degrades gracefully (logs an error, skips detection, still
  publishes features/topics) if they're missing.
- `wiz_bringup`: added `vision.launch.py` (camera+vision) and `all.launch.py`
  (status+vision together).
- Real error handling: cv_bridge conversion failures are caught and logged
  rather than crashing the node; a missing model is logged, not fatal.
- Updated docs/ROS2_CONVENTIONS.md with the new topics/messages, model setup
  step, and visualization instructions (`rqt_image_view` via WSLg).

**Tested:**
- `colcon build` — all 4 packages (incl. new `wiz_vision`) build clean.
- `colcon test --packages-select wiz_vision` — 11/11 pytest cases pass:
  preprocessing shape/dtype, ORB finds features on a textured image and none
  on a blank one, centroid tracker (registration, ID persistence while
  moving, drop-after-disappeared, multiple simultaneous objects), missing-
  model error handling, and a full rclpy integration test that runs
  camera_node + vision_node together and checks real topic output.
- **Semantic correctness**, not just plumbing: ran the detector on a real
  sample photo (`test/fixtures/sample_person.jpg`, a Pascal VOC image) and
  confirmed it correctly finds `person` at ~1.00 confidence — asserted in
  `test_detect_objects_finds_person_in_sample_image` and additionally
  eyeballed by rendering the annotated output to PNG and viewing it directly:
  correct bounding box, label, and ORB keypoints landing on actual texture
  (eyes, hair, jacket embroidery).
- Ran `ros2 launch wiz_bringup vision.launch.py` live: both nodes start,
  `/wiz/camera/annotated` publishes 640x480 bgr8 at ~12 Hz,
  `/wiz/vision/detections` publishes per frame (empty on the synthetic
  source, as expected — a plain circle isn't a VOC class).

**Open issues / risks:**
- `vision_node` processing time (ORB + DNN forward pass, ~80ms/frame) is
  close to camera_node's 15 Hz publish period, so the effective annotated
  rate is ~12 Hz and could lag further under sustained load. Fine for this
  sprint; worth revisiting (e.g. throttle camera rate, or process every Nth
  frame) if a later sprint needs tighter real-time behavior.
- `webcam` source mode is implemented but untested — WSL2 has no camera
  passthrough by default (would need `usbipd-win`), carried over as a noted
  limitation rather than solved here.
- Detector is a fixed pretrained model (VOC 2007, 20 classes) — no
  fine-tuning/training pipeline, by design for an "initial algorithm."
- Flight-control stack and LICENSE choice still open (carried over).

**Next candidates:** see docs/ROADMAP.md. Waiting on go-ahead before starting
any further sprint.

## Sprint 4 — Terrain Mapping

**Goal:** turn camera frames into a terrain representation the drone could
reason about, as the first step toward mapping.

**Delivered — "Wiz v0.4 — Terrain Mapping (single-frame)":**
- New `wiz_terrain` ROS 2 package, structured like `wiz_vision`:
  `terrain.py` holds the algorithms with **no ROS imports** (so the whole
  layer tests without a ROS installation) and `terrain_node.py` is a thin
  wrapper.
- Vision pipeline: `INTER_AREA` downscale + Gaussian blur, HSV vegetation
  segmentation, Sobel gradient magnitude for surface roughness, Canny +
  area-filtered contours for discrete obstacles, morphological cleanup.
- Output: a single-frame, camera-relative traversability grid published as
  `nav_msgs/OccupancyGrid` on `/wiz/terrain/grid` (standard message, no new
  custom type needed), plus an annotated overlay on `/wiz/terrain/overlay`.
- `scripts/render_terrain_demo.py` renders a four-panel montage (source,
  preprocessed, overlay, grid) with no ROS running.
- `terrain.launch.py` brings up camera + terrain together.

**Two implementation details worth recording:**
- *Morphology is not one-size-fits-all.* Region masks (vegetation) get
  open-then-close, but Canny edges are one pixel wide and an opening deletes
  them outright — edges get close only. The first version opened everything
  and found no obstacles at all.
- *Roughness must read the unblurred grey.* Running Sobel on the
  Gaussian-blurred image removed exactly the high-frequency texture that
  roughness exists to measure: a band of coarse rubble in the synthetic
  scene scored the same as flat ground. Caught by rendering the montage and
  looking at it, not by a passing test. Roughness now reads the sharp grey
  while Canny keeps the blurred one, and
  `test_textured_ground_costs_more_than_smooth_ground` locks the behaviour in.

**Testing — 61 tests, all passing:**
- 58 offline tests with no ROS environment at all (`test_terrain.py`,
  `test_grid.py`): input validation, parameter validation, preprocessing,
  HSV segmentation, roughness, obstacle extraction, cost semantics, cost
  monotonicity in roughness gain, unknown-cell handling, ground-sample-
  distance geometry, OccupancyGrid layout conversion, int8 range safety,
  and overlay rendering.
- 3 rclpy integration tests: camera_node + terrain_node publishing together,
  grid dimensions/frame/resolution/origin correctness, parameter overrides
  reaching the published message, and invalid parameters failing loudly at
  construction.
- Regression check: `wiz_vision` (11) and `wiz_status` (2) still pass.
- Live `ros2 launch wiz_bringup terrain.launch.py`: both nodes start,
  `/wiz/terrain/grid` publishes 24x32 at resolution 0.131 m/cell with origin
  y = -2.10 m (= -0.5 * 32 * 0.131, as designed).
- Visually inspected the rendered montage: the two solid blocks saturate at
  cost 100, vegetation patches read as elevated cost, the rubble band reads
  rough, and the smooth central path stays near 0.

**Assumptions and limitations (explicit):**
- **All imagery used is SYNTHETIC or file-based.** There has been no
  real-terrain, field, or in-flight validation, and none is claimed. The
  airframe is broken and has never flown.
- Metric scale is derived from declared `assumed_altitude_m` and
  `camera_hfov_deg` assuming a nadir camera over flat level ground with a
  rectilinear lens. None of that is measured; the node logs it as an
  assumption at startup.
- Single-frame only: every frame yields an independent grid. No pose, no
  accumulation, no persistence. `frame_id` stays `base_link`.
- `OccupancyGrid` values are a **traversability cost**, not occupancy
  probability.
- Classification is heuristic thresholding, not machine learning. No model
  is trained or used in this package.
- `-1` (unknown) is currently produced only by the optional
  `unknown_border_cells` parameter; it becomes meaningful in Sprint 5 for
  genuinely unobserved map cells.
- Cells must be square because `OccupancyGrid` carries one `resolution`; the
  node warns when parameters make them non-square.

**Carried over:** flight-control stack decision and LICENSE choice still
open; webcam source still untested (no WSL2 USB passthrough).

**Next:** Sprint 5 — pose-aware multi-frame terrain mapping (see
docs/ROADMAP.md). Waiting on go-ahead before starting.
