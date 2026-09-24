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
