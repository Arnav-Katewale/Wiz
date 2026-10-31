# Wiz Roadmap

Tracked as sprints. One sprint is in progress at a time; scope for a sprint
is only locked in when it starts, not committed here in advance.

## Sprint 1 — Repository & Foundation (done)
Repo, project structure, documented stack decisions, minimal working Python
package with a passing test.

## Sprint 2 — ROS 2 Integration & Drone Communication (done)
ROS 2 Humble installed; `wiz_interfaces`/`wiz_status`/`wiz_bringup` packages;
simulated status publisher + subscriber communicating over `/wiz/*` topics;
common launch file; message/frame/QoS conventions documented; automated
communication tests.

## Sprint 3 — Computer Vision & Camera Processing (done)
`wiz_vision` package: camera_node (synthetic/file/webcam-capable) + vision_node
(ORB feature detection, MobileNet-SSD object detection, centroid tracking),
publishing annotated frames and structured detections over ROS 2. Tested on
real sample imagery, not just synthetic data.

## Sprint 4 — Terrain Mapping (done)
`wiz_terrain` package: ROS-independent `terrain.py` (HSV vegetation
segmentation, Sobel roughness, Canny/contour obstacles, morphological
cleanup) plus a `terrain_node` ROS wrapper publishing a single-frame,
camera-relative traversability grid as `nav_msgs/OccupancyGrid` and an
annotated overlay. Scale derived from declared altitude/FOV assumptions.
Tested on synthetic imagery only — no real-terrain validation.

## Sprint 5 — Pose-aware multi-frame terrain mapping (next, not started)
Consume pose (position, altitude, heading), transform per-frame observations
into a persistent map with `odom`/`map` frames, accumulate across frames with
cell confidence and repeated-observation weighting. This is where the
single-frame limitation of Sprint 4 gets lifted.

## Candidate future sprints (order/scope not yet committed)
- Simulation environment: Gazebo + ROS 2 Humble bring-up
- Flight-control decision: resolve the Betaflight/ArduPilot/PX4 question
  (see docs/ARCHITECTURE.md) — a likely prerequisite for any real hardware
  bridge into the `/wiz/*` topics
- Navigation: waypoint following in simulation
- SLAM: full mapping + localization (builds on Sprint 5)
- Hardware bring-up: companion computer on the physical quad, real MSP/MAVLink
  data and a real camera replacing the simulated/file-based sources
- Vision improvements: real webcam input (usbipd-win passthrough), swap or
  fine-tune the detector, address the vision_node processing-rate limitation
  noted in DEVLOG.md
