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

## Candidate future sprints (order/scope not yet committed)
- Simulation environment: Gazebo + ROS 2 Humble bring-up
- Flight-control decision: resolve the Betaflight/ArduPilot/PX4 question
  (see docs/ARCHITECTURE.md) — a likely prerequisite for any real hardware
  bridge into the `/wiz/*` topics
- Navigation: waypoint following in simulation
- SLAM: mapping + localization
- Hardware bring-up: companion computer on the physical quad, real MSP/MAVLink
  data and a real camera replacing the simulated/file-based sources
- Vision improvements: real webcam input (usbipd-win passthrough), swap or
  fine-tune the detector, address the vision_node processing-rate limitation
  noted in DEVLOG.md
