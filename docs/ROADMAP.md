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

## Candidate future sprints (order/scope not yet committed)
- Computer vision: camera capture + a first basic vision task
- Simulation environment: Gazebo + ROS 2 Humble bring-up
- Flight-control decision: resolve the Betaflight/ArduPilot/PX4 question
  (see docs/ARCHITECTURE.md) — a likely prerequisite for any real hardware
  bridge into the `/wiz/*` topics
- Navigation: waypoint following in simulation
- SLAM: mapping + localization
- Hardware bring-up: companion computer on the physical quad, real MSP/MAVLink
  data replacing the simulated publisher
