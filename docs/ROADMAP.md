# Wiz Roadmap

Tracked as sprints. One sprint is in progress at a time; scope for a sprint
is only locked in when it starts, not committed here in advance.

## Sprint 1 — Repository & Foundation (this sprint)
Repo, project structure, documented stack decisions, minimal working Python
package with a passing test.

## Candidate future sprints (order/scope not yet committed)
- Computer vision: camera capture + a first basic vision task
- Simulation environment: Gazebo + ROS 2 Humble bring-up
- ROS 2 integration: first nodes/topics inside `ros2_ws/`
- Flight-control decision: resolve the Betaflight/ArduPilot/PX4 question
  (see docs/ARCHITECTURE.md)
- Navigation: waypoint following in simulation
- SLAM: mapping + localization
- Hardware bring-up: companion computer on the physical quad
