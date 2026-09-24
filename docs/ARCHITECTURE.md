# Wiz Architecture

Status: Sprint 1 (foundation). Most subsystems below are documented decisions,
not yet implemented — they land in later sprints.

## Host & OS
- Host: Windows 11
- Dev/runtime OS: Ubuntu 22.04 LTS (Jammy) via WSL2. ROS 2 and its tooling are
  Linux-first; WSLg provides GUI passthrough on this Windows 11 host for
  later simulation/visualization tools (RViz, Gazebo).

## ROS 2 Distribution
- ROS 2 Humble Hawksbill — the LTS release matched to Ubuntu 22.04 (supported
  through May 2027). Not installed yet; arrives in the ROS 2 integration
  sprint.

## Programming Languages
- Python 3.10+ is the primary language (perception, planning, tooling, node
  glue). C++ is not used yet and will only be introduced later for a specific
  ROS 2 node that needs it for a tight control loop.

## Simulation Environment
- Gazebo, paired with ROS 2 Humble (final variant — Classic 11 vs. Harmonic
  via ros_gz — to be decided in the simulation sprint). Chosen for native
  ROS 2 integration and existing ArduPilot/PX4 SITL support. Not installed
  yet.

## Flight-Control Stack — OPEN DECISION
The existing quad (see project history) runs **Betaflight 4.5.2** on a
SpeedyBee F405. Betaflight is a manual/acro-flight firmware: it has no
standard autonomous offboard/guided mode and doesn't speak MAVLink natively.
Autonomous navigation (SLAM-driven waypoints, velocity/position commands from
a companion computer) needs a stack that does — i.e. **ArduPilot** or
**PX4**, both of which support the F405 board family.

Decision deferred to the flight-control integration sprint. Options on the
table:
1. Reflash this FC to ArduPilot or PX4.
2. Keep Betaflight for manual flight; treat autonomy as simulation-only until
   a decision is made.
3. Add a second FC dedicated to autonomous flight.

Flagging this now, in Sprint 1, so it isn't a surprise once navigation work
starts.

## Companion Computer
TBD — not required for Sprint 1 (software-only foundation).

## Repository Layout
```
Wiz/
├── src/wiz/            Python package
│   ├── main.py         entry point (`wiz` console script)
│   ├── cv/              computer vision (empty - future sprint)
│   ├── navigation/       navigation/planning (empty - future sprint)
│   ├── simulation/       sim integration code (empty - future sprint)
│   └── config/           config-loading helpers (empty - future sprint)
├── config/              runtime config files (YAML)
├── ros2_ws/             reserved colcon workspace for ROS 2 packages
├── tests/               pytest test suite
└── docs/                architecture, roadmap
```
