# Wiz Architecture

Status: Sprint 2 (ROS 2 communication framework). Subsystems marked "not yet
implemented" below still land in later sprints.

## Host & OS
- Host: Windows 11
- Dev/runtime OS: Ubuntu 22.04 LTS (Jammy) via WSL2. ROS 2 and its tooling are
  Linux-first; WSLg provides GUI passthrough on this Windows 11 host for
  simulation/visualization tools (RViz2 is installed; not yet used in a
  sprint).

## ROS 2 Distribution
- ROS 2 Humble Hawksbill (Desktop variant — includes RViz2), installed in
  Sprint 2 via the official apt repo. Matches Ubuntu 22.04, supported through
  May 2027. Workspace lives at `ros2_ws/`; see docs/ROS2_CONVENTIONS.md for
  package layout, topics, and message conventions.

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
├── src/wiz/                     Python package (Sprint 1, unchanged)
│   ├── main.py                  entry point (`wiz` console script)
│   ├── cv/                       still empty - real CV lives in ros2_ws/src/wiz_vision
│   ├── navigation/                navigation/planning (empty - future sprint)
│   ├── simulation/                sim integration code (empty - future sprint)
│   └── config/                    config-loading helpers (empty - future sprint)
├── config/                      runtime config files (YAML)
├── models/                      downloaded model weights (gitignored, see scripts/)
├── scripts/
│   └── download_vision_models.sh   fetches MobileNet-SSD weights
├── ros2_ws/                     colcon workspace for ROS 2 packages
│   └── src/
│       ├── wiz_interfaces/       custom .msg definitions (ament_cmake)
│       ├── wiz_status/            status publisher/subscriber nodes + tests
│       ├── wiz_vision/            camera capture + CV processing nodes + tests
│       └── wiz_bringup/           launch files
├── tests/                       pytest test suite (src/wiz only)
└── docs/                        architecture, roadmap, ROS 2 conventions
```

Note: `src/wiz/cv/` was reserved in Sprint 1 as a placeholder but is intentionally
still empty — the actual computer-vision implementation (Sprint 3) is a ROS 2
package (`ros2_ws/src/wiz_vision/`), consistent with how Sprint 2's status
nodes also live under `ros2_ws/` rather than in `src/wiz/`.
