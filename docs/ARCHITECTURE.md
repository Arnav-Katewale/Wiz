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
│   ├── download_vision_models.sh   fetches MobileNet-SSD weights
│   └── render_terrain_demo.py      renders the terrain pipeline without ROS
├── ros2_ws/                     colcon workspace for ROS 2 packages
│   └── src/
│       ├── wiz_interfaces/       custom .msg definitions (ament_cmake)
│       ├── wiz_status/            status publisher/subscriber nodes + tests
│       ├── wiz_vision/            camera capture + CV processing nodes + tests
│       ├── wiz_terrain/           terrain analysis + traversability grid + tests
│       └── wiz_bringup/           launch files
├── tests/                       pytest test suite (src/wiz only)
└── docs/                        architecture, roadmap, ROS 2 conventions
```

Note: `src/wiz/cv/` was reserved in Sprint 1 as a placeholder but is intentionally
still empty — the actual computer-vision implementation (Sprint 3) is a ROS 2
package (`ros2_ws/src/wiz_vision/`), consistent with how Sprint 2's status
nodes also live under `ros2_ws/` rather than in `src/wiz/`.

## Terrain Mapping (Sprint 4)

`ros2_ws/src/wiz_terrain/` follows the same split as `wiz_vision`: the
algorithms live in `terrain.py` with **no ROS imports**, so they are testable
without a ROS installation, and `terrain_node.py` is a thin ROS wrapper.

Pipeline, per frame:

1. downscale (`INTER_AREA`) and Gaussian blur
2. HSV threshold for vegetation (hue is stabler than BGR under changing light)
3. Sobel gradient magnitude on the **unblurred** grey as a roughness proxy
4. Canny + area-filtered contours for discrete obstacles
5. morphological cleanup — open+close on region masks, close only on edges
   (a 1-pixel Canny edge would not survive an opening)
6. average each channel down to grid cells and combine into a cost

Roughness deliberately reads the sharp grey: blurring first removes exactly
the high-frequency texture it exists to measure, which made coarse rubble
score as clear ground.

### Why OccupancyGrid

`nav_msgs/OccupancyGrid` is reused as the transport because it is standard
and rviz2 renders it natively, avoiding a custom message. Its value range is
being used as a **traversability cost**, *not* an occupancy probability —
see docs/ROS2_CONVENTIONS.md.

### Scale assumptions

`OccupancyGrid.info.resolution` is computed from declared parameters, not
measured:

```
ground_sample_distance = 2 * assumed_altitude_m * tan(camera_hfov_deg / 2) / image_width
cell_size              = ground_sample_distance * (image_width / grid_cols)
```

This assumes a nadir camera, flat level ground, exact altitude, and a
rectilinear undistorted lens. None of these are verified in Sprint 4; the
node logs the assumption at startup. Sprint 5 replaces the altitude
assumption with real pose.

`OccupancyGrid` carries a single `resolution`, so cells must be square. The
defaults (320 px wide, 24x32 grid) give exactly 10x10 px cells; the node
warns if a parameter choice makes forward and lateral cell sizes differ by
more than 10%.
