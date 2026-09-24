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
