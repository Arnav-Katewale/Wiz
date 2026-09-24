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
