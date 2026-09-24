# Wiz

Wiz is an autonomous-flight upgrade for an existing FPV quadcopter (SpeedyBee
F405 stack). It's being built sprint by sprint: this repo starts as a plain
software foundation, with computer vision, ROS 2, SLAM, and autonomous
navigation layered in over future sprints.

## Stack (Sprint 1 decisions)

| Layer            | Choice                                   | Status        |
|-------------------|-------------------------------------------|---------------|
| OS                | Windows 11 host + WSL2 (Ubuntu 22.04 LTS) | active        |
| ROS 2             | Humble Hawksbill                          | not installed yet |
| Language          | Python 3.10+                              | active        |
| Simulation        | Gazebo (paired with Humble)               | not installed yet |
| Flight control    | TBD (Betaflight vs. ArduPilot/PX4)        | open decision |

See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for the reasoning behind
each choice, and [docs/ROADMAP.md](docs/ROADMAP.md) for what's planned next.

## Development process

Built in sprints: one focused goal at a time, planned before coding, tested
before merging, documented after. See [DEVLOG.md](DEVLOG.md) for the history.

## Getting started

Everything runs inside WSL2 (Ubuntu 22.04).

```bash
cd ~/Wiz
python3 -m venv .venv
source .venv/bin/activate
pip install -e ".[dev]"

wiz              # run the entry point
pytest           # run the test suite
```
