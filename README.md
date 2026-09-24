# Wiz

Wiz is an autonomous-flight upgrade for an existing FPV quadcopter (SpeedyBee
F405 stack). It's being built sprint by sprint: this repo starts as a plain
software foundation, with computer vision, ROS 2, SLAM, and autonomous
navigation layered in over future sprints.

## Stack

| Layer            | Choice                                   | Status        |
|-------------------|-------------------------------------------|---------------|
| OS                | Windows 11 host + WSL2 (Ubuntu 22.04 LTS) | active        |
| ROS 2             | Humble Hawksbill (Desktop)                | active        |
| Language          | Python 3.10+                              | active        |
| Computer vision   | OpenCV (ORB features + MobileNet-SSD)     | active        |
| Simulation        | Gazebo (paired with Humble)               | not installed yet |
| Flight control    | TBD (Betaflight vs. ArduPilot/PX4)        | open decision |

See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for the reasoning behind
each choice, [docs/ROS2_CONVENTIONS.md](docs/ROS2_CONVENTIONS.md) for the ROS 2
package/topic/message conventions, and [docs/ROADMAP.md](docs/ROADMAP.md) for
what's planned next.

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

### ROS 2

```bash
source /opt/ros/humble/setup.bash
cd ~/Wiz/ros2_ws
colcon build --symlink-install
source install/setup.bash

ros2 launch wiz_bringup wiz_bringup.launch.py   # run both nodes together
colcon test --packages-select wiz_status --event-handlers console_direct+
```

See [docs/ROS2_CONVENTIONS.md](docs/ROS2_CONVENTIONS.md) for topics, message
shapes, and conventions.

### Computer vision

One-time setup (downloads the ~23MB MobileNet-SSD model, not committed to git):
```bash
bash scripts/download_vision_models.sh
```

Run it:
```bash
ros2 launch wiz_bringup vision.launch.py          # camera + vision only
ros2 launch wiz_bringup all.launch.py             # status + vision together
ros2 run rqt_image_view rqt_image_view /wiz/camera/annotated   # view it (WSLg)
```

Defaults to a synthetic test pattern; pass `source_type:=file
source_path:=/path/to/video.mp4` to run it against a real video. See
[docs/ROS2_CONVENTIONS.md](docs/ROS2_CONVENTIONS.md) for details.
