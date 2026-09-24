# ROS 2 Conventions

## Workspace

All ROS 2 packages live in `ros2_ws/src/`. Build from `ros2_ws/`:

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Run everything together:

```bash
ros2 launch wiz_bringup wiz_bringup.launch.py
```

Run tests:

```bash
colcon test --packages-select wiz_status --event-handlers console_direct+
```

## Packages

| Package          | Build type    | Contents                                         |
|-------------------|---------------|---------------------------------------------------|
| `wiz_interfaces`  | ament_cmake   | Custom `.msg` definitions                         |
| `wiz_status`      | ament_python  | `status_publisher`, `status_subscriber` nodes + tests |
| `wiz_bringup`     | ament_cmake   | Launch files                                      |

## Topics

All Wiz topics are namespaced under `/wiz/`:

| Topic                | Message type                    | QoS                 |
|----------------------|-----------------------------------|----------------------|
| `/wiz/system_health` | `wiz_interfaces/SystemHealth`   | reliable, depth 10  |
| `/wiz/telemetry`     | `wiz_interfaces/Telemetry`      | reliable, depth 10  |
| `/wiz/sensor_data`   | `wiz_interfaces/SensorReading`  | reliable, depth 10  |

QoS is kept at simple reliable defaults for now. Revisit once real sensors
are attached (e.g. best-effort + smaller depth for a high-rate raw sensor
stream, where losing an occasional sample is fine but backpressure isn't).

## Messages

- `SystemHealth`: `header`, `armed`, `flight_mode`, `battery_voltage`,
  `battery_percent`, `ok`, `message`.
- `Telemetry`: `header`, `position` (`geometry_msgs/Point`), `orientation`
  (`geometry_msgs/Quaternion`), `linear_velocity` (`geometry_msgs/Vector3`).
- `SensorReading`: `header`, `sensor_name`, `value`, `unit` — a generic
  single named reading, extensible to any future sensor without a schema
  change.

## Units and frames

Following REP-103 / REP-105:
- Units are SI: meters, meters/second, radians, radians/second.
- `header.frame_id` on every message is `base_link` (the drone body frame:
  x-forward, y-left, z-up) until a localization/SLAM sprint introduces
  `odom` and `map`.

## Logging and error handling

Nodes use `get_logger()` exclusively — no `print()`:
- `info` for normal state (startup, routine status).
- `warn` for a recoverable condition (e.g. simulated low battery).
- `error` for a lost/stale topic (the subscriber runs a 1s watchdog timer
  and flags any topic silent for more than 5s).

## Data source

All data published this sprint is **simulated** — there is no link to the
physical flight controller yet. See docs/ARCHITECTURE.md: the flight-control
firmware stack (Betaflight vs. ArduPilot/PX4) is still an open decision, so a
real hardware bridge is deliberately out of scope until that's resolved.
