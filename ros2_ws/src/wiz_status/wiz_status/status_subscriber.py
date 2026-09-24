import time

import rclpy
from rclpy.node import Node

from wiz_interfaces.msg import SystemHealth, Telemetry, SensorReading

STALE_THRESHOLD_SEC = 5.0
WATCHDOG_PERIOD_SEC = 1.0


class StatusSubscriber(Node):

    def __init__(self):
        super().__init__('status_subscriber')

        self._last_seen = {'system_health': None, 'telemetry': None, 'sensor_data': None}

        self.create_subscription(SystemHealth, '/wiz/system_health', self._on_health, 10)
        self.create_subscription(Telemetry, '/wiz/telemetry', self._on_telemetry, 10)
        self.create_subscription(SensorReading, '/wiz/sensor_data', self._on_sensor, 10)

        self.create_timer(WATCHDOG_PERIOD_SEC, self._check_staleness)
        self.get_logger().info(
            'status_subscriber started, watching /wiz/system_health, /wiz/telemetry, /wiz/sensor_data'
        )

    def _on_health(self, msg: SystemHealth):
        self._last_seen['system_health'] = time.monotonic()
        state = 'OK' if msg.ok else 'WARN'
        self.get_logger().info(
            f'[{state}] armed={msg.armed} mode={msg.flight_mode} '
            f'battery={msg.battery_percent:.1f}% ({msg.message})'
        )

    def _on_telemetry(self, msg: Telemetry):
        self._last_seen['telemetry'] = time.monotonic()

    def _on_sensor(self, msg: SensorReading):
        self._last_seen['sensor_data'] = time.monotonic()

    def _check_staleness(self):
        now = time.monotonic()
        for topic, last in self._last_seen.items():
            if last is None:
                continue
            age = now - last
            if age > STALE_THRESHOLD_SEC:
                self.get_logger().error(f'{topic} is stale: no message in {age:.1f}s')


def main(args=None):
    rclpy.init(args=args)
    node = StatusSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
