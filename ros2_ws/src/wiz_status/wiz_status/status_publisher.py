import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Quaternion, Vector3

from wiz_interfaces.msg import SystemHealth, Telemetry, SensorReading

PUBLISH_RATE_HZ = 2.0
LOW_BATTERY_THRESHOLD = 20.0
BATTERY_DRAIN_PER_TICK = 0.05


class StatusPublisher(Node):

    def __init__(self):
        super().__init__('status_publisher')

        self._health_pub = self.create_publisher(SystemHealth, '/wiz/system_health', 10)
        self._telemetry_pub = self.create_publisher(Telemetry, '/wiz/telemetry', 10)
        self._sensor_pub = self.create_publisher(SensorReading, '/wiz/sensor_data', 10)

        self._battery_percent = 100.0
        self._start_time = time.monotonic()

        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._tick)
        self.get_logger().info(
            f'status_publisher started, publishing simulated telemetry at {PUBLISH_RATE_HZ:.1f} Hz'
        )

    def _header(self) -> Header:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'base_link'
        return header

    def _tick(self):
        self._battery_percent = max(0.0, self._battery_percent - BATTERY_DRAIN_PER_TICK)
        self._publish_health()
        self._publish_telemetry()
        self._publish_sensor_reading()

    def _publish_health(self):
        msg = SystemHealth()
        msg.header = self._header()
        msg.armed = True
        msg.flight_mode = 'SIMULATED'
        msg.battery_voltage = 11.1 * (self._battery_percent / 100.0)
        msg.battery_percent = self._battery_percent
        msg.ok = self._battery_percent > LOW_BATTERY_THRESHOLD
        msg.message = 'nominal' if msg.ok else 'low battery'

        if not msg.ok:
            self.get_logger().warn(f'battery low: {self._battery_percent:.1f}%')

        self._health_pub.publish(msg)

    def _publish_telemetry(self):
        t = time.monotonic() - self._start_time
        msg = Telemetry()
        msg.header = self._header()
        msg.position = Point(x=math.sin(t * 0.1), y=math.cos(t * 0.1), z=1.5)
        msg.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        msg.linear_velocity = Vector3(x=0.0, y=0.0, z=0.0)
        self._telemetry_pub.publish(msg)

    def _publish_sensor_reading(self):
        msg = SensorReading()
        msg.header = self._header()
        msg.sensor_name = 'sim_rangefinder'
        msg.value = 1.5 + 0.1 * math.sin(time.monotonic())
        msg.unit = 'm'
        self._sensor_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StatusPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
