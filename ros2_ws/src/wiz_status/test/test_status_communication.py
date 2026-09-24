import time

import rclpy
from rclpy.node import Node

from wiz_status.status_publisher import StatusPublisher
from wiz_interfaces.msg import SystemHealth, Telemetry, SensorReading


def _spin_until(nodes, predicate, timeout_sec=5.0):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        for node in nodes:
            rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return True
    return False


def test_all_topics_reach_subscriber_with_valid_fields():
    rclpy.init()
    publisher = StatusPublisher()
    listener = Node('test_listener_fields')
    received = {}

    listener.create_subscription(
        SystemHealth, '/wiz/system_health', lambda m: received.setdefault('health', m), 10)
    listener.create_subscription(
        Telemetry, '/wiz/telemetry', lambda m: received.setdefault('telemetry', m), 10)
    listener.create_subscription(
        SensorReading, '/wiz/sensor_data', lambda m: received.setdefault('sensor', m), 10)

    try:
        ok = _spin_until(
            [publisher, listener],
            lambda: {'health', 'telemetry', 'sensor'} <= received.keys(),
            timeout_sec=5.0,
        )
        assert ok, 'did not receive all three topics within timeout'
        assert received['health'].armed is True
        assert 0.0 <= received['health'].battery_percent <= 100.0
        assert received['sensor'].sensor_name == 'sim_rangefinder'
        assert received['telemetry'].position.z == 1.5
    finally:
        publisher.destroy_node()
        listener.destroy_node()
        rclpy.shutdown()


def test_sustained_delivery_over_multiple_ticks():
    rclpy.init()
    publisher = StatusPublisher()
    listener = Node('test_listener_count')
    count = {'health': 0}

    listener.create_subscription(
        SystemHealth, '/wiz/system_health', lambda m: count.__setitem__('health', count['health'] + 1), 10)

    try:
        ok = _spin_until([publisher, listener], lambda: count['health'] >= 3, timeout_sec=5.0)
        assert ok, 'did not receive at least 3 messages within timeout'
    finally:
        publisher.destroy_node()
        listener.destroy_node()
        rclpy.shutdown()
