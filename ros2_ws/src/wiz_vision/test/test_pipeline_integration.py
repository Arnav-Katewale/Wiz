import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from wiz_interfaces.msg import DetectionArray
from wiz_vision.camera_node import CameraNode
from wiz_vision.vision_node import VisionNode


def _spin_until(nodes, predicate, timeout_sec=8.0):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        for node in nodes:
            rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return True
    return False


def test_camera_and_vision_nodes_publish_together():
    rclpy.init()
    camera = CameraNode()
    vision = VisionNode()
    listener = Node('test_pipeline_listener')
    received = {}

    listener.create_subscription(
        Image, '/wiz/camera/annotated', lambda m: received.setdefault('annotated', m), 10)
    listener.create_subscription(
        DetectionArray, '/wiz/vision/detections', lambda m: received.setdefault('detections', m), 10)

    try:
        ok = _spin_until(
            [camera, vision, listener],
            lambda: {'annotated', 'detections'} <= received.keys(),
            timeout_sec=8.0,
        )
        assert ok, 'did not receive annotated image and detections within timeout'
        assert received['annotated'].encoding == 'bgr8'
        assert received['annotated'].width == 640
        assert received['annotated'].height == 480
    finally:
        camera.destroy_node()
        vision.destroy_node()
        listener.destroy_node()
        rclpy.shutdown()
