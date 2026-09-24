import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('source_type', 'synthetic')
        self.declare_parameter('source_path', '')
        self.declare_parameter('frame_rate', 15.0)
        self.declare_parameter('loop', True)

        self._source_type = self.get_parameter('source_type').value
        self._source_path = self.get_parameter('source_path').value
        self._frame_rate = float(self.get_parameter('frame_rate').value)
        self._loop = bool(self.get_parameter('loop').value)

        self._bridge = CvBridge()
        self._publisher = self.create_publisher(Image, '/wiz/camera/raw', 10)
        self._capture = None
        self._tick_count = 0

        if self._source_type == 'file':
            self._capture = cv2.VideoCapture(self._source_path)
            if not self._capture.isOpened():
                raise RuntimeError(f'could not open video file: {self._source_path}')
        elif self._source_type == 'webcam':
            self._capture = cv2.VideoCapture(int(self._source_path or 0))
            if not self._capture.isOpened():
                self.get_logger().error(
                    f'could not open webcam device {self._source_path!r} '
                    '(WSL2 needs usbipd-win USB passthrough for this to work)'
                )
        elif self._source_type != 'synthetic':
            raise ValueError(f'unknown source_type: {self._source_type}')

        self.create_timer(1.0 / self._frame_rate, self._tick)
        self.get_logger().info(
            f'camera_node started, source_type={self._source_type!r}, '
            f'publishing at {self._frame_rate:.1f} Hz'
        )

    def _tick(self):
        frame = self._read_frame()
        if frame is None:
            return
        msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        self._publisher.publish(msg)

    def _read_frame(self):
        if self._source_type == 'synthetic':
            return self._synthetic_frame()

        if self._capture is None or not self._capture.isOpened():
            return None

        ok, frame = self._capture.read()
        if not ok:
            if self._loop and self._source_type == 'file':
                self._capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ok, frame = self._capture.read()
            if not ok:
                self.get_logger().warn('failed to read frame from capture source')
                return None
        return frame

    def _synthetic_frame(self, width=640, height=480):
        frame = np.full((height, width, 3), 40, dtype=np.uint8)
        t = self._tick_count
        self._tick_count += 1
        cx = int(width / 2 + (width / 2 - 30) * np.sin(t * 0.05))
        cy = int(height / 2 + (height / 2 - 30) * np.cos(t * 0.03))
        cv2.circle(frame, (cx, cy), 25, (0, 200, 255), -1)
        return frame

    def destroy_node(self):
        if self._capture is not None:
            self._capture.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
