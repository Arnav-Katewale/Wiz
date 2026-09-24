import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from wiz_interfaces.msg import Detection, DetectionArray
from wiz_vision.processing import preprocess_frame, detect_features, load_detector, detect_objects
from wiz_vision.tracking import CentroidTracker

CONFIDENCE_THRESHOLD = 0.4


class VisionNode(Node):

    def __init__(self):
        super().__init__('vision_node')

        self.declare_parameter('model_dir', '')
        model_dir_param = self.get_parameter('model_dir').value or None

        self._bridge = CvBridge()
        self._tracker = CentroidTracker(max_disappeared=10)

        try:
            self._net = load_detector(model_dir_param)
            self._detector_ready = True
        except FileNotFoundError as exc:
            self.get_logger().error(str(exc))
            self._net = None
            self._detector_ready = False

        self._sub = self.create_subscription(Image, '/wiz/camera/raw', self._on_frame, 10)
        self._annotated_pub = self.create_publisher(Image, '/wiz/camera/annotated', 10)
        self._detections_pub = self.create_publisher(DetectionArray, '/wiz/vision/detections', 10)

        self.get_logger().info('vision_node started, subscribing to /wiz/camera/raw')

    def _on_frame(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().error(f'failed to convert incoming image: {exc}')
            return

        gray = preprocess_frame(frame)
        keypoints = detect_features(gray)

        detections = []
        if self._detector_ready:
            detections = detect_objects(self._net, frame, CONFIDENCE_THRESHOLD)

        tracked = self._tracker.update(detections)

        annotated = cv2.drawKeypoints(frame, keypoints, None, color=(255, 200, 0))

        detection_array = DetectionArray()
        detection_array.header = msg.header

        for object_id, (cx, cy, bbox, label, confidence) in tracked.items():
            x, y, w, h = bbox
            cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 255, 0), 2)
            label_y = y - 8 if y - 8 > 10 else y + 20
            cv2.putText(annotated, f'#{object_id} {label} {confidence:.2f}',
                        (x, label_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            det_msg = Detection()
            det_msg.track_id = int(object_id)
            det_msg.label = label
            det_msg.confidence = float(confidence)
            det_msg.x, det_msg.y = int(x), int(y)
            det_msg.width, det_msg.height = int(w), int(h)
            detection_array.detections.append(det_msg)

        self._detections_pub.publish(detection_array)

        try:
            annotated_msg = self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            annotated_msg.header = msg.header
            self._annotated_pub.publish(annotated_msg)
        except CvBridgeError as exc:
            self.get_logger().error(f'failed to publish annotated image: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
