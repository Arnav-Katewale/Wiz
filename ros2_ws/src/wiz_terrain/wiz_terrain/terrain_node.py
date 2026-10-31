"""ROS 2 wrapper around the terrain analysis layer.

Subscribes to camera frames and publishes a **single-frame, camera-relative
(local) traversability grid**. Nothing is accumulated between frames and no
pose is consumed, so the published grid describes only what the current frame
shows, in the ``base_link`` frame. It is not a persistent world map.

``nav_msgs/OccupancyGrid`` is reused as the transport because it is a standard
message that rviz2 renders natively, but its value range carries a
*traversability cost*, not an occupancy probability:

* ``0``   -- clear / low-cost terrain
* ``100`` -- impassable / highest-cost terrain
* ``-1``  -- unknown
"""

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import Image

from wiz_terrain.terrain import (
    TerrainConfig,
    build_traversability_grid,
    ground_sample_distance,
    render_overlay,
    to_occupancy_row_major,
)

#: Warn if no camera frame arrives for this long (matches wiz_status's watchdog).
FRAME_TIMEOUT_SEC = 5.0
#: Warn if forward and lateral cell sizes disagree by more than this fraction.
CELL_ASPECT_TOLERANCE = 0.10


class TerrainNode(Node):

    def __init__(self, **kwargs):
        # kwargs are forwarded to rclpy's Node (parameter_overrides in
        # particular), which lets tests drive the node without a launch file.
        super().__init__('terrain_node', **kwargs)

        defaults = TerrainConfig()
        self.declare_parameter('resize_width', defaults.resize_width)
        self.declare_parameter('grid_rows', defaults.grid_rows)
        self.declare_parameter('grid_cols', defaults.grid_cols)
        self.declare_parameter('blur_kernel', defaults.blur_kernel)
        self.declare_parameter('veg_hsv_lower', list(defaults.veg_hsv_lower))
        self.declare_parameter('veg_hsv_upper', list(defaults.veg_hsv_upper))
        self.declare_parameter('veg_weight', defaults.veg_weight)
        self.declare_parameter('roughness_threshold', defaults.roughness_threshold)
        self.declare_parameter('roughness_gain', defaults.roughness_gain)
        self.declare_parameter('canny_low', defaults.canny_low)
        self.declare_parameter('canny_high', defaults.canny_high)
        self.declare_parameter('min_contour_area', defaults.min_contour_area)
        self.declare_parameter(
            'obstacle_fraction_threshold', defaults.obstacle_fraction_threshold
        )
        self.declare_parameter('obstacle_cost', defaults.obstacle_cost)
        self.declare_parameter('morph_kernel', defaults.morph_kernel)
        self.declare_parameter('assumed_altitude_m', defaults.assumed_altitude_m)
        self.declare_parameter('camera_hfov_deg', defaults.camera_hfov_deg)
        self.declare_parameter('unknown_border_cells', defaults.unknown_border_cells)
        self.declare_parameter('publish_overlay', True)

        try:
            self._config = self._config_from_parameters()
            self._config.validate()
        except (ValueError, TypeError) as exc:
            self.get_logger().error(
                f'invalid terrain parameters, refusing to start: {exc}'
            )
            raise

        self._publish_overlay = bool(self.get_parameter('publish_overlay').value)
        self._bridge = CvBridge()
        self._frames_processed = 0
        self._frames_rejected = 0
        self._last_frame_time = None
        self._warned_cell_aspect = False

        self._sub = self.create_subscription(
            Image, '/wiz/camera/raw', self._on_frame, 10
        )
        self._grid_pub = self.create_publisher(OccupancyGrid, '/wiz/terrain/grid', 10)
        self._overlay_pub = self.create_publisher(
            Image, '/wiz/terrain/overlay', 10
        )
        self._watchdog = self.create_timer(1.0, self._check_frame_timeout)

        gsd = ground_sample_distance(self._config, self._config.resize_width)
        self.get_logger().info(
            'terrain_node started, subscribing to /wiz/camera/raw; '
            f'grid {self._config.grid_rows}x{self._config.grid_cols} (image-space)'
        )
        self.get_logger().info(
            'ground scale is ASSUMED, not measured: nadir camera over flat '
            f'ground at {self._config.assumed_altitude_m:.2f} m with '
            f'{self._config.camera_hfov_deg:.1f} deg HFOV '
            f'-> {gsd * 100.0:.2f} cm/pixel'
        )

    def _config_from_parameters(self) -> TerrainConfig:
        get = lambda name: self.get_parameter(name).value  # noqa: E731
        veg_lower = tuple(int(v) for v in get('veg_hsv_lower'))
        veg_upper = tuple(int(v) for v in get('veg_hsv_upper'))
        if len(veg_lower) != 3 or len(veg_upper) != 3:
            raise ValueError('veg_hsv_lower and veg_hsv_upper need exactly 3 values')
        return TerrainConfig(
            resize_width=int(get('resize_width')),
            grid_rows=int(get('grid_rows')),
            grid_cols=int(get('grid_cols')),
            blur_kernel=int(get('blur_kernel')),
            veg_hsv_lower=veg_lower,
            veg_hsv_upper=veg_upper,
            veg_weight=float(get('veg_weight')),
            roughness_threshold=float(get('roughness_threshold')),
            roughness_gain=float(get('roughness_gain')),
            canny_low=int(get('canny_low')),
            canny_high=int(get('canny_high')),
            min_contour_area=int(get('min_contour_area')),
            obstacle_fraction_threshold=float(get('obstacle_fraction_threshold')),
            obstacle_cost=int(get('obstacle_cost')),
            morph_kernel=int(get('morph_kernel')),
            assumed_altitude_m=float(get('assumed_altitude_m')),
            camera_hfov_deg=float(get('camera_hfov_deg')),
            unknown_border_cells=int(get('unknown_border_cells')),
        )

    def _check_frame_timeout(self) -> None:
        if self._last_frame_time is None:
            return
        age = (self.get_clock().now() - self._last_frame_time).nanoseconds / 1e9
        if age > FRAME_TIMEOUT_SEC:
            self.get_logger().error(
                f'/wiz/camera/raw silent for {age:.1f}s (is camera_node running?)'
            )

    def _on_frame(self, msg: Image) -> None:
        self._last_frame_time = self.get_clock().now()

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self._frames_rejected += 1
            self.get_logger().error(f'failed to convert incoming image: {exc}')
            return

        try:
            result = build_traversability_grid(frame, self._config)
        except (ValueError, TypeError) as exc:
            self._frames_rejected += 1
            self.get_logger().warn(f'skipping unusable frame: {exc}')
            return
        except cv2.error as exc:  # pragma: no cover - defensive
            self._frames_rejected += 1
            self.get_logger().error(f'OpenCV failure while analysing frame: {exc}')
            return

        self._frames_processed += 1
        self._publish_grid(result, msg)

        if self._publish_overlay:
            self._publish_overlay_image(result, msg)

    def _publish_grid(self, result, source_msg: Image) -> None:
        data, width, height = to_occupancy_row_major(result.grid)

        resized_h, resized_w = result.resized.shape[:2]
        lateral_cell = result.cell_size_m
        forward_cell = result.ground_sample_distance_m * (
            resized_h / float(self._config.grid_rows)
        )
        if lateral_cell > 0 and not self._warned_cell_aspect:
            skew = abs(forward_cell - lateral_cell) / lateral_cell
            if skew > CELL_ASPECT_TOLERANCE:
                self._warned_cell_aspect = True
                self.get_logger().warn(
                    'grid cells are not square '
                    f'(forward {forward_cell * 100:.1f} cm vs lateral '
                    f'{lateral_cell * 100:.1f} cm); OccupancyGrid carries a single '
                    'resolution, so the lateral size is published. Choose '
                    'grid_rows/grid_cols to match the image aspect ratio '
                    f'({resized_w}x{resized_h}) to remove the skew.'
                )

        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = source_msg.header.stamp
        grid_msg.header.frame_id = source_msg.header.frame_id or 'base_link'
        grid_msg.info.map_load_time = source_msg.header.stamp
        grid_msg.info.resolution = float(lateral_cell)
        grid_msg.info.width = int(width)
        grid_msg.info.height = int(height)

        # The grid sits directly ahead of the body origin and is centred
        # laterally: x in [0, forward_extent], y in [-lateral/2, +lateral/2].
        grid_msg.info.origin.position.x = 0.0
        grid_msg.info.origin.position.y = -0.5 * height * float(lateral_cell)
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0

        grid_msg.data = data
        self._grid_pub.publish(grid_msg)

    def _publish_overlay_image(self, result, source_msg: Image) -> None:
        try:
            overlay = render_overlay(result, self._config)
            overlay_msg = self._bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
            overlay_msg.header = source_msg.header
            self._overlay_pub.publish(overlay_msg)
        except CvBridgeError as exc:
            self.get_logger().error(f'failed to publish overlay image: {exc}')

    def destroy_node(self):
        self.get_logger().info(
            f'terrain_node shutting down after {self._frames_processed} frames '
            f'processed, {self._frames_rejected} rejected'
        )
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TerrainNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
