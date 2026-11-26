"""ROS 2 wrapper around the terrain analysis layer.

Subscribes to camera frames and publishes two grids:

* ``/wiz/terrain/grid``: the **single-frame, camera-relative (local)
  traversability grid** in ``base_link``. It describes only what the current
  frame shows.
* ``/wiz/terrain/map``: a **persistent map** in the ``map`` frame. Every frame
  that arrives after a pose is known is merged into an ``AccumulatedMap``
  (``map_grid.py``) at the drone pose *at that frame's timestamp*,
  interpolated from recent ``/wiz/telemetry`` (``pose.PoseBuffer``); each map
  cell holds the average cost of every observation of it. Frames whose time
  has no known pose (before the first telemetry, or telemetry gone quiet)
  are skipped for the map with a warning, because there is nowhere honest to
  put them.
* ``/wiz/terrain/map_confidence``: same layout as the map, but each cell is
  how well observed it is, ``0..100``: ``100 * min(observations /
  map_confidence_full_at, 1)``. Unseen cells are ``0``. An observation is
  one frame cell landing in the map cell, so one frame can add several when
  map cells are larger than frame cells. This is a count of observations,
  not a probability that the cost is correct.

A message with an unset (zero) stamp is timed by when it arrived instead.

``nav_msgs/OccupancyGrid`` is reused as the transport because it is a standard
message that rviz2 renders natively, but its value range carries a
*traversability cost*, not an occupancy probability:

* ``0``   -- clear / low-cost terrain
* ``100`` -- impassable / highest-cost terrain
* ``-1``  -- unknown
"""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import Image
from wiz_interfaces.msg import Telemetry

from wiz_terrain.map_grid import AccumulatedMap
from wiz_terrain.pose import Pose2D, PoseBuffer, pose_from_telemetry
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
        # Persistent map in the ``map`` frame. Defaults: 200 x 200 cells of
        # 0.15 m (a 30 m square centred on the map origin). Keep the
        # resolution at least the frame cell size, or the map gets gaps.
        self.declare_parameter('map_width_cells', 200)
        self.declare_parameter('map_height_cells', 200)
        self.declare_parameter('map_resolution_m', 0.15)
        self.declare_parameter('map_origin_x', -15.0)
        self.declare_parameter('map_origin_y', -15.0)
        # How much telemetry history to keep for matching frame timestamps,
        # and how long past the newest telemetry its pose may still be used.
        self.declare_parameter('pose_history_s', 2.0)
        self.declare_parameter('pose_hold_s', 0.2)
        # Observations of a cell after which its confidence reads 100.
        self.declare_parameter('map_confidence_full_at', 10)

        try:
            self._config = self._config_from_parameters()
            self._config.validate()
            get = lambda name: self.get_parameter(name).value  # noqa: E731
            self._map = AccumulatedMap(
                width=int(get('map_width_cells')),
                height=int(get('map_height_cells')),
                resolution_m=float(get('map_resolution_m')),
                origin_x=float(get('map_origin_x')),
                origin_y=float(get('map_origin_y')),
            )
            self._poses = PoseBuffer(
                max_history_s=float(get('pose_history_s')),
                max_hold_s=float(get('pose_hold_s')),
            )
            self._confidence_full_at = int(get('map_confidence_full_at'))
            if self._confidence_full_at < 1:
                raise ValueError(
                    f'map_confidence_full_at must be >= 1, got {self._confidence_full_at}'
                )
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
        self._pose = None
        self._telemetry_rejected = 0
        self._frames_without_pose = 0

        self._sub = self.create_subscription(
            Image, '/wiz/camera/raw', self._on_frame, 10
        )
        self._telemetry_sub = self.create_subscription(
            Telemetry, '/wiz/telemetry', self._on_telemetry, 10
        )
        self._grid_pub = self.create_publisher(OccupancyGrid, '/wiz/terrain/grid', 10)
        self._map_pub = self.create_publisher(OccupancyGrid, '/wiz/terrain/map', 10)
        self._confidence_pub = self.create_publisher(
            OccupancyGrid, '/wiz/terrain/map_confidence', 10
        )
        self._overlay_pub = self.create_publisher(
            Image, '/wiz/terrain/overlay', 10
        )
        self._watchdog = self.create_timer(1.0, self._check_frame_timeout)

        gsd = ground_sample_distance(self._config, self._config.resize_width)
        frame_cell = gsd * self._config.resize_width / float(self._config.grid_cols)
        if self._map.resolution_m < frame_cell:
            self.get_logger().warn(
                f'map_resolution_m {self._map.resolution_m:.3f} m is finer than the '
                f'frame cell size {frame_cell:.3f} m; the map will have gaps'
            )
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

    @property
    def latest_pose(self) -> 'Pose2D | None':
        """Most recent valid pose from /wiz/telemetry, or None if none yet."""
        return self._pose

    def _on_telemetry(self, msg: Telemetry) -> None:
        p, q = msg.position, msg.orientation
        try:
            pose = pose_from_telemetry(p.x, p.y, q.x, q.y, q.z, q.w)
        except ValueError as exc:
            self._telemetry_rejected += 1
            self.get_logger().warn(f'ignoring unusable telemetry pose: {exc}')
            return
        if self._pose is None:
            self.get_logger().info(
                f'first pose from /wiz/telemetry: x={pose.x:.2f} m, '
                f'y={pose.y:.2f} m, heading={pose.yaw:.2f} rad'
            )
        self._pose = pose
        self._poses.add(self._stamp_seconds(msg.header.stamp), pose)

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
        self._update_map(result, msg)

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

    def _stamp_seconds(self, stamp) -> float:
        """Message stamp in seconds; an unset (zero) stamp means 'now'."""
        if stamp.sec == 0 and stamp.nanosec == 0:
            return self.get_clock().now().nanoseconds / 1e9
        return stamp.sec + stamp.nanosec / 1e9

    def _update_map(self, result, source_msg: Image) -> None:
        """Merge this frame into the persistent map and publish the map."""
        pose = self._poses.pose_at(self._stamp_seconds(source_msg.header.stamp))
        if pose is None:
            self._frames_without_pose += 1
            self.get_logger().warn(
                'no /wiz/telemetry pose for this frame time; frame not added to the map',
                throttle_duration_sec=5.0,
            )
            return
        self._map.integrate(result.grid, pose, result.cell_size_m)

        data, _, _ = self._map.to_occupancy()
        self._map_pub.publish(self._map_message(data, source_msg.header.stamp))
        confidence = np.rint(self._map.confidence(self._confidence_full_at) * 100.0)
        self._confidence_pub.publish(self._map_message(
            confidence.astype(np.int8).flatten().tolist(), source_msg.header.stamp
        ))

    def _map_message(self, data, stamp) -> OccupancyGrid:
        """An OccupancyGrid in the ``map`` frame with the accumulated map's layout."""
        msg = OccupancyGrid()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        msg.info.map_load_time = stamp
        msg.info.resolution = float(self._map.resolution_m)
        msg.info.width = int(self._map.width)
        msg.info.height = int(self._map.height)
        msg.info.origin.position.x = float(self._map.origin_x)
        msg.info.origin.position.y = float(self._map.origin_y)
        msg.info.origin.orientation.w = 1.0
        msg.data = data
        return msg

    @property
    def accumulated_map(self) -> AccumulatedMap:
        return self._map

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
