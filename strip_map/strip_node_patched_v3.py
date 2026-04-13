#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Int64
from cv_bridge import CvBridge
import numpy as np
import cv2


class StripMapper(Node):
    def __init__(self):
        super().__init__('strip_mapper')

        self.declare_parameter('scan_topic', '/sick_tim_5xx/scan')
        self.declare_parameter('velocity_topic', '/odom')
        self.declare_parameter('enable_velocity_sync', True)
        self.declare_parameter('pixels_per_meter', 180.0)
        self.declare_parameter('min_velocity_threshold', 0.01)
        self.declare_parameter('max_columns_per_update', 10)

        self.declare_parameter('angle_start_deg', 42.0)
        self.declare_parameter('angle_end_deg', 100.0)
        self.declare_parameter('depth_clip_min_m', 0.20)
        self.declare_parameter('depth_clip_max_m', 0.80)
        self.declare_parameter('invalid_fill_m', 0.80)
        self.declare_parameter('depth_smoothing_alpha', 0.30)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.velocity_topic = self.get_parameter('velocity_topic').value
        self.enable_velocity_sync = self.get_parameter('enable_velocity_sync').value
        self.pixels_per_meter = float(self.get_parameter('pixels_per_meter').value)
        self.min_vel_threshold = float(self.get_parameter('min_velocity_threshold').value)
        self.max_columns_per_update = int(self.get_parameter('max_columns_per_update').value)

        self.angle_start_deg = float(self.get_parameter('angle_start_deg').value)
        self.angle_end_deg = float(self.get_parameter('angle_end_deg').value)
        self.depth_clip_min_m = float(self.get_parameter('depth_clip_min_m').value)
        self.depth_clip_max_m = float(self.get_parameter('depth_clip_max_m').value)
        self.invalid_fill_m = float(self.get_parameter('invalid_fill_m').value)
        self.depth_smoothing_alpha = float(self.get_parameter('depth_smoothing_alpha').value)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.image_pub = self.create_publisher(Image, '/strip/image', 10)
        self.column_count_pub = self.create_publisher(Int64, '/strip/column_count', 10)

        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, qos_profile)
        if self.enable_velocity_sync:
            self.vel_sub = self.create_subscription(Odometry, self.velocity_topic, self.odom_cb, 10)

        self.bridge = CvBridge()
        self.strip_height = 300
        self.strip_width = 600
        self.strip_image = np.zeros((self.strip_height, self.strip_width), dtype=np.uint8)

        self.current_velocity = 0.0
        self.distance_accumulator = 0.0
        self.column_spacing = 1.0 / max(self.pixels_per_meter, 1e-6)
        self.last_scan_column = None
        self.last_update_time = None
        self.total_columns_inserted = 0

        self._smoothed_ranges = None

        self.get_logger().info(
            f"✅ StripMapper v3 started on {self.scan_topic}\n"
            f"   Velocity sync: {'ENABLED' if self.enable_velocity_sync else 'DISABLED'}\n"
            f"   Angle window: {self.angle_start_deg:.1f}° to {self.angle_end_deg:.1f}°\n"
            f"   Depth clip: {self.depth_clip_min_m:.2f}m to {self.depth_clip_max_m:.2f}m\n"
            f"   Resolution: {self.pixels_per_meter:.1f} px/m"
        )

    def _publish_strip(self, stamp=None):
        image_msg = self.bridge.cv2_to_imgmsg(self.strip_image, encoding='mono8')
        if stamp is not None:
            image_msg.header.stamp = stamp
        image_msg.header.frame_id = 'strip_map'
        self.image_pub.publish(image_msg)
        self.column_count_pub.publish(Int64(data=int(self.total_columns_inserted)))

    def odom_cb(self, msg: Odometry):
        if not self.enable_velocity_sync or self.last_scan_column is None:
            return

        self.current_velocity = abs(msg.twist.twist.linear.x)
        now = self.get_clock().now()

        if self.last_update_time is None:
            self.last_update_time = now
            return

        dt = (now - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = now

        if dt <= 0.0 or self.current_velocity < self.min_vel_threshold:
            return

        self.distance_accumulator += self.current_velocity * dt
        raw_columns_to_add = int(self.distance_accumulator / self.column_spacing)
        columns_to_add = min(raw_columns_to_add, self.max_columns_per_update)

        if columns_to_add <= 0:
            return

        for _ in range(columns_to_add):
            self.strip_image = np.roll(self.strip_image, -1, axis=1)
            self.strip_image[:, -1] = self.last_scan_column[:, 0]
            self.total_columns_inserted += 1

        self.distance_accumulator -= columns_to_add * self.column_spacing
        self._publish_strip(stamp=now.to_msg())

    def scan_cb(self, msg: LaserScan):
        angles_full = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges), dtype=np.float32)
        mask = (
            angles_full > np.deg2rad(self.angle_start_deg)
        ) & (
            angles_full < np.deg2rad(self.angle_end_deg)
        )

        ranges = np.array(msg.ranges, dtype=np.float32)[mask]
        ranges = np.nan_to_num(ranges, nan=np.inf, posinf=np.inf, neginf=np.inf)
        invalid = ~np.isfinite(ranges)
        ranges[invalid] = self.invalid_fill_m
        ranges = np.clip(ranges, self.depth_clip_min_m, self.depth_clip_max_m)

        if self._smoothed_ranges is None or self._smoothed_ranges.shape != ranges.shape:
            self._smoothed_ranges = ranges.copy()
        else:
            a = np.clip(self.depth_smoothing_alpha, 0.0, 1.0)
            self._smoothed_ranges = a * ranges + (1.0 - a) * self._smoothed_ranges

        normalized = 255.0 * (self.depth_clip_max_m - self._smoothed_ranges) / max(self.depth_clip_max_m - self.depth_clip_min_m, 1e-6)
        normalized = np.clip(normalized, 0, 255).astype(np.uint8)
        column = cv2.resize(normalized.reshape(-1, 1), (1, self.strip_height), interpolation=cv2.INTER_LINEAR)
        self.last_scan_column = column

        if not self.enable_velocity_sync:
            self.strip_image = np.roll(self.strip_image, -1, axis=1)
            self.strip_image[:, -1] = column[:, 0]
            self.total_columns_inserted += 1
            self._publish_strip(stamp=msg.header.stamp)


def main(args=None):
    rclpy.init(args=args)
    node = StripMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
