#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, PointCloud2, PointField
from cv_bridge import CvBridge
import numpy as np
import cv2
from rclpy.qos import qos_profile_sensor_data
import sensor_msgs_py.point_cloud2 as pc2
import os


class StripMapper(Node):
    def __init__(self):
        super().__init__('strip_mapper')

        # Declare scan topic parameter
        self.declare_parameter('scan_topic', '/scan')
        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value

        # QoS settings (match SICK driver)
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers
        self.image_pub = self.create_publisher(Image, '/strip/image', 10)
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/strip/pointcloud', 10)

        # LiDAR subscriber
        self.subscriber = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_cb,
            qos_profile
        )

        # Bridge + visualization setup
        self.bridge = CvBridge()
        self.strip_height = 300   # more vertical resolution (shelf levels)
        self.strip_width = 600
        self.strip_image = np.zeros((self.strip_height, self.strip_width), dtype=np.uint8)

        # Track scanning progress
        self.accumulated_points = []
        self.get_logger().info(f"✅ StripMapper (Vertical Mode) started — waiting for LiDAR scans on {self.scan_topic}...")

    # ================================================================
    # CALLBACK
    # ================================================================
    def scan_cb(self, msg: LaserScan):
        self.get_logger().info(f"Received scan with {len(msg.ranges)} points")

        # --- Restrict Field of View (optional) ---
        ANGLE_FOV_DEG = 65  # degrees vertically (top to bottom)
        half_span = np.deg2rad(ANGLE_FOV_DEG / 2)
        angles_full = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        mask = (angles_full > -half_span) & (angles_full < half_span)

        angles = angles_full[mask]
        ranges = np.array(msg.ranges, dtype=np.float32)[mask]
        ranges = np.nan_to_num(ranges, nan=msg.range_max, posinf=msg.range_max, neginf=msg.range_max)

        # --- Build Vertical Column for Strip Map ---
        # Each scan = one vertical slice through shelves
        min_r, max_r = np.nanmin(ranges), np.nanmax(ranges)
        scaled = (ranges - min_r) / (max_r - min_r + 1e-6)
        normalized = 255 - np.clip(scaled * 255, 0, 255).astype(np.uint8)

        # Flip vertically so top = top shelf
        column = cv2.resize(normalized.reshape(-1, 1), (1, self.strip_height))
        column = cv2.flip(column, 0)

        # Add column to strip map (scrolling left)
        self.strip_image = np.roll(self.strip_image, -1, axis=1)
        self.strip_image[:, -1] = column[:, 0]

        # Publish strip image
        image_msg = self.bridge.cv2_to_imgmsg(self.strip_image, encoding='mono8')
        image_msg.header = msg.header
        self.image_pub.publish(image_msg)

        # --- Build Point Cloud ---
        xs = np.zeros_like(ranges)  # depth axis (into the shelf)
        ys = ranges * np.cos(angles)  # vertical axis
        zs = ranges * np.sin(angles)  # horizontal offset
        points = np.vstack((xs, ys, zs)).T.astype(np.float32)

        # Accumulate for visualization
        self.accumulated_points.append(points)
        if len(self.accumulated_points) > 1000:
            self.accumulated_points = self.accumulated_points[-1000:]

        merged = np.concatenate(self.accumulated_points, axis=0)
        cloud_msg = pc2.create_cloud_xyz32(msg.header, merged)
        self.pointcloud_pub.publish(cloud_msg)

        self.get_logger().info(f"Published accumulated vertical strip map ({len(self.accumulated_points)} frames)")

    # ================================================================
    # SAVE STRIP MAP
    # ================================================================
    def destroy_node(self):
        super().destroy_node()
        if len(self.accumulated_points) > 0:
            merged = np.concatenate(self.accumulated_points, axis=0)
            save_path = os.path.expanduser("~/vertical_strip_map.xyz")
            np.savetxt(save_path, merged[:, :3], fmt="%.4f")
            self.get_logger().info(f"💾 Saved vertical strip map to {save_path}")


def main(args=None):
    rclpy.init(args=args)
    node = StripMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Stopping and saving map...")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
