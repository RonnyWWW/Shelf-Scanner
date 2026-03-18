#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, PointCloud2
from cv_bridge import CvBridge
import numpy as np
import cv2
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import sensor_msgs_py.point_cloud2 as pc2
import os


class DualStripMapper(Node):
    def __init__(self):
        super().__init__('dual_strip_mapper')

        # Declare parameters
        self.declare_parameter('scan_topic', '/sick_tim_5xx/scan')
        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value

        # QoS settings
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers - separate for left and right
        self.image_pub_left = self.create_publisher(Image, '/strip/image/left', 10)
        self.image_pub_right = self.create_publisher(Image, '/strip/image/right', 10)
        self.pointcloud_pub_left = self.create_publisher(PointCloud2, '/strip/pointcloud/left', 10)
        self.pointcloud_pub_right = self.create_publisher(PointCloud2, '/strip/pointcloud/right', 10)

        # LiDAR subscriber
        self.subscriber = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_cb,
            qos_profile
        )

        # Bridge + visualization setup
        self.bridge = CvBridge()
        self.strip_height = 300
        self.strip_width = 600
        
        # Separate strip images for left and right
        self.strip_image_left = np.zeros((self.strip_height, self.strip_width), dtype=np.uint8)
        self.strip_image_right = np.zeros((self.strip_height, self.strip_width), dtype=np.uint8)

        # Track scanning progress
        self.accumulated_points_left = []
        self.accumulated_points_right = []
        
        self.get_logger().info(f"✅ DualStripMapper started — listening on {self.scan_topic}")

    # ================================================================
    # CALLBACK
    # ================================================================
    def scan_cb(self, msg: LaserScan):
        # Get full scan data
        angles_full = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ranges_full = np.array(msg.ranges, dtype=np.float32)
        ranges_full = np.nan_to_num(ranges_full, nan=msg.range_max, posinf=msg.range_max, neginf=msg.range_max)

        # ================================================================
        # SPLIT INTO LEFT AND RIGHT REGIONS
        # ================================================================
        # Assuming blind spot points down (at -90° or 270°)
        # LEFT SIDE: 45° to 135° 
        left_mask = (angles_full >= np.deg2rad(45)) & (angles_full <= np.deg2rad(135))
        
        # RIGHT SIDE: -135° to -45° 
        right_mask = (angles_full >= np.deg2rad(-135)) & (angles_full <= np.deg2rad(-45))

        # Extract left side data
        angles_left = angles_full[left_mask]
        ranges_left = ranges_full[left_mask]
        
        # Extract right side data  
        angles_right = angles_full[right_mask]
        ranges_right = ranges_full[right_mask]

        # ================================================================
        # PROCESS LEFT STRIP
        # ================================================================
        if len(ranges_left) > 0:
            self.process_strip(
                ranges_left,
                angles_left,
                self.strip_image_left,
                self.accumulated_points_left,
                msg.header,
                self.image_pub_left,
                self.pointcloud_pub_left
            )

        # ================================================================
        # PROCESS RIGHT STRIP
        # ================================================================
        if len(ranges_right) > 0:
            self.process_strip(
                ranges_right,
                angles_right,
                self.strip_image_right,
                self.accumulated_points_right,
                msg.header,
                self.image_pub_right,
                self.pointcloud_pub_right
            )

        self.get_logger().info(
            f"Processed scan: LEFT={len(ranges_left)} points, RIGHT={len(ranges_right)} points"
        )

    # ================================================================
    # PROCESS INDIVIDUAL STRIP (LEFT OR RIGHT)
    # ================================================================
    def process_strip(self, ranges, angles, strip_image, accumulated_points, 
                     header, image_pub, cloud_pub):
        """Process one side (left or right) of the scan."""
        
        # --- Build Vertical Column for Strip Map ---
        min_r, max_r = np.nanmin(ranges), np.nanmax(ranges)
        if max_r - min_r < 1e-6:
            return  # Skip if no depth variation
            
        scaled = (ranges - min_r) / (max_r - min_r + 1e-6)
        normalized = 255 - np.clip(scaled * 255, 0, 255).astype(np.uint8)

        # Flip vertically so top = top shelf
        column = cv2.resize(normalized.reshape(-1, 1), (1, self.strip_height))
        column = cv2.flip(column, 0)

        # Add column to strip map (scrolling left)
        strip_image[:] = np.roll(strip_image, -1, axis=1)
        strip_image[:, -1] = column[:, 0]

        # Publish strip image
        image_msg = self.bridge.cv2_to_imgmsg(strip_image, encoding='mono8')
        image_msg.header = header
        image_pub.publish(image_msg)

        # --- Build Point Cloud ---
        xs = ranges * np.cos(angles)  # horizontal distance
        ys = np.zeros_like(ranges)     # forward axis (into the aisle)
        zs = ranges * np.sin(angles)   # vertical component
        points = np.vstack((xs, ys, zs)).T.astype(np.float32)

        # Accumulate for visualization
        accumulated_points.append(points)
        if len(accumulated_points) > 1000:
            accumulated_points[:] = accumulated_points[-1000:]

        merged = np.concatenate(accumulated_points, axis=0)
        cloud_msg = pc2.create_cloud_xyz32(header, merged)
        cloud_pub.publish(cloud_msg)

    # ================================================================
    # SAVE STRIP MAPS
    # ================================================================
    def destroy_node(self):
        super().destroy_node()
        
        # Save left side
        if len(self.accumulated_points_left) > 0:
            merged = np.concatenate(self.accumulated_points_left, axis=0)
            save_path = os.path.expanduser("~/strip_map_left.xyz")
            np.savetxt(save_path, merged[:, :3], fmt="%.4f")
            self.get_logger().info(f"💾 Saved LEFT strip map to {save_path}")
        
        # Save right side
        if len(self.accumulated_points_right) > 0:
            merged = np.concatenate(self.accumulated_points_right, axis=0)
            save_path = os.path.expanduser("~/strip_map_right.xyz")
            np.savetxt(save_path, merged[:, :3], fmt="%.4f")
            self.get_logger().info(f"💾 Saved RIGHT strip map to {save_path}")


def main(args=None):
    rclpy.init(args=args)
    node = DualStripMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Stopping and saving maps...")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
