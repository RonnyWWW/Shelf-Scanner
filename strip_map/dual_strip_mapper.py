#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, PointCloud2
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import numpy as np
import cv2
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import sensor_msgs_py.point_cloud2 as pc2
import os


class DualStripMapper(Node):
    def __init__(self):
        super().__init__('dual_strip_mapper')

        # Declare scan topic parameter
        self.declare_parameter('scan_topic', '/scan')
        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value

        # ============ VELOCITY SYNC PARAMETERS ============
        self.declare_parameter('velocity_topic', '/odom')
        self.declare_parameter('enable_velocity_sync', True)
        self.declare_parameter('pixels_per_meter', 200.0)
        self.declare_parameter('min_velocity_threshold', 0.01)
        
        self.velocity_topic = self.get_parameter('velocity_topic').get_parameter_value().string_value
        self.enable_velocity_sync = self.get_parameter('enable_velocity_sync').get_parameter_value().bool_value
        self.pixels_per_meter = self.get_parameter('pixels_per_meter').get_parameter_value().double_value
        self.min_vel_threshold = self.get_parameter('min_velocity_threshold').get_parameter_value().double_value
        # ==================================================

        # QoS settings (match SICK driver)
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

        # ============ VELOCITY SUBSCRIBER ============
        if self.enable_velocity_sync:
            self.vel_sub = self.create_subscription(
                Odometry,
                self.velocity_topic,
                self.odom_cb,
                10
            )
        # =============================================

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
        
        # ============ VELOCITY SYNC STATE ============
        self.current_velocity = 0.0
        self.distance_accumulator = 0.0
        self.column_spacing = 1.0 / self.pixels_per_meter
        self.last_scan_column_left = None  # Store column for replay
        self.last_scan_column_right = None
        self.last_update_time = self.get_clock().now()
        # =============================================
        
        sync_status = "ENABLED" if self.enable_velocity_sync else "DISABLED"
        self.get_logger().info(
            f"✅ DualStripMapper started – waiting for LiDAR scans on {self.scan_topic}...\n"
            f"   Velocity Sync: {sync_status}"
            + (f"\n   Velocity topic: {self.velocity_topic}\n   Resolution: {self.pixels_per_meter:.1f} px/m" 
               if self.enable_velocity_sync else "")
        )

    # ============ VELOCITY CALLBACK ============
    def odom_cb(self, msg: Odometry):
        """Process odometry to update velocity and add columns based on distance."""
        if not self.enable_velocity_sync:
            return
        
        if self.last_scan_column_left is None and self.last_scan_column_right is None:
            return
        
        self.current_velocity = abs(msg.twist.twist.linear.x)
        
        # Calculate time delta
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = now
        
        # Only update if moving fast enough
        if self.current_velocity < self.min_vel_threshold:
            return
        
        # Calculate distance traveled
        distance = self.current_velocity * dt
        self.distance_accumulator += distance
        
        # Determine how many columns to add
        columns_to_add = int(self.distance_accumulator / self.column_spacing)
        
        if columns_to_add > 0:
            # Add columns (up to 10 per update to prevent overload)
            for _ in range(min(columns_to_add, 10)):
                # Update LEFT strip
                if self.last_scan_column_left is not None:
                    self.strip_image_left = np.roll(self.strip_image_left, -1, axis=1)
                    self.strip_image_left[:, -1] = self.last_scan_column_left[:, 0]
                
                # Update RIGHT strip
                if self.last_scan_column_right is not None:
                    self.strip_image_right = np.roll(self.strip_image_right, -1, axis=1)
                    self.strip_image_right[:, -1] = self.last_scan_column_right[:, 0]
            
            # Update accumulator
            self.distance_accumulator -= columns_to_add * self.column_spacing
            
            # Publish updated images
            if self.last_scan_column_left is not None:
                image_msg = self.bridge.cv2_to_imgmsg(self.strip_image_left, encoding='mono8')
                image_msg.header.stamp = now.to_msg()
                image_msg.header.frame_id = 'strip_map_left'
                self.image_pub_left.publish(image_msg)
            
            if self.last_scan_column_right is not None:
                image_msg = self.bridge.cv2_to_imgmsg(self.strip_image_right, encoding='mono8')
                image_msg.header.stamp = now.to_msg()
                image_msg.header.frame_id = 'strip_map_right'
                self.image_pub_right.publish(image_msg)
    # ===========================================

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
                self.pointcloud_pub_left,
                'left'
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
                self.pointcloud_pub_right,
                'right'
            )

        self.get_logger().info(
            f"Processed scan: LEFT={len(ranges_left)} points, RIGHT={len(ranges_right)} points"
        )

    # ================================================================
    # PROCESS INDIVIDUAL STRIP (LEFT OR RIGHT)
    # ================================================================
    def process_strip(self, ranges, angles, strip_image, accumulated_points, 
                     header, image_pub, cloud_pub, side):
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

        # ============ VELOCITY SYNC HANDLING ============
        if self.enable_velocity_sync:
            # Store column for velocity-based replay
            if side == 'left':
                self.last_scan_column_left = column
            else:
                self.last_scan_column_right = column
            # Don't add to strip here - velocity callback will do it
        else:
            # Original behavior: Add column immediately
            strip_image[:] = np.roll(strip_image, -1, axis=1)
            strip_image[:, -1] = column[:, 0]
            
            # Publish strip image
            image_msg = self.bridge.cv2_to_imgmsg(strip_image, encoding='mono8')
            image_msg.header = header
            image_pub.publish(image_msg)
        # ================================================

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
