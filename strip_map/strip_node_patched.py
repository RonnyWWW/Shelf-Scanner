#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, PointCloud2, PointField
from nav_msgs.msg import Odometry
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

        # ============ VELOCITY SYNC PARAMETERS (NEW) ============
        self.declare_parameter('velocity_topic', '/odom')
        self.declare_parameter('enable_velocity_sync', True)
        self.declare_parameter('pixels_per_meter', 200.0)
        self.declare_parameter('min_velocity_threshold', 0.01)
        
        self.velocity_topic = self.get_parameter('velocity_topic').get_parameter_value().string_value
        self.enable_velocity_sync = self.get_parameter('enable_velocity_sync').get_parameter_value().bool_value
        self.pixels_per_meter = self.get_parameter('pixels_per_meter').get_parameter_value().double_value
        self.min_vel_threshold = self.get_parameter('min_velocity_threshold').get_parameter_value().double_value
        # ========================================================

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
        #self.pointcloud_pub = self.create_publisher(PointCloud2, '/strip/pointcloud', 10)

        # LiDAR subscriber
        self.subscriber = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_cb,
            qos_profile
        )

        # ============ VELOCITY SUBSCRIBER============
        if self.enable_velocity_sync:
            self.vel_sub = self.create_subscription(
                Odometry,
                self.velocity_topic,
                self.odom_cb,
                10
            )
        # ===================================================

        # Bridge + visualization setup
        self.bridge = CvBridge()
        self.strip_height = 300   # more vertical resolution (shelf levels)
        self.strip_width = 600
        self.strip_image = np.zeros((self.strip_height, self.strip_width), dtype=np.uint8)

        # Track scanning progress
        self.accumulated_points = []
        
        # ============ VELOCITY SYNC STATE (NEW) ============
        self.current_velocity = 0.0
        self.distance_accumulator = 0.0
        self.column_spacing = 1.0 / self.pixels_per_meter
        self.last_scan_column = None  # Store column for replay
        self.last_update_time = self.get_clock().now()
        # ===================================================
        
        sync_status = "ENABLED" if self.enable_velocity_sync else "DISABLED"
        self.get_logger().info(
            f"✅ StripMapper (Vertical Mode) started – waiting for LiDAR scans on {self.scan_topic}...\n"
            f"   Velocity Sync: {sync_status}"
            + (f"\n   Velocity topic: {self.velocity_topic}\n   Resolution: {self.pixels_per_meter:.1f} px/m" 
               if self.enable_velocity_sync else "")
        )

    # ============ VELOCITY CALLBACK (NEW) ============
    def odom_cb(self, msg: Odometry):
        """Process odometry to update velocity and add columns based on distance."""
        if not self.enable_velocity_sync or self.last_scan_column is None:
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
                self.strip_image = np.roll(self.strip_image, -1, axis=1)
                self.strip_image[:, -1] = self.last_scan_column[:, 0]
            
            # Update accumulator
            self.distance_accumulator -= columns_to_add * self.column_spacing
            
            # Publish updated image
            image_msg = self.bridge.cv2_to_imgmsg(self.strip_image, encoding='mono8')
            image_msg.header.stamp = now.to_msg()
            image_msg.header.frame_id = 'strip_map'
            self.image_pub.publish(image_msg)
    # =================================================

    # ================================================================
    # CALLBACK
    # ================================================================
    def scan_cb(self, msg: LaserScan):
        self.get_logger().info(f"Received scan with {len(msg.ranges)} points")

        # --- Restrict Field of View (optional) ---
        ANGLE_START_DEG = 42.0   # 0=middle
        ANGLE_END_DEG   = 80.0   # 135 = edge

        angles_full = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges), dtype=np.float32)
        
        mask = (angles_full > np.deg2rad(ANGLE_START_DEG)) & \
       (angles_full < np.deg2rad(ANGLE_END_DEG))

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
        #column = cv2.flip(column, 0)

        # ============ MODIFIED SECTION ============
        if self.enable_velocity_sync:
            # Store column for velocity-based replay
            self.last_scan_column = column
            # Don't add to strip here - velocity callback will do it
        else:
            # Original behavior: Add column immediately
            self.strip_image = np.roll(self.strip_image, -1, axis=1)
            self.strip_image[:, -1] = column[:, 0]
            
            # Publish strip image
            image_msg = self.bridge.cv2_to_imgmsg(self.strip_image, encoding='mono8')
            image_msg.header = msg.header
            self.image_pub.publish(image_msg)
        # ==========================================

        # --- Build Point Cloud ---
        #xs = np.zeros_like(ranges)  # depth axis (into the shelf)
        #ys = ranges * np.cos(angles)  # vertical axis
        #zs = ranges * np.sin(angles)  # horizontal offset
        #points = np.vstack((xs, ys, zs)).T.astype(np.float32)

        # Accumulate for visualization
        #self.accumulated_points.append(points)
        #if len(self.accumulated_points) > 1000:
            #self.accumulated_points = self.accumulated_points[-1000:]

        #merged = np.concatenate(self.accumulated_points, axis=0)
        #cloud_msg = pc2.create_cloud_xyz32(msg.header, merged)
        #self.pointcloud_pub.publish(cloud_msg)

        self.get_logger().info(f"Published accumulated vertical strip map ({len(self.accumulated_points)} frames)")

    # ================================================================
    # SAVE STRIP MAP
    # ================================================================
    def destroy_node(self):
        super().destroy_node()
        #if len(self.accumulated_points) > 0:
            #merged = np.concatenate(self.accumulated_points, axis=0)
            #save_path = os.path.expanduser("~/vertical_strip_map.xyz")
            #np.savetxt(save_path, merged[:, :3], fmt="%.4f")
            #self.get_logger().info(f"💾 Saved vertical strip map to {save_path}")


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
