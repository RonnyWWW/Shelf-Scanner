#!/usr/bin/env python3
"""
Enhanced Velocity Simulator with Speed Control Subscription
Publishes fake odometry/velocity and listens to speed multiplier commands.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3, Point, Pose, Quaternion
from std_msgs.msg import Header, Float32
import numpy as np


class VelocitySimulator(Node):
    def __init__(self):
        super().__init__('velocity_simulator')
        
        # Parameters
        self.declare_parameter('linear_velocity', 0.3)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('enable_noise', False)
        self.declare_parameter('noise_stddev', 0.02)
        
        self.base_velocity = self.get_parameter('linear_velocity').get_parameter_value().double_value
        rate_hz = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.enable_noise = self.get_parameter('enable_noise').get_parameter_value().bool_value
        self.noise_stddev = self.get_parameter('noise_stddev').get_parameter_value().double_value
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber for speed control
        self.speed_sub = self.create_subscription(
            Float32,
            '/speed_multiplier',
            self.speed_callback,
            10
        )
        
        # State
        self.speed_multiplier = 1.0
        self.position_x = 0.0
        self.last_time = self.get_clock().now()
        
        # Timer
        self.timer = self.create_timer(1.0 / rate_hz, self.publish_velocity)
        
        self.get_logger().info(
            f"✅ VelocitySimulator (Enhanced) started:\n"
            f"   Base velocity: {self.base_velocity:.2f} m/s\n"
            f"   Publish rate: {rate_hz:.1f} Hz\n"
            f"   Noise: {'enabled' if self.enable_noise else 'disabled'}\n"
            f"   Listening for speed commands on /speed_multiplier"
        )
    
    def speed_callback(self, msg: Float32):
        """Update speed multiplier from external command."""
        self.speed_multiplier = max(0.0, msg.data)
        self.get_logger().info(f"🎚️  Speed multiplier updated: {self.speed_multiplier:.2f}x")
    
    def publish_velocity(self):
        """Publish odometry and velocity messages."""
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        
        # Calculate velocity
        vel = self.base_velocity * self.speed_multiplier
        if self.enable_noise:
            vel += np.random.normal(0, self.noise_stddev)
        
        # Update position
        self.position_x += vel * dt
        
        # Publish Twist
        twist_msg = Twist()
        twist_msg.linear = Vector3(x=vel, y=0.0, z=0.0)
        twist_msg.angular = Vector3(x=0.0, y=0.0, z=0.0)
        self.vel_pub.publish(twist_msg)
        
        # Publish Odometry
        odom_msg = Odometry()
        odom_msg.header = Header()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        odom_msg.pose.pose = Pose()
        odom_msg.pose.pose.position = Point(x=self.position_x, y=0.0, z=0.0)
        odom_msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        odom_msg.twist.twist = twist_msg
        
        odom_msg.pose.covariance[0] = 0.01
        odom_msg.twist.covariance[0] = 0.001
        
        self.odom_pub.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VelocitySimulator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 VelocitySimulator stopped")
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()