from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # LiDAR Driver
        Node(
            package='sick_scan_xd',
            executable='sick_generic_caller',
            name='sick_lidar',
            output='screen',
            arguments=['hostname:=192.168.0.1', 'scanner_type:=sick_tim_5xx']
        ),
        
        # Odometry Node
        Node(
            package='strip_map',
            executable='mcu_odom',
            name='odometry',
            output='screen'
        ),
        
        # Strip Mapper with Velocity Sync
        Node(
            package='strip_map',
            executable='strip_node_patched',
            name='strip_mapper',
            output='screen',
            parameters=[{
                'scan_topic': '/sick_tim_5xx/scan',
                'enable_velocity_sync': True,
                'velocity_topic': '/odom',
                'pixels_per_meter': 200.0,
                'min_velocity_threshold': 0.01
            }]
        ),
        
        # Gap Detector
        Node(
            package='strip_map',
            executable='gap_detector',
            name='gap_detector',
            output='screen'
        ),
    ])
