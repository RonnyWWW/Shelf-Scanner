from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sick_scan_xd',
            executable='sick_generic_caller',
            name='sick_lidar',
            output='screen',
            arguments=['hostname:=192.168.0.1', 'scanner_type:=sick_tim_5xx']
        ),
        Node(
            package='strip_map',
            executable='strip_node_patched_v3',
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
        Node(
            package='strip_map',
            executable='gap_detector_v3',
            name='gap_detector',
            output='screen',
            parameters=[{
                'process_every_n_frames': 2
            }]
        ),
        Node(
            package='strip_map',
            executable='firebase_uploader_v2',
            name='firebase_uploader',
            output='screen',
        ),
    ])
