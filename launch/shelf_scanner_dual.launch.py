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
        
        # Dual Strip Mapper (replaces strip_node_patched)
        Node(
            package='strip_map',
            executable='dual_strip_mapper',
            name='dual_strip_mapper',
            output='screen',
            parameters=[{
                'scan_topic': '/sick_tim_5xx/scan'
            }]
        ),
        
        # Gap Detector - LEFT SIDE
        Node(
            package='strip_map',
            executable='gap_detector',
            name='gap_detector_left',
            output='screen',
            remappings=[
                ('/strip/image', '/strip/image/left'),
                ('/strip/gaps', '/strip/gaps/left')
            ]
        ),
        
        # Gap Detector - RIGHT SIDE
        Node(
            package='strip_map',
            executable='gap_detector',
            name='gap_detector_right',
            output='screen',
            remappings=[
                ('/strip/image', '/strip/image/right'),
                ('/strip/gaps', '/strip/gaps/right')
            ]
        ),
    ])
