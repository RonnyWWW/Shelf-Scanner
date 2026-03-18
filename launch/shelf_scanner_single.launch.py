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
            executable='mcu_odom',
            name='odometry',
            output='screen'
        ),
        Node(
            package='strip_map',
            executable='strip_node_patched',  # Original single-view mapper
            name='strip_mapper',
            output='screen',
            parameters=[{
                'scan_topic': '/sick_tim_5xx/scan',
            }]
        ),
        Node(
            package='strip_map',
            executable='gap_detector',
            name='gap_detector',
            output='screen'
        ),
    ])
