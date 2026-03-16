from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sick_scan_xd',
            executable='sick_generic_caller',
            name='tim561',
            parameters=[{'hostname': '192.168.0.1'}],
            output='screen'
        ),
        Node(
            package='strip_map',
            executable='strip_node',
            name='strip_mapper',
            parameters=['config/params.yaml'],
            output='screen'
        ),
    ])
