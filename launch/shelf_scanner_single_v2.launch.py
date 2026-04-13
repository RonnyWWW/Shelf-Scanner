from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sick_scan_xd',
            executable='sick_generic_caller',
            name='sick_lidar',
            output='screen',
            arguments=[
                'hostname:=192.168.0.1',
                'scanner_type:=sick_tim_5xx'
            ]
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
                'pixels_per_meter': 180.0,
                'min_velocity_threshold': 0.01,
                'max_columns_per_update': 10,
                'angle_start_deg': 42.0,
                'angle_end_deg': 100.0,
                'depth_clip_min_m': 0.45,
                'depth_clip_max_m': 1.00,
                'invalid_fill_m': 1.00,
                'depth_smoothing_alpha': 0.30,
            }]
        ),

        Node(
            package='strip_map',
            executable='gap_detector_v3',
            name='gap_detector',
            output='screen',
            parameters=[{
                'process_every_n_frames': 2,
                'min_gap_width_m': 0.10,
                'pixels_per_meter': 180.0,
                'min_gap_height_px': 70,
                'min_gap_area_px': 300,
                'base_depth_delta': 45.0,
                'height_split_ratio': 0.55,
                'min_shelf_strength': 0.45,
                'shelf_window': 12,
                'active_width': 220,
                'new_track_min_x': 470.0,
                'confirm_frames': 3,
                'max_missed_frames_candidate': 3,
                'max_missed_frames_confirmed': 8,
                'match_x_tol_px': 45,
                'match_y_tol_px': 30,
                'match_w_tol_px': 40,
                'match_h_tol_px': 50,
                'angle_start_deg': 42.0,
                'angle_end_deg': 100.0,
            }]
        ),

        Node(
            package='strip_map',
            executable='firebase_uploader_v2',
            name='firebase_uploader',
            output='screen',
        ),
    ])
