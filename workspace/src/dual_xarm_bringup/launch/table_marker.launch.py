from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_path = Path(get_package_share_directory('dual_xarm_bringup')) / \
        'config' / 'handeye_calibration.yaml'
    settings = yaml.safe_load(config_path.read_text())
    camera = settings['camera']
    marker = settings['table_marker']

    table_marker_detector = Node(
        package='dual_xarm_bringup',
        executable='charuco_detector.py',
        name='table_marker_detector',
        output='screen',
        parameters=[{
            'image_topic': camera['image_topic'],
            'camera_info_topic': camera['camera_info_topic'],
            'optical_frame': camera['optical_frame'],
            'marker_type': marker.get('marker_type', 'aruco'),
            'aruco_id': marker.get('marker_id', -1),
            'marker_length_m': marker['marker_length_m'],
            'marker_separation_m': marker.get('marker_separation_m', 0.01),
            'markers_x': marker.get('markers_x', 4),
            'markers_y': marker.get('markers_y', 3),
            'grid_id_order': marker.get('grid_id_order', 'row_major'),
            'dictionary': marker['dictionary'],
            'marker_frame': marker['marker_frame'],
        }],
    )

    return LaunchDescription([
        table_marker_detector,
    ])
