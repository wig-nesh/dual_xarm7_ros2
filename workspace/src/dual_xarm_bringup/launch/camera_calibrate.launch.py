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
    board = settings['board']
    handeye = settings['handeye']

    charuco_detector = Node(
        package='dual_xarm_bringup',
        executable='charuco_detector.py',
        name='charuco_detector',
        output='screen',
        parameters=[{
            'image_topic': camera['image_topic'],
            'camera_info_topic': camera['camera_info_topic'],
            'optical_frame': camera['optical_frame'],
            'squares_x': board['squares_x'],
            'squares_y': board['squares_y'],
            'square_length_m': board['square_length_m'],
            'marker_length_m': board['marker_length_m'],
            'dictionary': board['dictionary'],
            'marker_frame': board['marker_frame'],
        }],
    )

    handeye_server = Node(
        package='easy_handeye2',
        executable='handeye_server',
        name='handeye_server',
        output='screen',
        parameters=[{
            'name': settings['calibration_name'],
            'calibration_type': handeye['calibration_type'],
            'robot_base_frame': handeye['robot_base_frame'],
            'robot_effector_frame': handeye['robot_effector_frame'],
            'tracking_base_frame': camera['optical_frame'],
            'tracking_marker_frame': board['marker_frame'],
            'freehand_robot_movement': True,
        }],
    )

    return LaunchDescription([
        charuco_detector,
        handeye_server,
    ])
