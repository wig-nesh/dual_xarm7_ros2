from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'calibration_file', default_value='/colcon_ws/calibrations/xarm1_cam1.yaml'),
        Node(
            package='dual_xarm_bringup',
            executable='publish_calibration.py',
            name='calibration_publisher',
            output='screen',
            parameters=[{
                'calibration_file': LaunchConfiguration('calibration_file'),
            }],
        ),
    ])
