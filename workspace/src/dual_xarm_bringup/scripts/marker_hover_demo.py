#!/usr/bin/env python3
import sys
import time
from pathlib import Path

import rclpy
import yaml
import numpy as np
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from tf_transformations import quaternion_matrix

from single_arm_motions import SingleArmMotions

CORNER_OFFSETS = {
    'center': (0.0, 0.0),
    'top_left': (-0.5, 0.5),
    'top_right': (0.5, 0.5),
    'bottom_left': (-0.5, -0.5),
    'bottom_right': (0.5, -0.5),
}


class MarkerHoverDemo(Node):

    def __init__(self, settings: dict) -> None:
        super().__init__('marker_hover_demo')
        self.base_frame = settings['handeye']['robot_base_frame']
        self.marker_frame = settings['table_marker']['marker_frame']
        self.hover_offset = settings['table_marker']['hover_offset_m']
        self.hover_orientation = settings['table_marker'].get('hover_orientation', 'down')
        corner_name = settings['table_marker'].get('hover_corner', 'center')
        if corner_name not in CORNER_OFFSETS:
            raise ValueError(f'hover_corner must be one of {sorted(CORNER_OFFSETS)}')
        self.corner_name = corner_name
        self.corner_scale = settings['table_marker']['marker_length_m']
        self.tf_node = rclpy.create_node('marker_hover_demo_tf')
        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self.tf_node, spin_thread=True)
        self.motions = SingleArmMotions()

    def marker_transform(self):
        try:
            stamped = self.tf_buffer.lookup_transform(
                self.base_frame, self.marker_frame, Time())
        except TransformException as error:
            return None, f'{self.marker_frame} tf unavailable: {error}'
        age = (self.get_clock().now() - Time.from_msg(stamped.header.stamp)).nanoseconds * 1e-9
        if age > 2.0:
            return None, f'{self.marker_frame} tf is stale ({age:.2f} s old)'
        return stamped, None

    def wait_for_marker(self, timeout_sec: float):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            stamped, error = self.marker_transform()
            if stamped is not None:
                return stamped, None
            time.sleep(0.2)
        return None, error

    def run(self) -> None:
        marker, error = self.wait_for_marker(10.0)
        if marker is None:
            raise RuntimeError(
                f'{error}; are the camera, the table marker detector and the '
                'calibration publisher running?')
        marker_position = marker.transform.translation
        self.get_logger().info(
            f'{self.marker_frame} in {self.base_frame}: '
            f'x {marker_position.x:.3f} y {marker_position.y:.3f} '
            f'z {marker_position.z:.3f} m')

        corner_x, corner_y = CORNER_OFFSETS[self.corner_name]
        marker_rotation = quaternion_matrix([
            marker.transform.rotation.x, marker.transform.rotation.y,
            marker.transform.rotation.z, marker.transform.rotation.w])
        corner_offset = marker_rotation[:3, :3] @ np.array(
            [corner_x * self.corner_scale, corner_y * self.corner_scale, 0.0])
        target_x = marker_position.x + float(corner_offset[0])
        target_y = marker_position.y + float(corner_offset[1])
        target_z = marker_position.z + float(corner_offset[2])
        self.get_logger().info(
            f"hovering over '{self.corner_name}' corner: "
            f'x {target_x:.3f} y {target_y:.3f} z {target_z:.3f} m')

        seed = self.motions.read_current_angles()
        current = self.motions.read_current_eef_pose()
        target = Pose()
        target.position.x = target_x
        target.position.y = target_y
        target.position.z = target_z + self.hover_offset
        if self.hover_orientation == 'down':
            target.orientation.x = 1.0
            target.orientation.y = 0.0
            target.orientation.z = 0.0
            target.orientation.w = 0.0
        else:
            target.orientation = current.orientation
        angles = self.motions.solve_ik(target, seed)
        self.get_logger().info(
            f'ik displacement '
            f'{[round(b - a, 4) for a, b in zip(seed, angles)]}')
        moved = self.motions.move_to_joint_angles(angles, 'hover above table marker')
        if moved:
            self.get_logger().info(
                'hovering over the marker, the tcp should now be directly above it')
        else:
            self.get_logger().info('hover skipped')


def load_settings() -> dict:
    config_path = Path(get_package_share_directory('dual_xarm_bringup')) / \
        'config' / 'handeye_calibration.yaml'
    return yaml.safe_load(config_path.read_text())


def main() -> None:
    settings = load_settings()
    rclpy.init()
    node = MarkerHoverDemo(settings)
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('interrupted')
    except Exception as error:
        node.get_logger().error(str(error))
        sys.exit(1)
    finally:
        node.destroy_node()
        node.tf_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
