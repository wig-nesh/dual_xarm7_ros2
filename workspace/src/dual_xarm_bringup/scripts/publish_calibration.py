#!/usr/bin/env python3
import sys
import time
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener
from tf_transformations import quaternion_from_matrix, quaternion_matrix

import numpy as np

from geometry_msgs.msg import TransformStamped


class CalibrationPublisher(Node):

    def __init__(self) -> None:
        super().__init__('calibration_publisher')
        self.declare_parameter('calibration_file', '/colcon_ws/calibrations/xarm1_cam1.yaml')
        self.declare_parameter('camera_root_frame', 'camera_link')

        calibration_file = Path(self.get_parameter('calibration_file').value)
        if not calibration_file.exists():
            raise RuntimeError(f'calibration file {calibration_file} does not exist')
        calibration = yaml.safe_load(calibration_file.read_text())
        parameters = calibration['parameters']
        optical_frame = parameters['tracking_base_frame']
        base_frame = parameters['robot_base_frame']

        base_to_optical = quaternion_matrix([
            calibration['transform']['rotation']['x'],
            calibration['transform']['rotation']['y'],
            calibration['transform']['rotation']['z'],
            calibration['transform']['rotation']['w']])
        base_to_optical[:3, 3] = [
            calibration['transform']['translation']['x'],
            calibration['transform']['translation']['y'],
            calibration['transform']['translation']['z']]

        camera_root_frame = self.get_parameter('camera_root_frame').value
        tf_node = rclpy.create_node('calibration_publisher_tf')
        buffer = Buffer()
        TransformListener(buffer, tf_node, spin_thread=True)
        root_to_optical = None
        end = self.get_clock().now() + rclpy.time.Duration(seconds=10.0)
        while self.get_clock().now() < end:
            try:
                root_to_optical = buffer.lookup_transform(
                    camera_root_frame, optical_frame, Time())
            except TransformException:
                root_to_optical = None
            if root_to_optical is not None:
                break
            time.sleep(0.2)
        tf_node.destroy_node()
        if root_to_optical is None:
            raise RuntimeError(
                f'no tf from {camera_root_frame} to {optical_frame}, '
                'is the camera driver running?')

        root_to_optical_matrix = quaternion_matrix([
            root_to_optical.transform.rotation.x,
            root_to_optical.transform.rotation.y,
            root_to_optical.transform.rotation.z,
            root_to_optical.transform.rotation.w])
        root_to_optical_matrix[:3, 3] = [
            root_to_optical.transform.translation.x,
            root_to_optical.transform.translation.y,
            root_to_optical.transform.translation.z]
        base_to_root = base_to_optical @ np.linalg.inv(root_to_optical_matrix)
        quaternion = quaternion_from_matrix(base_to_root)

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = base_frame
        transform.child_frame_id = camera_root_frame
        transform.transform.translation.x = float(base_to_root[0, 3])
        transform.transform.translation.y = float(base_to_root[1, 3])
        transform.transform.translation.z = float(base_to_root[2, 3])
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.static_broadcaster.sendTransform(transform)
        self.get_logger().info(
            f'publishing static tf {base_frame} -> {camera_root_frame} '
            f'[{transform.transform.translation.x:.4f} '
            f'{transform.transform.translation.y:.4f} '
            f'{transform.transform.translation.z:.4f}] m')


def main() -> None:
    rclpy.init()
    node = CalibrationPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
