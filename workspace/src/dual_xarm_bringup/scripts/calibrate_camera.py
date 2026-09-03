#!/usr/bin/env python3
import math
import shutil
import sys
import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from easy_handeye2_msgs.srv import ComputeCalibration, SaveCalibration, TakeSample
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from tf_transformations import quaternion_from_matrix, quaternion_matrix, rotation_matrix

from single_arm_motions import SingleArmMotions

CALIBRATIONS_DIR = Path('/colcon_ws/calibrations')


class CameraCalibrator(Node):

    def __init__(self, settings: dict) -> None:
        super().__init__('camera_calibrator')
        self.settings = settings
        self.handeye = settings['handeye']
        self.camera = settings['camera']
        self.sampling = settings['sampling']
        self.board = settings['board']

        self.tf_node = rclpy.create_node('camera_calibrator_tf')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.tf_node, spin_thread=True)
        self.take_sample_client = self.create_client(TakeSample, '/easy_handeye2/calibration/take_sample')
        self.compute_client = self.create_client(
            ComputeCalibration, '/easy_handeye2/calibration/compute_calibration')
        self.save_client = self.create_client(
            SaveCalibration, '/easy_handeye2/calibration/save_calibration')
        self.motions = SingleArmMotions()

    def call_service(self, client, request, timeout_sec: float, description: str):
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f'{description}: service unavailable, is handeye_server running?')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            raise RuntimeError(f'{description}: timed out')
        return future.result()

    def marker_transform(self):
        parent = self.camera['optical_frame']
        child = self.board['marker_frame']
        try:
            stamped = self.tf_buffer.lookup_transform(parent, child, Time())
        except TransformException as error:
            return None, f'marker tf unavailable: {error}'
        age = (self.get_clock().now() - Time.from_msg(stamped.header.stamp)).nanoseconds * 1e-9
        if age > self.sampling['tf_max_age_sec']:
            return None, f'marker tf is stale ({age:.2f} s old)'
        return stamped, None

    def wait_for_marker(self, timeout_sec: float):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            stamped, error = self.marker_transform()
            if stamped is not None:
                return stamped, None
            time.sleep(0.2)
        return None, error

    def generate_targets(self, nominal: Pose) -> list[Pose]:
        sampling = self.sampling
        nominal_matrix = quaternion_matrix([
            nominal.orientation.x, nominal.orientation.y,
            nominal.orientation.z, nominal.orientation.w])
        targets = []
        for tilt_deg in sampling['tilt_steps_deg']:
            for roll_deg in sampling['roll_steps_deg']:
                for azimuth_index in range(sampling['azimuth_count']):
                    azimuth = 2.0 * math.pi * azimuth_index / sampling['azimuth_count']
                    axis = (math.cos(azimuth), math.sin(azimuth), 0.0)
                    offset = rotation_matrix(math.radians(tilt_deg), axis) @ \
                        rotation_matrix(math.radians(roll_deg), (0.0, 0.0, 1.0))
                    quaternion = quaternion_from_matrix(nominal_matrix @ offset)
                    target = Pose()
                    target.position.x = nominal.position.x
                    target.position.y = nominal.position.y
                    target.position.z = nominal.position.z
                    target.orientation.x = quaternion[0]
                    target.orientation.y = quaternion[1]
                    target.orientation.z = quaternion[2]
                    target.orientation.w = quaternion[3]
                    targets.append(target)
        return targets

    def run(self) -> None:
        marker_stamped, error = self.wait_for_marker(10.0)
        if marker_stamped is None:
            raise RuntimeError(
                f'{error}; is the camera running and the board visible to charuco_detector?')
        distance = math.sqrt(
            marker_stamped.transform.translation.x ** 2 +
            marker_stamped.transform.translation.y ** 2 +
            marker_stamped.transform.translation.z ** 2)
        self.get_logger().info(f'marker visible at {distance:.3f} m from camera')

        nominal = self.motions.read_current_eef_pose()
        targets = self.generate_targets(nominal)
        self.get_logger().info(
            f'{len(targets)} orientation candidates, collecting '
            f"{self.sampling['target_samples']} samples")

        taken = 0
        skipped = 0
        for index, target in enumerate(targets):
            if taken >= self.sampling['target_samples']:
                break
            description = f'calibration pose {index + 1}/{len(targets)} (sample {taken + 1})'
            seed = self.motions.read_current_angles()
            try:
                goal_angles = self.motions.solve_ik(target, seed)
            except RuntimeError as error:
                self.get_logger().warn(f'{description}: ik failed, {error}')
                skipped += 1
                continue
            displacement = math.sqrt(sum(
                (b - a) ** 2 for a, b in zip(seed, goal_angles)))
            if displacement > self.sampling['max_joint_displacement_rad']:
                self.get_logger().warn(
                    f'{description}: ik displacement {displacement:.2f} rad too large, skipping')
                skipped += 1
                continue
            try:
                moved = self.motions.move_to_joint_angles(goal_angles, description)
            except RuntimeError as error:
                self.get_logger().warn(f'{description}: motion failed, {error}')
                skipped += 1
                continue
            if not moved:
                skipped += 1
                continue

            time.sleep(self.sampling['settle_time_sec'])
            marker_stamped, error = self.wait_for_marker(
                self.sampling['marker_wait_timeout_sec'])
            if marker_stamped is None:
                self.get_logger().warn(f'{description}: {error}, sample skipped')
                skipped += 1
                continue
            distance = math.sqrt(
                marker_stamped.transform.translation.x ** 2 +
                marker_stamped.transform.translation.y ** 2 +
                marker_stamped.transform.translation.z ** 2)
            response = self.call_service(
                self.take_sample_client, TakeSample.Request(), 5.0, 'take sample')
            taken = len(response.samples.samples)
            self.get_logger().info(
                f'{description}: sample {taken} taken, marker at {distance:.3f} m')

        if taken < self.sampling['target_samples']:
            self.get_logger().warn(
                f'only {taken} samples collected ({skipped} candidates skipped), '
                'calibration quality may be poor')

        compute = self.call_service(
            self.compute_client, ComputeCalibration.Request(), 30.0, 'compute calibration')
        if not compute.valid:
            raise RuntimeError('calibration computation failed, too few or inconsistent samples')
        translation = compute.calibration.transform.translation
        rotation = compute.calibration.transform.rotation
        self.get_logger().info(
            f'calibration: camera in {self.handeye["robot_base_frame"]} frame, '
            f'translation [{translation.x:.4f} {translation.y:.4f} {translation.z:.4f}] m, '
            f'rotation [x {rotation.x:.4f} y {rotation.y:.4f} '
            f'z {rotation.z:.4f} w {rotation.w:.4f}]')

        save = self.call_service(
            self.save_client, SaveCalibration.Request(), 10.0, 'save calibration')
        if not save.success:
            raise RuntimeError('saving the calibration on the server failed')
        source = Path(save.filepath.data)
        CALIBRATIONS_DIR.mkdir(parents=True, exist_ok=True)
        destination = CALIBRATIONS_DIR / f"{self.settings['calibration_name']}.yaml"
        shutil.copyfile(source, destination)
        self.get_logger().info(f'calibration copied to {destination}')
        self.get_logger().info(
            'sanity check against a tape measure: the camera translation above '
            'should roughly match the physical distance from '
            f"{self.handeye['robot_base_frame']} to the camera")


def load_settings() -> dict:
    config_path = Path(get_package_share_directory('dual_xarm_bringup')) / \
        'config' / 'handeye_calibration.yaml'
    return yaml.safe_load(config_path.read_text())


def main() -> None:
    settings = load_settings()
    rclpy.init()
    node = CameraCalibrator(settings)
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('interrupted, keeping any samples already taken')
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
