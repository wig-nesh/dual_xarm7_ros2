#!/usr/bin/env python3
import math

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

import cv2
import numpy as np


def quaternion_from_rotation(m: np.ndarray) -> tuple[float, float, float, float]:
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


class CharucoDetector(Node):

    def __init__(self) -> None:
        super().__init__('charuco_detector')
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('optical_frame', 'camera/camera/color/optical_frame')
        self.declare_parameter('marker_type', 'charuco')
        self.declare_parameter('squares_x', 5)
        self.declare_parameter('squares_y', 5)
        self.declare_parameter('square_length_m', 0.025)
        self.declare_parameter('marker_length_m', 0.025)
        self.declare_parameter('marker_separation_m', 0.01)
        self.declare_parameter('markers_x', 4)
        self.declare_parameter('markers_y', 3)
        self.declare_parameter('grid_id_order', 'row_major')
        self.declare_parameter('dictionary', 'DICT_4X4_50')
        self.declare_parameter('marker_frame', 'charuco_board')
        self.declare_parameter('aruco_id', -1)

        self.optical_frame = self.get_parameter('optical_frame').value
        self.marker_frame = self.get_parameter('marker_frame').value
        self.marker_type = self.get_parameter('marker_type').value
        self.aruco_id = self.get_parameter('aruco_id').value
        self.aruco_length = self.get_parameter('marker_length_m').value
        self.square_length_m = self.get_parameter('square_length_m').value
        self.dictionary = cv2.aruco.Dictionary_get(
            getattr(cv2.aruco, self.get_parameter('dictionary').value))
        self.board = None
        self.grid_points = {}
        if self.marker_type == 'charuco':
            self.board = cv2.aruco.CharucoBoard_create(
                self.get_parameter('squares_x').value,
                self.get_parameter('squares_y').value,
                self.square_length_m,
                self.get_parameter('marker_length_m').value,
                self.dictionary,
            )
        elif self.marker_type == 'grid':
            markers_x = self.get_parameter('markers_x').value
            markers_y = self.get_parameter('markers_y').value
            separation = self.get_parameter('marker_separation_m').value
            pitch = self.aruco_length + separation
            id_order = self.get_parameter('grid_id_order').value
            half = self.aruco_length / 2.0
            id0_column, id0_row = (0, 0) if id_order == 'row_major' else (0, markers_y - 1)
            origin_x = id0_column * pitch
            origin_y = (markers_y - 1 - id0_row) * pitch
            for column in range(markers_x):
                for row in range(markers_y):
                    if id_order == 'row_major':
                        marker_id = row * markers_x + column
                    elif id_order == 'column_bottom_up':
                        marker_id = column * markers_y + (markers_y - 1 - row)
                    else:
                        raise ValueError(f"unknown grid_id_order '{id_order}'")
                    center_x = column * pitch + half - origin_x
                    center_y = (markers_y - 1 - row) * pitch + half - origin_y
                    self.grid_points[marker_id] = np.array([
                        [center_x - half, center_y + half, 0.0],
                        [center_x + half, center_y + half, 0.0],
                        [center_x + half, center_y - half, 0.0],
                        [center_x - half, center_y - half, 0.0]], dtype=np.float64)
                    self.grid_points[marker_id] = np.array([
                        [center_x - half, center_y - half, 0.0],
                        [center_x + half, center_y - half, 0.0],
                        [center_x + half, center_y + half, 0.0],
                        [center_x - half, center_y + half, 0.0]], dtype=np.float64)
        self.detector_params = cv2.aruco.DetectorParameters_create()

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.distortion = None
        self.tf_broadcaster = TransformBroadcaster(self)

        sensor_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        image_topic = self.get_parameter('image_topic').value
        info_topic = self.get_parameter('camera_info_topic').value
        self.create_subscription(Image, image_topic, self.on_image, sensor_qos)
        self.create_subscription(CameraInfo, info_topic, self.on_camera_info, sensor_qos)
        self.detection_pub = self.create_publisher(
            Image, f'{image_topic}_{self.marker_frame}_detection', 10)
        self.get_logger().info(
            f"waiting for images on {image_topic}, marker type '{self.marker_type}'")

    def on_camera_info(self, msg: CameraInfo) -> None:
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.distortion = np.array(msg.d, dtype=np.float64)
            self.get_logger().info(f'camera info received, frame {msg.header.frame_id}')

    def on_image(self, msg: Image) -> None:
        try:
            self.process_image(msg)
        except Exception as error:
            self.get_logger().error(
                f'detection frame failed: {error}', throttle_duration_sec=5.0)

    def process_image(self, msg: Image) -> None:
        if self.camera_matrix is None:
            return
        color = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
        if self.marker_type == 'aruco':
            self.process_single_aruco(msg, color, gray)
        elif self.marker_type == 'grid':
            self.process_grid(msg, color, gray)
        else:
            self.process_charuco(msg, color, gray)

    def process_charuco(self, msg: Image, color: np.ndarray, gray: np.ndarray) -> None:
        marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.detector_params)
        if marker_ids is None or len(marker_ids) < 2:
            self.publish_no_detection(color)
            return
        interpolated, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
            marker_corners, marker_ids, gray, self.board)
        if interpolated is None or interpolated < 4 or charuco_corners is None:
            self.publish_no_detection(color)
            return
        object_points = self.board.chessboardCorners[np.asarray(charuco_ids).flatten()]
        found, rvec, tvec = cv2.solvePnP(
            np.asarray(object_points, dtype=np.float64),
            np.asarray(charuco_corners, dtype=np.float64),
            self.camera_matrix, self.distortion, flags=cv2.SOLVEPNP_IPPE)
        annotated = color.copy()
        cv2.aruco.drawDetectedCornersCharuco(
            annotated, charuco_corners, charuco_ids, (0, 255, 0))
        if not found:
            self.publish_no_detection(annotated)
            return
        self.emit_pose(msg, annotated, rvec, tvec)

    def process_grid(self, msg: Image, color: np.ndarray, gray: np.ndarray) -> None:
        marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.detector_params)
        if marker_ids is None or len(marker_ids) < 2:
            self.publish_no_detection(color)
            return
        flat_ids = np.asarray(marker_ids).flatten()
        object_points = []
        image_points = []
        used_corners = []
        used_ids = []
        for index, marker_id in enumerate(flat_ids):
            if int(marker_id) not in self.grid_points:
                continue
            object_points.append(self.grid_points[int(marker_id)].mean(axis=0))
            image_points.append(
                np.asarray(marker_corners[index], dtype=np.float64).reshape(4, 2).mean(axis=0))
            used_corners.append(marker_corners[index])
            used_ids.append(marker_ids[index])
        if len(object_points) < 2:
            self.publish_no_detection(color)
            return
        found, rvec, tvec = cv2.solvePnP(
            np.array(object_points, dtype=np.float64), np.array(image_points, dtype=np.float64),
            self.camera_matrix, self.distortion, flags=cv2.SOLVEPNP_IPPE)
        annotated = color.copy()
        cv2.aruco.drawDetectedMarkers(
            annotated, used_corners, np.asarray(used_ids), (0, 255, 0))
        if not found:
            self.publish_no_detection(annotated)
            return
        self.emit_pose(msg, annotated, rvec, tvec)

    def process_single_aruco(self, msg: Image, color: np.ndarray, gray: np.ndarray) -> None:
        marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.detector_params)
        if marker_ids is None or len(marker_ids) == 0:
            self.publish_no_detection(color)
            return
        if self.aruco_id >= 0:
            matches = [index for index, ids in enumerate(marker_ids)
                       if ids[0] == self.aruco_id]
            if not matches:
                self.publish_no_detection(color)
                return
            index = matches[0]
        else:
            index = 0
        half = self.aruco_length / 2.0
        object_points = np.array([
            [-half, half, 0.0], [half, half, 0.0],
            [half, -half, 0.0], [-half, -half, 0.0]], dtype=np.float64)
        found, rvec, tvec = cv2.solvePnP(
            object_points, np.asarray(marker_corners[index], dtype=np.float64),
            self.camera_matrix, self.distortion, flags=cv2.SOLVEPNP_IPPE)
        annotated = color.copy()
        cv2.aruco.drawDetectedMarkers(
            annotated, marker_corners, np.asarray(marker_ids), (0, 255, 0))
        if not found:
            self.publish_no_detection(annotated)
            return
        self.emit_pose(msg, annotated, rvec, tvec)

    def emit_pose(self, msg: Image, annotated: np.ndarray, rvec, tvec) -> None:
        rotation, _ = cv2.Rodrigues(rvec)
        if not np.isfinite(rotation).all() or not np.isfinite(tvec).all():
            self.publish_no_detection(annotated)
            return
        cv2.drawFrameAxes(
            annotated, self.camera_matrix, self.distortion, rvec, tvec, self.aruco_length)
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = msg.header.frame_id or self.optical_frame
        transform.child_frame_id = self.marker_frame
        transform.transform.translation.x = float(tvec[0])
        transform.transform.translation.y = float(tvec[1])
        transform.transform.translation.z = float(tvec[2])
        qx, qy, qz, qw = quaternion_from_rotation(rotation)
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)
        self.detection_pub.publish(self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8'))

    def publish_no_detection(self, annotated: np.ndarray) -> None:
        self.detection_pub.publish(self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8'))
        self.get_logger().warn('marker not detected', throttle_duration_sec=5.0)


def main() -> None:
    rclpy.init()
    node = CharucoDetector()
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
