#!/usr/bin/env python3
import sys
import time

import rclpy
from cv_bridge import CvBridge
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image

import cv2
import numpy as np


def main() -> None:
    image_topic = sys.argv[1] if len(sys.argv) > 1 else '/camera/camera/color/image_raw'
    rclpy.init()
    node = rclpy.create_node('probe_grid_layout')
    bridge = CvBridge()
    frame = {}
    qos = QoSProfile(
        depth=1,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
    )
    node.create_subscription(Image, image_topic, lambda msg: frame.__setitem__('msg', msg), qos)
    deadline = time.monotonic() + 5.0
    while 'msg' not in frame and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)
    if 'msg' not in frame:
        print(f'no image received on {image_topic}, is the camera running?')
        sys.exit(1)
    gray = cv2.cvtColor(bridge.imgmsg_to_cv2(frame['msg'], 'bgr8'), cv2.COLOR_BGR2GRAY)
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
    if ids is None or len(ids) == 0:
        print('no markers detected')
        sys.exit(1)

    centroids = {}
    sizes = []
    for marker_id, corner in zip(np.asarray(ids).flatten(), corners):
        points = np.asarray(corner).reshape(4, 2)
        centroids[int(marker_id)] = points.mean(axis=0)
        sizes.append(np.linalg.norm(points[0] - points[2]))
    marker_span = float(np.median(sizes))

    ordered = sorted(centroids.items(), key=lambda item: item[1][1])
    rows = []
    current = [ordered[0]]
    for marker_id, centroid in ordered[1:]:
        if centroid[1] - current[-1][1][1] > marker_span:
            rows.append(current)
            current = []
        current.append((marker_id, centroid))
    rows.append(current)

    print('physical marker layout as seen by the camera (image y grows downward):')
    for marker_id, centroid in sorted(centroids.items()):
        print(f'id {marker_id:3d}: cx {centroid[0]:7.1f}  cy {centroid[1]:7.1f}')
    for row in rows:
        row_sorted = sorted(row, key=lambda item: item[1][0])
        print('  '.join(f'id {marker_id:3d}' for marker_id, _ in row_sorted))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
