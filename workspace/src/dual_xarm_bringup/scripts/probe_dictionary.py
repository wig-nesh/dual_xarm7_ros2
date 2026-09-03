#!/usr/bin/env python3
import sys
import time

import rclpy
from cv_bridge import CvBridge
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image

import cv2


def main() -> None:
    image_topic = sys.argv[1] if len(sys.argv) > 1 else '/camera/camera/color/image_raw'
    rclpy.init()
    node = rclpy.create_node('probe_dictionary')
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
    matches = []
    for name in sorted(n for n in dir(cv2.aruco) if n.startswith('DICT_')):
        dictionary = cv2.aruco.Dictionary_get(getattr(cv2.aruco, name))
        _, ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
        if ids is not None and len(ids) > 0:
            matches.append((name, [int(i[0]) for i in ids]))
    if not matches:
        print('no dictionary detected any marker, check lighting, angle and focus')
    for name, ids in matches:
        print(f'{name}: marker ids {ids}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
