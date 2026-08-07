#!/usr/bin/env python3

import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage


class DualCheckerboardChecker(Node):
    def __init__(self):
        super().__init__('dual_checkerboard_checker')

        self.pattern_size = (8, 6)
        self.output_dir = os.path.expanduser('~/task3_results')
        os.makedirs(self.output_dir, exist_ok=True)

        self.found = {'ihawk1': False, 'ihawk2': False}
        self.frames = {'ihawk1': 0, 'ihawk2': 0}
        self.latest = {'ihawk1': None, 'ihawk2': None}

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE

        self.create_subscription(
            CompressedImage,
            '/ihawk1/color/color_raw/compressed',
            lambda msg: self.process(msg, 'ihawk1'),
            qos
        )
        self.create_subscription(
            CompressedImage,
            '/ihawk2/color/color_raw/compressed',
            lambda msg: self.process(msg, 'ihawk2'),
            qos
        )

        print('Looking for an 8x6 inner-corner checkerboard...')
        print('Waiting up to 30 seconds.')

    def process(self, msg, camera):
        if self.found[camera]:
            return

        self.frames[camera] += 1

        # Process every fifth frame to reduce CPU load.
        if self.frames[camera] % 5 != 0:
            return

        array = np.frombuffer(msg.data, dtype=np.uint8)
        image = cv2.imdecode(array, cv2.IMREAD_COLOR)

        if image is None:
            return

        self.latest[camera] = image.copy()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        detected, corners = cv2.findChessboardCornersSB(
            gray,
            self.pattern_size,
            flags=cv2.CALIB_CB_NORMALIZE_IMAGE
        )

        if detected:
            self.found[camera] = True
            cv2.drawChessboardCorners(
                image, self.pattern_size, corners, detected
            )
            path = os.path.join(
                self.output_dir, f'{camera}_checkerboard_detected.png'
            )
            cv2.imwrite(path, image)
            print(f'{camera}: PASS - detected 48 corners')
            print(f'Saved: {path}')


def main():
    rclpy.init()
    node = DualCheckerboardChecker()
    start = time.time()

    while rclpy.ok() and time.time() - start < 30.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if all(node.found.values()):
            break

    for camera in ('ihawk1', 'ihawk2'):
        if not node.found[camera]:
            print(f'{camera}: FAIL - complete 8x6 checkerboard not detected')
            if node.latest[camera] is not None:
                path = os.path.join(
                    node.output_dir, f'{camera}_checkerboard_failed.png'
                )
                cv2.imwrite(path, node.latest[camera])
                print(f'Saved latest image: {path}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
