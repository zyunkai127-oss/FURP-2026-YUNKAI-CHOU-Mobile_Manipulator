#!/usr/bin/env python3

import argparse
from collections import deque
import json
import math
import os

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


def quaternion_to_matrix(q):
    x, y, z, w = q
    q = np.array([x, y, z, w], dtype=np.float64)
    q /= np.linalg.norm(q)
    x, y, z, w = q

    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def pose_to_matrix(msg):
    transform = np.eye(4)
    transform[:3, :3] = quaternion_to_matrix([
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w,
    ])
    transform[:3, 3] = [
        msg.pose.position.x,
        msg.pose.position.y,
        msg.pose.position.z,
    ]
    return transform


def stamp_seconds(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def rotation_error_deg(r1, r2):
    relative = r1.T @ r2
    value = np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0)
    return math.degrees(math.acos(value))


class ExtrinsicValidator(Node):
    def __init__(self, args):
        super().__init__('dual_camera_extrinsic_validator')

        self.args = args
        self.cam1_buffer = deque(maxlen=40)
        self.cam2_buffer = deque(maxlen=40)
        self.errors_xyz = []
        self.rotation_errors = []
        self.skews_ms = []

        with open(args.extrinsics, 'r') as file:
            config = yaml.safe_load(file)

        translation = config['translation']
        rotation = config['rotation']

        self.transform_cam1_cam2 = np.eye(4)
        self.transform_cam1_cam2[:3, :3] = quaternion_to_matrix([
            rotation['x'],
            rotation['y'],
            rotation['z'],
            rotation['w'],
        ])
        self.transform_cam1_cam2[:3, 3] = [
            translation['x'],
            translation['y'],
            translation['z'],
        ]

        qos = QoSProfile(depth=20)
        qos.reliability = ReliabilityPolicy.RELIABLE

        self.create_subscription(
            PoseStamped,
            '/ihawk1/checkerboard_pose',
            self.cam1_callback,
            qos
        )
        self.create_subscription(
            PoseStamped,
            '/ihawk2/checkerboard_pose',
            self.cam2_callback,
            qos
        )

        print(f'Validation label: {args.label}')
        print(f'Collecting {args.samples} synchronized Pose pairs...')
        print('Keep cameras and calibration board completely still.')

    def cam1_callback(self, msg):
        self.cam1_buffer.append(msg)
        self.match()

    def cam2_callback(self, msg):
        self.cam2_buffer.append(msg)
        self.match()

    def match(self):
        while self.cam1_buffer and self.cam2_buffer:
            best = None

            for index1, pose1 in enumerate(self.cam1_buffer):
                time1 = stamp_seconds(pose1)

                for index2, pose2 in enumerate(self.cam2_buffer):
                    skew = abs(time1 - stamp_seconds(pose2))

                    if best is None or skew < best[0]:
                        best = (skew, index1, index2, pose1, pose2)

            skew, index1, index2, pose1, pose2 = best

            if skew > 0.030:
                if stamp_seconds(self.cam1_buffer[0]) < \
                        stamp_seconds(self.cam2_buffer[0]):
                    self.cam1_buffer.popleft()
                else:
                    self.cam2_buffer.popleft()
                continue

            del self.cam1_buffer[index1]
            del self.cam2_buffer[index2]

            measured_cam1_board = pose_to_matrix(pose1)
            measured_cam2_board = pose_to_matrix(pose2)

            predicted_cam1_board = (
                self.transform_cam1_cam2 @ measured_cam2_board
            )

            error_xyz = (
                predicted_cam1_board[:3, 3]
                - measured_cam1_board[:3, 3]
            )

            rotation_error = rotation_error_deg(
                measured_cam1_board[:3, :3],
                predicted_cam1_board[:3, :3]
            )

            self.errors_xyz.append(error_xyz)
            self.rotation_errors.append(rotation_error)
            self.skews_ms.append(skew * 1000.0)

            count = len(self.errors_xyz)
            if count % 10 == 0:
                print(f'Collected {count}/{self.args.samples}')

            if count >= self.args.samples:
                return

    def complete(self):
        return len(self.errors_xyz) >= self.args.samples

    def save_results(self):
        errors_xyz = np.array(self.errors_xyz)
        errors_3d = np.linalg.norm(errors_xyz, axis=1)
        rotation_errors = np.array(self.rotation_errors)

        result = {
            'label': self.args.label,
            'samples': len(errors_xyz),
            'mean_error_xyz_mm': (
                np.mean(errors_xyz, axis=0) * 1000.0
            ).tolist(),
            'std_error_xyz_mm': (
                np.std(errors_xyz, axis=0) * 1000.0
            ).tolist(),
            'rmse_xyz_mm': (
                np.sqrt(np.mean(errors_xyz ** 2, axis=0)) * 1000.0
            ).tolist(),
            'mean_3d_error_mm': float(
                np.mean(errors_3d) * 1000.0
            ),
            'median_3d_error_mm': float(
                np.median(errors_3d) * 1000.0
            ),
            'rmse_3d_error_mm': float(
                np.sqrt(np.mean(errors_3d ** 2)) * 1000.0
            ),
            'p95_3d_error_mm': float(
                np.percentile(errors_3d, 95) * 1000.0
            ),
            'max_3d_error_mm': float(
                np.max(errors_3d) * 1000.0
            ),
            'mean_rotation_error_deg': float(
                np.mean(rotation_errors)
            ),
            'rmse_rotation_error_deg': float(
                np.sqrt(np.mean(rotation_errors ** 2))
            ),
            'max_rotation_error_deg': float(
                np.max(rotation_errors)
            ),
            'mean_timestamp_skew_ms': float(
                np.mean(self.skews_ms)
            ),
            'max_timestamp_skew_ms': float(
                np.max(self.skews_ms)
            ),
        }

        output = os.path.expanduser(
            f'~/task5_results/{self.args.label}.json'
        )

        with open(output, 'w') as file:
            json.dump(result, file, indent=2)

        print(json.dumps(result, indent=2))
        print(f'Saved: {output}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--label', required=True)
    parser.add_argument('--samples', type=int, default=100)
    parser.add_argument(
        '--extrinsics',
        default=os.path.expanduser(
            '~/berxel_ros2_ws/src/dual_berxel_calibration/'
            'config/dual_camera_extrinsics.yaml'
        )
    )
    args = parser.parse_args()

    rclpy.init()
    node = ExtrinsicValidator(args)

    try:
        while rclpy.ok() and not node.complete():
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass

    if node.complete():
        node.save_results()
    else:
        print('Validation interrupted before completion.')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
