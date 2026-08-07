#!/usr/bin/env python3

from collections import deque
import math
import os

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger


def quaternion_to_matrix(q):
    x, y, z, w = q
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    x, y, z, w = x/norm, y/norm, z/norm, w/norm

    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ], dtype=np.float64)


def matrix_to_quaternion(r):
    trace = np.trace(r)

    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        q = np.array([
            (r[2, 1] - r[1, 2]) / s,
            (r[0, 2] - r[2, 0]) / s,
            (r[1, 0] - r[0, 1]) / s,
            0.25 * s,
        ])
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1+r[0, 0]-r[1, 1]-r[2, 2]) * 2
        q = np.array([
            0.25*s,
            (r[0, 1]+r[1, 0])/s,
            (r[0, 2]+r[2, 0])/s,
            (r[2, 1]-r[1, 2])/s,
        ])
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1+r[1, 1]-r[0, 0]-r[2, 2]) * 2
        q = np.array([
            (r[0, 1]+r[1, 0])/s,
            0.25*s,
            (r[1, 2]+r[2, 1])/s,
            (r[0, 2]-r[2, 0])/s,
        ])
    else:
        s = math.sqrt(1+r[2, 2]-r[0, 0]-r[1, 1]) * 2
        q = np.array([
            (r[0, 2]+r[2, 0])/s,
            (r[1, 2]+r[2, 1])/s,
            0.25*s,
            (r[1, 0]-r[0, 1])/s,
        ])

    return q / np.linalg.norm(q)


def pose_to_matrix(msg):
    q = [
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w,
    ]

    transform = np.eye(4)
    transform[:3, :3] = quaternion_to_matrix(q)
    transform[:3, 3] = [
        msg.pose.position.x,
        msg.pose.position.y,
        msg.pose.position.z,
    ]
    return transform


def rotation_difference_deg(r1, r2):
    relative = r1.T @ r2
    cosine = np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0)
    return math.degrees(math.acos(cosine))


def average_rotation(rotations):
    quaternions = np.array(
        [matrix_to_quaternion(rotation) for rotation in rotations]
    )

    reference = quaternions[0]
    for index in range(len(quaternions)):
        if np.dot(quaternions[index], reference) < 0:
            quaternions[index] *= -1

    accumulator = np.zeros((4, 4))
    for quaternion in quaternions:
        accumulator += np.outer(quaternion, quaternion)

    eigenvalues, eigenvectors = np.linalg.eigh(accumulator)
    mean_q = eigenvectors[:, np.argmax(eigenvalues)]

    if np.dot(mean_q, reference) < 0:
        mean_q *= -1

    return quaternion_to_matrix(mean_q), mean_q


class ExtrinsicCalibrator(Node):
    def __init__(self):
        super().__init__('dual_camera_extrinsic_calibrator')

        self.max_skew_s = 0.030
        self.target_samples = 30
        self.pose_buffers = {
            'ihawk1': deque(maxlen=60),
            'ihawk2': deque(maxlen=60),
        }
        self.samples = []
        self.board_poses_cam1 = []

        self.output_file = os.path.expanduser(
            '~/berxel_ros2_ws/src/dual_berxel_calibration/'
            'config/dual_camera_extrinsics.yaml'
        )

        qos = QoSProfile(depth=20)
        qos.reliability = ReliabilityPolicy.RELIABLE

        self.create_subscription(
            PoseStamped,
            '/ihawk1/checkerboard_pose',
            lambda msg: self.pose_callback(msg, 'ihawk1'),
            qos
        )
        self.create_subscription(
            PoseStamped,
            '/ihawk2/checkerboard_pose',
            lambda msg: self.pose_callback(msg, 'ihawk2'),
            qos
        )

        self.create_service(
            Trigger,
            '/dual_camera/capture_extrinsic_sample',
            self.capture_sample
        )
        self.create_service(
            Trigger,
            '/dual_camera/save_extrinsics',
            self.save_service
        )
        self.create_service(
            Trigger,
            '/dual_camera/reset_extrinsic_samples',
            self.reset_samples
        )

        self.get_logger().info(
            'Extrinsic calibrator ready. Target: 30 samples.'
        )

    @staticmethod
    def timestamp(msg):
        return (
            msg.header.stamp.sec
            + msg.header.stamp.nanosec * 1e-9
        )

    def pose_callback(self, msg, camera):
        self.pose_buffers[camera].append(msg)

    def closest_pair(self):
        if not self.pose_buffers['ihawk1'] or \
                not self.pose_buffers['ihawk2']:
            return None

        best = None

        for pose1 in self.pose_buffers['ihawk1']:
            time1 = self.timestamp(pose1)

            for pose2 in self.pose_buffers['ihawk2']:
                skew = abs(time1 - self.timestamp(pose2))

                if best is None or skew < best[0]:
                    best = (skew, pose1, pose2)

        return best

    def capture_sample(self, request, response):
        pair = self.closest_pair()

        if pair is None:
            response.success = False
            response.message = 'No paired Pose available.'
            return response

        skew, pose1, pose2 = pair

        if skew > self.max_skew_s:
            response.success = False
            response.message = (
                f'Timestamp skew {skew*1000:.2f} ms exceeds 30 ms.'
            )
            return response

        transform_cam1_board = pose_to_matrix(pose1)
        transform_cam2_board = pose_to_matrix(pose2)

        if self.board_poses_cam1:
            previous = self.board_poses_cam1[-1]
            movement_m = np.linalg.norm(
                transform_cam1_board[:3, 3] - previous[:3, 3]
            )
            rotation_deg = rotation_difference_deg(
                previous[:3, :3],
                transform_cam1_board[:3, :3]
            )

            if movement_m < 0.020 and rotation_deg < 5.0:
                response.success = False
                response.message = (
                    'Board pose too similar. Move at least 20 mm '
                    'or rotate at least 5 degrees.'
                )
                return response

        transform_cam1_cam2 = (
            transform_cam1_board
            @ np.linalg.inv(transform_cam2_board)
        )

        self.samples.append({
            'transform': transform_cam1_cam2,
            'skew_s': skew,
        })
        self.board_poses_cam1.append(transform_cam1_board)

        count = len(self.samples)
        response.success = True
        response.message = (
            f'Captured sample {count}/{self.target_samples}; '
            f'skew={skew*1000:.2f} ms'
        )

        self.get_logger().info(response.message)

        if count == self.target_samples:
            success, message = self.calculate_and_save()
            self.get_logger().info(message)

        return response

    def calculate_and_save(self):
        if len(self.samples) < 10:
            return False, 'Need at least 10 samples before saving.'

        transforms = [sample['transform'] for sample in self.samples]
        translations = np.array([t[:3, 3] for t in transforms])
        rotations = [t[:3, :3] for t in transforms]

        preliminary_translation = np.mean(translations, axis=0)
        preliminary_rotation, _ = average_rotation(rotations)

        translation_residuals = np.linalg.norm(
            translations - preliminary_translation,
            axis=1
        )
        rotation_residuals = np.array([
            rotation_difference_deg(preliminary_rotation, rotation)
            for rotation in rotations
        ])

        translation_median = np.median(translation_residuals)
        translation_mad = np.median(
            np.abs(translation_residuals - translation_median)
        )
        rotation_median = np.median(rotation_residuals)
        rotation_mad = np.median(
            np.abs(rotation_residuals - rotation_median)
        )

        translation_limit = max(
            translation_median + 3.0 * 1.4826 * translation_mad,
            0.003
        )
        rotation_limit = max(
            rotation_median + 3.0 * 1.4826 * rotation_mad,
            0.3
        )

        inliers = (
            (translation_residuals <= translation_limit)
            & (rotation_residuals <= rotation_limit)
        )

        if np.count_nonzero(inliers) < 10:
            return False, 'Too few inliers after rejection.'

        used_translations = translations[inliers]
        used_rotations = [
            rotation for rotation, keep in zip(rotations, inliers)
            if keep
        ]

        final_translation = np.mean(used_translations, axis=0)
        final_rotation, final_quaternion = average_rotation(used_rotations)

        final_translation_residuals = np.linalg.norm(
            used_translations - final_translation,
            axis=1
        )
        final_rotation_residuals = np.array([
            rotation_difference_deg(final_rotation, rotation)
            for rotation in used_rotations
        ])

        result = {
            'parent_frame': 'ihawk1/berxel_camera_color_optical_frame',
            'child_frame': 'ihawk2/berxel_camera_color_optical_frame',
            'translation': {
                'x': float(final_translation[0]),
                'y': float(final_translation[1]),
                'z': float(final_translation[2]),
            },
            'rotation': {
                'x': float(final_quaternion[0]),
                'y': float(final_quaternion[1]),
                'z': float(final_quaternion[2]),
                'w': float(final_quaternion[3]),
            },
            'calibration_samples_total': len(self.samples),
            'calibration_samples_used': int(np.count_nonzero(inliers)),
            'translation_rmse_m': float(np.sqrt(np.mean(
                final_translation_residuals ** 2
            ))),
            'translation_max_residual_m': float(np.max(
                final_translation_residuals
            )),
            'rotation_rmse_deg': float(np.sqrt(np.mean(
                final_rotation_residuals ** 2
            ))),
            'rotation_max_residual_deg': float(np.max(
                final_rotation_residuals
            )),
            'mean_timestamp_skew_ms': float(np.mean([
                sample['skew_s'] for sample in self.samples
            ]) * 1000.0),
            'max_timestamp_skew_ms': float(np.max([
                sample['skew_s'] for sample in self.samples
            ]) * 1000.0),
            'translation_outlier_limit_m': float(translation_limit),
            'rotation_outlier_limit_deg': float(rotation_limit),
        }

        with open(self.output_file, 'w') as file:
            yaml.safe_dump(result, file, sort_keys=False)

        return True, (
            f'Saved {np.count_nonzero(inliers)}/{len(self.samples)} '
            f'inliers to {self.output_file}'
        )

    def save_service(self, request, response):
        response.success, response.message = self.calculate_and_save()
        return response

    def reset_samples(self, request, response):
        self.samples.clear()
        self.board_poses_cam1.clear()
        response.success = True
        response.message = 'All calibration samples cleared.'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ExtrinsicCalibrator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
