#!/usr/bin/env python3

import json
import math

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import String


def rotation_matrix_to_quaternion(matrix):
    trace = np.trace(matrix)

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (matrix[2, 1] - matrix[1, 2]) / s
        qy = (matrix[0, 2] - matrix[2, 0]) / s
        qz = (matrix[1, 0] - matrix[0, 1]) / s
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        s = math.sqrt(
            1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]
        ) * 2.0
        qw = (matrix[2, 1] - matrix[1, 2]) / s
        qx = 0.25 * s
        qy = (matrix[0, 1] + matrix[1, 0]) / s
        qz = (matrix[0, 2] + matrix[2, 0]) / s
    elif matrix[1, 1] > matrix[2, 2]:
        s = math.sqrt(
            1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]
        ) * 2.0
        qw = (matrix[0, 2] - matrix[2, 0]) / s
        qx = (matrix[0, 1] + matrix[1, 0]) / s
        qy = 0.25 * s
        qz = (matrix[1, 2] + matrix[2, 1]) / s
    else:
        s = math.sqrt(
            1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]
        ) * 2.0
        qw = (matrix[1, 0] - matrix[0, 1]) / s
        qx = (matrix[0, 2] + matrix[2, 0]) / s
        qy = (matrix[1, 2] + matrix[2, 1]) / s
        qz = 0.25 * s

    quaternion = np.array([qx, qy, qz, qw], dtype=np.float64)
    quaternion /= np.linalg.norm(quaternion)
    return quaternion


class CameraState:
    def __init__(self, name, optical_frame):
        self.name = name
        self.optical_frame = optical_frame
        self.camera_matrix = None
        self.distortion = None
        self.frame_count = 0


class CheckerboardPoseDetector(Node):
    def __init__(self):
        super().__init__('checkerboard_pose_detector')

        self.pattern_size = (8, 6)
        self.square_length = 0.025
        self.reprojection_threshold = 1.0
        self.process_every_n = 3

        self.states = {
            'ihawk1': CameraState(
                'ihawk1',
                'ihawk1/berxel_camera_color_optical_frame'
            ),
            'ihawk2': CameraState(
                'ihawk2',
                'ihawk2/berxel_camera_color_optical_frame'
            ),
        }

        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )
        self.aruco_detector = cv2.aruco.ArucoDetector(self.dictionary)

        self.object_points = np.zeros((8 * 6, 3), dtype=np.float32)
        grid = np.mgrid[0:8, 0:6].T.reshape(-1, 2)
        self.object_points[:, :2] = grid * self.square_length

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE

        self.pose_publishers = {}
        self.debug_publishers = {}
        self.status_publishers = {}

        for camera in ('ihawk1', 'ihawk2'):
            self.pose_publishers[camera] = self.create_publisher(
                PoseStamped,
                f'/{camera}/checkerboard_pose',
                10
            )
            self.debug_publishers[camera] = self.create_publisher(
                CompressedImage,
                f'/{camera}/checkerboard_debug/compressed',
                10
            )
            self.status_publishers[camera] = self.create_publisher(
                String,
                f'/{camera}/checkerboard_status',
                10
            )

            self.create_subscription(
                CameraInfo,
                f'/{camera}/color/camera_info',
                lambda msg, name=camera: self.camera_info_callback(msg, name),
                qos
            )
            self.create_subscription(
                CompressedImage,
                f'/{camera}/color/color_raw/compressed',
                lambda msg, name=camera: self.image_callback(msg, name),
                qos
            )

        self.get_logger().info(
            'Dual checkerboard pose detector started'
        )

    def camera_info_callback(self, msg, camera):
        state = self.states[camera]
        state.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        state.distortion = np.array(msg.d, dtype=np.float64)

    def orient_corners(self, corners, marker_centres):
        grid = corners.reshape(6, 8, 2)

        expected = {
            0: (0, 0),
            1: (0, 7),
            2: (5, 0),
            3: (5, 7),
        }

        candidates = {
            'normal': grid,
            'rotated_180': grid[::-1, ::-1],
            'flip_horizontal': grid[:, ::-1],
            'flip_vertical': grid[::-1, :],
        }

        used_ids = [
            marker_id
            for marker_id in marker_centres
            if marker_id in expected
        ]

        if not used_ids:
            return None, [], None

        scores = {}

        for name, candidate in candidates.items():
            score = 0.0

            for marker_id in used_ids:
                row, column = expected[marker_id]
                score += np.linalg.norm(
                    candidate[row, column]
                    - marker_centres[marker_id]
                )

            scores[name] = score

        best_name = min(scores, key=scores.get)
        best_grid = candidates[best_name]

        return (
            best_grid.reshape(-1, 1, 2),
            used_ids,
            best_name
        )

    def publish_status(self, camera, stamp, data):
        message = String()
        message.data = json.dumps(data)
        self.status_publishers[camera].publish(message)

    def publish_debug(self, camera, stamp, image):
        success, encoded = cv2.imencode(
            '.jpg',
            image,
            [cv2.IMWRITE_JPEG_QUALITY, 90]
        )

        if not success:
            return

        message = CompressedImage()
        message.header.stamp = stamp
        message.header.frame_id = self.states[camera].optical_frame
        message.format = 'jpeg'
        message.data = encoded.tobytes()
        self.debug_publishers[camera].publish(message)

    def image_callback(self, msg, camera):
        state = self.states[camera]
        state.frame_count += 1

        if state.frame_count % self.process_every_n != 0:
            return

        if state.camera_matrix is None:
            return

        image = cv2.imdecode(
            np.frombuffer(msg.data, dtype=np.uint8),
            cv2.IMREAD_COLOR
        )

        if image is None:
            return

        debug = image.copy()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        checker_found, corners = cv2.findChessboardCornersSB(
            gray,
            self.pattern_size,
            flags=(
                cv2.CALIB_CB_NORMALIZE_IMAGE
                | cv2.CALIB_CB_EXHAUSTIVE
            )
        )

        marker_corners, marker_ids, rejected = \
            self.aruco_detector.detectMarkers(gray)

        marker_centres = {}
        detected_ids = []

        if marker_ids is not None:
            cv2.aruco.drawDetectedMarkers(
                debug,
                marker_corners,
                marker_ids
            )

            for marker_corner, marker_id in zip(
                marker_corners,
                marker_ids.flatten()
            ):
                marker_id = int(marker_id)
                marker_centres[marker_id] = (
                    marker_corner.reshape(4, 2).mean(axis=0)
                )
                detected_ids.append(marker_id)

        status = {
            'camera': camera,
            'checkerboard_corners': 48 if checker_found else 0,
            'aruco_ids': sorted(detected_ids),
            'orientation_markers': [],
            'orientation': None,
            'reprojection_error_px': None,
            'accepted': False,
        }

        if not checker_found:
            cv2.putText(
                debug,
                'REJECT: checkerboard not found',
                (15, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2
            )
            self.publish_status(camera, msg.header.stamp, status)
            self.publish_debug(camera, msg.header.stamp, debug)
            return

        oriented_corners, used_ids, orientation = \
            self.orient_corners(corners, marker_centres)

        status['orientation_markers'] = sorted(used_ids)
        status['orientation'] = orientation

        cv2.drawChessboardCorners(
            debug,
            self.pattern_size,
            corners,
            checker_found
        )

        if oriented_corners is None:
            cv2.putText(
                debug,
                'REJECT: no orientation ArUco',
                (15, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2
            )
            self.publish_status(camera, msg.header.stamp, status)
            self.publish_debug(camera, msg.header.stamp, debug)
            return

        solved, rvec, tvec = cv2.solvePnP(
            self.object_points,
            oriented_corners,
            state.camera_matrix,
            state.distortion,
            flags=cv2.SOLVEPNP_ITERATIVE
        )

        if not solved:
            self.publish_status(camera, msg.header.stamp, status)
            self.publish_debug(camera, msg.header.stamp, debug)
            return

        projected, _ = cv2.projectPoints(
            self.object_points,
            rvec,
            tvec,
            state.camera_matrix,
            state.distortion
        )

        errors = projected.reshape(-1, 2) - \
            oriented_corners.reshape(-1, 2)

        reprojection_error = float(
            np.sqrt(np.mean(np.sum(errors * errors, axis=1)))
        )

        status['reprojection_error_px'] = reprojection_error
        status['accepted'] = (
            reprojection_error < self.reprojection_threshold
        )

        colour = (0, 255, 0) if status['accepted'] else (0, 0, 255)

        cv2.drawFrameAxes(
            debug,
            state.camera_matrix,
            state.distortion,
            rvec,
            tvec,
            0.075,
            2
        )

        cv2.rectangle(debug, (5, 5), (635, 72), (0, 0, 0), -1)
        cv2.putText(
            debug,
            (
                f'{camera} corners=48 IDs={sorted(detected_ids)} '
                f'error={reprojection_error:.3f}px'
            ),
            (12, 31),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.57,
            colour,
            2
        )
        cv2.putText(
            debug,
            (
                f'orientation={orientation} '
                f'accepted={status["accepted"]}'
            ),
            (12, 59),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.57,
            colour,
            2
        )

        if status['accepted']:
            rotation_matrix, _ = cv2.Rodrigues(rvec)
            quaternion = rotation_matrix_to_quaternion(rotation_matrix)

            pose = PoseStamped()
            pose.header.stamp = msg.header.stamp
            pose.header.frame_id = state.optical_frame

            pose.pose.position.x = float(tvec[0, 0])
            pose.pose.position.y = float(tvec[1, 0])
            pose.pose.position.z = float(tvec[2, 0])

            pose.pose.orientation.x = float(quaternion[0])
            pose.pose.orientation.y = float(quaternion[1])
            pose.pose.orientation.z = float(quaternion[2])
            pose.pose.orientation.w = float(quaternion[3])

            self.pose_publishers[camera].publish(pose)

        self.publish_status(camera, msg.header.stamp, status)
        self.publish_debug(camera, msg.header.stamp, debug)


def main(args=None):
    rclpy.init(args=args)
    node = CheckerboardPoseDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
