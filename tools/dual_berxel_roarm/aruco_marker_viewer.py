#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image


class ArucoMarkerViewer(Node):

    def __init__(self):
        super().__init__("aruco_marker_viewer")

        self.declare_parameter("target_marker_id", 4)
        self.target_id = int(
            self.get_parameter(
                "target_marker_id"
            ).value
        )

        dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )
        parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(
            dictionary,
            parameters,
        )

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.frames = {
            "ihawk1": None,
            "ihawk2": None,
        }

        self.create_subscription(
            Image,
            "/ihawk1/color/color_raw",
            lambda msg: self.image_callback(
                "ihawk1",
                msg,
            ),
            qos,
        )

        self.create_subscription(
            Image,
            "/ihawk2/color/color_raw",
            lambda msg: self.image_callback(
                "ihawk2",
                msg,
            ),
            qos,
        )

        self.combined_publisher = (
            self.create_publisher(
                Image,
                "/dual_camera/aruco_detection_view",
                10,
            )
        )

        self.stop_requested = False
        self.create_timer(0.05, self.display_callback)

        cv2.namedWindow(
            "Dual Camera ArUco Detection",
            cv2.WINDOW_NORMAL,
        )

        self.get_logger().info(
            f"ArUco viewer started. "
            f"Target ID={self.target_id}. "
            "Press q in the image window to quit."
        )

    @staticmethod
    def image_to_bgr(msg):
        rows = np.frombuffer(
            msg.data,
            dtype=np.uint8,
        ).reshape(msg.height, msg.step)

        image = rows[
            :, :msg.width * 3
        ].reshape(
            msg.height,
            msg.width,
            3,
        )

        encoding = msg.encoding.lower()

        if encoding == "rgb8":
            return cv2.cvtColor(
                image,
                cv2.COLOR_RGB2BGR,
            )

        return image.copy()

    def image_callback(self, camera, msg):
        try:
            image = self.image_to_bgr(msg)
        except Exception as error:
            self.get_logger().warning(
                f"{camera}: conversion failed: {error}"
            )
            return

        corners, ids, _ = self.detector.detectMarkers(
            image
        )

        detected_ids = []

        if ids is not None:
            detected_ids = [
                int(value)
                for value in ids.flatten()
            ]

            cv2.aruco.drawDetectedMarkers(
                image,
                corners,
                ids,
            )

            for marker_corners, marker_id in zip(
                corners,
                detected_ids,
            ):
                points = marker_corners.reshape(
                    4,
                    2,
                )
                center = np.mean(
                    points,
                    axis=0,
                ).astype(int)

                color = (
                    (0, 255, 0)
                    if marker_id == self.target_id
                    else (0, 180, 255)
                )

                cv2.circle(
                    image,
                    tuple(center),
                    7,
                    color,
                    -1,
                )

                cv2.drawMarker(
                    image,
                    tuple(center),
                    color,
                    cv2.MARKER_CROSS,
                    25,
                    2,
                )

                cv2.putText(
                    image,
                    f"ID {marker_id}",
                    (
                        int(center[0]) + 10,
                        int(center[1]) - 10,
                    ),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    color,
                    2,
                    cv2.LINE_AA,
                )

        target_detected = (
            self.target_id in detected_ids
        )

        status_color = (
            (0, 255, 0)
            if target_detected
            else (0, 0, 255)
        )

        cv2.rectangle(
            image,
            (0, 0),
            (image.shape[1], 70),
            (0, 0, 0),
            -1,
        )

        cv2.putText(
            image,
            (
                f"{camera} | ID {self.target_id}: "
                + (
                    "DETECTED"
                    if target_detected
                    else "NOT DETECTED"
                )
            ),
            (12, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            status_color,
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            image,
            f"All IDs: {sorted(detected_ids)}",
            (12, 58),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        self.frames[camera] = image

    def display_callback(self):
        if (
            self.frames["ihawk1"] is None
            or self.frames["ihawk2"] is None
        ):
            return

        left = self.frames["ihawk1"]
        right = self.frames["ihawk2"]

        height = min(
            left.shape[0],
            right.shape[0],
        )

        left = cv2.resize(
            left,
            (
                int(
                    left.shape[1]
                    * height
                    / left.shape[0]
                ),
                height,
            ),
        )
        right = cv2.resize(
            right,
            (
                int(
                    right.shape[1]
                    * height
                    / right.shape[0]
                ),
                height,
            ),
        )

        combined = np.hstack([left, right])

        cv2.imshow(
            "Dual Camera ArUco Detection",
            combined,
        )

        output = Image()
        output.header.stamp = (
            self.get_clock().now().to_msg()
        )
        output.header.frame_id = "base_link"
        output.height = combined.shape[0]
        output.width = combined.shape[1]
        output.encoding = "bgr8"
        output.is_bigendian = False
        output.step = combined.shape[1] * 3
        output.data = combined.tobytes()

        self.combined_publisher.publish(output)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            self.stop_requested = True


def main():
    rclpy.init()
    node = ArucoMarkerViewer()

    try:
        while (
            rclpy.ok()
            and not node.stop_requested
        ):
            rclpy.spin_once(
                node,
                timeout_sec=0.05,
            )
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
