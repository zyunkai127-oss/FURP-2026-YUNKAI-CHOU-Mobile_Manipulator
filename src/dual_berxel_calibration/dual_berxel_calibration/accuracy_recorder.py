#!/usr/bin/env python3

import argparse
import csv
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class AccuracyRecorder(Node):

    def __init__(self, label, samples, output):
        super().__init__('accuracy_recorder')

        self.label = label
        self.target_samples = samples
        self.output_path = Path(output).expanduser()
        self.output_path.parent.mkdir(
            parents=True,
            exist_ok=True,
        )

        self.fields = [
            'label',
            'receipt_time_sec',
            'point_difference_mm',
            'timestamp_skew_ms',
            'ihawk1_depth_std_mm',
            'ihawk2_depth_std_mm',
            'ihawk1_weight',
            'ihawk2_weight',
            'ihawk1_x_m',
            'ihawk1_y_m',
            'ihawk1_z_m',
            'ihawk2_x_m',
            'ihawk2_y_m',
            'ihawk2_z_m',
            'fused_x_m',
            'fused_y_m',
            'fused_z_m',
            'robot_corrected_x_m',
            'robot_corrected_y_m',
            'robot_corrected_z_m',
        ]

        self.file = self.output_path.open(
            'w',
            newline='',
            encoding='utf-8',
        )

        self.writer = csv.DictWriter(
            self.file,
            fieldnames=self.fields,
        )
        self.writer.writeheader()
        self.file.flush()

        self.count = 0
        self.done = False

        self.subscription = self.create_subscription(
            String,
            '/dual_camera/fusion_status',
            self.status_callback,
            10,
        )

        self.get_logger().info(
            f'Recording {samples} valid samples '
            f'for {label} to {self.output_path}'
        )

    def status_callback(self, message):
        try:
            values = json.loads(message.data)
        except json.JSONDecodeError:
            return

        if not values.get('accepted', False):
            return

        required = [
            'ihawk1_corrected_base_m',
            'ihawk2_base_m',
            'fused_base_m',
            'robot_corrected_base_m',
        ]

        if any(key not in values for key in required):
            return

        point1 = values[
            'ihawk1_corrected_base_m'
        ]
        point2 = values['ihawk2_base_m']
        fused = values['fused_base_m']
        robot_corrected = values[
            'robot_corrected_base_m'
        ]

        if not all(
            len(point) == 3
            for point in (
                point1,
                point2,
                fused,
                robot_corrected,
            )
        ):
            return

        receipt_time = (
            self.get_clock().now().nanoseconds
            / 1.0e9
        )

        row = {
            'label': self.label,
            'receipt_time_sec': receipt_time,
            'point_difference_mm':
                values['point_difference_mm'],
            'timestamp_skew_ms':
                values['timestamp_skew_ms'],
            'ihawk1_depth_std_mm':
                values['ihawk1_depth_std_mm'],
            'ihawk2_depth_std_mm':
                values['ihawk2_depth_std_mm'],
            'ihawk1_weight':
                values['ihawk1_weight'],
            'ihawk2_weight':
                values['ihawk2_weight'],
            'ihawk1_x_m': point1[0],
            'ihawk1_y_m': point1[1],
            'ihawk1_z_m': point1[2],
            'ihawk2_x_m': point2[0],
            'ihawk2_y_m': point2[1],
            'ihawk2_z_m': point2[2],
            'fused_x_m': fused[0],
            'fused_y_m': fused[1],
            'fused_z_m': fused[2],
            'robot_corrected_x_m':
                robot_corrected[0],
            'robot_corrected_y_m':
                robot_corrected[1],
            'robot_corrected_z_m':
                robot_corrected[2],
        }

        self.writer.writerow(row)
        self.file.flush()

        self.count += 1

        self.get_logger().info(
            f'Captured {self.count}/'
            f'{self.target_samples}'
        )

        if self.count >= self.target_samples:
            self.done = True

    def close(self):
        if not self.file.closed:
            self.file.close()


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        '--label',
        required=True,
    )
    parser.add_argument(
        '--samples',
        type=int,
        default=30,
    )
    parser.add_argument(
        '--output',
        required=True,
    )

    args = parser.parse_args()

    rclpy.init()

    node = AccuracyRecorder(
        label=args.label,
        samples=args.samples,
        output=args.output,
    )

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(
                node,
                timeout_sec=0.5,
            )
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    print(
        f'Saved {node.count} samples '
        f'to {node.output_path}'
    )


if __name__ == '__main__':
    main()
