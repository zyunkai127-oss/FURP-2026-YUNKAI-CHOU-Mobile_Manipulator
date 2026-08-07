#!/usr/bin/env python3

import argparse
import csv
import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class DepthROIStats(Node):
    def __init__(self, args):
        super().__init__('depth_roi_stats')

        self.args = args
        self.records = []
        self.all_valid_depths = []

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE

        self.subscription = self.create_subscription(
            Image, args.topic, self.callback, qos)

        print(f'Topic: {args.topic}')
        print(f'ROI: centre ({args.cx}, {args.cy}), size {args.size}x{args.size}')
        print(f'Waiting for {args.frames} frames...')

    def callback(self, msg):
        if len(self.records) >= self.args.frames:
            return

        dtype = np.dtype('>u2' if msg.is_bigendian else '<u2')
        row_pixels = msg.step // 2
        image = np.frombuffer(msg.data, dtype=dtype)
        image = image.reshape(msg.height, row_pixels)[:, :msg.width]

        half = self.args.size // 2
        x1 = max(0, self.args.cx - half)
        x2 = min(msg.width, self.args.cx + half)
        y1 = max(0, self.args.cy - half)
        y2 = min(msg.height, self.args.cy + half)

        roi = image[y1:y2, x1:x2].astype(np.float64)
        valid = roi[roi > 0]

        total_count = roi.size
        valid_count = valid.size
        invalid_ratio = 1.0 - valid_count / total_count

        if valid_count:
            median = float(np.median(valid))
            mad = float(np.median(np.abs(valid - median)))

            record = {
                'frame': len(self.records) + 1,
                'sec': msg.header.stamp.sec,
                'nanosec': msg.header.stamp.nanosec,
                'valid_pixels': int(valid_count),
                'invalid_ratio': float(invalid_ratio),
                'mean_mm': float(np.mean(valid)),
                'median_mm': median,
                'std_mm': float(np.std(valid)),
                'mad_mm': mad,
                'min_mm': float(np.min(valid)),
                'max_mm': float(np.max(valid)),
            }
            self.all_valid_depths.append(valid)
        else:
            record = {
                'frame': len(self.records) + 1,
                'sec': msg.header.stamp.sec,
                'nanosec': msg.header.stamp.nanosec,
                'valid_pixels': 0,
                'invalid_ratio': 1.0,
                'mean_mm': None,
                'median_mm': None,
                'std_mm': None,
                'mad_mm': None,
                'min_mm': None,
                'max_mm': None,
            }

        self.records.append(record)
        print(f'\rCollected {len(self.records)}/{self.args.frames}', end='', flush=True)

        if len(self.records) == self.args.frames:
            self.save_results()

    def save_results(self):
        csv_path = self.args.output + '.csv'
        json_path = self.args.output + '_summary.json'

        with open(csv_path, 'w', newline='') as file:
            writer = csv.DictWriter(file, fieldnames=self.records[0].keys())
            writer.writeheader()
            writer.writerows(self.records)

        combined = (
            np.concatenate(self.all_valid_depths)
            if self.all_valid_depths else np.array([])
        )

        valid_records = [r for r in self.records if r['median_mm'] is not None]

        summary = {
            'mode': self.args.mode,
            'topic': self.args.topic,
            'frames': len(self.records),
            'roi': {
                'center_x': self.args.cx,
                'center_y': self.args.cy,
                'size': self.args.size,
            },
            'mean_invalid_ratio': float(np.mean(
                [r['invalid_ratio'] for r in self.records])),
            'mean_frame_median_mm': float(np.mean(
                [r['median_mm'] for r in valid_records])),
            'mean_frame_std_mm': float(np.mean(
                [r['std_mm'] for r in valid_records])),
            'mean_frame_mad_mm': float(np.mean(
                [r['mad_mm'] for r in valid_records])),
            'overall_mean_mm': float(np.mean(combined)),
            'overall_median_mm': float(np.median(combined)),
            'overall_std_mm': float(np.std(combined)),
            'overall_min_mm': float(np.min(combined)),
            'overall_max_mm': float(np.max(combined)),
        }

        with open(json_path, 'w') as file:
            json.dump(summary, file, indent=2)

        print(f'\nSaved: {csv_path}')
        print(f'Saved: {json_path}')
        print(json.dumps(summary, indent=2))
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--mode', required=True)
    parser.add_argument('--frames', type=int, default=100)
    parser.add_argument('--cx', type=int, default=320)
    parser.add_argument('--cy', type=int, default=200)
    parser.add_argument('--size', type=int, default=80)
    args = parser.parse_args()

    rclpy.init()
    node = DepthROIStats(args)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
