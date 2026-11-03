#!/usr/bin/env python3
"""Count live G1 calibration topics without printing message payloads."""

from __future__ import annotations

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from unitree_hg.msg import IMUState, LowState

from g1_calibration.constants import (
    CLOUD_TOPIC,
    IMU_TOPIC,
    LIVOX_IMU_TOPIC,
    LOWSTATE_TOPIC,
)


class TopicProbe(Node):
    def __init__(self) -> None:
        super().__init__("g1_lidar_imu_capture_probe")
        self.received: dict[str, list[int]] = {
            CLOUD_TOPIC: [],
            IMU_TOPIC: [],
            LOWSTATE_TOPIC: [],
            LIVOX_IMU_TOPIC: [],
        }
        message_types = {
            CLOUD_TOPIC: PointCloud2,
            IMU_TOPIC: IMUState,
            LOWSTATE_TOPIC: LowState,
            LIVOX_IMU_TOPIC: Imu,
        }
        self._probe_subscriptions = [
            self.create_subscription(
                message_type,
                topic,
                lambda _message, name=topic: self._record(name),
                qos_profile_sensor_data,
            )
            for topic, message_type in message_types.items()
        ]

    def _record(self, topic: str) -> None:
        self.received[topic].append(time.monotonic_ns())

    def report(self) -> dict[str, dict[str, float | int]]:
        result: dict[str, dict[str, float | int]] = {}
        for topic, timestamps in self.received.items():
            duration_s = (
                (timestamps[-1] - timestamps[0]) / 1e9 if len(timestamps) >= 2 else 0
            )
            result[topic] = {
                "count": len(timestamps),
                "mean_hz": (
                    (len(timestamps) - 1) / duration_s if duration_s > 0 else 0.0
                ),
            }
        return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=5.0)
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("--seconds must be positive")

    rclpy.init(args=[])
    node = TopicProbe()
    try:
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.1, deadline - time.monotonic()))
        print(json.dumps(node.report(), sort_keys=True))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
