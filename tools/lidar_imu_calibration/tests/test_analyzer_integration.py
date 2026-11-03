from __future__ import annotations

import math
import struct
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path
from types import SimpleNamespace

TOOL_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL_DIR))


class AnalyzerIntegrationTests(unittest.TestCase):
    def test_native_mcap_analyzer(self):
        try:
            import rosbag2_py
            from builtin_interfaces.msg import Time
            from rclpy.serialization import serialize_message
            from sensor_msgs.msg import Imu, PointCloud2, PointField
            from std_msgs.msg import Header
            from unitree_hg.msg import IMUState, LowState
        except ImportError as exc:
            self.skipTest(f"ROS2 message environment unavailable: {exc}")

        from analyze_g1_lidar_imu import inspect_bag, main as analyzer_main
        from g1_calibration.constants import (
            CLOUD_TOPIC,
            IMU_TOPIC,
            LIVOX_IMU_TOPIC,
            LOWSTATE_TOPIC,
        )
        from g1_calibration.manifest import load_yaml, write_yaml

        def cloud_message(frame: int, timestamp_ns: int) -> PointCloud2:
            fields = [
                PointField(name="x", offset=0, datatype=7, count=1),
                PointField(name="y", offset=4, datatype=7, count=1),
                PointField(name="z", offset=8, datatype=7, count=1),
                PointField(name="intensity", offset=12, datatype=7, count=1),
                PointField(name="tag", offset=16, datatype=2, count=1),
                PointField(name="line", offset=17, datatype=2, count=1),
                PointField(name="timestamp", offset=24, datatype=8, count=1),
            ]
            point_step = 32
            payload = bytearray(4 * point_step)
            for index in range(4):
                offset = index * point_step
                struct.pack_into("<ffff", payload, offset, 2.0 + index, -1.0, 0.5, 10.0)
                struct.pack_into("<BB", payload, offset + 16, 0, index)
                struct.pack_into(
                    "<d", payload, offset + 24, timestamp_ns + index * 10_000_000
                )
            return PointCloud2(
                header=Header(
                    stamp=Time(
                        sec=timestamp_ns // 1_000_000_000,
                        nanosec=timestamp_ns % 1_000_000_000,
                    ),
                    frame_id="mid360",
                ),
                height=1,
                width=4,
                fields=fields,
                is_bigendian=False,
                point_step=point_step,
                row_step=4 * point_step,
                data=payload,
                is_dense=True,
            )

        def imu_message(index: int) -> IMUState:
            message = IMUState()
            message.quaternion = [0.0, 0.0, 0.0, 1.0]
            message.gyroscope = [
                0.01 * math.sin(index * 0.001),
                0.01 * math.cos(index * 0.001),
                0.01 * math.sin(index * 0.0005),
            ]
            message.accelerometer = [0.0, 0.0, 9.81]
            message.rpy = [0.0, 0.0, 0.0]
            return message

        with tempfile.TemporaryDirectory() as temporary:
            bag_dir = Path(temporary) / "smoke"
            writer = rosbag2_py.SequentialWriter()
            writer.open(
                rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
                rosbag2_py.ConverterOptions(
                    input_serialization_format="cdr",
                    output_serialization_format="cdr",
                ),
            )
            topic_types = {
                CLOUD_TOPIC: "sensor_msgs/msg/PointCloud2",
                IMU_TOPIC: "unitree_hg/msg/IMUState",
                LOWSTATE_TOPIC: "unitree_hg/msg/LowState",
                LIVOX_IMU_TOPIC: "sensor_msgs/msg/Imu",
            }
            for topic, message_type in topic_types.items():
                writer.create_topic(
                    rosbag2_py.TopicMetadata(
                        name=topic,
                        type=message_type,
                        serialization_format="cdr",
                    )
                )

            start_ns = 1_700_000_000_000_000_000
            duration_ns = 12_000_000_000
            for timestamp in range(
                start_ns, start_ns + duration_ns + 1, 100_000_000
            ):
                frame = (timestamp - start_ns) // 100_000_000
                writer.write(
                    CLOUD_TOPIC,
                    serialize_message(cloud_message(frame, timestamp)),
                    timestamp,
                )
            for timestamp in range(
                start_ns, start_ns + duration_ns + 1, 1_000_000
            ):
                index = (timestamp - start_ns) // 1_000_000
                writer.write(IMU_TOPIC, serialize_message(imu_message(index)), timestamp)
                livox_imu = Imu()
                livox_imu.header = Header(
                    stamp=Time(
                        sec=timestamp // 1_000_000_000,
                        nanosec=timestamp % 1_000_000_000,
                    ),
                    frame_id="livox_imu",
                )
                writer.write(
                    LIVOX_IMU_TOPIC, serialize_message(livox_imu), timestamp
                )
            for timestamp in range(
                start_ns, start_ns + duration_ns + 1, 10_000_000
            ):
                writer.write(LOWSTATE_TOPIC, serialize_message(LowState()), timestamp)
            del writer

            report = inspect_bag(bag_dir, cloud_time_mode="auto")
            self.assertEqual(
                tuple(report["cloud"]["field_names"]),
                ("x", "y", "z", "intensity", "tag", "line", "timestamp"),
            )
            self.assertFalse(
                [item for item in report["failures"] if "cloud:" in item]
            )
            self.assertGreater(report["rates"]["cloud"]["mean_hz"], 9.0)
            self.assertGreater(report["rates"]["torso_imu"]["mean_hz"], 950.0)

            write_yaml(
                bag_dir / "capture_manifest.yaml",
                {"schema_version": 1, "profile": "smoke", "status": "recorded"},
            )
            previous_argv = sys.argv
            sys.argv = ["analyze_g1_lidar_imu.py", str(bag_dir), "--profile", "smoke"]
            try:
                with redirect_stdout(StringIO()):
                    self.assertEqual(analyzer_main(), 0)
            finally:
                sys.argv = previous_argv
            analysis = load_yaml(bag_dir / "analysis.yaml")
            manifest = load_yaml(bag_dir / "capture_manifest.yaml")
            self.assertEqual(analysis["status"], "passed")
            self.assertEqual(manifest["status"], "validated_passed")
            self.assertIn("cloud", manifest["analysis"]["rates"])


if __name__ == "__main__":
    unittest.main()
