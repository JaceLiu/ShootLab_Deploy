from __future__ import annotations

import math
import struct
import sys
import tempfile
import unittest
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

TOOL_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL_DIR))

from compare_extrinsics import (  # noqa: E402
    parse_extrinsics,
    relative_rotation_deg,
    translation_delta_m,
)
from generate_ikalibr_config import build_config  # noqa: E402
from g1_calibration.cloud import (  # noqa: E402
    header_stamp_ns,
    inspect_cloud,
    scan_time_offsets_s,
    validate_cloud_inspection,
)
from g1_calibration.manifest import (  # noqa: E402
    dump_yaml,
    load_yaml,
    validate_capture_id,
)
from g1_calibration.metrics import (  # noqa: E402
    coverage_report,
    motion_report,
    rate_failures,
    summarize_timestamps,
)


def native_cloud(point_count: int = 4):
    fields = [
        SimpleNamespace(name="x", offset=0, datatype=7, count=1),
        SimpleNamespace(name="y", offset=4, datatype=7, count=1),
        SimpleNamespace(name="z", offset=8, datatype=7, count=1),
        SimpleNamespace(name="intensity", offset=12, datatype=7, count=1),
        SimpleNamespace(name="tag", offset=16, datatype=2, count=1),
        SimpleNamespace(name="line", offset=17, datatype=2, count=1),
        SimpleNamespace(name="timestamp", offset=24, datatype=8, count=1),
    ]
    point_step = 32
    data = bytearray(point_count * point_step)
    base_ns = 1_700_000_000_000_000_000
    for index in range(point_count):
        offset = index * point_step
        struct.pack_into("<ffff", data, offset, 1.0, 2.0, 3.0, float(index))
        struct.pack_into("<BB", data, offset + 16, 0, index % 4)
        struct.pack_into("<d", data, offset + 24, base_ns + index * 10_000_000)
    header = SimpleNamespace(stamp=SimpleNamespace(sec=1_700_000_000, nanosec=0))
    return SimpleNamespace(
        fields=fields,
        width=point_count,
        height=1,
        point_step=point_step,
        row_step=point_count * point_step,
        is_bigendian=False,
        is_dense=True,
        data=data,
        header=header,
    )


class CloudTests(unittest.TestCase):
    def test_native_scan_times_are_relative_to_header(self):
        message = native_cloud()
        inspection = inspect_cloud(message)
        self.assertEqual(inspection.ring_field, "line")
        self.assertEqual(inspection.time_field, "timestamp")
        self.assertEqual(inspection.field_datatypes["line"], 2)
        self.assertEqual(inspection.field_datatypes["timestamp"], 8)
        self.assertAlmostEqual(inspection.scan_time_min_s, 0.0)
        self.assertAlmostEqual(inspection.scan_time_max_s, 0.03, delta=2e-7)
        self.assertTrue(inspection.monotonic)
        self.assertEqual(validate_cloud_inspection(
            inspection, time_min_s=0.0, time_max_s=0.15
        ), [])

    def test_scan_time_mode_rejects_obvious_absolute_seconds(self):
        offsets = scan_time_offsets_s(
            "time",
            [1_700_000_000.0, 1_700_000_000.01],
            0,
            mode="converted",
        )
        self.assertGreater(offsets[0], 1e8)

    def test_cloud_conversion_contract(self):
        try:
            import numpy as np
        except ImportError:
            self.skipTest("numpy unavailable")
        from convert_g1_lidar_imu import convert_cloud_message

        converted = convert_cloud_message(native_cloud())
        self.assertEqual(
            [field["name"] for field in converted.fields],
            ["x", "y", "z", "intensity", "ring", "time"],
        )
        self.assertEqual(converted.fields[4]["datatype"], 4)  # uint16 ring
        self.assertEqual(converted.fields[5]["datatype"], 7)  # float32 time
        points = np.frombuffer(converted.data.tobytes(), dtype=np.uint8).reshape(-1, 24)
        ring = np.ndarray(
            shape=(4,), dtype="<u2", buffer=points, offset=16, strides=(24,)
        )
        point_time = np.ndarray(
            shape=(4,), dtype="<f4", buffer=points, offset=20, strides=(24,)
        )
        self.assertEqual(ring.tolist(), [0, 1, 2, 3])
        np.testing.assert_allclose(point_time, [0.0, 0.01, 0.02, 0.03], atol=2e-7)

    def test_header_stamp(self):
        self.assertEqual(header_stamp_ns(native_cloud()), 1_700_000_000_000_000_000)

    def test_imu_quaternion_wxyz_to_xyzw(self):
        try:
            import numpy as np
        except ImportError:
            self.skipTest("numpy unavailable")
        from convert_g1_lidar_imu import _make_imu

        @dataclass
        class Time:
            sec: int
            nanosec: int

        @dataclass
        class Header:
            seq: int
            stamp: Time
            frame_id: str

        @dataclass
        class Quaternion:
            x: float
            y: float
            z: float
            w: float

        @dataclass
        class Vector3:
            x: float
            y: float
            z: float

        @dataclass
        class Imu:
            header: Header
            orientation: Quaternion
            orientation_covariance: object
            angular_velocity: Vector3
            angular_velocity_covariance: object
            linear_acceleration: Vector3
            linear_acceleration_covariance: object

        typestore = SimpleNamespace(
            types={
                "builtin_interfaces/msg/Time": Time,
                "std_msgs/msg/Header": Header,
                "geometry_msgs/msg/Quaternion": Quaternion,
                "geometry_msgs/msg/Vector3": Vector3,
                "sensor_msgs/msg/Imu": Imu,
            }
        )
        source = SimpleNamespace(
            quaternion=[1.0, 0.0, 0.0, 0.0],
            gyroscope=[0.1, 0.2, 0.3],
            accelerometer=[0.0, 0.0, 9.81],
        )
        message = _make_imu(typestore, source, 1_700_000_000_000_000_000)
        self.assertEqual(
            [message.orientation.x, message.orientation.y, message.orientation.z,
             message.orientation.w],
            [0.0, 0.0, 0.0, 1.0],
        )
        self.assertTrue(
            np.allclose(
                list(message.angular_velocity.__dict__.values()),
                [0.1, 0.2, 0.3],
            )
        )


class MetricsTests(unittest.TestCase):
    def test_rate_limits(self):
        timestamps = [index * 100_000_000 for index in range(100)]
        metrics = summarize_timestamps(timestamps)
        self.assertAlmostEqual(metrics.mean_hz or 0.0, 10.0)
        self.assertEqual(
            rate_failures(
                "cloud",
                metrics,
                min_mean_hz=9.0,
                max_interval_s=0.25,
                max_percentile=(0.95, 0.12),
            ),
            [],
        )

    def test_coverage_detects_missing_tail(self):
        streams = {
            "cloud": list(range(0, 10_000_000_000, 100_000_000)),
            "imu": list(range(0, 8_000_000_000, 1_000_000)),
        }
        _, failures = coverage_report(
            streams, edge_slack_s=1.0, min_fraction=0.98
        )
        self.assertTrue(any("missing final" in item for item in failures))

    def test_motion_requires_three_axes_and_static_edges(self):
        timestamps = [index * 10_000_000 for index in range(12_001)]
        gyro = [[], [], []]
        accel = [[], [], []]
        for index, _timestamp in enumerate(timestamps):
            moving = 1000 <= index < 11_000
            for axis in range(3):
                value = (
                    math.sin(index * 0.01 + axis * 0.7)
                    if moving
                    else 0.0
                )
                gyro[axis].append(value)
                accel[axis].append(value)
        detail, failures = motion_report(
            timestamps,
            gyro,
            accel,
            static_window_s=10.0,
            axis_gyro_span_rad_s=0.1,
            static_max_gyro_span_rad_s=0.25,
            motion_to_static_ratio=2.0,
            motion_accel_span_m_s2=0.3,
        )
        self.assertTrue(detail["has_static_windows"])
        self.assertEqual(failures, [])


class ExtrinsicsTests(unittest.TestCase):
    def test_matrix_comparison(self):
        identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        self.assertAlmostEqual(relative_rotation_deg(identity, identity), 0.0)
        self.assertAlmostEqual(translation_delta_m([1, 2, 3], [1, 2, 3.02]), 0.02)

    def test_ikalibr_style_quaternion_and_position(self):
        document = {
            "CalibParam": {
                "EXTRI": {
                    "SO3_LkToBr": {"/g1/mid360": [0.0, 0.0, 0.0, 1.0]},
                    "POS_LkInBr": {"/g1/mid360": [0.1, 0.2, 0.3]},
                }
            }
        }
        rotation, translation = parse_extrinsics(document)
        self.assertEqual(rotation, [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
        self.assertEqual(translation, [0.1, 0.2, 0.3])


class ManifestTests(unittest.TestCase):
    def test_capture_id_rejects_paths(self):
        with self.assertRaises(ValueError):
            validate_capture_id("../escape")
        self.assertEqual(validate_capture_id("g1_A_001"), "g1_A_001")

    def test_yaml_round_trip(self):
        data = {"capture": "A", "items": [1, 2, 3], "unicode": "真机"}
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "manifest.yaml"
            path.write_text(dump_yaml(data), encoding="utf-8")
            self.assertEqual(load_yaml(path), data)

    def test_ikalibr_config_contract(self):
        config = build_config(
            bag_path=Path("/tmp/input.bag"),
            output_path=Path("/tmp/output"),
            imu_intrinsics=Path("/tmp/imu-intri.yaml"),
            begin_time_s=0.0,
            duration_s=120.0,
        )
        stream = config["Configor"]["DataStream"]
        self.assertEqual(stream["ReferIMU"], "/g1/torso_imu")
        self.assertEqual(stream["LiDARTopics"][0]["value"]["Type"], "VLP_POINTS")
        self.assertEqual(stream["IMUTopics"][0]["value"]["Type"], "SENSOR_IMU")
        self.assertEqual(stream["RGBDTopics"], [])
        self.assertEqual(
            config["Configor"]["Prior"]["TimeOffsetPadding"],
            0.10,
        )


if __name__ == "__main__":
    unittest.main()
