#!/usr/bin/env python3
"""Validate one native G1 MID360/torso-IMU ROS2 MCAP capture."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

from g1_calibration.cloud import (
    CloudInspection,
    inspect_cloud,
    validate_cloud_inspection,
)
from g1_calibration.constants import (
    ANALYSIS_LIMITS,
    CLOUD_TOPIC,
    IMU_TOPIC,
    LIVOX_IMU_TOPIC,
    LOWSTATE_TOPIC,
    TOPIC_TYPES,
)
from g1_calibration.manifest import load_yaml, utc_now, write_yaml
from g1_calibration.metrics import (
    coverage_report,
    motion_report,
    rate_failures,
    summarize_timestamps,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path, help="capture directory or .mcap path")
    parser.add_argument("--profile", choices=("smoke", "A", "B"))
    parser.add_argument("--json-out", type=Path)
    parser.add_argument(
        "--cloud-time-mode",
        choices=("auto", "native", "converted"),
        default="auto",
    )
    parser.add_argument(
        "--skip-motion-check",
        action="store_true",
        help="Diagnostics only; does not satisfy the A/B acceptance procedure",
    )
    return parser.parse_args()


def resolve_bag_dir(path: Path) -> Path:
    resolved = path.expanduser().resolve()
    return resolved.parent if resolved.suffix == ".mcap" else resolved


def add_cloud_inspection(
    aggregate: dict[str, Any] | None,
    inspection: CloudInspection,
) -> dict[str, Any]:
    current = inspection.to_dict()
    if aggregate is None:
        return current
    aggregate["scan_time_min_s"] = min(
        float(aggregate["scan_time_min_s"]), current["scan_time_min_s"]
    )
    aggregate["scan_time_max_s"] = max(
        float(aggregate["scan_time_max_s"]), current["scan_time_max_s"]
    )
    aggregate["monotonic"] = bool(aggregate["monotonic"] and current["monotonic"])
    aggregate["sampled_frames"] = int(aggregate["sampled_frames"]) + 1
    aggregate["point_count_min"] = min(
        int(aggregate["point_count_min"]), int(current["point_count"])
    )
    aggregate["point_count_max"] = max(
        int(aggregate["point_count_max"]), int(current["point_count"])
    )
    return aggregate


def inspect_bag(
    bag_dir: Path,
    *,
    cloud_time_mode: str,
) -> dict[str, Any]:
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise RuntimeError(
            "analysis must run in the sourced ROS2 Python environment: " + str(exc)
        ) from exc

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    topic_metadata = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    failures = []
    for topic, expected in TOPIC_TYPES.items():
        actual = topic_metadata.get(topic)
        if actual is None:
            failures.append(f"{topic}: topic absent from bag")
        elif actual != expected:
            failures.append(f"{topic}: type {actual}, expected {expected}")

    timestamps: dict[str, list[int]] = {
        CLOUD_TOPIC: [],
        IMU_TOPIC: [],
        LOWSTATE_TOPIC: [],
        LIVOX_IMU_TOPIC: [],
    }
    imu_gyro = [[], [], []]
    imu_accel = [[], [], []]
    cloud_inspection: dict[str, Any] | None = None
    cloud_inspection_errors: list[str] = []
    cloud_count = 0
    cloud_sample_period = 50

    while reader.has_next():
        topic, raw, receive_time_ns = reader.read_next()
        if topic in timestamps:
            timestamps[topic].append(int(receive_time_ns))
        if topic == IMU_TOPIC:
            message = deserialize_message(raw, get_message(topic_metadata[topic]))
            for axis in range(3):
                imu_gyro[axis].append(float(message.gyroscope[axis]))
                imu_accel[axis].append(float(message.accelerometer[axis]))
        elif topic == CLOUD_TOPIC:
            cloud_count += 1
            # Inspect about one frame every five seconds, plus the first frame.
            # Deserializing every MID360 frame into Python objects adds no value
            # to the field/time checks.
            if cloud_count == 1 or cloud_count % cloud_sample_period == 0:
                message = deserialize_message(raw, get_message(topic_metadata[topic]))
                try:
                    inspection = inspect_cloud(
                        message, mode=cloud_time_mode, max_samples=100_000
                    )
                    if cloud_inspection is None:
                        cloud_inspection = inspection.to_dict()
                        cloud_inspection.update(
                            {
                                "sampled_frames": 1,
                                "point_count_min": inspection.point_count,
                                "point_count_max": inspection.point_count,
                            }
                        )
                    else:
                        cloud_inspection = add_cloud_inspection(
                            cloud_inspection, inspection
                        )
                except Exception as exc:
                    cloud_inspection_errors.append(
                        f"cloud frame {cloud_count}: {exc}"
                    )

    cloud_metrics = summarize_timestamps(timestamps[CLOUD_TOPIC])
    imu_metrics = summarize_timestamps(timestamps[IMU_TOPIC])
    lowstate_metrics = summarize_timestamps(timestamps[LOWSTATE_TOPIC])
    livox_imu_metrics = summarize_timestamps(timestamps[LIVOX_IMU_TOPIC])

    failures.extend(
        rate_failures(
            "cloud",
            cloud_metrics,
            min_mean_hz=ANALYSIS_LIMITS["cloud_min_mean_hz"],
            max_interval_s=ANALYSIS_LIMITS["cloud_max_interval_s"],
            max_percentile=(0.95, ANALYSIS_LIMITS["cloud_max_p95_interval_s"]),
        )
    )
    failures.extend(
        rate_failures(
            "torso_imu",
            imu_metrics,
            min_mean_hz=ANALYSIS_LIMITS["imu_min_mean_hz"],
            max_interval_s=ANALYSIS_LIMITS["imu_max_interval_s"],
            max_percentile=(0.99, ANALYSIS_LIMITS["imu_max_p99_interval_s"]),
        )
    )
    failures.extend(rate_failures("lowstate", lowstate_metrics))
    failures.extend(rate_failures("livox_imu_diagnostic", livox_imu_metrics))

    coverage, coverage_failures = coverage_report(
        {
            "cloud": timestamps[CLOUD_TOPIC],
            "torso_imu": timestamps[IMU_TOPIC],
            "lowstate": timestamps[LOWSTATE_TOPIC],
        },
        edge_slack_s=ANALYSIS_LIMITS["coverage_edge_slack_s"],
        min_fraction=ANALYSIS_LIMITS["coverage_min_fraction"],
    )
    failures.extend(coverage_failures)

    if cloud_inspection is None:
        failures.append("cloud: no frame could be inspected")
    else:
        representative = CloudInspection(
            field_names=tuple(cloud_inspection["field_names"]),
            field_datatypes={
                str(name): int(datatype)
                for name, datatype in cloud_inspection["field_datatypes"].items()
            },
            ring_field=str(cloud_inspection["ring_field"]),
            time_field=str(cloud_inspection["time_field"]),
            point_count=int(cloud_inspection["point_count"]),
            sampled_points=int(cloud_inspection["sampled_points"]),
            scan_time_min_s=float(cloud_inspection["scan_time_min_s"]),
            scan_time_max_s=float(cloud_inspection["scan_time_max_s"]),
            monotonic=bool(cloud_inspection["monotonic"]),
        )
        failures.extend(
            validate_cloud_inspection(
                representative,
                time_min_s=ANALYSIS_LIMITS["cloud_scan_time_min_s"],
                time_max_s=ANALYSIS_LIMITS["cloud_scan_time_max_s"],
            )
        )
    failures.extend(cloud_inspection_errors)

    motion, motion_failures = motion_report(
        timestamps[IMU_TOPIC],
        imu_gyro,
        imu_accel,
        static_window_s=ANALYSIS_LIMITS["static_window_s"],
        axis_gyro_span_rad_s=ANALYSIS_LIMITS["motion_axis_gyro_span_rad_s"],
        static_max_gyro_span_rad_s=ANALYSIS_LIMITS["static_max_gyro_span_rad_s"],
        motion_to_static_ratio=ANALYSIS_LIMITS["motion_to_static_ratio"],
        motion_accel_span_m_s2=ANALYSIS_LIMITS["motion_accel_span_m_s2"],
    )
    motion["failures"] = motion_failures

    bag_duration = float(coverage.get("bag_duration_s", 0.0))
    report: dict[str, Any] = {
        "schema_version": 1,
        "generated_at_utc": utc_now(),
        "bag": str(bag_dir),
        "profile": None,
        "status": "pending",
        "duration_s": bag_duration,
        "topics": {
            topic: {"type": topic_metadata.get(topic)}
            for topic in TOPIC_TYPES
        },
        "rates": {
            "cloud": cloud_metrics.to_dict(),
            "torso_imu": imu_metrics.to_dict(),
            "lowstate": lowstate_metrics.to_dict(),
            "livox_imu_diagnostic": livox_imu_metrics.to_dict(),
        },
        "coverage": coverage,
        "cloud": cloud_inspection,
        "motion": motion,
        "failures": failures,
        "warnings": [],
    }
    return report


def main() -> int:
    args = parse_args()
    bag_dir = resolve_bag_dir(args.bag)
    manifest_path = bag_dir / "capture_manifest.yaml"
    manifest = load_yaml(manifest_path) if manifest_path.is_file() else {}
    profile = args.profile or manifest.get("profile") or "smoke"
    try:
        report = inspect_bag(bag_dir, cloud_time_mode=args.cloud_time_mode)
    except Exception as exc:
        print(f"ANALYSIS ERROR: {exc}", file=sys.stderr)
        return 2
    report["profile"] = profile

    if profile in {"A", "B"}:
        duration = float(report["duration_s"])
        if not (
            ANALYSIS_LIMITS["formal_min_duration_s"]
            <= duration
            <= ANALYSIS_LIMITS["formal_max_duration_s"]
        ):
            report["failures"].append(
                f"profile {profile}: duration {duration:.3f} s is outside "
                f"{ANALYSIS_LIMITS['formal_min_duration_s']:.0f}-"
                f"{ANALYSIS_LIMITS['formal_max_duration_s']:.0f} s"
            )
        if args.skip_motion_check:
            report["warnings"].append(
                "motion check skipped; this bag does not satisfy formal acceptance"
            )
            report["failures"].append(
                "formal profile requires the motion check"
            )
        else:
            report["failures"].extend(report["motion"].get("failures", []))
        report["motion"]["status"] = (
            "passed" if not report["motion"].get("failures") else "failed"
        )
    else:
        report["motion"]["status"] = "not_required"
        report["motion"]["failures"] = []

    report["status"] = "passed" if not report["failures"] else "failed"
    write_yaml(bag_dir / "analysis.yaml", report)
    if args.json_out:
        args.json_out.write_text(
            json.dumps(report, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
    if manifest_path.is_file():
        manifest["analysis"] = {
            "status": report["status"],
            "profile": profile,
            "generated_at_utc": report["generated_at_utc"],
            "failure_count": len(report["failures"]),
            "rates": report["rates"],
            "coverage": report["coverage"],
            "cloud_fields": report["cloud"].get("field_names")
            if report["cloud"]
            else None,
            "motion_status": report["motion"].get("status"),
        }
        manifest["status"] = (
            "validated_passed"
            if report["status"] == "passed"
            else "validation_failed"
        )
        manifest["updated_at_utc"] = utc_now()
        write_yaml(manifest_path, manifest)

    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
