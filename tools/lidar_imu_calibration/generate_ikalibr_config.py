#!/usr/bin/env python3
"""Generate an iKalibr YAML matching the converted G1 ROS1 bag contract."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from g1_calibration.manifest import write_yaml


def build_config(
    *,
    bag_path: Path,
    output_path: Path,
    imu_intrinsics: Path,
    begin_time_s: float,
    duration_s: float,
) -> dict[str, object]:
    return {
        "Configor": {
            "DataStream": {
                "IMUTopics": [
                    {
                        "key": "/g1/torso_imu",
                        "value": {
                            "Type": "SENSOR_IMU",
                            "Intrinsics": str(imu_intrinsics),
                            "AcceWeight": 17.68,
                            "GyroWeight": 57.66,
                        },
                    }
                ],
                "RadarTopics": [],
                "LiDARTopics": [
                    {
                        "key": "/g1/mid360",
                        "value": {
                            "Type": "VLP_POINTS",
                            "Weight": 100.0,
                        },
                    }
                ],
                "CameraTopics": [],
                "RGBDTopics": [],
                "ReferIMU": "/g1/torso_imu",
                "BagPath": str(bag_path),
                "BeginTime": begin_time_s,
                "Duration": duration_s,
                "OutputPath": str(output_path),
            },
            "Prior": {
                "GravityNorm": 9.81,
                "SpatTempPrioriPath": "",
                "OptTemporalParams": True,
                "TimeOffsetPadding": 0.10,
                "ReadoutTimePadding": 0.01,
                "MapDownSample": 0.10,
                "KnotTimeDist": {
                    "SO3Spline": 0.05,
                    "ScaleSpline": 0.05,
                },
                "NDTLiDAROdometer": {
                    "Resolution": 0.5,
                    "KeyFrameDownSample": 0.1,
                },
                "LiDARDataAssociate": {
                    "MapDownSample": 0.10,
                    "PointToSurfelMax": 0.1,
                    "PlanarityMin": 0.6,
                },
            },
            "Preference": {
                "UseCudaInSolving": False,
                "Outputs": ["ParamInEachIter"],
                "OutputDataFormat": "YAML",
                "ThreadsToUse": -1,
                "SplineScaleInViewer": 3.0,
                "CoordSScaleInViewer": 0.3,
            },
        }
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, required=True, help="converted ROS1 .bag")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--ikalibr-root",
        type=Path,
        default=Path("/workspace/unitree-jetson-container/ljc/iKalibr"),
    )
    parser.add_argument("--config", type=Path)
    parser.add_argument("--begin-time", type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=120.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    bag_path = args.bag.expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()
    ikalibr_root = args.ikalibr_root.expanduser().resolve()
    intrinsics = ikalibr_root / "config" / "imu-intri.yaml"
    if not bag_path.is_file():
        raise SystemExit(f"bag not found: {bag_path}")
    if not intrinsics.is_file():
        raise SystemExit(f"iKalibr IMU intrinsics not found: {intrinsics}")
    output_dir.mkdir(parents=True, exist_ok=True)
    config_path = args.config or (output_dir / "config-ikalibr.yaml")
    config = build_config(
        bag_path=bag_path,
        output_path=output_dir,
        imu_intrinsics=intrinsics,
        begin_time_s=args.begin_time,
        duration_s=args.duration,
    )
    write_yaml(config_path, config)
    print(f"Wrote iKalibr config: {config_path}")
    print(json.dumps(config["Configor"]["DataStream"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
