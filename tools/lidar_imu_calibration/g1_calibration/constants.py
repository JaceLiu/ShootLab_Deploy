"""Shared data contract and defaults for G1 LiDAR-IMU calibration bags."""

from __future__ import annotations

from pathlib import Path

CLOUD_TOPIC = "/utlidar/cloud_livox_mid360"
IMU_TOPIC = "/secondary_imu"
LOWSTATE_TOPIC = "/lowstate"
LIVOX_IMU_TOPIC = "/utlidar/imu_livox_mid360"

TOPIC_TYPES = {
    CLOUD_TOPIC: "sensor_msgs/msg/PointCloud2",
    IMU_TOPIC: "unitree_hg/msg/IMUState",
    LOWSTATE_TOPIC: "unitree_hg/msg/LowState",
    LIVOX_IMU_TOPIC: "sensor_msgs/msg/Imu",
}

CAPTURE_PROFILES = {
    "smoke": {
        "duration_s": 15,
        "motion": "static smoke test",
        "requires_motion": False,
    },
    "A": {
        "duration_s": 120,
        "motion": (
            "10 s static; slow yaw, roll, and pitch sweeps in both directions; "
            "combined three-axis motion with modest translation; 10 s static"
        ),
        "requires_motion": True,
    },
    "B": {
        "duration_s": 120,
        "motion": (
            "10 s static; reverse-order pitch, yaw, and roll sweeps; "
            "different combined motion and translation direction; 10 s static"
        ),
        "requires_motion": True,
    },
}

CONFLICT_PROCESS_PATTERNS = (
    "ball_detector.py",
    "target_ball_detector.py",
    "ball_fuser.py",
    "deploy_real.py",
    "deploy_policy.py",
    "deploy_bridge_py.py",
    "policy_runtime.py",
    "FreeKick.py",
    "cpp_bridge_main",
    "livox_ros_driver2_node",
    "msg_MID360_launch",
)

# The installed Livox MID360 PointCloud2 schema differs from the PC contract:
# native: x/y/z/intensity/tag/line/timestamp (ns, absolute)
# converted: x/y/z/intensity/ring/time (s, scan-relative)
CLOUD_XYZI_FIELDS = ("x", "y", "z", "intensity")
CLOUD_RING_ALIASES = ("ring", "line")
CLOUD_TIME_ALIASES = ("time", "timestamp")

REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_DATA_ROOT = REPO_ROOT.parent / "calibration_data"
DEFAULT_PC_OUTPUT_ROOT = REPO_ROOT / "calibration_data_ros1"

ANALYSIS_LIMITS = {
    "cloud_min_mean_hz": 9.0,
    "cloud_max_p95_interval_s": 0.120,
    "cloud_max_interval_s": 0.250,
    "imu_min_mean_hz": 950.0,
    "imu_max_p99_interval_s": 0.003,
    "imu_max_interval_s": 0.020,
    "cloud_scan_time_min_s": 0.0,
    "cloud_scan_time_max_s": 0.150,
    "coverage_edge_slack_s": 2.0,
    "coverage_min_fraction": 0.98,
    "formal_min_duration_s": 90.0,
    "formal_max_duration_s": 120.0,
    "motion_axis_gyro_span_rad_s": 0.10,
    "static_max_gyro_span_rad_s": 0.25,
    "motion_to_static_ratio": 2.0,
    "motion_accel_span_m_s2": 0.30,
    "static_window_s": 10.0,
}
