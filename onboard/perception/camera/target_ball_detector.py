"""Chest-camera AprilTag target + bright-ball detector for the G1 onboard computer.

One grayscale/IR camera process detects BOTH the AprilTag target and the bright
ball, each published on its own DDS topic.

Subscribes to:
  /lowstate  (Unitree G1 joint states via ROS2, for waist angles)

Publishes via DDS:
  "rt/target_state"    (AprilTag target position in pelvis body frame)
  "rt/cam_ball_state"  (bright-ball position in pelvis body frame; with --ball-bright)

Usage (launch via run_gray.sh, which fills the grayscale/IR V4L2 defaults):
    bash onboard/perception/camera/run_gray.sh
    bash onboard/perception/camera/run_gray.sh --show
    bash onboard/perception/camera/run_gray.sh --with-fuser --show
    bash onboard/perception/camera/run_gray.sh --tag-id 0 --tag-size 0.12
"""

from __future__ import annotations

import argparse
import math
import multiprocessing as mp
import os
import queue
import signal
import socket
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
try:
    from unitree_hg.msg import LowState
except ModuleNotFoundError:
    LowState = None
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as LowStateHG

sys.path.append(str(Path(__file__).parent.parent.parent.parent.absolute()))

from common.target_state_dds import (  # noqa: E402
    INVALID_CLASS_ID,
    SOURCE_CHEST_CAMERA,
    TargetStatePublisher,
)
from common.ball_state_dds import BallStatePublisher, SOURCE_CAM, SOURCE_NONE  # noqa: E402
from onboard.perception.camera.camera_to_base import (  # noqa: E402
    get_default_chest_extrinsics,
    optical_to_body,
    transform_point_chest_camera_to_base_with_extrinsics,
)
from onboard.perception.camera.timing import StageTimer  # noqa: E402


COAST_FRAMES = 8
EMA_ALPHA = 0.5
EMA_GATE = 0.4


def _read_real_config_net():
    real_cfg_path = Path(__file__).parent.parent.parent.parent / "deploy_real" / "config" / "real.yaml"
    with open(real_cfg_path, "r", encoding="utf-8") as f:
        for line in f:
            stripped = line.strip()
            if stripped.startswith("net:"):
                return stripped.split(":", 1)[1].strip().strip("\"'")
    return None


def _available_net_interfaces():
    try:
        return {name for name in os.listdir("/sys/class/net") if name != "lo"}
    except OSError:
        return set()


def _select_lowstate_net(config_net):
    available = _available_net_interfaces()
    if config_net in available:
        return config_net
    for candidate in ("enP8p1s0", "enp5s0f1", "eth0", "usb0", "usb1"):
        if candidate in available:
            print(
                f"[WARN] configured lowstate net '{config_net}' is unavailable; using '{candidate}'",
                flush=True,
            )
            return candidate
    if available:
        selected = sorted(available)[0]
        print(
            f"[WARN] configured lowstate net '{config_net}' is unavailable; using '{selected}'",
            flush=True,
        )
        return selected
    return config_net

# ── Ball detection constants (shared by the bright-ball detector) ─────────
_BALL_DEPTH_SAMPLE_R = 5
_BALL_DEPTH_MIN      = 0.1    # m — minimum valid depth
_BALL_DEPTH_MAX      = 10.0   # m — maximum valid depth
_BALL_RADIUS         = 0.115  # m — physical ball radius; depth sensor sees front surface
_BALL_COAST          = 10     # frames to hold last position after a miss
_BALL_EMA_ALPHA      = 0.5
_BALL_EMA_GATE       = 0.6    # m — EMA reset threshold

# Bright-ball detection constants (for RealSense IR/greyscale UVC stream).
_BRIGHT_MIN_R       = 12    # px
_BRIGHT_MAX_R       = 180   # px
_BRIGHT_MIN_FILL    = 0.18  # white pixels inside enclosing circle
_BRIGHT_MIN_CIRC    = 0.35  # contour circularity
_BRIGHT_ROI_Y_FRAC  = 0.45  # ignore bright wall/ceiling in the upper image
_BRIGHT_MIN_ASPECT  = 0.65  # reject elongated bright blobs such as shoes
_BRIGHT_MAX_ASPECT  = 1.55
_BRIGHT_MAX_CENTER_OFFSET = 0.35  # contour centroid offset / enclosing radius
_BRIGHT_RADIUS_CORRECTION = 6.0  # px added by the 13x13 dilation used to merge dots

STATUS_PRINT_PERIOD_S = 0.5


_CAMERA_PROFILES = {
    # D435 IR/greyscale UVC stream. Its FOV is substantially wider than the
    # color imager, so sharing the color focal length biases AprilTag pose and
    # monocular ball depth.
    "gray-ir": {
        "intrinsics": {
            "fx_width_ratio": 0.527,
            "fy_width_ratio": 0.508,
            "cx_width_ratio": 0.500,
            "cy_height_ratio": 0.500,
        },
        "chest_xyz_delta": (0.0, 0.020, 0.0),
        "chest_rpy_delta": (0.0, 0.0, 0.0),
        "bright": {
            "threshold": 190,
            "roi_y": 0.42,
            "min_depth": 0.35,
            "max_depth": 10.0,
            "max_abs_y": 5.0,
            "z_min": -1.35,
            "z_max": 0.5,
            "radius_correction": 6.0,
            "min_fill": 0.16,
            "min_circularity": 0.32,
            "min_aspect": 0.68,
            "max_aspect": 1.47,
            "max_center_offset": 0.32,
        },
    },
    # V4L2 YUYV color stream. Keep the previous focal-length approximation,
    # but separate it from the IR profile and use stricter bright-blob filters.
    "color-v4l2": {
        "intrinsics": {
            "fx_width_ratio": 0.712,
            "fy_width_ratio": 0.712,
            "cx_width_ratio": 0.500,
            "cy_height_ratio": 0.500,
        },
        "chest_xyz_delta": (0.0, 0.0, 0.0),
        "chest_rpy_delta": (0.0, 0.0, 0.0),
        "bright": {
            "threshold": 215,
            "roi_y": 0.50,
            "min_depth": 0.40,
            "max_depth": 5.0,
            "max_abs_y": 2.0,
            "z_min": -1.25,
            "z_max": 0.20,
            "radius_correction": 4.0,
            "min_fill": 0.22,
            "min_circularity": 0.40,
            "min_aspect": 0.75,
            "max_aspect": 1.33,
            "max_center_offset": 0.25,
        },
    },
    "realsense": {
        "intrinsics": None,
        "chest_xyz_delta": (0.0, 0.0, 0.0),
        "chest_rpy_delta": (0.0, 0.0, 0.0),
        "bright": {},
    },
}


def _resolve_camera_profile(args) -> str:
    if args.camera_profile != "auto":
        return args.camera_profile
    if args.color_backend != "v4l2":
        return "realsense"
    fourcc = str(args.v4l2_fourcc).upper()
    return "gray-ir" if fourcc in ("GREY", "GRAY", "Y8", "Y800") else "color-v4l2"


def _profile_value(args, attr: str, profile: dict, key: str, fallback):
    value = getattr(args, attr)
    if value is not None:
        return value
    return profile.get("bright", {}).get(key, fallback)


def _build_profile_intrinsics(args, profile: dict):
    intr = profile.get("intrinsics") or {}
    fx = args.fx if args.fx is not None else args.width * intr.get("fx_width_ratio", 0.712)
    fy = args.fy if args.fy is not None else args.width * intr.get("fy_width_ratio", 0.712)
    cx = args.cx if args.cx is not None else args.width * intr.get("cx_width_ratio", 0.5)
    cy = args.cy if args.cy is not None else args.height * intr.get("cy_height_ratio", 0.5)
    return _ApproxIntrinsics(args.width, args.height, fx, fy, cx, cy)


def _detect_ball_bright(
    color_bgr: np.ndarray,
    threshold: int = 180,
    min_r: int = _BRIGHT_MIN_R,
    max_r: int = _BRIGHT_MAX_R,
    min_fill: float = _BRIGHT_MIN_FILL,
    min_circularity: float = _BRIGHT_MIN_CIRC,
    roi_y_frac: float = _BRIGHT_ROI_Y_FRAC,
    min_aspect: float = _BRIGHT_MIN_ASPECT,
    max_aspect: float = _BRIGHT_MAX_ASPECT,
    max_center_offset: float = _BRIGHT_MAX_CENTER_OFFSET,
) -> tuple:
    """Detect a bright, mostly round ball in the IR/greyscale camera stream.

    This is intended for the D435I UVC stream, where the soccer ball appears as
    a bright object but normal HSV color segmentation is meaningless.
    Returns (cx_px, cy_px, r_px) or (None, None, 0.0).
    """
    candidates = _detect_ball_bright_candidates(
        color_bgr,
        threshold=threshold,
        min_r=min_r,
        max_r=max_r,
        min_fill=min_fill,
        min_circularity=min_circularity,
        roi_y_frac=roi_y_frac,
        min_aspect=min_aspect,
        max_aspect=max_aspect,
        max_center_offset=max_center_offset,
    )
    if not candidates:
        return None, None, 0.0
    best = candidates[0]
    return best["cx"], best["cy"], best["r"]


def _detect_ball_bright_candidates(
    image: np.ndarray,
    threshold: int = 180,
    min_r: int = _BRIGHT_MIN_R,
    max_r: int = _BRIGHT_MAX_R,
    min_fill: float = _BRIGHT_MIN_FILL,
    min_circularity: float = _BRIGHT_MIN_CIRC,
    roi_y_frac: float = _BRIGHT_ROI_Y_FRAC,
    min_aspect: float = _BRIGHT_MIN_ASPECT,
    max_aspect: float = _BRIGHT_MAX_ASPECT,
    max_center_offset: float = _BRIGHT_MAX_CENTER_OFFSET,
) -> list:
    """Return bright round candidates sorted by visual confidence."""
    if image.ndim == 2:
        gray = image
    else:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape
    y_min = int(max(0.0, min(0.95, roi_y_frac)) * h)
    roi = gray[y_min:, :]
    if roi.size == 0:
        return []

    # Use a fixed lower bound but adapt upward gently in overexposed scenes.
    # A high percentile makes dotted/reflective balls flicker, so cap its effect.
    dyn_thr = int(max(threshold, min(np.percentile(roi, 90), threshold + 20)))
    _, seed_mask = cv2.threshold(roi, dyn_thr, 255, cv2.THRESH_BINARY)
    bright_pts = cv2.findNonZero(seed_mask)
    if bright_pts is None:
        return []

    # Most frames contain only a compact bright region. Crop expensive morphology
    # and contour extraction to the bright-pixel bounding box instead of the full ROI.
    bx, by, bw, bh = cv2.boundingRect(bright_pts)
    margin = int(max(max_r, 16))
    x0 = max(0, bx - margin)
    y0 = max(0, by - margin)
    x1 = min(w, bx + bw + margin)
    y1 = min(roi.shape[0], by + bh + margin)
    mask = seed_mask[y0:y1, x0:x1]
    k_clean = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    k_merge = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (13, 13))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k_clean)
    # Merge separate reflective dots on the ball into one candidate blob.
    mask = cv2.dilate(mask, k_merge, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k_merge)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for cnt in contours:
        area = float(cv2.contourArea(cnt))
        if area < 40:
            continue
        (cx, cy_roi), r = cv2.minEnclosingCircle(cnt)
        r = float(r)
        if not (min_r <= r <= max_r):
            continue
        perimeter = float(cv2.arcLength(cnt, True))
        if perimeter <= 1e-6:
            continue
        circularity = 4.0 * np.pi * area / (perimeter * perimeter)
        circle_area = np.pi * r * r
        fill = area / circle_area if circle_area > 1e-6 else 0.0
        if circularity < min_circularity or fill < min_fill:
            continue

        rect = cv2.minAreaRect(cnt)
        rw, rh = rect[1]
        if rw <= 1e-6 or rh <= 1e-6:
            continue
        aspect = min(rw, rh) / max(rw, rh)
        if not (min_aspect <= aspect <= max_aspect):
            continue

        moments = cv2.moments(cnt)
        if abs(moments["m00"]) <= 1e-6:
            continue
        centroid_x = float(moments["m10"] / moments["m00"])
        centroid_y = float(moments["m01"] / moments["m00"])
        center_offset = float(np.hypot(centroid_x - cx, centroid_y - cy_roi) / max(r, 1.0))
        if center_offset > max_center_offset:
            continue

        cx += x0
        cy_roi += y0
        cy = cy_roi + y_min
        # Prefer solid, symmetric, round blobs in the lower image. White shoes
        # tend to be elongated or have an off-centre contour centroid.
        lower_bonus = 0.25 * (cy / max(1, h))
        symmetry = 1.0 - center_offset
        score = fill * 1.5 + circularity + aspect + symmetry + lower_bonus + min(r / 80.0, 1.0)
        candidates.append(
            {
                "score": float(score),
                "cx": int(cx),
                "cy": int(cy),
                "r": float(r),
                "r_depth": float(max(min_r, r - _BRIGHT_RADIUS_CORRECTION)),
                "fill": float(fill),
                "circularity": float(circularity),
                "aspect": float(aspect),
                "center_offset": float(center_offset),
                "area": float(area),
            }
        )

    candidates.sort(key=lambda item: item["score"], reverse=True)
    return candidates


class _FPS:
    def __init__(self, window=30):
        self._timestamps = []
        self._window = window

    def tick(self):
        now = time.perf_counter()
        self._timestamps.append(now)
        if len(self._timestamps) > self._window:
            self._timestamps.pop(0)

    @property
    def fps(self):
        if len(self._timestamps) < 2:
            return 0.0
        return (len(self._timestamps) - 1) / (self._timestamps[-1] - self._timestamps[0])


class _JointListener(Node):
    """Background ROS2 node that reads waist joint angles from /lowstate."""

    def __init__(self):
        super().__init__("camera_apriltag_detector_joint_listener")
        self.q_wy = 0.0
        self.q_wr = 0.0
        self.q_wp = 0.0
        self.last_msg_s = 0.0
        if LowState is not None:
            self.create_subscription(LowState, "/lowstate", self._cb, qos_profile_sensor_data)
            self.get_logger().info("camera_apriltag_detector: subscribed to /lowstate")
        else:
            self.get_logger().warn(
                "unitree_hg ROS msg not found; using default waist angles"
            )

    def _cb(self, msg: LowState):
        q = [m.q for m in msg.motor_state]
        self.q_wy = q[12]
        self.q_wr = q[13]
        self.q_wp = q[14]
        self.last_msg_s = time.time()


class _UnitreeDdsJointListener:
    """Read waist joint angles from Unitree DDS rt/lowstate."""

    def __init__(self, net: str | None, topic: str = "rt/lowstate", max_hz: float = 50.0):
        self.q_wy = 0.0
        self.q_wr = 0.0
        self.q_wp = 0.0
        self.last_msg_s = 0.0
        self._min_update_period_s = 1.0 / max_hz if max_hz > 0 else 0.0
        self._last_update_s = 0.0
        self._cb_count = 0
        self._applied_count = 0
        self._skipped_count = 0
        self._cb_time_acc_s = 0.0
        self._cb_time_max_s = 0.0
        self._stats_window_start_s = time.time()
        self._lock = threading.Lock()
        ChannelFactoryInitialize(0, net)
        self._subscriber = ChannelSubscriber(topic, LowStateHG)
        self._subscriber.Init(self._cb, 10)
        print(f"[INFO] Unitree DDS joint listener started ({topic}, max {max_hz:.0f} Hz)")

    def _cb(self, msg: LowStateHG):
        _t0 = time.perf_counter()
        now_s = time.time()
        self._cb_count += 1
        if now_s - self._last_update_s < self._min_update_period_s:
            self._skipped_count += 1
        else:
            with self._lock:
                self._last_update_s = now_s
                self.q_wy = float(msg.motor_state[12].q)
                self.q_wr = float(msg.motor_state[13].q)
                self.q_wp = float(msg.motor_state[14].q)
                self.last_msg_s = now_s
            self._applied_count += 1

        _dt = time.perf_counter() - _t0
        self._cb_time_acc_s += _dt
        self._cb_time_max_s = max(self._cb_time_max_s, _dt)

    def pop_stats(self):
        with self._lock:
            now_s = time.time()
            elapsed_s = max(1e-6, now_s - self._stats_window_start_s)
            stats = {
                "lowstate_cb_hz": self._cb_count / elapsed_s,
                "lowstate_applied_hz": self._applied_count / elapsed_s,
                "lowstate_skipped_hz": self._skipped_count / elapsed_s,
                "lowstate_cb_avg_ms": (
                    (self._cb_time_acc_s / self._cb_count) * 1000.0
                    if self._cb_count else 0.0
                ),
                "lowstate_cb_max_ms": self._cb_time_max_s * 1000.0,
            }
            self._cb_count = 0
            self._applied_count = 0
            self._skipped_count = 0
            self._cb_time_acc_s = 0.0
            self._cb_time_max_s = 0.0
            self._stats_window_start_s = now_s
            return stats


def _parse_tag_ids(raw_values):
    if not raw_values:
        return [0]
    parsed = []
    for raw in raw_values:
        for item in str(raw).split(","):
            item = item.strip()
            if item:
                parsed.append(int(item))
    if not parsed:
        raise ValueError("At least one valid --tag-id is required.")
    return parsed


def _parse_tag_offsets(raw_values):
    offsets = {}
    if not raw_values:
        return offsets
    for raw in raw_values:
        if len(raw) != 4:
            raise ValueError("--tag-offset expects: TAG_ID DX DY DZ")
        tag_id = int(raw[0])
        if tag_id in offsets:
            raise ValueError(f"Duplicate --tag-offset for tag id {tag_id}.")
        offsets[tag_id] = np.array(
            [float(raw[1]), float(raw[2]), float(raw[3])],
            dtype=np.float32,
        )
    return offsets


def _get_apriltag_dictionary(tag_family):
    family_key = str(tag_family).strip().lower()
    families = {
        "tag16h5": cv2.aruco.DICT_APRILTAG_16h5,
        "tag25h9": cv2.aruco.DICT_APRILTAG_25h9,
        "tag36h10": cv2.aruco.DICT_APRILTAG_36h10,
        "tag36h11": cv2.aruco.DICT_APRILTAG_36h11,
    }
    if family_key not in families:
        raise ValueError(
            f"Unsupported AprilTag family '{tag_family}'. "
            f"Choose one of: {sorted(families.keys())}"
        )
    return cv2.aruco.getPredefinedDictionary(families[family_key]), family_key


def _start_camera_pipeline(args, with_depth: bool = False):
    ctx = rs.context()
    devs = ctx.query_devices()
    all_sns = [d.get_info(rs.camera_info.serial_number) for d in devs]
    all_names = [d.get_info(rs.camera_info.name) for d in devs]
    all_usb = []

    print("[INFO] Connected RealSense devices:")
    for idx, (d, sn, nm) in enumerate(zip(devs, all_sns, all_names)):
        try:
            usb = d.get_info(rs.camera_info.usb_type_descriptor)
        except Exception:
            usb = "?"
        all_usb.append(usb)
        usb_warn = "  ⚠ USB2 – frame drops likely!" if usb.startswith("2") else ""
        print(f"  [{idx}] serial={sn}  {nm}  USB {usb}{usb_warn}")

    if args.list_cameras:
        return None, None
    if not all_sns:
        raise RuntimeError("No RealSense device found.")

    serial = args.camera_serial or all_sns[0]
    if serial not in all_sns:
        raise RuntimeError(
            f"Requested RealSense serial {serial} not found. Available: {all_sns}"
        )
    # Use cached enumeration data here. On some Jetson/librealsense builds,
    # touching the same device handle again can raise "failed to set power state".
    sel_idx = all_sns.index(serial)
    usb = all_usb[sel_idx] if sel_idx < len(all_usb) else "?"
    usb_ok = usb.startswith("3")
    tag = "OK" if usb_ok else "WARN – frame drops likely if USB2!"
    print(f"[INFO] CHEST AprilTag camera -> serial {serial}  USB {usb}  [{tag}]")

    pipeline = rs.pipeline()
    fps_tries = [30, 15, 10, 5]
    last_err = None
    for color_fps in fps_tries:
        rs_cfg = rs.config()
        rs_cfg.enable_device(serial)
        rs_cfg.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, color_fps)
        if with_depth:
            # Depth at 848×480 — D455 supports 1280×720 color + 848×480 depth @ 30fps.
            # 424×240 is NOT a valid paired mode at 30fps and causes fallback to 15fps.
            rs_cfg.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, color_fps)
        for attempt in range(2):
            try:
                print(
                    f"[INFO] Starting RealSense serial={serial} "
                    f"(color {color_fps} Hz, attempt {attempt + 1})..."
                )
                profile = pipeline.start(rs_cfg)
            except RuntimeError as exc:
                last_err = exc
                print(f"[WARN] Stream profile not supported: {exc}")
                break
            try:
                pipeline.wait_for_frames(timeout_ms=5000)
                print(
                    f"[INFO] RealSense pipeline OK for serial={serial} "
                    f"(color {color_fps} Hz)"
                )
                return pipeline, profile
            except RuntimeError:
                print("[WARN] Frame timeout. Performing hardware reset...")
                pipeline.stop()
                reset_ctx = rs.context()
                matched = [
                    d
                    for d in reset_ctx.query_devices()
                    if d.get_info(rs.camera_info.serial_number) == serial
                ]
                if not matched:
                    raise RuntimeError(f"RealSense serial {serial} disappeared during reset.")
                matched[0].hardware_reset()
                time.sleep(3)

    msg = f"RealSense failed to start for serial={serial}. Tried FPS: {fps_tries}."
    if last_err is not None:
        msg += f" Last error: {last_err!r}"
    raise RuntimeError(msg)


def _start_depth_pipeline(args):
    """Start a depth-only RealSense pipeline for V4L2 bright-ball validation."""
    serial = args.camera_serial
    if serial is None:
        ctx = rs.context()
        devs = ctx.query_devices()
        if len(devs) == 0:
            raise RuntimeError("No RealSense device found for depth stream.")
        serial = devs[0].get_info(rs.camera_info.serial_number)

    pipeline = rs.pipeline()
    tries = [
        (args.width, args.height, 30),
        (848, 480, 30),
        (640, 480, 30),
        (848, 480, 15),
    ]
    last_err = None
    for width, height, fps in tries:
        cfg = rs.config()
        cfg.enable_device(serial)
        cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        try:
            print(f"[INFO] Starting RealSense depth-only serial={serial} ({width}x{height}@{fps})...")
            profile = pipeline.start(cfg)
            pipeline.wait_for_frames(timeout_ms=5000)
            depth_profile = profile.get_stream(rs.stream.depth).as_video_stream_profile()
            depth_intrin = depth_profile.get_intrinsics()
            depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
            print(
                f"[INFO] Depth-only pipeline OK ({depth_intrin.width}x{depth_intrin.height}) "
                f"scale={depth_scale:.4f}"
            )
            return pipeline, profile, depth_intrin, depth_scale
        except RuntimeError as exc:
            last_err = exc
            try:
                pipeline.stop()
            except Exception:
                pass
            print(f"[WARN] Depth-only stream failed: {exc}")
    raise RuntimeError(f"Failed to start depth-only RealSense stream: {last_err!r}")


def _start_v4l2_color_capture(args):
    """Open the RealSense RGB UVC node via V4L2.

    On this G1 Jetson, librealsense can return all-zero RGB frames while the
    UVC RGB node is healthy. This fallback is for AprilTag-only mode.
    """
    from glob import glob
    import subprocess

    def _device_candidates():
        devices = [args.v4l2_device]
        devices.extend(sorted(glob("/dev/video*")))
        out = []
        seen = set()
        for item in devices:
            if item in seen:
                continue
            seen.add(item)
            out.append(item)
        return out

    def _device_supports_fourcc(device):
        requested = str(args.v4l2_fourcc).upper()
        try:
            proc = subprocess.run(
                ["v4l2-ctl", "-d", device, "--list-formats-ext"],
                text=True,
                capture_output=True,
                timeout=2,
                check=False,
            )
        except Exception:
            return True  # If v4l2-ctl is unavailable, let OpenCV try.
        out = proc.stdout.upper()
        if requested == "GREY":
            return "'GREY'" in out or "8-BIT GREYSCALE" in out
        if requested in ("YUYV", "UYVY"):
            return f"'{requested}'" in out
        return f"'{requested}'" in out

    last_err = None
    for device in _device_candidates():
        if isinstance(device, str) and device.startswith("/dev/video") and not _device_supports_fourcc(device):
            continue
        try:
            return _try_open_v4l2_color_device(args, device)
        except RuntimeError as exc:
            last_err = exc
            print(f"[WARN] V4L2 color open failed on {device}: {exc}")
    raise RuntimeError(f"Failed to open any V4L2 color camera. Last error: {last_err}")


def _try_open_v4l2_color_device(args, device):
    """Open one V4L2 color/IR device node."""
    # OpenCV's V4L2 backend on Jetson can fail when passed "/dev/videoN" as a
    # string even though the same node works by numeric index.
    if isinstance(device, str) and device.startswith("/dev/video"):
        try:
            device_for_cv = int(device.removeprefix("/dev/video"))
        except ValueError:
            device_for_cv = device
    else:
        device_for_cv = device
    cap = cv2.VideoCapture(device_for_cv, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError("OpenCV VideoCapture failed")
    fourcc = str(args.v4l2_fourcc).upper()
    if len(fourcc) != 4:
        raise RuntimeError(f"invalid --v4l2-fourcc '{args.v4l2_fourcc}'")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.v4l2_fps)
    frame = None
    for _ in range(10):
        ok, candidate = cap.read()
        if not ok or candidate is None:
            time.sleep(0.03)
            continue
        if candidate.ndim == 2:
            pass
        elif candidate.ndim == 3 and candidate.shape[2] == 3:
            pass
        else:
            cap.release()
            raise RuntimeError(
                f"not a usable color/IR frame: shape={getattr(candidate, 'shape', None)}"
            )
        if float(candidate.std()) >= 1.0:
            frame = candidate
            break
        time.sleep(0.03)
    if frame is None:
        cap.release()
        raise RuntimeError("only received empty/constant frames during warmup")
    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or args.width)
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or args.height)
    actual_fps = float(cap.get(cv2.CAP_PROP_FPS) or args.v4l2_fps)
    print(
        f"[INFO] V4L2 camera OK -> {device} fourcc={fourcc} "
        f"({actual_w}x{actual_h} @ {actual_fps:.0f}fps)"
    )
    return cap, frame, actual_fps, device


class _ApproxIntrinsics:
    def __init__(self, width, height, fx, fy, ppx, ppy):
        self.width = int(width)
        self.height = int(height)
        self.fx = float(fx)
        self.fy = float(fy)
        self.ppx = float(ppx)
        self.ppy = float(ppy)
        self.coeffs = [0.0, 0.0, 0.0, 0.0, 0.0]


def _deproject_pixel_to_point(intrin, pixel, depth_m: float):
    """Project pixel + depth to the optical camera frame.

    pyrealsense2 only accepts native rs.intrinsics. V4L2 fallback uses
    _ApproxIntrinsics, so keep the pinhole math in Python for both paths.
    """
    u, v = float(pixel[0]), float(pixel[1])
    z = float(depth_m)
    x = (u - intrin.ppx) / intrin.fx * z
    y = (v - intrin.ppy) / intrin.fy * z
    return [x, y, z]


def _sample_depth_patch_for_color_pixel(
    cx: int,
    cy: int,
    depth_arr: np.ndarray,
    depth_scale: float,
    color_intrin,
    depth_intrin,
    sample_r: int = _BALL_DEPTH_SAMPLE_R,
) -> float:
    """Approximate depth lookup for V4L2 IR/color pixel.

    The V4L2 stream is an IR-like UVC node, not a librealsense color stream, so
    exact color->depth extrinsics are unavailable. On D435I this IR stream is
    close enough to depth image geometry for candidate validation; use normalized
    pinhole coordinates to map into the depth image, then take a median patch.
    """
    if depth_arr is None or depth_intrin is None:
        return 0.0
    dh, dw = depth_arr.shape
    ndcx = (float(cx) - color_intrin.ppx) / color_intrin.fx
    ndcy = (float(cy) - color_intrin.ppy) / color_intrin.fy
    dx = int(ndcx * depth_intrin.fx + depth_intrin.ppx + 0.5)
    dy = int(ndcy * depth_intrin.fy + depth_intrin.ppy + 0.5)
    dx = max(0, min(dw - 1, dx))
    dy = max(0, min(dh - 1, dy))
    patch = (
        depth_arr[max(0, dy - sample_r):dy + sample_r + 1,
                  max(0, dx - sample_r):dx + sample_r + 1]
        .astype(np.float32) * depth_scale
    )
    valid_d = patch[(patch > _BALL_DEPTH_MIN) & (patch < _BALL_DEPTH_MAX)]
    if len(valid_d) == 0:
        return 0.0
    return float(np.median(valid_d))


def _put_latest(q, item):
    try:
        while True:
            q.get_nowait()
    except queue.Empty:
        pass
    try:
        q.put_nowait(item)
    except queue.Full:
        pass


def _bright_ball_worker(
    frame_q,
    result_q,
    topic,
    color_intrin_tuple,
    chest_xyz,
    chest_rpy,
    bright_cfg,
):
    color_intrin = _ApproxIntrinsics(*color_intrin_tuple)
    ball_dds = BallStatePublisher(domain_id=0, topic_name=topic)
    center_ema = None
    miss_count = 0

    while True:
        item = frame_q.get()
        if item is None:
            break

        color, waist_q = item
        q_wy, q_wr, q_wp = waist_q
        candidates = _detect_ball_bright_candidates(
            color,
            threshold=bright_cfg["threshold"],
            roi_y_frac=bright_cfg["roi_y"],
            min_fill=bright_cfg["min_fill"],
            min_circularity=bright_cfg["min_circularity"],
            min_aspect=bright_cfg["min_aspect"],
            max_aspect=bright_cfg["max_aspect"],
            max_center_offset=bright_cfg["max_center_offset"],
        )
        selected = None

        for cand in candidates:
            radius_for_depth = max(
                _BRIGHT_MIN_R,
                cand["r"] - bright_cfg["radius_correction"],
            )
            cand_depth = (
                color_intrin.fx * _BALL_RADIUS / radius_for_depth
                if radius_for_depth > 0 else 0.0
            )
            if not (bright_cfg["min_depth"] <= cand_depth <= bright_cfg["max_depth"]):
                continue
            cand_opt = _deproject_pixel_to_point(
                color_intrin, [cand["cx"], cand["cy"]], cand_depth)
            cand_cam = optical_to_body(cand_opt)
            cand_base = transform_point_chest_camera_to_base_with_extrinsics(
                np.array(cand_cam, dtype=np.float32),
                q_wy, q_wr, q_wp,
                chest_xyz=chest_xyz,
                chest_rpy=chest_rpy,
            )
            cx_b, cy_b, cz_b = (float(cand_base[0]), float(cand_base[1]), float(cand_base[2]))
            if abs(cy_b) > bright_cfg["max_abs_y"]:
                continue
            if not (bright_cfg["z_min"] <= cz_b <= bright_cfg["z_max"]):
                continue
            cand["depth"] = float(cand_depth)
            cand["p_cam"] = np.array(cand_cam, dtype=np.float32)
            cand["p_base"] = (cx_b, cy_b, cz_b)
            selected = cand
            break

        if selected is not None:
            miss_count = 0
            p_arr = selected["p_cam"]
            if center_ema is None:
                center_ema = p_arr.copy()
            elif np.linalg.norm(p_arr - center_ema) < _BALL_EMA_GATE:
                center_ema = _BALL_EMA_ALPHA * p_arr + (1 - _BALL_EMA_ALPHA) * center_ema
            else:
                center_ema = p_arr.copy()

            p_base = transform_point_chest_camera_to_base_with_extrinsics(
                center_ema,
                q_wy, q_wr, q_wp,
                chest_xyz=chest_xyz,
                chest_rpy=chest_rpy,
            )
            bx, by, bz = (float(p_base[0]), float(p_base[1]), float(p_base[2]))
            ball_dds.publish(bx, by, bz, valid=True, source=SOURCE_CAM)
            r_int = int(selected["r"])
            overlay = {
                "bbox": (
                    int(selected["cx"] - r_int),
                    int(selected["cy"] - r_int),
                    int(selected["cx"] + r_int),
                    int(selected["cy"] + r_int),
                ),
                "pelvis": (bx, by, bz),
                "depth": float(selected["depth"]),
                "valid": True,
                "miss": 0,
                "source": "bright",
                "status": (
                    f"BRGT valid pelvis=({bx:+.3f},{by:+.3f},{bz:+.3f}) "
                    f"center={float(selected['depth']):.2f}m cand={len(candidates)}"
                ),
            }
        else:
            miss_count += 1
            if miss_count <= _BALL_COAST and center_ema is not None:
                p_base = transform_point_chest_camera_to_base_with_extrinsics(
                    center_ema,
                    q_wy, q_wr, q_wp,
                    chest_xyz=chest_xyz,
                    chest_rpy=chest_rpy,
                )
                bx, by, bz = (float(p_base[0]), float(p_base[1]), float(p_base[2]))
                ball_dds.publish(bx, by, bz, valid=False, source=SOURCE_CAM)
                overlay = {
                    "bbox": None,
                    "pelvis": (bx, by, bz),
                    "depth": 0.0,
                    "valid": False,
                    "miss": miss_count,
                    "source": "bright",
                    "status": (
                        f"BRGT coast {miss_count}/{_BALL_COAST} "
                        f"pelvis=({bx:+.3f},{by:+.3f},{bz:+.3f}) cand={len(candidates)}"
                    ),
                }
            else:
                center_ema = None
                ball_dds.publish(0.0, 0.0, 0.0, valid=False, source=SOURCE_NONE)
                overlay = {
                    "bbox": None,
                    "pelvis": None,
                    "depth": 0.0,
                    "valid": False,
                    "miss": miss_count,
                    "source": "bright",
                    "status": f"BRGT no ball cand={len(candidates)}",
                }
        _put_latest(result_q, overlay)


def _start_mjpeg_server(port: int = 8080):
    import http.server
    import socketserver

    mjpeg_frame = [None]
    mjpeg_lock = threading.Lock()

    class _MJPEGHandler(http.server.BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            if self.path == "/":
                self.send_response(302)
                self.send_header("Location", "/stream")
                self.end_headers()
                return
            if self.path != "/stream":
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                last_sent = None
                while True:
                    with mjpeg_lock:
                        jpg = mjpeg_frame[0]
                    if jpg is None or jpg is last_sent:
                        time.sleep(0.02)
                        continue
                    last_sent = jpg
                    self.wfile.write(
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"
                    )
            except Exception:
                pass

    socketserver.ThreadingTCPServer.allow_reuse_address = True
    try:
        httpd = socketserver.ThreadingTCPServer(("0.0.0.0", port), _MJPEGHandler)
    except OSError as exc:
        if exc.errno == 98:
            raise RuntimeError(
                f"MJPEG port {port} is already in use. Stop the old preview process "
                f"or restart with --show-port {port + 1}."
            ) from exc
        raise
    httpd.daemon_threads = True
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    return httpd, mjpeg_frame, mjpeg_lock


def _get_stream_url(port=8080, path="/stream"):
    host = "127.0.0.1"
    sock = None
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.connect(("192.168.123.1", 1))
        host = sock.getsockname()[0]
    except OSError:
        try:
            host = socket.gethostbyname(socket.gethostname())
        except OSError:
            pass
    finally:
        if sock is not None:
            sock.close()
    return f"http://{host}:{port}{path}"


def _build_camera_matrix(intrinsics):
    return np.array(
        [
            [intrinsics.fx, 0.0, intrinsics.ppx],
            [0.0, intrinsics.fy, intrinsics.ppy],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float32,
    )


def _build_dist_coeffs(intrinsics):
    coeffs = list(intrinsics.coeffs)
    if not coeffs:
        coeffs = [0.0] * 5
    if len(coeffs) < 5:
        coeffs += [0.0] * (5 - len(coeffs))
    return np.array(coeffs[:5], dtype=np.float32).reshape(-1, 1)


def _estimate_tag_pose(corners, camera_matrix, dist_coeffs, tag_size_m):
    # Define the printed tag plane in its own local frame. The tag center is the
    # published target point in this first version of the pipeline.
    half = 0.5 * float(tag_size_m)
    object_points = np.array(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float32,
    )
    image_points = np.asarray(corners, dtype=np.float32).reshape(4, 2)
    solvepnp_flag = getattr(cv2, "SOLVEPNP_IPPE_SQUARE", cv2.SOLVEPNP_ITERATIVE)
    ok, rvec, tvec = cv2.solvePnP(
        object_points,
        image_points,
        camera_matrix,
        dist_coeffs,
        flags=solvepnp_flag,
    )
    if not ok:
        return None, None
    # solvePnP returns the pose in the camera optical frame.
    return rvec, tvec.reshape(3)


def _select_detection(detections, target_tag_ids):
    if not detections:
        return None
    priority = {tag_id: idx for idx, tag_id in enumerate(target_tag_ids)}
    ranked = sorted(
        detections,
        key=lambda det: (
            priority.get(det["tag_id"], len(priority)),
            -det["area"],
            det["distance_m"],
        ),
    )
    return ranked[0]


def _detection_confidence(area_px, image_shape):
    img_h, img_w = image_shape
    area_norm = float(area_px) / max(1.0, float(img_h * img_w))
    return max(0.05, min(1.0, 5.0 * math.sqrt(max(0.0, area_norm))))


def _resolve_target_point_optical(detection, tag_offsets):
    if detection["tvec"] is None:
        return None, False

    target_optical = np.asarray(detection["tvec"], dtype=np.float32).reshape(3)
    offset_tag = tag_offsets.get(detection["tag_id"])
    if offset_tag is None:
        return target_optical, False

    rot_optical_from_tag, _ = cv2.Rodrigues(
        np.asarray(detection["rvec"], dtype=np.float32).reshape(3, 1)
    )
    return target_optical + rot_optical_from_tag @ offset_tag, True


def _fuse_target_points_optical(detections):
    if not detections:
        return None, 0

    weights = []
    points = []
    for det in detections:
        # Larger tags are usually more reliable, so use image area as the
        # dominant fusion weight while keeping a small floor for single-tag use.
        weights.append(max(1.0, float(det["area"])))
        points.append(np.asarray(det["target_tvec"], dtype=np.float32).reshape(3))

    weights_arr = np.asarray(weights, dtype=np.float32)
    points_arr = np.stack(points, axis=0)
    fused = (weights_arr[:, None] * points_arr).sum(axis=0) / weights_arr.sum()
    return fused, len(detections)


def _build_parser():
    parser = argparse.ArgumentParser(
        description="Chest D455 + AprilTag target detector -> rt/target_state"
    )
    parser.add_argument("--width", type=int, default=848)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument(
        "--detect-scale", type=float, default=1.0,
        help="Scale factor applied to gray frame before ArUco detection "
             "(default 1.0 = full resolution). Set <1.0 if bad_alloc occurs "
             "on low-memory systems; corners are scaled back automatically.",
    )
    parser.add_argument(
        "--camera-serial",
        default=None,
        help="RealSense serial for the chest camera.",
    )
    parser.add_argument(
        "--list-cameras",
        action="store_true",
        help="Print connected RealSense serials and exit.",
    )
    parser.add_argument(
        "--color-backend",
        choices=("auto", "realsense", "v4l2"),
        default="auto",
        help="Color capture backend. auto uses v4l2 for --ball-bright, otherwise realsense.",
    )
    parser.add_argument(
        "--camera-profile",
        choices=("auto", "gray-ir", "color-v4l2", "realsense"),
        default="auto",
        help="Camera calibration/detection profile. auto selects from backend/fourcc.",
    )
    parser.add_argument(
        "--v4l2-device",
        default="/dev/video2",
        help="V4L2 RGB camera node used when --color-backend=v4l2.",
    )
    parser.add_argument(
        "--v4l2-fps",
        type=float,
        default=15.0,
        help="V4L2 RGB capture FPS used when --color-backend=v4l2.",
    )
    parser.add_argument(
        "--v4l2-fourcc",
        default="UYVY",
        help="V4L2 pixel format, e.g. GREY for IR grayscale or UYVY/YUYV for color.",
    )
    parser.add_argument("--fx", type=float, default=None,
                        help="Override color camera fx; useful with --color-backend=v4l2.")
    parser.add_argument("--fy", type=float, default=None,
                        help="Override color camera fy; useful with --color-backend=v4l2.")
    parser.add_argument("--cx", type=float, default=None,
                        help="Override color camera principal point x.")
    parser.add_argument("--cy", type=float, default=None,
                        help="Override color camera principal point y.")
    parser.add_argument(
        "--show",
        action="store_true",
        help="Stream annotated video via MJPEG on port 8080.",
    )
    parser.add_argument(
        "--show-port",
        type=int,
        default=8080,
        help="MJPEG preview port used with --show (default: 8080).",
    )
    parser.add_argument(
        "--preview-max-hz",
        type=float,
        default=12.0,
        help="Maximum MJPEG encode/update rate when --show is active (default 12 Hz).",
    )
    parser.add_argument(
        "--status-hz",
        type=float,
        default=1.0 / STATUS_PRINT_PERIOD_S,
        help="Maximum terminal status refresh rate; <=0 disables status prints (default 2 Hz).",
    )
    parser.add_argument(
        "--profile-timing",
        action="store_true",
        help="Print camera pipeline mean/p95 stage timings every --profile-window frames.",
    )
    parser.add_argument(
        "--profile-window",
        type=int,
        default=30,
        help="Timing profiler window in frames (default 30).",
    )
    parser.add_argument(
        "--record",
        action="store_true",
        help="Record annotated camera view to MP4 (saved to --record-dir).",
    )
    parser.add_argument(
        "--record-dir",
        default="recordings",
        help="Directory for recorded MP4 files, relative to repo root (default: recordings/).",
    )
    parser.add_argument(
        "--dds-topic",
        default="rt/target_state",
        help="DDS topic name to publish to.",
    )
    parser.add_argument(
        "--lowstate-net",
        default=None,
        help="Network interface for Unitree DDS lowstate. Defaults to deploy_real/config/real.yaml net.",
    )
    parser.add_argument(
        "--lowstate-topic",
        default="rt/lowstate",
        help="Unitree DDS lowstate topic for waist joint angles.",
    )
    parser.add_argument(
        "--lowstate-max-hz",
        type=float,
        default=50.0,
        help="Maximum rate for applying lowstate waist joint updates (default 50 Hz).",
    )
    parser.add_argument(
        "--tag-id",
        action="append",
        dest="tag_ids",
        help="AprilTag id(s) to track. Can be repeated or comma-separated.",
    )
    parser.add_argument(
        "--tag-family",
        default="tag36h11",
        help="AprilTag family, e.g. tag36h11.",
    )
    parser.add_argument(
        "--tag-size",
        type=float,
        default=0.15,
        help="Printed tag edge length in metres, excluding any white paper margin.",
    )
    parser.add_argument(
        "--tag-offset",
        action="append",
        nargs=4,
        default=None,
        metavar=("TAG_ID", "DX", "DY", "DZ"),
        help=(
            "Optional shared target offset in the tag frame (metres). Repeat per tag as: "
            "--tag-offset TAG_ID DX DY DZ"
        ),
    )
    parser.add_argument("--coast-frames", type=int, default=COAST_FRAMES)
    parser.add_argument("--ema-alpha", type=float, default=EMA_ALPHA)
    parser.add_argument("--ema-gate", type=float, default=EMA_GATE)

    # ── Bright ball detection for IR/greyscale V4L2 stream ────────────────
    parser.add_argument(
        "--ball-bright", action="store_true",
        help="Detect the white soccer ball in the bright IR/greyscale stream. "
             "Uses apparent ball radius for monocular distance and publishes to "
             "--ball-bright-topic (default rt/cam_ball_state).",
    )
    parser.add_argument(
        "--ball-bright-inline",
        action="store_true",
        help="Run bright-ball detection inline in the AprilTag process instead of a worker process.",
    )
    parser.add_argument("--ball-bright-topic", default="rt/cam_ball_state")
    parser.add_argument(
        "--ball-bright-max-hz",
        type=float,
        default=10.0,
        help="Maximum bright-ball detection/publish rate. Default 10 Hz; <=0 runs every frame.",
    )
    parser.add_argument("--ball-bright-threshold", type=int, default=None,
                        help="Minimum grayscale threshold for bright ball detection.")
    parser.add_argument("--ball-bright-roi-y", type=float, default=None,
                        help="Ignore image rows above this fraction (default 0.45).")
    parser.add_argument("--ball-bright-min-depth", type=float, default=None,
                        help="Reject apparent-radius depth below this value in metres.")
    parser.add_argument("--ball-bright-max-depth", type=float, default=None,
                        help="Reject apparent-radius depth above this value in metres.")
    parser.add_argument("--ball-bright-max-abs-y", type=float, default=None,
                        help="Reject pelvis-frame lateral ball positions outside +/- this value.")
    parser.add_argument("--ball-bright-z-min", type=float, default=None,
                        help="Reject pelvis-frame ball z below this value.")
    parser.add_argument("--ball-bright-z-max", type=float, default=None,
                        help="Reject pelvis-frame ball z above this value.")
    parser.add_argument("--ball-bright-radius-correction", type=float,
                        default=None,
                        help="Pixels subtracted from the dilated bright blob radius before depth estimation.")
    parser.add_argument("--ball-bright-min-fill", type=float, default=None,
                        help="Minimum contour fill ratio for bright-ball candidates.")
    parser.add_argument("--ball-bright-min-circularity", type=float, default=None,
                        help="Minimum contour circularity for bright-ball candidates.")
    parser.add_argument("--ball-bright-min-aspect", type=float, default=None,
                        help="Minimum minAreaRect aspect ratio for bright-ball candidates.")
    parser.add_argument("--ball-bright-max-aspect", type=float, default=None,
                        help="Maximum minAreaRect aspect ratio for bright-ball candidates.")
    parser.add_argument("--ball-bright-max-center-offset", type=float, default=None,
                        help="Maximum contour-centroid offset divided by enclosing radius.")
    parser.add_argument("--ball-bright-use-depth", action="store_true",
                        help="Use RealSense depth to verify physical radius and publish ball center.")
    parser.add_argument("--ball-bright-radius-tol", type=float, default=0.08,
                        help="Allowed physical-radius error in metres when --ball-bright-use-depth is active.")
    parser.add_argument("--ball-bright-show-mask", action="store_true",
                        help="Draw the detected bright-ball circle on the input frame.")

    parser.add_argument(
        "--chest-xyz",
        type=float,
        nargs=3,
        default=None,
        metavar=("X", "Y", "Z"),
        help="Override chest camera translation in waist_pitch frame (metres).",
    )
    parser.add_argument(
        "--chest-rpy",
        type=float,
        nargs=3,
        default=None,
        metavar=("ROLL", "PITCH", "YAW"),
        help="Override chest camera rotation in radians.",
    )
    return parser


def main():
    def _raise_keyboard_interrupt(signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)

    parser = _build_parser()
    args = parser.parse_args()
    if args.color_backend == "auto":
        args.color_backend = "v4l2" if args.ball_bright else "realsense"
    args.camera_profile = _resolve_camera_profile(args)
    camera_profile = _CAMERA_PROFILES[args.camera_profile]
    args.ball_bright_threshold = _profile_value(
        args, "ball_bright_threshold", camera_profile, "threshold", 190)
    args.ball_bright_roi_y = _profile_value(
        args, "ball_bright_roi_y", camera_profile, "roi_y", _BRIGHT_ROI_Y_FRAC)
    args.ball_bright_min_depth = _profile_value(
        args, "ball_bright_min_depth", camera_profile, "min_depth", 0.4)
    args.ball_bright_max_depth = _profile_value(
        args, "ball_bright_max_depth", camera_profile, "max_depth", 6.0)
    args.ball_bright_max_abs_y = _profile_value(
        args, "ball_bright_max_abs_y", camera_profile, "max_abs_y", 2.5)
    args.ball_bright_z_min = _profile_value(
        args, "ball_bright_z_min", camera_profile, "z_min", -1.4)
    args.ball_bright_z_max = _profile_value(
        args, "ball_bright_z_max", camera_profile, "z_max", 0.3)
    args.ball_bright_radius_correction = _profile_value(
        args, "ball_bright_radius_correction", camera_profile,
        "radius_correction", _BRIGHT_RADIUS_CORRECTION)
    args.ball_bright_min_fill = _profile_value(
        args, "ball_bright_min_fill", camera_profile, "min_fill", _BRIGHT_MIN_FILL)
    args.ball_bright_min_circularity = _profile_value(
        args, "ball_bright_min_circularity", camera_profile,
        "min_circularity", _BRIGHT_MIN_CIRC)
    args.ball_bright_min_aspect = _profile_value(
        args, "ball_bright_min_aspect", camera_profile, "min_aspect", _BRIGHT_MIN_ASPECT)
    args.ball_bright_max_aspect = _profile_value(
        args, "ball_bright_max_aspect", camera_profile, "max_aspect", _BRIGHT_MAX_ASPECT)
    args.ball_bright_max_center_offset = _profile_value(
        args, "ball_bright_max_center_offset", camera_profile,
        "max_center_offset", _BRIGHT_MAX_CENTER_OFFSET)

    target_tag_ids = _parse_tag_ids(args.tag_ids)
    tag_offsets = _parse_tag_offsets(args.tag_offset)
    print(f"[INFO] Target AprilTag ids requested: {target_tag_ids}")
    if tag_offsets:
        print(
            "[INFO] Tag-frame target offsets: "
            + ", ".join(
                f"id={tag_id}->{tuple(round(float(v), 4) for v in offset)}"
                for tag_id, offset in sorted(tag_offsets.items())
            )
        )

    april_dict, family_name = _get_apriltag_dictionary(args.tag_family)
    detector_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(april_dict, detector_params)
    print(f"[INFO] AprilTag family: {family_name}")
    print(f"[INFO] Tag size: {args.tag_size:.4f} m")

    # Init lowstate + DDS before the camera pipeline — mirroring ball_detector.py.
    # Starting the camera first lets the frame queue fill during DDS setup
    # (several seconds), which overflows librealsense's frame pool → bad_alloc.
    if args.lowstate_net is None:
        args.lowstate_net = _select_lowstate_net(_read_real_config_net())
    joint = _UnitreeDdsJointListener(
        args.lowstate_net,
        topic=args.lowstate_topic,
        max_hz=args.lowstate_max_hz,
    )
    ros_stop = threading.Event()
    ros_thread = None

    dds = TargetStatePublisher(domain_id=0, topic_name=args.dds_topic)
    print(f"[INFO] DDS publisher ready on '{args.dds_topic}'")

    # Depth stream is only used by the optional --ball-bright-use-depth validation.
    pipeline = None
    profile = None
    depth_pipeline = None
    depth_profile = None
    v4l2_cap = None
    v4l2_first_frame = None
    v4l2_device_active = None
    if args.color_backend == "v4l2":
        v4l2_cap, v4l2_first_frame, v4l2_fps, v4l2_device_active = _start_v4l2_color_capture(args)
        if args.ball_bright and args.ball_bright_use_depth:
            try:
                depth_pipeline, depth_profile, depth_intrin, depth_scale = _start_depth_pipeline(args)
            except RuntimeError as exc:
                print(
                    "[WARN] Depth validation unavailable while V4L2 color is active; "
                    f"falling back to monocular bright-ball radius estimate. ({exc})"
                )
                depth_pipeline = None
                depth_profile = None
                depth_intrin = None
                depth_scale = 1.0
                args.ball_bright_use_depth = False
    else:
        pipeline, profile = _start_camera_pipeline(args, with_depth=False)
        if args.list_cameras:
            return

    default_xyz, default_rpy = get_default_chest_extrinsics()
    chest_xyz = (
        tuple(args.chest_xyz)
        if args.chest_xyz is not None
        else tuple(
            float(base) + float(delta)
            for base, delta in zip(default_xyz, camera_profile["chest_xyz_delta"])
        )
    )
    chest_rpy = (
        tuple(args.chest_rpy)
        if args.chest_rpy is not None
        else tuple(
            float(base) + float(delta)
            for base, delta in zip(default_rpy, camera_profile["chest_rpy_delta"])
        )
    )
    print(f"[INFO] Camera profile: {args.camera_profile}")
    print(f"[INFO] Chest extrinsics xyz={tuple(round(v, 5) for v in chest_xyz)}")
    print(f"[INFO] Chest extrinsics rpy={tuple(round(v, 5) for v in chest_rpy)}")

    if args.color_backend == "v4l2":
        color_intrin = _build_profile_intrinsics(args, camera_profile)
        rec_fps = v4l2_fps
    else:
        color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
        color_intrin = color_profile.get_intrinsics()
        rec_fps = color_profile.fps()
    camera_matrix = _build_camera_matrix(color_intrin)
    dist_coeffs = _build_dist_coeffs(color_intrin)
    print(f"[INFO] Color intrinsics fx={color_intrin.fx:.1f} fy={color_intrin.fy:.1f}")

    # ── Ball detection setup ───────────────────────────────────────────────
    # Shared dict written by ball detectors and read by MJPEG/status rendering
    # (dict-key writes are atomic in CPython; stale-by-one-frame reads are OK).
    _ball_overlay = {
        "bbox": None,
        "pelvis": None,
        "depth": 0.0,
        "valid": False,
        "miss": 0,
        "source": None,
        "status": "ball disabled",
    }

    # Depth resources for the optional V4L2 --ball-bright-use-depth validation
    # (a depth-only RealSense stream populates depth_intrin/depth_scale earlier).
    depth_intrin        = locals().get("depth_intrin", None)
    depth_scale         = locals().get("depth_scale", 1.0)

    # Bright IR/greyscale ball detection state (main-loop, no thread).
    _bright_ball_dds = None
    _bright_center_ema = None
    _bright_miss_count = 0
    _bright_frame_q = None
    _bright_result_q = None
    _bright_proc = None
    if args.ball_bright:
        if args.ball_bright_inline:
            _bright_ball_dds = BallStatePublisher(domain_id=0, topic_name=args.ball_bright_topic)
            mode_txt = "inline"
        else:
            # Use spawn so the worker does not inherit already-started DDS,
            # RealSense, or OpenCV background threads from the parent process.
            _bright_mp_ctx = mp.get_context("spawn")
            _bright_frame_q = _bright_mp_ctx.Queue(maxsize=1)
            _bright_result_q = _bright_mp_ctx.Queue(maxsize=1)
            bright_cfg = {
                "threshold": args.ball_bright_threshold,
                "roi_y": args.ball_bright_roi_y,
                "min_depth": args.ball_bright_min_depth,
                "max_depth": args.ball_bright_max_depth,
                "max_abs_y": args.ball_bright_max_abs_y,
                "z_min": args.ball_bright_z_min,
                "z_max": args.ball_bright_z_max,
                "radius_correction": args.ball_bright_radius_correction,
                "min_fill": args.ball_bright_min_fill,
                "min_circularity": args.ball_bright_min_circularity,
                "min_aspect": args.ball_bright_min_aspect,
                "max_aspect": args.ball_bright_max_aspect,
                "max_center_offset": args.ball_bright_max_center_offset,
                "status_hz": args.status_hz,
            }
            _bright_proc = _bright_mp_ctx.Process(
                target=_bright_ball_worker,
                args=(
                    _bright_frame_q,
                    _bright_result_q,
                    args.ball_bright_topic,
                    (
                        color_intrin.width,
                        color_intrin.height,
                        color_intrin.fx,
                        color_intrin.fy,
                        color_intrin.ppx,
                        color_intrin.ppy,
                    ),
                    tuple(chest_xyz),
                    tuple(chest_rpy),
                    bright_cfg,
                ),
                daemon=True,
            )
            _bright_proc.start()
            mode_txt = f"worker pid={_bright_proc.pid}"
        print(
            f"[INFO] Bright ball detection active ({mode_txt})  threshold≥{args.ball_bright_threshold} "
            f"roi_y≥{args.ball_bright_roi_y:.2f} -> topic '{args.ball_bright_topic}'"
        )

    if args.show:
        httpd, mjpeg_frame, mjpeg_lock = _start_mjpeg_server(args.show_port)
        print(f"[INFO] MJPEG stream started -> open {_get_stream_url(args.show_port)}")
    else:
        httpd = None
        mjpeg_frame = None
        mjpeg_lock = None

    video_writer = None
    if args.record:
        rec_dir = Path(args.record_dir)
        rec_dir.mkdir(parents=True, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        rec_path = rec_dir / f"apriltag_{ts}.mp4"
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        video_writer = cv2.VideoWriter(
            str(rec_path), fourcc, rec_fps, (args.width, args.height)
        )
        print(f"[INFO] Recording -> {rec_path}  ({args.width}×{args.height} @ {rec_fps:.0f}fps)")

    center_ema = None
    last_detection = None
    last_pelvis_xyz = None
    miss_count = 0
    fps = _FPS()
    v4l2_fail_count = 0
    _last_bright_run_s = 0.0
    _bright_period_s = 1.0 / args.ball_bright_max_hz if args.ball_bright_max_hz > 0 else 0.0
    _last_status_print_s = 0.0
    _status_period_s = 1.0 / args.status_hz if args.status_hz > 0 else None
    _last_preview_s = 0.0
    _preview_period_s = 1.0 / args.preview_max_hz if args.preview_max_hz > 0 else 0.0
    timing = StageTimer(
        enabled=args.profile_timing,
        window=args.profile_window,
        label="apriltag",
    )

    def _status_print(*print_args, **print_kwargs):
        nonlocal _last_status_print_s
        if _status_period_s is None:
            return
        now_s = time.monotonic()
        if now_s - _last_status_print_s < _status_period_s:
            return
        _last_status_print_s = now_s
        print(*print_args, **print_kwargs)

    def _format_ball_status():
        if not args.ball_bright:
            return None
        return _ball_overlay.get("status") or "ball waiting"

    def _status_line(tag_status: str):
        parts = [tag_status, f"apriltag={fps.fps:4.1f}fps"]
        ball_status = _format_ball_status()
        if ball_status:
            parts.append(ball_status)
        _status_print("\r" + " | ".join(parts) + " " * 8, end="", flush=True)

    print("[INFO] Camera running. Press Ctrl+C to stop.")
    try:
        while True:
            _t_frame0 = time.perf_counter()
            frames = None
            if args.color_backend == "v4l2":
                if v4l2_first_frame is not None:
                    color = v4l2_first_frame
                    v4l2_first_frame = None
                else:
                    ok, color = v4l2_cap.read()
                    if not ok or color is None:
                        v4l2_fail_count += 1
                        if v4l2_fail_count >= 5:
                            print(
                                f"\n[WARN] V4L2 read timeout on {v4l2_device_active}; "
                                "reopening/scanning video devices..."
                            )
                            try:
                                v4l2_cap.release()
                            except Exception:
                                pass
                            v4l2_cap, v4l2_first_frame, v4l2_fps, v4l2_device_active = (
                                _start_v4l2_color_capture(args)
                            )
                            rec_fps = v4l2_fps
                            v4l2_fail_count = 0
                        continue
                    v4l2_fail_count = 0
            else:
                frames = pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue
                color = np.asanyarray(color_frame.get_data()).copy()
            _t_capture = time.perf_counter()

            _bright_depth_np = None
            if args.ball_bright and args.ball_bright_inline and args.ball_bright_use_depth:
                if depth_pipeline is not None:
                    try:
                        _depth_frames = depth_pipeline.wait_for_frames(timeout_ms=5)
                    except RuntimeError:
                        _depth_frames = None
                    if _depth_frames is not None:
                        _df = _depth_frames.get_depth_frame()
                        _bright_depth_np = np.asanyarray(_df.get_data()).copy() if _df else None
                elif frames is not None:
                    _df = frames.get_depth_frame()
                    _bright_depth_np = np.asanyarray(_df.get_data()).copy() if _df else None
            _t_depth = time.perf_counter()

            _t_hsv = time.perf_counter()

            # Bright IR/greyscale ball detection: useful when V4L2 exposes an
            # IR-like stream where the white ball is bright but color is absent.
            _now_loop_s = time.time()
            _run_bright = (
                args.ball_bright
                and args.ball_bright_inline
                and (
                    _bright_period_s <= 0.0
                    or _now_loop_s - _last_bright_run_s >= _bright_period_s
                )
            )
            if args.ball_bright and not args.ball_bright_inline:
                try:
                    while True:
                        _ball_overlay.update(_bright_result_q.get_nowait())
                except queue.Empty:
                    pass
                if (
                    _bright_period_s <= 0.0
                    or _now_loop_s - _last_bright_run_s >= _bright_period_s
                ):
                    _last_bright_run_s = _now_loop_s
                    bright_frame = color.copy() if color.ndim == 2 else color
                    _put_latest(
                        _bright_frame_q,
                        (bright_frame, (joint.q_wy, joint.q_wr, joint.q_wp)),
                    )
            if _run_bright:
                _last_bright_run_s = _now_loop_s
                _br_candidates = _detect_ball_bright_candidates(
                    color,
                    threshold=args.ball_bright_threshold,
                    roi_y_frac=args.ball_bright_roi_y,
                    min_fill=args.ball_bright_min_fill,
                    min_circularity=args.ball_bright_min_circularity,
                    min_aspect=args.ball_bright_min_aspect,
                    max_aspect=args.ball_bright_max_aspect,
                    max_center_offset=args.ball_bright_max_center_offset,
                )
                _br_selected = None
                for _cand in _br_candidates:
                    _surface_depth = 0.0
                    _physical_radius = 0.0
                    if args.ball_bright_use_depth:
                        _surface_depth = _sample_depth_patch_for_color_pixel(
                            _cand["cx"], _cand["cy"],
                            _bright_depth_np, depth_scale,
                            color_intrin, depth_intrin,
                        )
                        if _surface_depth <= 0:
                            continue
                        _physical_radius = float(_cand["r"] * _surface_depth / color_intrin.fx)
                        if abs(_physical_radius - _BALL_RADIUS) > args.ball_bright_radius_tol:
                            continue
                        # Depth image measures the visible front surface. Publish ball center.
                        _cand_depth = _surface_depth + _BALL_RADIUS
                        _cand["r_depth"] = float(_cand["r"])
                    else:
                        _radius_for_depth = max(
                            _BRIGHT_MIN_R,
                            _cand["r"] - args.ball_bright_radius_correction,
                        )
                        _cand["r_depth"] = float(_radius_for_depth)
                        _cand_depth = (color_intrin.fx * _BALL_RADIUS / _radius_for_depth
                                       if _radius_for_depth > 0 else 0.0)
                        _physical_radius = _BALL_RADIUS
                    if not (args.ball_bright_min_depth <= _cand_depth <= args.ball_bright_max_depth):
                        continue
                    _cand_opt = _deproject_pixel_to_point(
                        color_intrin, [_cand["cx"], _cand["cy"]], _cand_depth)
                    _cand_cam = optical_to_body(_cand_opt)
                    _cand_base = transform_point_chest_camera_to_base_with_extrinsics(
                        np.array(_cand_cam, dtype=np.float32),
                        joint.q_wy, joint.q_wr, joint.q_wp,
                        chest_xyz=chest_xyz, chest_rpy=chest_rpy,
                    )
                    _cx_b, _cy_b, _cz_b = (float(_cand_base[0]),
                                           float(_cand_base[1]),
                                           float(_cand_base[2]))
                    if abs(_cy_b) > args.ball_bright_max_abs_y:
                        continue
                    if not (args.ball_bright_z_min <= _cz_b <= args.ball_bright_z_max):
                        continue
                    _cand["depth"] = float(_cand_depth)
                    _cand["surface_depth"] = float(_surface_depth)
                    _cand["physical_radius"] = float(_physical_radius)
                    _cand["p_cam"] = np.array(_cand_cam, dtype=np.float32)
                    _cand["p_base"] = (_cx_b, _cy_b, _cz_b)
                    _br_selected = _cand
                    break

                if _br_selected is not None:
                    _bright_miss_count = 0
                    _br_cx = int(_br_selected["cx"])
                    _br_cy = int(_br_selected["cy"])
                    _br_r = float(_br_selected["r"])
                    _br_r_depth = float(_br_selected.get("r_depth", _br_r))
                    _br_depth = float(_br_selected["depth"])
                    _br_radius_m = float(_br_selected.get("physical_radius", _BALL_RADIUS))
                    _p_arr = _br_selected["p_cam"]

                    if _bright_center_ema is None:
                        _bright_center_ema = _p_arr.copy()
                    elif np.linalg.norm(_p_arr - _bright_center_ema) < _BALL_EMA_GATE:
                        _bright_center_ema = (_BALL_EMA_ALPHA * _p_arr
                                              + (1 - _BALL_EMA_ALPHA) * _bright_center_ema)
                    else:
                        _bright_center_ema = _p_arr.copy()

                    _p_base = transform_point_chest_camera_to_base_with_extrinsics(
                        _bright_center_ema,
                        joint.q_wy, joint.q_wr, joint.q_wp,
                        chest_xyz=chest_xyz, chest_rpy=chest_rpy,
                    )
                    _bbx, _bby, _bbz = (float(_p_base[0]),
                                         float(_p_base[1]),
                                         float(_p_base[2]))
                    _bright_ball_dds.publish(_bbx, _bby, _bbz,
                                             valid=True, source=SOURCE_CAM)

                    _r_int = int(_br_r)
                    _ball_overlay["bbox"]   = (_br_cx - _r_int, _br_cy - _r_int,
                                               _br_cx + _r_int, _br_cy + _r_int)
                    _ball_overlay["pelvis"] = (_bbx, _bby, _bbz)
                    _ball_overlay["depth"]  = _br_depth
                    _ball_overlay["valid"]  = True
                    _ball_overlay["miss"]   = 0
                    _ball_overlay["source"] = "bright"
                    _ball_overlay["status"] = (
                        f"BRGT valid pelvis=({_bbx:+.3f},{_bby:+.3f},{_bbz:+.3f}) "
                        f"center={_br_depth:.2f}m r={_br_r:.1f}/{_br_r_depth:.1f}px "
                        f"R={_br_radius_m:.3f}m"
                    )

                    if args.ball_bright_show_mask:
                        cv2.circle(color, (_br_cx, _br_cy), _r_int,
                                   (0, 255, 180), 2)
                else:
                    _bright_miss_count += 1
                    if _bright_miss_count <= _BALL_COAST and _bright_center_ema is not None:
                        _p_base = transform_point_chest_camera_to_base_with_extrinsics(
                            _bright_center_ema,
                            joint.q_wy, joint.q_wr, joint.q_wp,
                            chest_xyz=chest_xyz, chest_rpy=chest_rpy,
                        )
                        _bbx, _bby, _bbz = (float(_p_base[0]),
                                             float(_p_base[1]),
                                             float(_p_base[2]))
                        _bright_ball_dds.publish(_bbx, _bby, _bbz,
                                                 valid=False, source=SOURCE_CAM)
                        _ball_overlay["miss"]  = _bright_miss_count
                        _ball_overlay["valid"] = False
                        _ball_overlay["source"] = "bright"
                        _ball_overlay["status"] = (
                            f"BRGT coast {_bright_miss_count}/{_BALL_COAST} "
                            f"pelvis=({_bbx:+.3f},{_bby:+.3f},{_bbz:+.3f})"
                        )
                    else:
                        if _bright_miss_count > _BALL_COAST:
                            _bright_center_ema = None
                            _ball_overlay["bbox"] = None
                            _ball_overlay["pelvis"] = None
                            _ball_overlay["depth"] = 0.0
                        _bright_ball_dds.publish(0.0, 0.0, 0.0,
                                                 valid=False, source=SOURCE_NONE)
                        _ball_overlay["valid"] = False
                        _ball_overlay["miss"] = _bright_miss_count
                        _ball_overlay["source"] = "bright"
                        _ball_overlay["status"] = f"BRGT no ball cand={len(_br_candidates)}"
            _t_bright = time.perf_counter()

            gray = color if color.ndim == 2 else cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
            # Downscale before ArUco to avoid bad_alloc in OpenCV C++ on Jetson
            # at high resolutions/fps. Corners are scaled back so pose estimation
            # uses the original camera_matrix unchanged.
            if args.detect_scale != 1.0:
                h, w = gray.shape
                gray_det = cv2.resize(
                    gray, (int(w * args.detect_scale), int(h * args.detect_scale))
                )
            else:
                gray_det = gray
            corners_list, ids, _ = detector.detectMarkers(gray_det)
            _t_detect = time.perf_counter()
            if args.detect_scale != 1.0 and corners_list:
                corners_list = tuple(c / args.detect_scale for c in corners_list)
            ids_flat = ids.reshape(-1).tolist() if ids is not None else []
            all_detections = []
            target_detections = []

            for corners, tag_id in zip(corners_list, ids_flat):
                corners = np.asarray(corners, dtype=np.float32).reshape(4, 2)
                area = abs(cv2.contourArea(corners))
                center_xy = corners.mean(axis=0)
                detection = {
                    "tag_id": int(tag_id),
                    "corners": corners,
                    "center_xy": center_xy,
                    "area": float(area),
                    "distance_m": 0.0,
                    "rvec": None,
                    "tvec": None,
                    "confidence": _detection_confidence(area, color.shape[:2]),
                }
                rvec, tvec = _estimate_tag_pose(corners, camera_matrix, dist_coeffs, args.tag_size)
                if tvec is not None:
                    detection["rvec"] = rvec
                    detection["tvec"] = tvec
                    detection["distance_m"] = float(np.linalg.norm(tvec))
                    target_tvec, uses_offset = _resolve_target_point_optical(
                        detection,
                        tag_offsets,
                    )
                    detection["target_tvec"] = target_tvec
                    detection["target_distance_m"] = float(np.linalg.norm(target_tvec))
                    detection["uses_offset"] = uses_offset
                else:
                    detection["target_tvec"] = None
                    detection["target_distance_m"] = 0.0
                    detection["uses_offset"] = False
                all_detections.append(detection)
                if (
                    detection["tag_id"] in target_tag_ids
                    and detection["target_tvec"] is not None
                ):
                    target_detections.append(detection)

            representative = _select_detection(target_detections, target_tag_ids)
            fused_target_tvec, fused_count = _fuse_target_points_optical(target_detections)
            published = False
            pelvis_xyz = last_pelvis_xyz if last_pelvis_xyz is not None else (0.0, 0.0, 0.0)
            tag_conf = 0.0
            tag_distance = 0.0
            fused_tag_ids = []
            overlay_mode_txt = "mode: waiting"
            tag_status = (
                f"[     ] no tag ({','.join(str(tag_id) for tag_id in target_tag_ids)})"
            )

            if representative is not None and fused_target_tvec is not None:
                miss_count = 0
                last_detection = representative
                fused_tag_ids = [det["tag_id"] for det in target_detections]
                # Each visible tag proposes the same shared target point in the
                # optical frame. Fuse all visible proposals, then convert once.
                p_cam_arr = optical_to_body(fused_target_tvec)

                if center_ema is None:
                    center_ema = p_cam_arr.copy()
                else:
                    gate_dist = float(np.linalg.norm(p_cam_arr - center_ema))
                    if gate_dist < args.ema_gate:
                        center_ema = args.ema_alpha * p_cam_arr + (1.0 - args.ema_alpha) * center_ema
                    else:
                        center_ema = p_cam_arr.copy()

                p_base = transform_point_chest_camera_to_base_with_extrinsics(
                    center_ema,
                    joint.q_wy,
                    joint.q_wr,
                    joint.q_wp,
                    chest_xyz=chest_xyz,
                    chest_rpy=chest_rpy,
                )
                pelvis_xyz = (float(p_base[0]), float(p_base[1]), float(p_base[2]))
                last_pelvis_xyz = pelvis_xyz
                tag_conf = float(max(det["confidence"] for det in target_detections))
                tag_distance = float(np.linalg.norm(fused_target_tvec))
                published = True
                dds.publish(
                    *pelvis_xyz,
                    valid=True,
                    class_id=representative["tag_id"],
                    confidence=tag_conf,
                    source=SOURCE_CHEST_CAMERA,
                )
                target_mode = "fused" if fused_count > 1 else ("offset" if representative["uses_offset"] else "center")
                contributors = ",".join(str(tag_id) for tag_id in fused_tag_ids)
                if fused_count > 1:
                    overlay_mode_txt = f"mode: fused tags {contributors}"
                else:
                    overlay_mode_txt = f"mode: single tag {representative['tag_id']} ({target_mode})"
                tag_status = (
                    f"[TAG {representative['tag_id']}] "
                    f"pelvis=({pelvis_xyz[0]:+.3f},{pelvis_xyz[1]:+.3f},{pelvis_xyz[2]:+.3f}) "
                    # f"dist={tag_distance:.2f}m mode={target_mode} tags={contributors} conf={tag_conf:.2f}"
                )
            else:
                miss_count += 1
                if last_detection is not None and last_pelvis_xyz is not None and miss_count <= args.coast_frames:
                    published = True
                    tag_conf = float(last_detection["confidence"])
                    tag_distance = float(last_detection["target_distance_m"])
                    overlay_mode_txt = f"mode: coast from tag {last_detection['tag_id']}"
                    dds.publish(
                        *last_pelvis_xyz,
                        valid=False,
                        class_id=last_detection["tag_id"],
                        confidence=tag_conf,
                        source=SOURCE_CHEST_CAMERA,
                    )
                    tag_status = (
                        f"[COAST {last_detection['tag_id']}] "
                        f"pelvis=({last_pelvis_xyz[0]:+.3f},{last_pelvis_xyz[1]:+.3f},{last_pelvis_xyz[2]:+.3f}) "
                        # f"dist={tag_distance:.2f}m miss={miss_count}/{args.coast_frames}"
                    )
                else:
                    center_ema = None
                    last_detection = None
                    last_pelvis_xyz = None
                    overlay_mode_txt = "mode: no visible target tags"
                    dds.publish(
                        0.0,
                        0.0,
                        0.0,
                        valid=False,
                        class_id=INVALID_CLASS_ID,
                        confidence=0.0,
                        source=SOURCE_CHEST_CAMERA,
                    )
                    tag_status = (
                        f"[     ] no tag ({','.join(str(tag_id) for tag_id in target_tag_ids)})"
                    )
            _t_publish = time.perf_counter()

            _now_preview_s = time.monotonic()
            _render_show = (
                args.show
                and (
                    _preview_period_s <= 0.0
                    or _now_preview_s - _last_preview_s >= _preview_period_s
                )
            )
            if _render_show or video_writer is not None:
                if color.ndim == 2:
                    vis = cv2.cvtColor(color, cv2.COLOR_GRAY2BGR)
                else:
                    vis = color.copy()
                for det in all_detections:
                    corners = det["corners"].astype(np.int32)
                    is_target = det["tag_id"] in target_tag_ids
                    is_selected = representative is not None and det["tag_id"] == representative["tag_id"] and np.allclose(
                        det["center_xy"], representative["center_xy"]
                    )
                    color_box = (0, 255, 0) if is_selected else ((255, 160, 0) if is_target else (160, 160, 160))
                    cv2.polylines(vis, [corners], True, color_box, 2)
                    cx, cy = det["center_xy"].astype(int)
                    cv2.circle(vis, (cx, cy), 4, color_box, -1)
                    label = f"id={det['tag_id']} d={det['distance_m']:.2f}m"
                    cv2.putText(
                        vis,
                        label,
                        (int(corners[0][0]), max(24, int(corners[0][1]) - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        color_box,
                        2,
                    )

                if representative is None and last_detection is not None and miss_count <= args.coast_frames:
                    coast_corners = last_detection["corners"].astype(np.int32)
                    cv2.polylines(vis, [coast_corners], True, (0, 165, 255), 2)
                    cv2.putText(
                        vis,
                        f"coast {last_detection['tag_id']} {miss_count}/{args.coast_frames}",
                        (int(coast_corners[0][0]), max(24, int(coast_corners[0][1]) - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (0, 165, 255),
                        2,
                    )

                fps_txt = f"AprilTag {fps.fps:.1f} fps"
                ids_txt = f"target ids: {', '.join(str(tag_id) for tag_id in target_tag_ids)}"
                fam_txt = f"family: {family_name} size={args.tag_size:.3f}m"
                extr_txt = f"xyz={tuple(round(v, 3) for v in chest_xyz)}"
                cv2.putText(
                    vis, fps_txt, (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2,
                )
                cv2.putText(
                    vis, ids_txt, (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2,
                )
                cv2.putText(
                    vis, fam_txt, (10, 76),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
                )
                cv2.putText(
                    vis, extr_txt, (10, 98),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
                )
                if tag_offsets:
                    cv2.putText(
                        vis, "target mode: per-tag offset", (10, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
                    )
                cv2.putText(
                    vis,
                    overlay_mode_txt,
                    (10, 142),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    1,
                )
                if published and last_pelvis_xyz is not None:
                    info = (
                        f"pelvis ({pelvis_xyz[0]:+.2f}, {pelvis_xyz[1]:+.2f}, {pelvis_xyz[2]:+.2f})m "
                        f"dist {tag_distance:.2f}m"
                    )
                    cv2.putText(vis, info, (10, vis.shape[0] - 12),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)

                # Draw ball detection overlay (bright-ball mode).
                if args.ball_bright:
                    bo = _ball_overlay
                    if bo["bbox"] is not None:
                        bx1, by1, bx2, by2 = bo["bbox"]
                        is_ball_valid = bo["valid"]
                        box_color = (0, 255, 128) if is_ball_valid else (0, 165, 255)
                        cv2.rectangle(vis, (bx1, by1), (bx2, by2), box_color, 2)
                        bcx, bcy = (bx1 + bx2) // 2, (by1 + by2) // 2
                        cv2.circle(vis, (bcx, bcy), 5, box_color, -1)
                        miss = bo["miss"]
                        label = (f"ball d={bo['depth']:.2f}m" if is_ball_valid
                                 else f"ball coast {miss}")
                        cv2.putText(vis, label, (bx1, max(16, by1 - 6)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, box_color, 2)
                        if bo["pelvis"] is not None:
                            px, py, pz = bo["pelvis"]
                            cv2.putText(
                                vis,
                                f"ball pelvis ({px:+.2f},{py:+.2f},{pz:+.2f})m",
                                (10, vis.shape[0] - 34),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2,
                            )

                if video_writer is not None:
                    video_writer.write(vis)

                if _render_show:
                    ok, jpg_buf = cv2.imencode(".jpg", vis, [cv2.IMWRITE_JPEG_QUALITY, 60])
                    if ok:
                        with mjpeg_lock:
                            mjpeg_frame[0] = jpg_buf.tobytes()
                    _last_preview_s = _now_preview_s
            _t_vis = time.perf_counter()

            fps.tick()
            _status_line(tag_status)
            timing.add("capture", _t_frame0, _t_capture)
            timing.add("depth", _t_capture, _t_depth)
            timing.add("hsv", _t_depth, _t_hsv)
            timing.add("bright", _t_hsv, _t_bright)
            timing.add("apriltag", _t_bright, _t_detect)
            timing.add("publish", _t_detect, _t_publish)
            timing.add("preview", _t_publish, _t_vis)
            timing.add("total", _t_frame0, _t_vis)
            timing.tick()

    except KeyboardInterrupt:
        print("\n[INFO] Interrupted.")
    finally:
        ros_stop.set()
        if _bright_frame_q is not None:
            _put_latest(_bright_frame_q, None)
        if _bright_proc is not None:
            _bright_proc.join(timeout=1.0)
            if _bright_proc.is_alive():
                _bright_proc.terminate()
                _bright_proc.join(timeout=1.0)
        if pipeline is not None:
            pipeline.stop()
        if depth_pipeline is not None:
            depth_pipeline.stop()
        if v4l2_cap is not None:
            v4l2_cap.release()
        if video_writer is not None:
            video_writer.release()
            print(f"[INFO] Recording saved.")
        if httpd is not None:
            httpd.shutdown()
        if ros_thread is not None and ros_thread.is_alive():
            ros_thread.join(timeout=1.0)
        if ros_thread is not None and rclpy.ok():
            rclpy.shutdown()
        print("[INFO] Done.")


if __name__ == "__main__":
    main()
