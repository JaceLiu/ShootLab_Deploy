#!/usr/bin/env python3
"""Ball detector service — runs on G1 onboard computer.

Subscribes to:
  /livox/lidar  (Livox MID360 point cloud)
  rt/lowstate   (Unitree DDS lowstate, for waist angles)

Publishes via DDS:
  "rt/lidar_ball_state"  (raw lidar BallState in pelvis body frame, ~10 Hz)

Usage (from this repository root, e.g. RoboMimic_Deploy):

    python3 onboard/perception/lidar/ball_detector.py

Must be run with the Python interpreter (after sourcing ROS + workspace if needed).
Do not run: ``bash onboard/perception/lidar/ball_detector.py`` — that feeds the file to bash.
"""

import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent.parent.parent.parent.absolute()))

import time
import threading
import select
import termios
import tty
from typing import Dict
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import PointCloud2, PointField
try:
    from unitree_hg.msg import LowState
except ModuleNotFoundError:
    LowState = None

from onboard.perception.lidar.center_kalman_filter import CenterKalmanFilter
from onboard.perception.lidar.mid360_to_base import compute_mid360_to_base_transform
from onboard.perception.lidar.rviz_publisher import RvizPublisher
from onboard.perception.lowstate_dds import (
    UnitreeDdsJointListener,
    UnitreeDdsJointProcessListener,
    read_real_config_net,
    select_lowstate_net,
)
from common.ball_state_dds import BallStatePublisher, SOURCE_LIDAR, SOURCE_NONE
from common.lidar_ball_debug_dds import LidarBallDebugPublisher


# ---------------------------------------------------------------------------
# Least-squares ball center estimator
# ---------------------------------------------------------------------------

def estimate_ball_center_ls(points, r=0.115, offset=0.095, max_iter=10):
    """Fit sphere of known radius r to point cloud. Returns center (3,)."""
    center = points.mean(axis=0).astype(np.float64)
    dist   = np.linalg.norm(points - center, axis=1)
    core   = points[dist < 0.2]
    # If the 0.2m neighbourhood is empty (e.g. scattered noise), fall back
    # to the full-set centroid so we never produce NaN.
    c = core.mean(axis=0).astype(np.float64) if len(core) > 0 else center
    norm_c = np.linalg.norm(c)
    if norm_c < 1e-6:
        return c   # guard against divide-by-zero at the origin
    pc = c + c / norm_c * offset
    dists = np.linalg.norm(points - pc[None, :], axis=1)
    in_shell = (dists < (r + 0.01)) & (dists > (r - 0.01))
    in_n = int(in_shell.sum())
    return pc, in_n
    # for _ in range(max_iter):
    #     v     = points - c[None, :]
    #     dist  = np.linalg.norm(v, axis=1) + 1e-9
    #     resid = dist - r
    #     J     = -(v / dist[:, None])
    #     A     = J.T @ J + 1e-6 * np.eye(3)
    #     dc    = np.linalg.solve(A, -J.T @ resid)
    #     c    += dc
    #     if np.linalg.norm(dc) < 1e-5:
    #         break
    # return c.astype(np.float32)


def _pc2_field_map(msg: PointCloud2) -> Dict[str, PointField]:
    return {f.name: f for f in msg.fields}


def _extract_xyz_intensity_from_pc2(msg: PointCloud2):
    """Fast zero-copy-ish extraction for Livox PointCloud2."""
    fields = _pc2_field_map(msg)
    required = ("x", "y", "z", "intensity")
    if not all(name in fields for name in required):
        return (
            np.zeros((0, 3), dtype=np.float32),
            np.zeros((0,), dtype=np.float32),
        )

    raw = np.frombuffer(msg.data, dtype=np.uint8)
    point_count = int(msg.width) * int(msg.height)
    if point_count == 0 or msg.point_step <= 0:
        return (
            np.zeros((0, 3), dtype=np.float32),
            np.zeros((0,), dtype=np.float32),
        )

    points_u8 = raw.reshape(point_count, msg.point_step)

    def _float32_field(name: str) -> np.ndarray:
        off = fields[name].offset
        return np.frombuffer(
            points_u8[:, off:off + 4].copy().tobytes(), dtype=np.float32
        )

    xyz = np.stack([
        _float32_field("x"),
        _float32_field("y"),
        _float32_field("z"),
    ], axis=1).astype(np.float32, copy=False)
    intensity = _float32_field("intensity")
    return xyz, intensity


# ---------------------------------------------------------------------------
# ROS2 node
# ---------------------------------------------------------------------------

class BallDetector(Node):
    def __init__(
        self,
        dds_topic: str = "rt/ball_state",
        debug_topic: str = "rt/lidar_ball_debug",
        base_y_bias: float = 0.0,
        show: bool = False,
        msg_type: str = "pc2",
        lowstate_net: str | None = None,
        lowstate_topic: str = "rt/lowstate",
        lowstate_max_hz: float = 50.0,
        joint_stale_s: float = 0.25,
        lowstate_listener: UnitreeDdsJointListener | None = None,
    ):
        super().__init__("ball_detector")

        # ---- Detection params ----
        self.r           = 0.115   # ball radius [m]
        self.reflect_thr = 190
        self.min_points  = 5
        self.max_range   = 10
        self.min_range   = 0.2
        self.z_low       = -5.0
        self.z_high      =  5.0
        self.x_low       =  0.0
        self.x_high      =  5.0
        self.y_low       = -5.0
        self.y_high      =  5.0
        self.center_offset = 0.085  # [m], adjustable at runtime from keyboard
        self.base_y_bias = float(base_y_bias)

        # ---- Temporal smoothing (Kalman filter) ----
        self.center_kf = CenterKalmanFilter()
        self._last_lidar_ts = None

        # ---- Joint angles (updated from Unitree DDS rt/lowstate) ----
        self.q_wy   = 0.0
        self.q_wr   = 0.0
        self.q_wp   = 0.0
        self.q_head = 0.593412
        self.q_mid  = 0.0
        self._joint_age_s = None
        self._joint_stale_s = float(joint_stale_s)
        self._last_joint_warn_s = 0.0
        self._last_joint_error_warn_s = 0.0
        self._ros_lowstate_s = 0.0
        self._joint_listener = lowstate_listener
        if self._joint_listener is not None:
            self.get_logger().info(
                f"Unitree DDS lowstate process listener active "
                f"(net={lowstate_net or 'auto'}, topic={lowstate_topic})"
            )
        else:
            self.get_logger().warn("Unitree DDS lowstate listener unavailable")
        self._T_base_mid360 = compute_mid360_to_base_transform(
            self.q_wy, self.q_wr, self.q_wp, self.q_head, self.q_mid
        )
        self._joint_dirty = False

        # ---- DDS publisher ----
        self._dds_topic = dds_topic
        self._dds = BallStatePublisher(domain_id=0, topic_name=dds_topic)
        self.get_logger().info(f"DDS publisher ready on '{dds_topic}'")
        self._debug = LidarBallDebugPublisher(domain_id=0, topic_name=debug_topic)
        self.get_logger().info(
            f"LiDAR debug DDS ready on '{debug_topic}', base_y_bias={self.base_y_bias:+.3f}m"
        )

        # ---- RViz2 publisher ----
        self._show = show
        self._msg_type = msg_type
        self._rviz = RvizPublisher(
            self, frame_id="livox_frame", ball_r=self.r,
            enable=show, cloud_publish_hz=5.0,
        )

        # ---- ROS2 subscriptions ----
        if msg_type == "pc2":
            self.create_subscription(PointCloud2, "/livox/lidar",
                                     self.cb_lidar, 5)
        else:
            self.create_subscription(CustomMsg, "/livox/lidar",
                                     self.cb_lidar, 5)
        if self._joint_listener is None and LowState is not None:
            self.create_subscription(LowState, "/lowstate",
                                    self.cb_lowstate, qos_profile_sensor_data)
            self.get_logger().warn(
                "falling back to ROS /lowstate; prefer Unitree DDS rt/lowstate"
            )
        else:
            if self._joint_listener is None:
                self.get_logger().warn(
                    "unitree_hg ROS msg not found; using default waist/head angles"
                )

        # ---- Worker thread: heavy processing decoupled from ROS callback ----
        # cb_lidar() just swaps the message reference (O(1), non-blocking).
        # All numpy/RViz work runs here, so the ROS executor stays responsive
        # and /lowstate at 500 Hz never starves the lidar processing.
        self._buf_lock       = threading.Lock()
        self._buf_msg        = None
        self._buf_recv_wall  = None
        self._buf_event      = threading.Event()
        self._stop_flag = threading.Event()
        self._worker_thread = threading.Thread(
            target=self._process_loop, daemon=True
        )
        self._worker_thread.start()

        self._stdin_fd = None
        self._stdin_old_term = None

        self.get_logger().info("BallDetector ready.")

    # ------------------------------------------------------------------

    def cb_lowstate(self, msg: LowState):
        q = [m.q for m in msg.motor_state]
        self.q_wy = q[12]
        self.q_wr = q[13]
        self.q_wp = q[14]
        self._ros_lowstate_s = time.time()
        self._joint_age_s = 0.0
        self._joint_dirty = True

    def _refresh_waist_joints(self):
        if self._joint_listener is not None:
            waist_q, age_s = self._joint_listener.snapshot()
            self.q_wy, self.q_wr, self.q_wp = waist_q
            self._joint_age_s = age_s
            self._joint_dirty = True
        elif self._ros_lowstate_s > 0.0:
            self._joint_age_s = time.time() - self._ros_lowstate_s

        now_s = time.monotonic()
        if self._joint_age_s is None or self._joint_age_s > self._joint_stale_s:
            if now_s - self._last_joint_warn_s >= 2.0:
                age = "null" if self._joint_age_s is None else f"{self._joint_age_s:.3f}s"
                self.get_logger().warn(
                    f"lowstate stale/missing (joint_age_s={age}); "
                    "MID360->pelvis transform is using default or stale waist joints"
                )
                self._last_joint_warn_s = now_s
            err = getattr(self._joint_listener, "last_error", None)
            if err and now_s - self._last_joint_error_warn_s >= 5.0:
                self.get_logger().warn(f"Unitree DDS lowstate child error: {err}")
                self._last_joint_error_warn_s = now_s

    def _transform_point_mid360_to_base_fast(self, point_mid360: np.ndarray) -> np.ndarray:
        self._refresh_waist_joints()
        if self._joint_dirty:
            self._T_base_mid360 = compute_mid360_to_base_transform(
                self.q_wy, self.q_wr, self.q_wp, self.q_head, self.q_mid
            )
            self._joint_dirty = False
        p_homogeneous = np.empty(4, dtype=np.float64)
        p_homogeneous[:3] = point_mid360
        p_homogeneous[3] = 1.0
        return (self._T_base_mid360 @ p_homogeneous)[:3]

    def cb_lidar(self, msg):
        # Non-blocking: just hand the message to the worker thread.
        # Python GIL ensures the reference swap is atomic.
        recv_wall = time.time()   # wall-clock when ROS delivered this message
        with self._buf_lock:
            self._buf_msg      = msg
            self._buf_recv_wall = recv_wall
        self._buf_event.set()

    # ------------------------------------------------------------------

    def _process_loop(self):
        """Worker thread — all heavy processing runs here, not in the callback."""
        _frame_n       = 0
        _prev_lidar_ts = None   # lidar header stamp of previous frame (seconds)
        _prev_recv_wall = None  # wall-clock receipt time of previous frame

        while not self._stop_flag.is_set():
            if not self._buf_event.wait(timeout=1.0):
                continue
            self._buf_event.clear()

            t_worker_start = time.time()   # wall-clock when worker wakes up

            with self._buf_lock:
                msg       = self._buf_msg
                recv_wall = self._buf_recv_wall
            if msg is None:
                continue

            t0    = time.perf_counter()
            stamp = msg.header.stamp
            if self._msg_type == "pc2":
                xyz_all, refl_all = _extract_xyz_intensity_from_pc2(msg)
                pts = None
                n = xyz_all.shape[0]
            else:
                pts = msg.points
                xyz_all = None
                refl_all = np.array([p.reflectivity for p in pts], dtype=np.uint8)
                n = len(pts)

            # ── Timestamp diagnostics ─────────────────────────────────
            # lidar_ts  : header stamp embedded by the Livox driver
            # recv_wall : wall-clock when cb_lidar() was called by ROS
            # t_worker  : wall-clock when worker picked up the message
            lidar_ts = stamp.sec + stamp.nanosec * 1e-9

            dt_lidar_ms  = (lidar_ts       - _prev_lidar_ts)  * 1000 if _prev_lidar_ts  else float('nan')
            dt_recv_ms   = (recv_wall      - _prev_recv_wall)  * 1000 if _prev_recv_wall else float('nan')
            age_ms       = (t_worker_start - recv_wall)        * 1000  # how stale the msg is

            _prev_lidar_ts  = lidar_ts
            _prev_recv_wall = recv_wall

            # print(
            #     f"\n[TS] lidar_stamp={lidar_ts:.3f}  "
            #     f"Δlidar={dt_lidar_ms:6.1f}ms  "
            #     f"Δrecv={dt_recv_ms:6.1f}ms  "
            #     f"age={age_ms:5.1f}ms",
            #     flush=True,
            # )

            if n == 0:
                self.center_kf.freeze_motion()
                self._publish_invalid()
                continue

            # ── 2-Pass deserialization ────────────────────────────────
            # Root cause of the original 200-300 ms cost:
            #   4-field tuple comprehension × 20000 pts = 80000 Python
            #   attribute accesses on ROS2 message objects → ~200 ms.
            #
            # Fix: split into two cheap passes.
            #
            # Pass 1 — reflectivity only (1 attr × N points)
            #   Typical high-reflectivity ball hits: M ≈ 10-200 pts << N
            t_pass1 = time.perf_counter()

            # Pass 2a — xyz only for high-refl candidates (3 attr × M)
            high_idx = np.where(refl_all >= self.reflect_thr)[0].tolist()
            if high_idx:
                if xyz_all is not None:
                    cand_raw = xyz_all[high_idx]
                else:
                    cand_raw = np.array(
                        [(pts[i].x, pts[i].y, pts[i].z) for i in high_idx],
                        dtype=np.float32,
                    )
                d_c  = np.linalg.norm(cand_raw, axis=1)
                roi  = (
                    (d_c >= self.min_range) & (d_c <= self.max_range) &
                    (cand_raw[:, 2] >= self.z_low)  & (cand_raw[:, 2] <= self.z_high) &
                    (cand_raw[:, 0] >= self.x_low)  & (cand_raw[:, 0] <= self.x_high) &
                    (cand_raw[:, 1] >= self.y_low)  & (cand_raw[:, 1] <= self.y_high)
                )
                cand = cand_raw[roi]
            else:
                cand = np.zeros((0, 3), dtype=np.float32)
            t_pass2a = time.perf_counter()

            if self._rviz.should_publish_clouds():
                # Full 20000-pt cloud is not needed for RViz debugging.
                step     = max(1, n // 1500)
                if xyz_all is not None:
                    xyz_disp = xyz_all[::step]
                else:
                    xyz_disp = np.array(
                        [(p.x, p.y, p.z) for p in pts[::step]],
                        dtype=np.float32,
                    )
            else:
                xyz_disp = None
            t_pass2b = time.perf_counter()

            if xyz_disp is not None:
                self._rviz.publish_clouds(xyz_disp, cand, stamp)
            t_pub = time.perf_counter()

            if cand.shape[0] < self.min_points:
                self.center_kf.freeze_motion()
                self._publish_invalid()
                if _frame_n % 30 == 0:
                    age = "null" if self._joint_age_s is None else f"{self._joint_age_s:.3f}"
                    print(
                        f"\r[lidar] no ball  n={n} hi={len(high_idx)} cand={cand.shape[0]}  "
                        f"waist_q=({self.q_wy:+.2f},{self.q_wr:+.2f},{self.q_wp:+.2f}) "
                        f"joint_age_s={age}  "
                        f"refl={1000*(t_pass1-t0):.0f}ms "
                        f"cand_xyz={1000*(t_pass2a-t_pass1):.0f}ms "
                        f"disp_xyz={1000*(t_pass2b-t_pass2a):.0f}ms "
                        f"pub={1000*(t_pub-t_pass2b):.0f}ms",
                        end="", flush=True,
                    )
                _frame_n += 1
                continue

            center_lidar, in_n = estimate_ball_center_ls(
                cand, r=self.r, offset=self.center_offset,
            )
            self._rviz.publish_ball_raw(center_lidar, stamp)

            now = time.time()
            dt  = 0.1 if self._last_lidar_ts is None else now - self._last_lidar_ts
            self._last_lidar_ts = now
            center_filtered = self.center_kf.step(center_lidar, dt)
            self._rviz.publish_ball_kf(center_filtered, stamp)

            center_base = self._transform_point_mid360_to_base_fast(center_filtered)
            center_base[1] += self.base_y_bias

            x, y, z = float(center_base[0]), float(center_base[1]), float(center_base[2])
            self._dds.publish(x, y, z, valid=True, source=SOURCE_LIDAR)
            self._debug.publish(
                center_lidar,
                center_filtered,
                center_base,
                base_y_bias=self.base_y_bias,
                candidate_count=cand.shape[0],
                in_shell_count=in_n,
                valid=True,
            )

            dt_ms = (time.perf_counter() - t0) * 1000.0
            # self._rviz.publish_text(center_filtered, cand.shape[0],
            #                         self.center_offset, dt_ms, stamp)

            # Throttle console output to avoid terminal I/O jitter.
            _frame_n += 1
            if _frame_n % 10 == 0:
                age = "null" if self._joint_age_s is None else f"{self._joint_age_s:.3f}"
                print(
                    f"\r[lidar] pelvis=({x:+.3f},{y:+.3f},{z:+.3f})  "
                    f"raw=({center_lidar[0]:.3f},{center_lidar[1]:.3f},{center_lidar[2]:.3f})  "
                    f"waist_q=({self.q_wy:+.2f},{self.q_wr:+.2f},{self.q_wp:+.2f}) "
                    f"joint_age_s={age}  "
                    f"surf_n={in_n} hi={len(high_idx)} cand={cand.shape[0]}  "
                    f"refl={1000*(t_pass1-t0):.0f}ms "
                    f"cand_xyz={1000*(t_pass2a-t_pass1):.0f}ms "
                    f"disp_xyz={1000*(t_pass2b-t_pass2a):.0f}ms "
                    f"pub={1000*(t_pub-t_pass2b):.0f}ms "
                    f"total={dt_ms:.0f}ms",
                    end="", flush=True,
                )

    def _publish_invalid(self):
        if self.center_kf.initialized:
            raw = self.center_kf.position
            cb = self._transform_point_mid360_to_base_fast(raw)
            cb[1] += self.base_y_bias
            self._dds.publish(float(cb[0]), float(cb[1]), float(cb[2]), valid=False, source=SOURCE_LIDAR)
            self._debug.publish(
                raw,
                raw,
                cb,
                base_y_bias=self.base_y_bias,
                valid=False,
            )
        else:
            self._refresh_waist_joints()
            self._dds.publish(0.0, 0.0, 0.0, valid=False, source=SOURCE_NONE)
            zero = np.zeros(3, dtype=np.float32)
            self._debug.publish(
                zero,
                zero,
                zero,
                base_y_bias=self.base_y_bias,
                valid=False,
            )

    def destroy_node(self):
        self._stop_flag.set()
        self._buf_event.set()          # unblock worker if waiting
        self._worker_thread.join(timeout=2)
        if self._joint_listener is not None and hasattr(self._joint_listener, "stop"):
            self._joint_listener.stop()
        if self._stdin_fd is not None and self._stdin_old_term is not None:
            termios.tcsetattr(self._stdin_fd, termios.TCSADRAIN, self._stdin_old_term)
        return super().destroy_node()


# ---------------------------------------------------------------------------

def main():
    import argparse
    parser = argparse.ArgumentParser(description="LiDAR ball detector")
    parser.add_argument("--dds-topic", default="rt/lidar_ball_state",
                        help="DDS topic name to publish to (default: rt/lidar_ball_state)")
    parser.add_argument("--debug-topic", default="rt/lidar_ball_debug",
                        help="DDS debug topic for pre-FK LiDAR values")
    parser.add_argument("--base-y-bias", type=float, default=0.0,
                        help="Add this bias to pelvis-frame y before publishing, e.g. 0.02 for +2cm")
    parser.add_argument("--show", action="store_true",
                        help="Enable RViz debug publishers (slower)")
    parser.add_argument("--msg-type", choices=("pc2", "custom"), default="pc2",
                        help="Livox ROS message type on /livox/lidar (default: pc2)")
    parser.add_argument("--lowstate-net", default=None,
                        help="Network interface for Unitree DDS lowstate. Defaults to deploy_real/config/real.yaml net.")
    parser.add_argument("--lowstate-topic", default="rt/lowstate",
                        help="Unitree DDS lowstate topic for waist joint angles.")
    parser.add_argument("--lowstate-max-hz", type=float, default=50.0,
                        help="Maximum rate for applying lowstate waist joint updates (default 50 Hz).")
    parser.add_argument("--joint-stale-s", type=float, default=0.25,
                        help="Warn when lowstate joint age exceeds this many seconds.")
    args, _ = parser.parse_known_args()

    if args.lowstate_net is None:
        args.lowstate_net = select_lowstate_net(read_real_config_net())
    print(
        f"[ball_detector] Lowstate DDS config: "
        f"net={args.lowstate_net or 'auto'} topic={args.lowstate_topic} "
        f"max_hz={args.lowstate_max_hz:.0f}",
        flush=True,
    )

    lowstate_listener = None
    try:
        lowstate_listener = UnitreeDdsJointProcessListener(
            args.lowstate_net,
            topic=args.lowstate_topic,
            max_hz=args.lowstate_max_hz,
        )
    except Exception as exc:
        print(
            f"[ball_detector] WARN: Unitree DDS lowstate listener failed: {exc}; "
            "will try ROS /lowstate fallback",
            flush=True,
        )

    rclpy.init()
    node = BallDetector(
        dds_topic=args.dds_topic,
        debug_topic=args.debug_topic,
        base_y_bias=args.base_y_bias,
        show=args.show,
        msg_type=args.msg_type,
        lowstate_net=args.lowstate_net,
        lowstate_topic=args.lowstate_topic,
        lowstate_max_hz=args.lowstate_max_hz,
        joint_stale_s=args.joint_stale_s,
        lowstate_listener=lowstate_listener,
    )
    # Use spin_once + sleep instead of spin() to avoid /lowstate 500 Hz
    # saturating the GIL and starving the worker thread.  The sleep yields
    # the GIL every 2 ms so the worker thread can run Python between spins.
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.002)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
