#!/usr/bin/env python3
"""Ball-state sensor fusion: lidar + chest camera -> single rt/ball_state.

Subscribes to:
  rt/lidar_ball_state  - raw MID360 lidar ball observation
  rt/cam_ball_state    - raw chest camera ball observation

Publishes:
  rt/ball_state        — fused authoritative ball position for deploy_policy.py

Fusion strategy:
  1. choose the authoritative raw observation with lidar priority
  2. feed that observation into one final CenterKalmanFilter
  3. publish rt/ball_state for policy

Usage (from repo root):
    bash onboard/perception/run_ball_fuser.sh
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.append(str(Path(__file__).parent.parent.parent.absolute()))

from common.ball_state_dds import (
    BallState,
    BallStatePublisher,
    BallStateSubscriber,
    SOURCE_CAM,
    SOURCE_LIDAR,
    SOURCE_NONE,
)
from onboard.perception.lidar.center_kalman_filter import CenterKalmanFilter

# ── Thresholds ──────────────────────────────────────────────────────────────
LIDAR_MAX_RANGE  = 2.0   # m — beyond this, lidar is unreliable / out of range
CAM_MIN_RANGE    = 0.8   # m — closer than this, camera depth is noisy
FUSE_HZ          = 50    # Hz — fuser publish rate
STALE_MS         = 300   # ms — treat sensor data as stale beyond this

# SOURCE_FUSED is not a new constant in the IDL — we reuse SOURCE_CAM / SOURCE_LIDAR
# to indicate which sensor was authoritative, so deploy_policy.py can distinguish.


def _dist(s: BallState) -> float:
    return (s.x ** 2 + s.y ** 2 + s.z ** 2) ** 0.5


def choose_observation(lidar: BallState, cam: BallState) -> BallState:
    """Return the best raw observation before final Kalman smoothing."""
    lidar_real   = lidar.valid == 1 and lidar.source == SOURCE_LIDAR
    cam_valid    = cam.valid == 1 and cam.source == SOURCE_CAM

    d_lidar = _dist(lidar) if lidar_real else 0.0
    d_cam   = _dist(cam)   if cam_valid else 0.0

    if lidar_real:
        if d_lidar <= LIDAR_MAX_RANGE or not cam_valid:
            return BallState(
                timestamp_us=lidar.timestamp_us,
                x=lidar.x, y=lidar.y, z=lidar.z,
                valid=1, source=SOURCE_LIDAR,
            )
        # Lidar reports a far out-of-range point; camera is more trustworthy there.
        return BallState(
            timestamp_us=cam.timestamp_us,
            x=cam.x, y=cam.y, z=cam.z,
            valid=1, source=SOURCE_CAM,
        )

    if cam_valid and d_cam >= CAM_MIN_RANGE:
        return BallState(
            timestamp_us=cam.timestamp_us,
            x=cam.x, y=cam.y, z=cam.z,
            valid=1, source=SOURCE_CAM,
        )

    return BallState(valid=0, source=SOURCE_NONE)


def fuse(lidar: BallState, cam: BallState) -> BallState:
    """Backward-compatible selector without Kalman state."""
    return choose_observation(lidar, cam)


def main():
    global LIDAR_MAX_RANGE, CAM_MIN_RANGE

    parser = argparse.ArgumentParser(
        description="Fuse lidar + camera ball estimates → rt/ball_state"
    )
    parser.add_argument("--lidar-topic",  default="rt/lidar_ball_state")
    parser.add_argument("--cam-topic",    default="rt/cam_ball_state")
    parser.add_argument("--output-topic", default="rt/ball_state")
    parser.add_argument("--hz", type=float, default=FUSE_HZ)
    parser.add_argument("--lidar-max-range", type=float, default=LIDAR_MAX_RANGE)
    parser.add_argument("--cam-min-range",   type=float, default=CAM_MIN_RANGE)
    parser.add_argument("--kf-max-jump", type=float, default=0.8,
                        help="Final Kalman measurement gate in metres (default 0.8)")
    parser.add_argument("--status-hz", type=float, default=4.0,
                        help="Terminal status refresh rate; <=0 disables status prints")
    args = parser.parse_args()

    LIDAR_MAX_RANGE = args.lidar_max_range
    CAM_MIN_RANGE   = args.cam_min_range

    lidar_sub = BallStateSubscriber(topic_name=args.lidar_topic)
    cam_sub   = BallStateSubscriber(topic_name=args.cam_topic)
    out_pub   = BallStatePublisher(topic_name=args.output_topic)

    lidar_sub.start()
    cam_sub.start()
    kf = CenterKalmanFilter()
    kf.max_jump = args.kf_max_jump

    print(f"[fuser] Subscribing: {args.lidar_topic}  +  {args.cam_topic}")
    print(f"[fuser] Publishing:  {args.output_topic}  @ {args.hz:.0f} Hz")
    print(f"[fuser] Ranges: lidar ≤ {LIDAR_MAX_RANGE}m | cam ≥ {CAM_MIN_RANGE}m")

    dt = 1.0 / args.hz
    last_kf_s = time.monotonic()
    last_status_s = 0.0
    status_period_s = 1.0 / args.status_hz if args.status_hz > 0 else None
    try:
        while True:
            t0 = time.monotonic()

            lidar = lidar_sub.latest()
            cam   = cam_sub.latest()
            obs = choose_observation(lidar, cam)
            now_s = time.monotonic()
            kf_dt = now_s - last_kf_s
            last_kf_s = now_s

            if obs.valid == 1:
                pos = kf.step(np.array([obs.x, obs.y, obs.z], dtype=np.float32), kf_dt)
                result = BallState(
                    timestamp_us=obs.timestamp_us,
                    x=float(pos[0]), y=float(pos[1]), z=float(pos[2]),
                    valid=1, source=obs.source,
                )
            elif kf.initialized:
                pos = kf.predict_only(kf_dt)
                result = BallState(
                    timestamp_us=int(time.time() * 1e6),
                    x=float(pos[0]), y=float(pos[1]), z=float(pos[2]),
                    valid=0, source=SOURCE_NONE,
                )
            else:
                result = BallState(valid=0, source=SOURCE_NONE)

            out_pub.publish(
                result.x, result.y, result.z,
                valid=result.valid == 1,
                source=result.source,
            )

            if status_period_s is not None and (time.monotonic() - last_status_s) >= status_period_s:
                last_status_s = time.monotonic()
                src_str = {SOURCE_LIDAR: "lidar", SOURCE_CAM: "cam", SOURCE_NONE: "none"}.get(
                    result.source, "?"
                )
                obs_str = {SOURCE_LIDAR: "lidar", SOURCE_CAM: "cam", SOURCE_NONE: "none"}.get(
                    obs.source, "?"
                )
                v_str = "valid" if result.valid else "pred " if kf.initialized else "inval"
                print(
                    f"\r[fuser] src={src_str:<5} obs={obs_str:<5} {v_str}  "
                    f"xyz=({result.x:+.2f},{result.y:+.2f},{result.z:+.2f})"
                    f"  lidar={'R' if lidar.valid and lidar.source==SOURCE_LIDAR else '-'}"
                    f"  cam={'V' if cam.valid and cam.source==SOURCE_CAM else '-'}",
                    end="", flush=True,
                )

            elapsed = time.monotonic() - t0
            sleep_t = dt - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        print("\n[fuser] Stopped.")


if __name__ == "__main__":
    main()
