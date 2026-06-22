#!/usr/bin/env python3
"""Export a freekick log replay to MP4 using MuJoCo offscreen rendering.

Usage:
    conda run -n robomimic python tools/export_video.py <log.bin> [options]

Examples:
    # Quick preview (first 200 frames, 640×360)
    conda run -n robomimic python tools/export_video.py logs/20260324_155110_freekick.bin \
      --width 640 --height 360 --frames 200

    # Full export at 1280×720
    conda run -n robomimic python tools/export_video.py 20260428_224118_freekick.bin \
      --width 1280 --height 720
logs/20260428_224118_freekick.bin
Options:
    --xml path/to/robot.xml      Override XML (default: from log meta or g1_liao.xml)
    --output out.mp4             Output path (default: <log_stem>.mp4 next to log)
    --fps 50                     Output FPS (default: 1 / control_dt)
    --width 1280                 Frame width  (default 640)
    --height 720                 Frame height (default 360)
    --cam-pos X Y Z              Camera position in world frame (default: -1.5 0 1.2)
    --cam-dir DX DY DZ           Camera look direction (default: 1 0 -0.5)
    --cam-dist D                 Distance from cam to lookat point (default 3.0)
    --speed S                    Playback speed multiplier, affects output FPS (default 1.0)
    --frames N                   Only render first N frames (quick preview)
    --start N                    Start from frame N (default 0)

Coordinate system (MuJoCo world frame):
    x = robot forward,  y = robot left,  z = up
    Ground plane is z = 0.  Robot pelvis is near z = 0 in real-robot logs.

Ball markers (ball_pos_b transformed to world):
    Red    — ball_valid
    Orange — not valid but ball_pos_b ≠ 0 (e.g. coast)
    Blue   — sim ground truth (ball_pos_w) when present
Target markers:
    Magenta — target actually used by FreeKick (debug_target_pos_b → world)
    Purple  — raw target sensor/state (target_pos_b → world) when it differs
"""

import sys
import os
import argparse
import copy
import numpy as np
import cv2

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

import mujoco

from common.logger import Logger
from common.path_config import PROJECT_ROOT


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _quat_to_matrix(q):
    """[w, x, y, z] → 3×3 rotation matrix."""
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z),  2*(x*y - w*z),  2*(x*z + w*y)],
        [2*(x*y + w*z),  1 - 2*(x*x + z*z),  2*(y*z - w*x)],
        [2*(x*z - w*y),      2*(y*z + w*x),  1 - 2*(x*x + y*y)],
    ], dtype=np.float64)


def _add_sphere(scn, pos, radius: float, rgba) -> None:
    if scn.ngeom >= scn.maxgeom:
        return
    g = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(
        g,
        mujoco.mjtGeom.mjGEOM_SPHERE,
        np.array([radius, radius, radius], dtype=np.float64),
        np.asarray(pos,   dtype=np.float64),
        np.eye(3, dtype=np.float64).flatten(),
        np.asarray(rgba,  dtype=np.float32),
    )
    scn.ngeom += 1


def _field_or_default(data: dict, name: str, shape, dtype=np.float32):
    if name in data:
        return data[name]
    return np.zeros(shape, dtype=dtype)


def make_camera(pos, direction, distance: float = 3.0) -> mujoco.MjvCamera:
    """Build a MjvCamera placed at `pos` looking in `direction`.

    MuJoCo free-cam forward vector:
        fwd = [-cos(el)*sin(az),  cos(el)*cos(az),  -sin(el)]

    Solving for az, el given desired unit forward direction d:
        el  = -arcsin(d.z)
        az  = atan2(-d.x, d.y)
        lookat = pos + dist * d_norm
    """
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE

    d = np.array(direction, dtype=np.float64)
    d /= np.linalg.norm(d)

    cam.elevation = float(np.degrees(np.arcsin(float(np.clip(d[2], -1.0, 1.0)))))
    cam.azimuth   = float(np.degrees(np.arctan2(-d[0], d[1])))
    cam.distance  = float(distance)
    cam.lookat    = np.array(pos, dtype=np.float64) + distance * d

    return cam


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="Export freekick log to MP4.")
    parser.add_argument("log",   help="Path to .bin log file")
    parser.add_argument("--xml",      default=None)
    parser.add_argument("--output",   default=None)
    parser.add_argument("--fps",      type=float, default=None,
                        help="Output FPS (default: 1/control_dt)")
    parser.add_argument("--width",    type=int,   default=1280)
    parser.add_argument("--height",   type=int,   default=720)
    # on the left side of the robot
    # parser.add_argument("--cam-pos",  nargs=3, type=float,
    #                     default=[-1.5, 1.0, 1.2], metavar=("X", "Y", "Z"))
    # parser.add_argument("--cam-dir",  nargs=3, type=float,
    #                     default=[1.0, 0.0, -0.5],  metavar=("DX", "DY", "DZ"))
    # on the rght side of the robot
    parser.add_argument("--cam-pos",  nargs=3, type=float,
                        default=[4.0, -1.0, 1.0], metavar=("X", "Y", "Z"),
                        help="Camera position in world frame (x=fwd, y=left, z=up)")
    parser.add_argument("--cam-dir",  nargs=3, type=float,
                        default=[-1.0, 0.0, -0.4],  metavar=("DX", "DY", "DZ"),
                        help="Camera look direction in world frame")

    parser.add_argument("--cam-dist", type=float, default=3.0,
                        help="Distance from camera to lookat (default 3.0)")
    parser.add_argument("--speed",    type=float, default=1.0,
                        help="Playback speed multiplier (default 1.0 = realtime)")
    parser.add_argument("--frames",   type=int,   default=None,
                        help="Only render first N frames (quick preview)")
    parser.add_argument("--start",    type=int,   default=0,
                        help="Start from frame N (default 0)")
    args = parser.parse_args()

    # ---- Load log ----------------------------------------------------------
    print(f"Loading {args.log} …")
    data = Logger.load(args.log)
    meta = data["_meta"]
    T    = data["q"].shape[0]
    dt   = float(meta.get("control_dt", 0.02))
    fps  = (args.fps if args.fps else 1.0 / dt) * args.speed

    # Apply --start / --frames clipping
    frame_start = max(0, args.start)
    frame_end   = min(T, frame_start + args.frames) if args.frames else T
    frame_ids   = range(frame_start, frame_end)
    N           = len(frame_ids)

    print(f"  total {T} frames · {T*dt:.1f}s · {dt*1000:.0f}ms/frame · {fps:.1f}fps out")
    if frame_start > 0 or frame_end < T:
        print(f"  rendering frames [{frame_start}, {frame_end}) = {N} frames  "
              f"({frame_start*dt:.1f}s – {frame_end*dt:.1f}s)")

    has_ball_gt = bool(np.any(np.abs(data["ball_pos_w"]) > 1e-6))
    target_pos_b_log = _field_or_default(data, "target_pos_b", (T, 3))
    target_valid_log = _field_or_default(data, "target_valid", (T,))
    debug_target_pos_b_log = _field_or_default(data, "debug_target_pos_b", (T, 3))

    # ---- Resolve XML -------------------------------------------------------
    xml_path = args.xml or meta.get("xml_path") or os.path.join(
        PROJECT_ROOT, "g1_description", "g1_liao.xml")
    if not os.path.isabs(xml_path):
        xml_path = os.path.join(PROJECT_ROOT, xml_path)
    print(f"  XML: {xml_path}")

    # ---- Load MuJoCo model -------------------------------------------------
    m = mujoco.MjModel.from_xml_path(xml_path)

    ghost_m = copy.deepcopy(m)
    ghost_m.geom_rgba[:, :3] = [0.2, 0.9, 0.4]
    ghost_m.geom_rgba[:,  3] = 0.35
    ghost_d = mujoco.MjData(ghost_m)

    num_joints = m.nu

    # ---- Output path -------------------------------------------------------
    if args.output is None:
        stem   = os.path.splitext(os.path.basename(args.log))[0]
        suffix = f"_f{frame_start}-{frame_end}" if (args.frames or args.start) else ""
        args.output = os.path.join(os.path.dirname(args.log), stem + suffix + ".mp4")
    print(f"  Output: {args.output}")

    # ---- Camera ------------------------------------------------------------
    cam    = make_camera(args.cam_pos, args.cam_dir, args.cam_dist)
    d_norm = np.array(args.cam_dir) / np.linalg.norm(args.cam_dir)
    print(f"  Camera: pos={args.cam_pos}  dir={[round(v,3) for v in d_norm.tolist()]}"
          f"  dist={args.cam_dist}")
    print(f"          → azimuth={cam.azimuth:.1f}°  elevation={cam.elevation:.1f}°"
          f"  lookat={[round(v,3) for v in cam.lookat.tolist()]}")

    # ---- Expand offscreen framebuffer to match requested resolution ---------
    # MuJoCo's default offscreen buffer is 640×480; must be enlarged explicitly.
    ghost_m.vis.global_.offwidth  = args.width
    ghost_m.vis.global_.offheight = args.height

    # ---- MuJoCo offscreen renderer -----------------------------------------
    renderer  = mujoco.Renderer(ghost_m, height=args.height, width=args.width)
    scene_opt = mujoco.MjvOption()

    # ---- OpenCV video writer -----------------------------------------------
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(args.output, fourcc, fps, (args.width, args.height))
    if not writer.isOpened():
        sys.exit(f"Error: could not open VideoWriter for {args.output}")

    # ---- Render frames -----------------------------------------------------
    print(f"\nRendering {N} frames …")
    for i, fi in enumerate(frame_ids):
        # Apply ghost robot pose from log
        ghost_d.qpos[0:3] = data["pelvis_pos_w"][fi]
        ghost_d.qpos[3:7] = data["pelvis_quat_w"][fi]          # [w,x,y,z]
        ghost_d.qpos[7 : 7 + num_joints] = data["q"][fi]
        mujoco.mj_forward(ghost_m, ghost_d)

        # Render base scene
        renderer.update_scene(ghost_d, camera=cam, scene_option=scene_opt)

        # ball_pos_b → world: red if valid, orange if invalid but non-zero (coast)
        R_pelvis = _quat_to_matrix(data["pelvis_quat_w"][fi].astype(np.float64))
        ball_pos_b = data["ball_pos_b"][fi].astype(np.float64)
        ball_sensor_w = (data["pelvis_pos_w"][fi].astype(np.float64)
                         + R_pelvis @ ball_pos_b)
        ball_pos_norm = float(np.linalg.norm(ball_pos_b))
        if data["ball_valid"][fi] > 0.5:
            _add_sphere(renderer.scene, ball_sensor_w,
                        radius=0.11, rgba=[1.0, 0.2, 0.2, 0.85])
        elif ball_pos_norm > 1e-3:
            _add_sphere(renderer.scene, ball_sensor_w,
                        radius=0.11, rgba=[1.0, 0.55, 0.05, 0.82])

        # Blue sphere: ground-truth ball position (sim logs only)
        if has_ball_gt:
            _add_sphere(renderer.scene, data["ball_pos_w"][fi],
                        radius=0.11, rgba=[0.2, 0.4, 1.0, 0.6])

        # Magenta sphere: target actually used by FreeKick in pelvis body frame → world.
        debug_target_pos_b = debug_target_pos_b_log[fi].astype(np.float64)
        debug_target_norm = float(np.linalg.norm(debug_target_pos_b))
        if debug_target_norm > 1e-3:
            debug_target_w = (
                data["pelvis_pos_w"][fi].astype(np.float64) + R_pelvis @ debug_target_pos_b
            )
            _add_sphere(renderer.scene, debug_target_w,
                        radius=0.08, rgba=[1.0, 0.1, 0.9, 0.90])

        # Purple sphere: raw target state / sensor value from log, when available.
        target_pos_b = target_pos_b_log[fi].astype(np.float64)
        target_pos_norm = float(np.linalg.norm(target_pos_b))
        if target_valid_log[fi] > 0.5 and target_pos_norm > 1e-3:
            target_sensor_w = (
                data["pelvis_pos_w"][fi].astype(np.float64) + R_pelvis @ target_pos_b
            )
            _add_sphere(renderer.scene, target_sensor_w,
                        radius=0.05, rgba=[0.65, 0.2, 1.0, 0.70])

        # RGB → BGR for OpenCV
        pixels = renderer.render()
        writer.write(cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR))

        if i % 50 == 0 or i == N - 1:
            pct = (i + 1) / N
            bar = "█" * int(40 * pct) + "─" * (40 - int(40 * pct))
            print(f"\r  [{bar}] {pct:5.1%}  {i+1}/{N}  (log frame {fi})",
                  end="", flush=True)

    writer.release()
    renderer.close()
    print(f"\n\nDone → {args.output}")


if __name__ == "__main__":
    main()
