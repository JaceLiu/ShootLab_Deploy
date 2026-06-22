#!/usr/bin/env python3
"""Replay a freekick log in MuJoCo with robot pose and ball visualization.

Usage:
    python tools/playback_log.py <log.bin> [--xml path/to/robot.xml] [--speed 1.0]

Controls:
    Space       pause / resume
    →  / ←      step +1 / -1 frame  (when paused)
    ]  / [      jump +10 / -10 frames
    f  / s      faster (×2) / slower (×0.5)
    r           rewind to frame 0
    q           quit

Visualization:
    Robot          joint pose from log
    Red sphere     ball_pos_b → world when ball_valid (trusted detection)
    Orange sphere  same transform when not valid but ball_pos_b ≠ 0 (e.g. coast)
    Blue sphere    ball ground-truth world position  (ball_pos_w, sim only)
"""

import sys
import os
import argparse
import time
import copy
import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

import mujoco
import mujoco.viewer

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
    """Inject a sphere into a MjvScene's user geom list."""
    if scn.ngeom >= scn.maxgeom:
        return
    g = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(
        g,
        mujoco.mjtGeom.mjGEOM_SPHERE,
        np.array([radius, radius, radius], dtype=np.float64),
        np.asarray(pos,  dtype=np.float64),
        np.eye(3, dtype=np.float64).flatten(),
        np.asarray(rgba, dtype=np.float32),
    )
    scn.ngeom += 1


def _progress(frame: int, total: int, speed: float, paused: bool,
              width: int = 50) -> str:
    pct  = frame / max(total - 1, 1)
    fill = int(width * pct)
    bar  = "█" * fill + "─" * (width - fill)
    tag  = "PAUSED" if paused else f"{speed:.1f}×"
    return f"\r[{bar}] {pct:5.1%}  frame {frame:5d}/{total-1}  {tag}   "


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="Replay a freekick log in MuJoCo.")
    parser.add_argument("log",   help="Path to .bin log file")
    parser.add_argument("--xml", default=None,
                        help="Robot XML path (overrides meta; default: g1_liao.xml)")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="Initial playback speed (default 1.0)")
    args = parser.parse_args()

    # ---- Load log ----------------------------------------------------------
    print(f"Loading {args.log} …")
    data = Logger.load(args.log)
    meta = data["_meta"]
    T    = data["q"].shape[0]
    dt   = float(meta.get("control_dt", 0.02))
    print(f"  {T} frames  ·  {T * dt:.1f} s  ·  {dt*1000:.0f} ms/frame")

    # Check whether ground-truth ball data is available (sim logs only).
    has_ball_gt = bool(np.any(np.abs(data["ball_pos_w"]) > 1e-6))

    # ---- Resolve XML -------------------------------------------------------
    xml_path = args.xml
    if xml_path is None:
        xml_path = meta.get("xml_path", None)
    if xml_path is None:
        xml_path = os.path.join(PROJECT_ROOT, "g1_description", "g1_liao.xml")
    if not os.path.isabs(xml_path):
        xml_path = os.path.join(PROJECT_ROOT, xml_path)
    print(f"  XML: {xml_path}")

    # ---- Load MuJoCo model -------------------------------------------------
    m = mujoco.MjModel.from_xml_path(xml_path)
    d = mujoco.MjData(m)

    # Ghost model: translucent green, used to show robot pose from log.
    ghost_m = copy.deepcopy(m)
    ghost_m.geom_rgba[:, :3] = [0.2, 0.9, 0.4]
    ghost_m.geom_rgba[:,  3] = 0.35
    ghost_d = mujoco.MjData(ghost_m)

    num_joints = m.nu

    # ---- Shared playback state (mutated inside key_callback) ---------------
    state = {
        "frame":  0,
        "speed":  args.speed,
        "paused": False,
        "quit":   False,
    }

    # GLFW key codes
    KEY_SPACE = 32
    KEY_RIGHT = 262
    KEY_LEFT  = 263
    KEY_RBKT  = 93   # ]
    KEY_LBKT  = 91   # [
    KEY_F     = 70
    KEY_S     = 83
    KEY_R     = 82
    KEY_Q     = 81

    def key_callback(keycode: int) -> None:
        f = state["frame"]
        if keycode == KEY_SPACE:
            state["paused"] = not state["paused"]
        elif keycode == KEY_RIGHT:
            state["frame"]  = min(f + 1, T - 1)
        elif keycode == KEY_LEFT:
            state["frame"]  = max(f - 1, 0)
        elif keycode == KEY_RBKT:
            state["frame"]  = min(f + 10, T - 1)
        elif keycode == KEY_LBKT:
            state["frame"]  = max(f - 10, 0)
        elif keycode == KEY_F:
            state["speed"]  = min(state["speed"] * 2.0, 16.0)
        elif keycode == KEY_S:
            state["speed"]  = max(state["speed"] * 0.5, 0.125)
        elif keycode == KEY_R:
            state["frame"]  = 0
            state["paused"] = True
        elif keycode == KEY_Q:
            state["quit"]   = True

    # ---- Playback loop -----------------------------------------------------
    log_time   = 0.0          # current log-time position (seconds)
    prev_wall  = time.time()

    with mujoco.viewer.launch_passive(m, d, key_callback=key_callback) as viewer:
        print("\nPlayback started.  Space=pause  ←/→=step  [/]=±10  f/s=speed  r=rewind  q=quit\n")

        while viewer.is_running() and not state["quit"]:
            now         = time.time()
            wall_dt     = now - prev_wall
            prev_wall   = now

            # Advance log-time based on wall-clock elapsed and speed.
            if not state["paused"]:
                log_time += wall_dt * state["speed"]
                log_time  = min(log_time, (T - 1) * dt)
                state["frame"] = int(log_time / dt)
                if state["frame"] >= T - 1:
                    state["paused"] = True
                    print("\n[Playback] reached end — press Space to replay from here or R to rewind")
            else:
                # When paused, key_callback may have changed frame; sync log_time.
                log_time = state["frame"] * dt

            fi = state["frame"]

            # ---- Set ghost robot pose --------------------------------------
            ghost_d.qpos[0:3] = data["pelvis_pos_w"][fi]
            ghost_d.qpos[3:7] = data["pelvis_quat_w"][fi]   # [w,x,y,z]
            ghost_d.qpos[7 : 7 + num_joints] = data["q"][fi]
            mujoco.mj_forward(ghost_m, ghost_d)

            # ---- Compute ball-sensor position in world frame ---------------
            # ball_pos_b is in pelvis body frame → transform to world.
            R_pelvis = _quat_to_matrix(data["pelvis_quat_w"][fi].astype(np.float64))
            ball_pos_b = data["ball_pos_b"][fi].astype(np.float64)
            ball_sensor_world = (data["pelvis_pos_w"][fi].astype(np.float64)
                                 + R_pelvis @ ball_pos_b)
            ball_valid = data["ball_valid"][fi] > 0.5
            ball_pos_norm = float(np.linalg.norm(ball_pos_b))

            # ---- Build scene -----------------------------------------------
            with viewer.lock():
                viewer.user_scn.ngeom = 0

                # Ghost robot
                mujoco.mjv_addGeoms(
                    ghost_m, ghost_d,
                    mujoco.MjvOption(), mujoco.MjvPerturb(),
                    mujoco.mjtCatBit.mjCAT_DYNAMIC.value,
                    viewer.user_scn,
                )

                # ball_pos_b → world: red if valid, orange if invalid but non-zero (coast / stale)
                if ball_valid:
                    _add_sphere(viewer.user_scn, ball_sensor_world,
                                radius=0.11, rgba=[1.0, 0.2, 0.2, 0.85])
                elif ball_pos_norm > 1e-3:
                    _add_sphere(viewer.user_scn, ball_sensor_world,
                                radius=0.11, rgba=[1.0, 0.55, 0.05, 0.82])

                # Blue sphere: ground-truth ball position (sim logs only)
                if has_ball_gt:
                    _add_sphere(viewer.user_scn, data["ball_pos_w"][fi],
                                radius=0.11, rgba=[0.2, 0.4, 1.0, 0.6])

            viewer.sync()

            # ---- Terminal progress -----------------------------------------
            print(_progress(fi, T, state["speed"], state["paused"]),
                  end="", flush=True)

            # ---- Throttle to target display rate ---------------------------
            render_time = time.time() - now
            sleep = max(0.0, dt - render_time)
            time.sleep(sleep)

    print()  # newline after progress bar


if __name__ == "__main__":
    main()
