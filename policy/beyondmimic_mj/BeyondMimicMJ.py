import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent.parent.parent.absolute()))

import os
import numpy as np
import yaml
import onnxruntime

from FSM.FSMState import FSMState
from common.ctrlcomp import StateAndCmd, PolicyOutput
from common.utils import FSMStateName, FSMCommand, progress_bar

# ---------------------------------------------------------------------------
# Joint index mapping: none needed.
#
# This policy is trained with mjlab, which uses MuJoCo's standard DFS joint
# order.  That order is identical to the hardware order (Unitree SDK), so
# no reindexing is required between NPZ / policy / hardware.
#
# MuJoCo DFS joint order (== hardware order):
#   l_hip_p(0) l_hip_r(1) l_hip_y(2) l_knee(3) l_ank_p(4) l_ank_r(5)
#   r_hip_p(6) r_hip_r(7) r_hip_y(8) r_knee(9) r_ank_p(10) r_ank_r(11)
#   waist_y(12) waist_r(13) waist_p(14)
#   l_sho_p(15) l_sho_r(16) l_sho_y(17) l_elbow(18)
#   l_wrist_r(19) l_wrist_p(20) l_wrist_y(21)
#   r_sho_p(22) r_sho_r(23) r_sho_y(24) r_elbow(25)
#   r_wrist_r(26) r_wrist_p(27) r_wrist_y(28)
# ---------------------------------------------------------------------------

# Index of torso_link in the NPZ body list (30 bodies, DFS, no world body).
# DFS traversal: pelvis(0) → left_leg(1-6) → right_leg(7-12)
#   → waist_yaw(13) → waist_roll(14) → torso_link(15) → arms(16-29)
NPZ_ANCHOR_IDX = 15


def _resolve_model_path(current_dir: str, configured_path: str) -> str:
    """Resolve asset paths from config.

    Relative paths are loaded from `policy/beyondmimic_mj/model`, while absolute
    paths are used directly so local experiments can point at exported mjlab
    artifacts without copying them into this repo.
    """
    if os.path.isabs(configured_path):
        return configured_path
    return os.path.join(current_dir, "model", configured_path)


# ---------------------------------------------------------------------------
# Math helpers  (quaternion convention: [w, x, y, z])
# ---------------------------------------------------------------------------

def _yaw_quat(q):
    """Return the yaw-only quaternion extracted from q=[w,x,y,z]."""
    w, x, y, z = q
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y**2 + z**2))
    return np.array([np.cos(yaw * 0.5), 0.0, 0.0, np.sin(yaw * 0.5)],
                    dtype=np.float64)


def _quat_to_matrix(q):
    """[w,x,y,z] -> 3x3 rotation matrix."""
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z),  2*(x*y - w*z),  2*(x*z + w*y)],
        [2*(x*y + w*z),  1 - 2*(x*x + z*z),  2*(y*z - w*x)],
        [2*(x*z - w*y),      2*(y*z + w*x),  1 - 2*(x*x + y*y)],
    ], dtype=np.float64)


def _matrix_to_quat(R):
    """3x3 rotation matrix -> [w,x,y,z]."""
    R = np.asarray(R, dtype=np.float64)
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z], dtype=np.float64)
    return q / np.linalg.norm(q)


def _quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)


def _quat_mul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ], dtype=np.float64)


def _rot6d_from_quat(q):
    """First two columns of rotation matrix, flattened row-major -> (6,).
    Matches mjlab matrix_from_quat()[..., :2].reshape(N, -1)."""
    R = _quat_to_matrix(q)
    return R[:, :2].flatten().astype(np.float32)


# ---------------------------------------------------------------------------
# Policy
# ---------------------------------------------------------------------------

class BeyondMimicMJ(FSMState):
    def __init__(self, state_cmd: StateAndCmd, policy_output: PolicyOutput,
                 config_file: str = "beyondmimic_mj.yaml",
                 state_name: FSMStateName = FSMStateName.SKILL_BEYONDMIMIC_MJ):
        super().__init__()
        self.state_cmd    = state_cmd
        self.policy_output = policy_output
        self.name     = state_name
        self.name_str = "skill_" + config_file.replace(".yaml", "")

        current_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(current_dir, "config", config_file)
        with open(config_path, "r") as f:
            cfg = yaml.load(f, Loader=yaml.FullLoader)

        onnx_path        = _resolve_model_path(current_dir, cfg["onnx_path"])
        motion_path      = _resolve_model_path(current_dir, cfg["motion_path"])
        self.control_dt  = float(cfg["control_dt"])   # needed before motion slicing

        # ---- Load motion from NPZ (all arrays in MuJoCo DFS order) ----
        motion = np.load(motion_path)
        joint_pos_full = motion["joint_pos"]   # [T, 29]  MuJoCo order
        joint_vel_full = motion["joint_vel"]   # [T, 29]  MuJoCo order
        body_quat_full = motion["body_quat_w"] # [T, 30, 4]  [w,x,y,z]
        body_pos_full  = motion["body_pos_w"] if "body_pos_w" in motion else None  # [T, 30, 3]
        T_full = joint_pos_full.shape[0]

        # Optional time window: motion_start_s / motion_end_s in seconds
        start_s = float(cfg.get("motion_start_s", 0.0))
        end_s   = cfg.get("motion_end_s", None)
        i0 = int(round(start_s / self.control_dt))
        i1 = int(round(float(end_s) / self.control_dt)) if end_s is not None else T_full
        i0 = max(0, min(i0, T_full))
        i1 = max(i0 + 1, min(i1, T_full))
        self.motion_joint_pos = joint_pos_full[i0:i1]
        self.motion_joint_vel = joint_vel_full[i0:i1]
        self.motion_body_quat = body_quat_full[i0:i1]
        self.motion_body_pos  = body_pos_full[i0:i1] if body_pos_full is not None else None
        self.motion_total_steps = self.motion_joint_pos.shape[0]
        print(f"BeyondMimicMJ motion window: {i0 * self.control_dt:.2f}s ~ "
              f"{i1 * self.control_dt:.2f}s  ({self.motion_total_steps} frames)")

        # ---- Config (all arrays in MuJoCo order, directly from ONNX metadata) ----
        self.kps          = np.array(cfg["kps"],               dtype=np.float32)
        self.kds          = np.array(cfg["kds"],               dtype=np.float32)
        self.tau_limit    = np.array(cfg["tau_limit"],         dtype=np.float32)
        self.default_q    = np.array(cfg["default_joint_pos"], dtype=np.float32)
        self.action_scale = np.array(cfg["action_scale"],      dtype=np.float32)
        self.clip_actions = float(cfg.get("clip_actions", 3.0))
        # self.control_dt already set above (needed for motion slicing)
        self.WARMUP_STEPS = int(cfg.get("warmup_steps", 30))

        # ---- ONNX session ----
        self.ort_session = onnxruntime.InferenceSession(onnx_path)

        # ---- Running state ----
        self.time_step      = 0
        self.last_action    = np.zeros(29, dtype=np.float32)  # MuJoCo order
        self._init_to_world = np.eye(3, dtype=np.float64)
        self._entry_q       = self.default_q.copy()
        self._t0_target_q   = self.default_q.copy()

        # Warm-up ONNX
        _dummy_obs = np.zeros((1, 154), dtype=np.float32)
        _dummy_ts  = np.zeros((1, 1),   dtype=np.float32)
        for _ in range(5):
            self.ort_session.run(["actions"], {"obs": _dummy_obs, "time_step": _dummy_ts})

        print("BeyondMimicMJ policy initialized "
              f"(154-dim, {self.motion_total_steps} motion frames).")

    # ------------------------------------------------------------------

    def enter(self):
        self.time_step   = 0
        self.last_action = np.zeros(29, dtype=np.float32)

        # ---- Yaw alignment (init_to_world) ----
        # Rotate motion reference so that its initial facing direction aligns
        # with the robot's current yaw.
        motion_t0_quat = self.motion_body_quat[0, NPZ_ANCHOR_IDX].astype(np.float64)
        robot_quat     = self.state_cmd.torso_quat_w.astype(np.float64)
        # Compute yaw alignment from the RELATIVE quaternion q_robot ⊗ q_motion0⁻¹.
        # This avoids ZYX singularity when the robot is lying down: if both have
        # the same tilt (e.g. pitch=-90°), q_rel is a pure Z-rotation regardless
        # of the individual yaw-extraction instability.
        q_rel = _quat_mul(robot_quat, _quat_conj(motion_t0_quat))
        self._init_to_world = _quat_to_matrix(_yaw_quat(q_rel))

        # ---- Warm-up: interpolate current pose -> motion t=0 pose ----
        # motion_joint_pos is already in MuJoCo order.
        self._entry_q     = self.state_cmd.q.copy()
        self._t0_target_q = self.motion_joint_pos[0].copy()

        max_delta = np.abs(self._t0_target_q - self._entry_q).max()
        print(f"BeyondMimicMJ enter: warmup {self.WARMUP_STEPS} steps, "
              f"max joint delta = {max_delta:.3f} rad")

    # ------------------------------------------------------------------

    def _build_obs(self) -> np.ndarray:
        """154-dim obs (MuJoCo DFS order throughout):
        [ref_jpos(29) | ref_jvel(29) | anchor_ori_b(6) |
         base_ang_vel(3) | joint_pos_rel(29) | joint_vel(29) | last_action(29)]
        """
        t = min(self.time_step - self.WARMUP_STEPS, self.motion_total_steps - 1)

        # ---- Motion reference (MuJoCo order, directly from NPZ) ----
        ref_jpos = self.motion_joint_pos[t]   # (29,)
        ref_jvel = self.motion_joint_vel[t]   # (29,)

        # ---- Anchor orientation with yaw alignment ----
        ref_anchor_quat  = self.motion_body_quat[t, NPZ_ANCHOR_IDX].astype(np.float64)
        robot_torso_quat = self.state_cmd.torso_quat_w.astype(np.float64)
        init_world_quat  = _matrix_to_quat(self._init_to_world)
        aligned_quat     = _quat_mul(init_world_quat, ref_anchor_quat)
        rel_quat = _quat_mul(_quat_conj(robot_torso_quat), aligned_quat)
        rel_quat = rel_quat / np.linalg.norm(rel_quat)
        anchor_ori_6d = _rot6d_from_quat(rel_quat)   # (6,)

        # ---- Joint state: already in MuJoCo order from hardware ----
        joint_pos_rel = self.state_cmd.q - self.default_q   # (29,)
        joint_vel     = self.state_cmd.dq                   # (29,)

        # ---- Angular velocity (body frame) ----
        ang_vel = self.state_cmd.root_ang_vel_b   # (3,)

        obs = np.concatenate([
            ref_jpos,          # command: ref joint_pos   (29)
            ref_jvel,          # command: ref joint_vel   (29)
            anchor_ori_6d,     # motion_anchor_ori_b       (6)
            ang_vel,           # base_ang_vel              (3)
            joint_pos_rel,     # joint_pos                (29)
            joint_vel,         # joint_vel                (29)
            self.last_action,  # last actions             (29)
        ], dtype=np.float32)
        return obs   # shape (154,)

    # ------------------------------------------------------------------

    def run(self):
        # ---- Warm-up: interpolate from entry pose to motion t=0 pose ----
        if self.time_step < self.WARMUP_STEPS:
            alpha    = (self.time_step + 1) / self.WARMUP_STEPS
            target_q = (1.0 - alpha) * self._entry_q + alpha * self._t0_target_q
            self.policy_output.actions = target_q
            self.policy_output.kps     = self.kps
            self.policy_output.kds     = self.kds
            self.time_step += 1
            return

        # ---- Policy phase ----
        policy_step = self.time_step - self.WARMUP_STEPS

        obs = self._build_obs()
        ts  = np.array([[policy_step]], dtype=np.float32)

        out = self.ort_session.run(
            ["actions"],
            {"obs": obs[None, :], "time_step": ts},
        )
        # Actions are in MuJoCo order — no reindexing needed.
        actions = out[0].squeeze(0)   # (29,) MuJoCo order
        actions = np.clip(actions, -self.clip_actions, self.clip_actions)
        self.last_action = actions.copy()

        target_q = self.default_q + self.action_scale * actions

        # Debug: print for first 3 policy steps
        if policy_step < 3:
            print(f"\n[BeyondMimicMJ policy_step={policy_step}]")
            print(f"  anchor_ori_6d : {obs[58:64]}")
            print(f"  ang_vel       : {obs[64:67]}")
            print(f"  joint_pos_rel : min={obs[67:96].min():.3f}  max={obs[67:96].max():.3f}")
            print(f"  actions       : min={actions.min():.3f}  max={actions.max():.3f}")
            print(f"  delta_q       : min={(target_q - self.state_cmd.q).min():.3f}"
                  f"  max={(target_q - self.state_cmd.q).max():.3f}")

        self.policy_output.actions = target_q
        self.policy_output.kps     = self.kps
        self.policy_output.kds     = self.kds

        self.time_step += 1
        capped = min(policy_step, self.motion_total_steps - 1)
        self.policy_output.ghost_qpos = self._compute_ghost_qpos(capped)
        print(progress_bar(capped * self.control_dt,
                           self.motion_total_steps * self.control_dt),
              end="", flush=True)

    # ------------------------------------------------------------------

    def _compute_ghost_qpos(self, t: int):
        """Compute ghost robot qpos for reference motion visualization.

        Transforms the reference motion into the robot's local frame following
        the same approach as mjlab's MotionCommand._update_command():

          ghost_root_pos  = [robot_anchor.xy, ref_anchor.z]
                          + R_init @ (ref_root_pos - ref_anchor_pos)
          ghost_root_quat = init_world_quat ⊗ ref_root_quat
          ghost_joints    = ref_joint_pos  (MuJoCo order)

        Returns None if body_pos_w is not available in the NPZ.
        """
        if self.motion_body_pos is None:
            return None

        ref_anchor_pos = self.motion_body_pos[t, NPZ_ANCHOR_IDX].astype(np.float64)
        ref_root_pos   = self.motion_body_pos[t, 0].astype(np.float64)
        ref_root_quat  = self.motion_body_quat[t, 0].astype(np.float64)

        # Yaw-align the anchor and root positions (same R_init as used for anchor_ori_6d).
        aligned_anchor_pos = self._init_to_world @ ref_anchor_pos

        # XY from robot anchor, Z from yaw-aligned reference anchor (mirrors mjlab).
        torso_pos = self.state_cmd.torso_pos_w.astype(np.float64)
        delta_pos = np.array([torso_pos[0], torso_pos[1], aligned_anchor_pos[2]])

        ghost_root_pos  = delta_pos + self._init_to_world @ (ref_root_pos - ref_anchor_pos)
        ghost_root_quat = _quat_mul(_matrix_to_quat(self._init_to_world), ref_root_quat)

        qpos = np.empty(7 + 29, dtype=np.float32)
        qpos[0:3] = ghost_root_pos
        qpos[3:7] = ghost_root_quat          # [w, x, y, z] — MuJoCo free-joint order
        qpos[7:]  = self.motion_joint_pos[t] # MuJoCo order directly
        return qpos

    # ------------------------------------------------------------------

    def exit(self):
        self.time_step   = 0
        self.last_action = np.zeros(29, dtype=np.float32)
        self.policy_output.ghost_qpos = None
        print()

    def checkChange(self):
        cmd = self.state_cmd.skill_cmd
        if cmd == FSMCommand.LOCO:
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.LOCOMODE
        elif cmd == FSMCommand.PASSIVE:
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.PASSIVE
        elif cmd == FSMCommand.POS_RESET:
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.FIXEDPOSE
        elif cmd == FSMCommand.CMD_BEYONDMIMIC_MJ:
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.SKILL_BEYONDMIMIC_MJ
        elif cmd == FSMCommand.CMD_STANDUP_MJ:
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.SKILL_STANDUP_MJ
        elif cmd == FSMCommand.CMD_PINOCCHIO_1_6_MJ:
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.SKILL_PINOCCHIO_1_6_MJ
        elif cmd == FSMCommand.CMD_AMP:
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.SKILL_AMP
        elif cmd == FSMCommand.CMD_FREEKICK:
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.SKILL_FREEKICK
        else:
            return self.name
