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
# Joint index mapping: MuJoCo order <-> Isaac Lab order
#
# MuJoCo:    l_hip_p(0) l_hip_r(1) l_hip_y(2) l_knee(3) l_ank_p(4) l_ank_r(5)
#            r_hip_p(6) r_hip_r(7) r_hip_y(8) r_knee(9) r_ank_p(10) r_ank_r(11)
#            waist_y(12) waist_r(13) waist_p(14)
#            l_sho_p(15) l_sho_r(16) l_sho_y(17) l_elbow(18)
#            l_wrist_r(19) l_wrist_p(20) l_wrist_y(21)
#            r_sho_p(22) r_sho_r(23) r_sho_y(24) r_elbow(25)
#            r_wrist_r(26) r_wrist_p(27) r_wrist_y(28)
#
# Isaac Lab: l_hip_p(0) r_hip_p(1) waist_y(2) l_hip_r(3) r_hip_r(4) waist_r(5)
#            l_hip_y(6) r_hip_y(7) waist_p(8) l_knee(9) r_knee(10)
#            l_sho_p(11) r_sho_p(12) l_ank_p(13) r_ank_p(14)
#            l_sho_r(15) r_sho_r(16) l_ank_r(17) r_ank_r(18)
#            l_sho_y(19) r_sho_y(20) l_elbow(21) r_elbow(22)
#            l_wrist_r(23) r_wrist_r(24) l_wrist_p(25) r_wrist_p(26)
#            l_wrist_y(27) r_wrist_y(28)
#
# Usage:
#   isaac_array = mujoco_array[ISAAC_TO_MUJOCO]   (mujoco -> isaac)
#   mujoco_array = isaac_array[MUJOCO_TO_ISAAC]    (isaac  -> mujoco)
# ---------------------------------------------------------------------------
MUJOCO_TO_ISAAC = np.array([
    0,  3,  6,  9, 13, 17,   # left leg
    1,  4,  7, 10, 14, 18,   # right leg
    2,  5,  8,               # waist
   11, 15, 19, 21, 23, 25, 27,  # left arm
   12, 16, 20, 22, 24, 26, 28,  # right arm
], dtype=np.int64)

ISAAC_TO_MUJOCO = np.argsort(MUJOCO_TO_ISAAC).astype(np.int64)

# Index of torso_link in the full NPZ body list (30 bodies).
# Isaac Lab / PhysX numbers bodies in joint_names order (base=0, then each
# joint's child body follows its joint index).  waist_pitch_joint is the 9th
# joint (index 8), so its child torso_link is body index 9.
NPZ_ANCHOR_IDX = 9


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
    Matches Isaac Lab matrix_from_quat()[..., :2].reshape(N, -1)."""
    R = _quat_to_matrix(q)
    return R[:, :2].flatten().astype(np.float32)


# ---------------------------------------------------------------------------
# Policy
# ---------------------------------------------------------------------------

class BeyondMimic(FSMState):
    def __init__(self, state_cmd: StateAndCmd, policy_output: PolicyOutput):
        super().__init__()
        self.state_cmd    = state_cmd
        self.policy_output = policy_output
        self.name     = FSMStateName.SKILL_BEYONDMIMIC
        self.name_str = "skill_beyondmimic"

        current_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(current_dir, "config", "beyondmimic.yaml")
        with open(config_path, "r") as f:
            cfg = yaml.load(f, Loader=yaml.FullLoader)

        onnx_path   = os.path.join(current_dir, "model", cfg["onnx_path"])
        motion_path = os.path.join(current_dir, "model", cfg["motion_path"])

        # ---- Load motion from NPZ ----
        motion = np.load(motion_path)
        self.motion_joint_pos = motion["joint_pos"]   # [T, 29]  Isaac Lab order
        self.motion_joint_vel = motion["joint_vel"]   # [T, 29]  Isaac Lab order
        self.motion_body_quat = motion["body_quat_w"] # [T, 30, 4]  [w,x,y,z]
        self.motion_total_steps = self.motion_joint_pos.shape[0]

        # ---- Config ----
        self.kps             = np.array(cfg["kps"],               dtype=np.float32)
        self.kds             = np.array(cfg["kds"],               dtype=np.float32)
        self.tau_limit       = np.array(cfg["tau_limit"],         dtype=np.float32)
        self.default_q_mj    = np.array(cfg["default_joint_pos"], dtype=np.float32)  # MuJoCo order
        self.action_scale_mj = np.array(cfg["action_scale"],      dtype=np.float32)  # MuJoCo order
        self.clip_actions    = float(cfg.get("clip_actions", 3.0))
        self.control_dt      = float(cfg["control_dt"])
        self.WARMUP_STEPS    = int(cfg.get("warmup_steps", 30))

        # Default joint pos in Isaac Lab order (for joint_pos_rel obs)
        self.default_q_il = self.default_q_mj[ISAAC_TO_MUJOCO]

        # ---- ONNX session ----
        self.ort_session = onnxruntime.InferenceSession(onnx_path)

        # ---- Running state ----
        self.time_step      = 0
        self.last_action_il = np.zeros(29, dtype=np.float32)
        self._init_to_world = np.eye(3, dtype=np.float64)  # yaw alignment matrix
        self._entry_q       = self.default_q_mj.copy()
        self._t0_target_q   = self.default_q_mj.copy()

        # Warm-up ONNX
        _dummy_obs = np.zeros((1, 154), dtype=np.float32)
        _dummy_ts  = np.zeros((1, 1),   dtype=np.float32)
        for _ in range(5):
            self.ort_session.run(["actions"], {"obs": _dummy_obs, "time_step": _dummy_ts})

        print("BeyondMimic policy initialized "
              f"(154-dim, {self.motion_total_steps} motion frames).")

    # ------------------------------------------------------------------

    def enter(self):
        self.time_step      = 0
        self.last_action_il = np.zeros(29, dtype=np.float32)

        # ---- Yaw alignment (init_to_world) ----
        # Rotate motion reference so that its initial facing direction aligns
        # with the robot's current yaw.  Without this, anchor_ori_b has a
        # constant bias equal to the yaw difference, pushing the policy
        # permanently out of distribution.
        motion_t0_quat = self.motion_body_quat[0, NPZ_ANCHOR_IDX].astype(np.float64)
        robot_quat     = self.state_cmd.torso_quat_w.astype(np.float64)
        # Extract yaw from the RELATIVE quaternion to avoid ZYX singularity when
        # the robot is lying down.  q_rel = q_robot ⊗ q_motion0⁻¹ is a near-pure
        # Z-rotation even when both poses share a large tilt (e.g. pitch = ±90°).
        q_rel = _quat_mul(robot_quat, _quat_conj(motion_t0_quat))
        self._init_to_world = _quat_to_matrix(_yaw_quat(q_rel))

        # ---- Warm-up: interpolate current pose -> motion t=0 pose ----
        # motion_joint_pos is in Isaac Lab order; convert to MuJoCo order
        self._entry_q     = self.state_cmd.q.copy()
        self._t0_target_q = self.motion_joint_pos[0][MUJOCO_TO_ISAAC].copy()

        max_delta = np.abs(self._t0_target_q - self._entry_q).max()
        print(f"BeyondMimic enter: warmup {self.WARMUP_STEPS} steps, "
              f"max joint delta = {max_delta:.3f} rad")

    # ------------------------------------------------------------------

    def _build_obs(self) -> np.ndarray:
        """154-dim obs matching Tracking-Flat-G1-Wo-State-Estimation-v0:
        [ref_jpos(29) | ref_jvel(29) | anchor_ori_b(6) |
         base_ang_vel(3) | joint_pos_rel(29) | joint_vel(29) | last_action(29)]
        """
        t = min(self.time_step - self.WARMUP_STEPS, self.motion_total_steps - 1)

        # ---- Motion reference (Isaac Lab order, from NPZ) ----
        ref_jpos = self.motion_joint_pos[t]   # (29,)
        ref_jvel = self.motion_joint_vel[t]   # (29,)

        # ---- Anchor orientation with yaw alignment ----
        # 1. Get motion torso quaternion at t
        ref_anchor_quat  = self.motion_body_quat[t, NPZ_ANCHOR_IDX].astype(np.float64)
        # 2. Get robot torso quaternion
        robot_torso_quat = self.state_cmd.torso_quat_w.astype(np.float64)
        # 3. Rotate reference into robot's world frame (yaw alignment)
        init_world_quat  = _matrix_to_quat(self._init_to_world)
        aligned_quat     = _quat_mul(init_world_quat, ref_anchor_quat)
        # 4. Express in robot's body frame
        rel_quat = _quat_mul(_quat_conj(robot_torso_quat), aligned_quat)
        rel_quat = rel_quat / np.linalg.norm(rel_quat)
        anchor_ori_6d = _rot6d_from_quat(rel_quat)   # (6,)

        # ---- Joint state: MuJoCo -> Isaac Lab order ----
        qj_il  = self.state_cmd.q[ISAAC_TO_MUJOCO]
        dqj_il = self.state_cmd.dq[ISAAC_TO_MUJOCO]
        joint_pos_rel = qj_il - self.default_q_il   # (29,)

        # ---- Angular velocity (body frame, from deploy_mujoco) ----
        ang_vel = self.state_cmd.root_ang_vel_b   # (3,)

        obs = np.concatenate([
            ref_jpos,            # command: ref joint_pos   (29)
            ref_jvel,            # command: ref joint_vel   (29)
            anchor_ori_6d,       # motion_anchor_ori_b       (6)
            ang_vel,             # base_ang_vel              (3)
            joint_pos_rel,       # joint_pos                (29)
            dqj_il,              # joint_vel                (29)
            self.last_action_il, # last actions             (29)
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
        actions_il = out[0].squeeze(0)   # (29,) Isaac Lab order
        actions_il = np.clip(actions_il, -self.clip_actions, self.clip_actions)
        self.last_action_il = actions_il.copy()

        # Convert: Isaac Lab order -> MuJoCo order, then scale + offset
        actions_mj = actions_il[MUJOCO_TO_ISAAC]
        target_q   = self.default_q_mj + self.action_scale_mj * actions_mj

        # Debug: print for first 3 policy steps
        if policy_step < 3:
            print(f"\n[BeyondMimic policy_step={policy_step}]")
            print(f"  anchor_ori_6d : {obs[58:64]}")
            print(f"  ang_vel       : {obs[64:67]}")
            print(f"  joint_pos_rel : min={obs[67:96].min():.3f}  max={obs[96:125].max():.3f}")
            print(f"  actions_il    : min={actions_il.min():.3f}  max={actions_il.max():.3f}")
            print(f"  delta_q       : min={(target_q - self.state_cmd.q).min():.3f}"
                  f"  max={(target_q - self.state_cmd.q).max():.3f}")

        self.policy_output.actions = target_q
        self.policy_output.kps     = self.kps
        self.policy_output.kds     = self.kds

        self.time_step += 1
        capped = min(policy_step, self.motion_total_steps - 1)
        print(progress_bar(capped * self.control_dt,
                           self.motion_total_steps * self.control_dt),
              end="", flush=True)

    # ------------------------------------------------------------------

    def exit(self):
        self.time_step      = 0
        self.last_action_il = np.zeros(29, dtype=np.float32)
        print()

    def checkChange(self):
        cmd = self.state_cmd.skill_cmd
        if cmd == FSMCommand.LOCO:
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.LOCOMODE
        elif cmd == FSMCommand.CMD_AMP:
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.SKILL_AMP
        elif cmd == FSMCommand.CMD_FREEKICK:
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.SKILL_FREEKICK
        elif cmd == FSMCommand.CMD_BEYONDMIMIC_MJ:
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.SKILL_BEYONDMIMIC_MJ
        elif cmd == FSMCommand.CMD_STANDUP_MJ:
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.SKILL_STANDUP_MJ
        elif cmd == FSMCommand.CMD_PINOCCHIO_1_6_MJ:
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.SKILL_PINOCCHIO_1_6_MJ
        elif cmd == FSMCommand.PASSIVE:
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.PASSIVE
        elif cmd == FSMCommand.POS_RESET:
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.FIXEDPOSE
        else:
            return FSMStateName.SKILL_BEYONDMIMIC
