"""AMP locomotion policy.

Ported from wbc_fsm/src/FSM/State_Amp.cpp.

Observation: 96-dim per frame × 4 history frames = 384-dim ONNX input.
  [body_ang_vel(3) | proj_gravity(3) | vel_cmd(3) |
   dof_pos_rel(29) | dof_vel(29) | last_action(29)]

Action: raw(29) * 0.25 + default_dof_pos[motor_idx] → policy_output.actions
"""

import os
import numpy as np
import yaml
import onnxruntime as ort

from FSM.FSMState import FSMState, FSMStateName
from common.ctrlcomp import StateAndCmd, PolicyOutput, FSMCommand

class Amp(FSMState):
    def __init__(self, state_cmd: StateAndCmd, policy_output: PolicyOutput):
        super().__init__()
        self.state_cmd   = state_cmd
        self.policy_output = policy_output
        self.name        = FSMStateName.SKILL_AMP
        self.name_str    = "AMP"

        cfg_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "config", "amp.yaml")
        with open(cfg_path) as f:
            cfg = yaml.safe_load(f)

        # ── Gains and defaults ────────────────────────────────────────────
        self.kps              = np.array(cfg["kps"],             dtype=np.float32)
        self.kds              = np.array(cfg["kds"],             dtype=np.float32)
        self.default_dof_pos  = np.array(cfg["default_dof_pos"], dtype=np.float32)  # motor order
        self.joint2motor_idx  = np.array(cfg["joint2motor_idx"], dtype=np.int32)

        # ── Observation scaling ───────────────────────────────────────────
        self.ang_vel_scale  = float(cfg["ang_vel_scale"])
        self.dof_pos_scale  = float(cfg["dof_pos_scale"])
        self.dof_vel_scale  = float(cfg["dof_vel_scale"])
        self.action_scale   = float(cfg["action_scale"])

        # ── Velocity command limits ───────────────────────────────────────
        cr      = cfg["cmd_range"]
        cr_fast = cfg["cmd_range_fast"]
        self._range_slow = [cr["lin_vel_x"],  cr["lin_vel_y"],  cr["ang_vel_z"]]
        self._range_fast = [cr_fast["lin_vel_x"], cr_fast["lin_vel_y"], cr_fast["ang_vel_z"]]
        self._cmd_smooth  = float(cfg["cmd_smooth"])
        self._high_speed  = True

        # ── Safety ───────────────────────────────────────────────────────
        self._tilt_thresh = float(cfg["safe_tilt_threshold"])

        # ── History buffer: 4 frames × 96 dim ────────────────────────────
        self._obs_dim   = int(cfg["obs_dim_per_frame"])
        self._hist_len  = int(cfg["history_len"])
        self._obs_buf   = np.zeros(self._obs_dim * self._hist_len, dtype=np.float32)
        self._last_action = np.zeros(len(self.joint2motor_idx), dtype=np.float32)
        self._cmd_smooth_val = np.zeros(3, dtype=np.float32)

        # ── ONNX session ─────────────────────────────────────────────────
        model_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  "model", cfg["model_path"])
        sess_opts = ort.SessionOptions()
        sess_opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        self._session = ort.InferenceSession(model_path, sess_opts)

        # Warmup
        dummy = np.zeros((1, self._obs_dim * self._hist_len), dtype=np.float32)
        for _ in range(5):
            self._session.run(["actions"], {"obs": dummy})

        print(f"[AMP] Initialized. Model: {model_path}")

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _build_single_obs(self) -> np.ndarray:
        """Build one 96-dim frame from current state_cmd."""
        idx = self.joint2motor_idx

        ang_vel     = self.state_cmd.ang_vel   * self.ang_vel_scale
        proj_grav   = self.state_cmd.gravity_ori
        dof_pos_rel = (self.state_cmd.q[idx] - self.default_dof_pos[idx]) * self.dof_pos_scale
        dof_vel     = self.state_cmd.dq[idx]  * self.dof_vel_scale

        obs = np.concatenate([
            ang_vel,           # 3
            proj_grav,         # 3
            self._cmd_smooth_val,  # 3  (smoothed command)
            dof_pos_rel,       # 29
            dof_vel,           # 29
            self._last_action, # 29  (raw, pre-scale)
        ]).astype(np.float32)
        return obs

    def _push_obs(self):
        """Shift history buffer and append current frame."""
        frame = self._build_single_obs()
        # Roll oldest frame out, append newest at the end
        self._obs_buf[:self._obs_dim * (self._hist_len - 1)] = \
            self._obs_buf[self._obs_dim:]
        self._obs_buf[-self._obs_dim:] = frame

    @staticmethod
    def _piecewise_scale(val: float, lo: float, hi: float,
                         deadzone: float = 0.05) -> float:
        """Map joystick axis [-1, 1] → [lo, hi] with centre = 0.

        Piecewise linear so that the stick resting position always produces
        zero velocity regardless of whether [lo, hi] is symmetric:
          val ∈ [+deadzone, 1]  →  [0, hi]
          val ∈ [-1, -deadzone] →  [lo, 0]
          |val| < deadzone      →  0
        """
        if val > deadzone:
            return hi * (val - deadzone) / (1.0 - deadzone)
        elif val < -deadzone:
            return lo * (-val - deadzone) / (1.0 - deadzone)
        return 0.0

    def _update_cmd(self):
        """Scale joystick input and apply EMA smoothing."""
        cmd_range = self._range_fast if self._high_speed else self._range_slow
        raw_cmd = np.array([
            self._piecewise_scale(self.state_cmd.vel_cmd[i],
                                  cmd_range[i][0], cmd_range[i][1])
            for i in range(3)
        ], dtype=np.float32)
        alpha = self._cmd_smooth
        self._cmd_smooth_val = alpha * self._cmd_smooth_val + (1 - alpha) * raw_cmd

    # ── FSMState interface ────────────────────────────────────────────────────

    def enter(self):
        self._high_speed     = True
        self._last_action[:] = 0.0
        self._cmd_smooth_val[:] = 0.0
        # Fill history with current state (4 calls, mirroring C++ _init_buffers)
        for _ in range(self._hist_len):
            self._push_obs()

    def run(self):
        self._update_cmd()
        self._push_obs()

        obs = np.clip(self._obs_buf, -100.0, 100.0).reshape(1, -1)
        raw_action = self._session.run(
            ["actions"], {"obs": obs}
        )[0].squeeze().astype(np.float32)

        raw_action = np.clip(raw_action, -100.0, 100.0)
        self._last_action[:] = raw_action   # store raw for next observation

        # Map policy DOFs → motor order and add default offset
        actions_out = np.zeros(29, dtype=np.float32)
        actions_out[self.joint2motor_idx] = (raw_action * self.action_scale
                                             + self.default_dof_pos[self.joint2motor_idx])

        self.policy_output.actions = actions_out
        self.policy_output.kps     = self.kps
        self.policy_output.kds     = self.kds

    def exit(self):
        self._last_action[:] = 0.0
        self._cmd_smooth_val[:] = 0.0
        self._obs_buf[:] = 0.0

    def checkChange(self) -> FSMStateName:
        cmd = self.state_cmd.skill_cmd

        # State transitions
        if cmd == FSMCommand.POS_RESET:       # START
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.FIXEDPOSE
        if cmd == FSMCommand.PASSIVE:
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.PASSIVE
        if cmd == FSMCommand.CMD_FREEKICK:
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.SKILL_FREEKICK
        if cmd == FSMCommand.CMD_BEYONDMIMIC_MJ:
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.SKILL_BEYONDMIMIC_MJ
        if cmd == FSMCommand.CMD_STANDUP_MJ:
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.SKILL_STANDUP_MJ
        if cmd == FSMCommand.CMD_PINOCCHIO_1_6_MJ:
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.SKILL_PINOCCHIO_1_6_MJ
        if cmd == FSMCommand.LOCO:            # B -> back to LocoMode
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.LOCOMODE

        return FSMStateName.SKILL_AMP
