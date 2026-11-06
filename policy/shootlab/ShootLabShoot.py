"""Deterministic ShootLab V24/V34 shooting policy for G1 deployment."""

from __future__ import annotations

from collections import deque
import hashlib
import json
from pathlib import Path

import numpy as np
import onnxruntime
import yaml

from FSM.FSMState import FSMState, FSMStateName
from common.ctrlcomp import PolicyOutput, StateAndCmd
from common.utils import FSMCommand


NUM_JOINTS = 29
HISTORY_LEN = 5
LATENT_DIM = 16

MUJOCO_TO_ISAAC = np.array(
    [
        0, 3, 6, 9, 13, 17,
        1, 4, 7, 10, 14, 18,
        2, 5, 8,
        11, 15, 19, 21, 23, 25, 27,
        12, 16, 20, 22, 24, 26, 28,
    ],
    dtype=np.int64,
)
ISAAC_TO_MUJOCO = np.argsort(MUJOCO_TO_ISAAC).astype(np.int64)

SELF_OBS_NAMES = [
    "base_ang_vel",
    "joint_pos_rel",
    "joint_vel",
    "last_decoded_action",
    "projected_gravity",
]


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _quat_to_matrix(quat_wxyz: np.ndarray) -> np.ndarray:
    quat = np.asarray(quat_wxyz, dtype=np.float64)
    quat /= np.linalg.norm(quat)
    w, x, y, z = quat
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
            [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
            [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _yaw_matrix(quat_wxyz: np.ndarray) -> np.ndarray:
    w, x, y, z = np.asarray(quat_wxyz, dtype=np.float64)
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    cosine = np.cos(yaw)
    sine = np.sin(yaw)
    return np.array([[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]])


def projected_gravity_b(quat_wxyz: np.ndarray) -> np.ndarray:
    return (_quat_to_matrix(quat_wxyz).T @ np.array([0.0, 0.0, -1.0])).astype(np.float32)


class ShootLabShoot(FSMState):
    """Run the fused V24/V34 policy with deployment-safe state handling."""

    TARGET_SOURCE_TO_CODE = {
        "none": 0.0,
        "fixed": 1.0,
        "fixed_fallback": 2.0,
        "apriltag": 3.0,
        "imu_hold": 4.0,
        "fixed_sim": 5.0,
    }

    def __init__(
        self,
        state_cmd: StateAndCmd,
        policy_output: PolicyOutput,
        runtime_mode: str = "sim",
    ):
        super().__init__()
        self.state_cmd = state_cmd
        self.policy_output = policy_output
        self.name = FSMStateName.SKILL_SHOOTLAB
        self.name_str = "shootlab_v24_v34"
        if runtime_mode not in {"sim", "real"}:
            raise ValueError(f"runtime_mode must be 'sim' or 'real', got {runtime_mode!r}")
        self.runtime_mode = runtime_mode

        current_dir = Path(__file__).resolve().parent
        with (current_dir / "config/shootlab_shoot.yaml").open("r") as file:
            cfg = yaml.safe_load(file)
        model_dir = current_dir / "model"
        self.onnx_path = model_dir / cfg["onnx_path"]
        self.metadata_path = model_dir / cfg["metadata_path"]
        self.manifest_path = model_dir / cfg["manifest_path"]

        self.control_dt = float(cfg["control_dt"])
        self.warmup_steps = int(cfg["warmup_steps"])
        self.max_policy_steps = int(round(float(cfg["policy_duration_s"]) / self.control_dt))
        self.max_ball_invalid_steps = int(
            np.ceil(float(cfg["ball_stale_timeout_s"]) / self.control_dt)
        )
        self.clip_joint_targets = bool(cfg.get("clip_joint_targets", False))
        self.target_source = str(cfg["target_source"]).strip().lower()
        if self.target_source not in {"fixed", "apriltag"}:
            raise ValueError(f"Unsupported target_source: {self.target_source!r}")
        self.target_pos = np.asarray(cfg["target_pos"], dtype=np.float64)
        self.target_hold_on_loss_with_imu = bool(cfg["target_hold_on_loss_with_imu"])
        self.target_use_fixed_fallback = bool(cfg["target_use_fixed_fallback"])
        if self.target_pos.shape != (3,):
            raise ValueError("target_pos must contain three values")

        self.metadata = json.loads(self.metadata_path.read_text())
        self.manifest = json.loads(self.manifest_path.read_text())
        self._validate_artifacts()
        self._validate_metadata()

        self.default_q_isaac = np.asarray(self.metadata["default_joint_pos"], dtype=np.float32)
        self.default_q = self.default_q_isaac[MUJOCO_TO_ISAAC].copy()
        self.kps = np.asarray(self.metadata["kps"], dtype=np.float32)[MUJOCO_TO_ISAAC].copy()
        self.kds = np.asarray(self.metadata["kds"], dtype=np.float32)[MUJOCO_TO_ISAAC].copy()
        self.joint_min = np.asarray(
            self.metadata["joint_pos_min"], dtype=np.float32
        )[MUJOCO_TO_ISAAC]
        self.joint_max = np.asarray(
            self.metadata["joint_pos_max"], dtype=np.float32
        )[MUJOCO_TO_ISAAC]

        session_options = onnxruntime.SessionOptions()
        session_options.graph_optimization_level = (
            onnxruntime.GraphOptimizationLevel.ORT_ENABLE_ALL
        )
        self.ort_session = onnxruntime.InferenceSession(
            str(self.onnx_path),
            sess_options=session_options,
            providers=["CPUExecutionProvider"],
        )
        self._validate_onnx_io()

        self._gravity_history = deque(maxlen=HISTORY_LEN)
        self._ang_vel_history = deque(maxlen=HISTORY_LEN)
        self._joint_pos_history = deque(maxlen=HISTORY_LEN)
        self._joint_vel_history = deque(maxlen=HISTORY_LEN)
        self._action_history = deque(maxlen=HISTORY_LEN)
        self._ball_history = deque(maxlen=HISTORY_LEN)
        self._target_history = deque(maxlen=HISTORY_LEN)
        self._reset_history()

        self.time_step = 0
        self._entry_q = self.default_q.copy()
        self._last_decoded_action_isaac = np.zeros(NUM_JOINTS, dtype=np.float32)
        self._last_raw_z = np.zeros(LATENT_DIM, dtype=np.float32)
        self._last_valid_ball_b = np.zeros(3, dtype=np.float32)
        self._ball_invalid_steps = 0
        self._entry_yaw = np.eye(3, dtype=np.float64)
        self._last_target_world_yaw = None
        self._target_source_runtime = "none"
        self._debug_ball_pos_b = np.zeros(3, dtype=np.float32)
        self._debug_target_pos_b = np.zeros(3, dtype=np.float32)
        self._exit_reason = ""

        dummy_obs = np.zeros((1, self.metadata["policy_obs_dim"]), dtype=np.float32)
        dummy_self = np.zeros((1, self.metadata["self_obs_dim"]), dtype=np.float32)
        for _ in range(5):
            self.ort_session.run(
                ["joint_targets", "raw_z"],
                {"policy_obs": dummy_obs, "self_obs_raw": dummy_self},
            )
        print(f"[ShootLab] initialized runtime={runtime_mode}, model={self.onnx_path.name}")

    def _validate_artifacts(self) -> None:
        for key, path in (
            ("onnx_sha256", self.onnx_path),
            ("metadata_sha256", self.metadata_path),
            ("test_vectors_sha256", self.onnx_path.parent / self.manifest["test_vectors"]),
        ):
            if _sha256(path) != self.manifest[key]:
                raise ValueError(f"Artifact hash mismatch for {path}")

    def _validate_metadata(self) -> None:
        if self.metadata.get("format") != "shootlab_shoot_fused_v3":
            raise ValueError("Unsupported ShootLab ONNX metadata format")
        expected = {
            "policy_obs_dim": 430,
            "self_obs_dim": 93,
            "latent_dim": LATENT_DIM,
            "joint_targets_order": "isaac_lab_bfs",
        }
        for key, value in expected.items():
            if self.metadata.get(key) != value:
                raise ValueError(f"Metadata {key!r}: expected {value!r}, got {self.metadata.get(key)!r}")
        self_obs_names = [term["name"] for term in self.metadata["self_obs_terms"]]
        if self_obs_names != SELF_OBS_NAMES:
            raise ValueError(f"Unexpected self_obs layout: {self_obs_names}")
        for key in (
            "joint_names",
            "default_joint_pos",
            "action_scale",
            "kps",
            "kds",
            "effort_limit",
            "joint_pos_min",
            "joint_pos_max",
        ):
            if len(self.metadata[key]) != NUM_JOINTS:
                raise ValueError(f"Metadata {key} must contain {NUM_JOINTS} values")

    def _validate_onnx_io(self) -> None:
        inputs = {item.name: item for item in self.ort_session.get_inputs()}
        outputs = {item.name for item in self.ort_session.get_outputs()}
        for name, dim in (("policy_obs", 430), ("self_obs_raw", 93)):
            if name not in inputs or int(inputs[name].shape[-1]) != dim:
                raise ValueError(f"Invalid or missing ONNX input {name}")
        if "joint_targets" not in outputs or "raw_z" not in outputs:
            raise ValueError("ONNX model must expose joint_targets and raw_z")

    def _reset_history(self) -> None:
        for buffer, dim in (
            (self._gravity_history, 3),
            (self._ang_vel_history, 3),
            (self._joint_pos_history, NUM_JOINTS),
            (self._joint_vel_history, NUM_JOINTS),
            (self._action_history, LATENT_DIM),
            (self._ball_history, 3),
            (self._target_history, 3),
        ):
            buffer.clear()
            buffer.extend([np.zeros(dim, dtype=np.float32) for _ in range(HISTORY_LEN)])

    def enter(self) -> None:
        self.time_step = 0
        self._entry_q = np.asarray(self.state_cmd.q, dtype=np.float32).copy()
        self._last_decoded_action_isaac.fill(0.0)
        self._last_raw_z.fill(0.0)
        self._ball_invalid_steps = 0
        self._exit_reason = ""
        self._entry_yaw = _yaw_matrix(self.state_cmd.pelvis_quat_w)
        self._last_target_world_yaw = None
        self._reset_history()

        if self.state_cmd.ball_valid:
            self._last_valid_ball_b = np.asarray(self.state_cmd.ball_pos_b, dtype=np.float32).copy()
        else:
            self._last_valid_ball_b.fill(0.0)
        self.policy_output.viz_spheres = None
        max_delta = float(np.max(np.abs(self.default_q - self._entry_q)))
        print(
            f"[ShootLab] enter warmup={self.warmup_steps}, "
            f"policy_steps={self.max_policy_steps}, max_joint_delta={max_delta:.3f} rad, "
            f"target_source={self.target_source}"
        )

    def _body_rotation(self) -> np.ndarray:
        return _quat_to_matrix(self.state_cmd.pelvis_quat_w)

    def _ball_pos_b(self) -> np.ndarray:
        ball_valid = bool(self.state_cmd.ball_valid) and np.isfinite(
            self.state_cmd.ball_pos_b
        ).all()
        if ball_valid:
            if self.runtime_mode == "real":
                ball_b = np.asarray(self.state_cmd.ball_pos_b, dtype=np.float64)
            else:
                ball_b = self._body_rotation().T @ (
                    np.asarray(self.state_cmd.ball_pos_w, dtype=np.float64)
                    - np.asarray(self.state_cmd.pelvis_pos_w, dtype=np.float64)
                )
            self._last_valid_ball_b = np.clip(ball_b, -8.0, 8.0).astype(np.float32)
            self._ball_invalid_steps = 0
        elif self.time_step >= self.warmup_steps:
            self._ball_invalid_steps += 1
        ball_b = self._last_valid_ball_b.copy()
        ball_b[1] += float(self.state_cmd.ball_y_bias)
        return np.clip(ball_b, -8.0, 8.0).astype(np.float32)

    def _fixed_target_pos_b(self, current_yaw: np.ndarray) -> np.ndarray:
        target_world = self._entry_yaw @ self.target_pos
        return np.clip(current_yaw.T @ target_world, -8.0, 8.0).astype(np.float32)

    def _target_pos_b(self) -> np.ndarray:
        if self.runtime_mode == "sim":
            target_b = self._body_rotation().T @ (
                self.target_pos - np.asarray(self.state_cmd.pelvis_pos_w, dtype=np.float64)
            )
            self._target_source_runtime = "fixed_sim"
        else:
            current_yaw = _yaw_matrix(self.state_cmd.pelvis_quat_w)
            if self.target_source == "apriltag":
                target_valid = bool(self.state_cmd.target_valid) and np.isfinite(
                    self.state_cmd.target_pos_b
                ).all()
                if target_valid:
                    target_b = np.asarray(self.state_cmd.target_pos_b, dtype=np.float64)
                    self._last_target_world_yaw = current_yaw @ target_b
                    self._target_source_runtime = "apriltag"
                elif self.target_hold_on_loss_with_imu and self._last_target_world_yaw is not None:
                    target_b = current_yaw.T @ self._last_target_world_yaw
                    self._target_source_runtime = "imu_hold"
                elif self.target_use_fixed_fallback:
                    target_b = self._fixed_target_pos_b(current_yaw)
                    self._target_source_runtime = "fixed_fallback"
                else:
                    target_b = np.zeros(3, dtype=np.float64)
                    self._target_source_runtime = "none"
            else:
                target_b = self._fixed_target_pos_b(current_yaw)
                self._target_source_runtime = "fixed"
        target_b = np.asarray(target_b, dtype=np.float32)
        target_b[1] += float(self.state_cmd.target_y_bias)
        return np.clip(target_b, -8.0, 8.0).astype(np.float32)

    def _build_observations(self) -> tuple[np.ndarray, np.ndarray]:
        joint_pos_isaac = np.asarray(self.state_cmd.q, dtype=np.float32)[ISAAC_TO_MUJOCO]
        joint_vel_isaac = np.asarray(self.state_cmd.dq, dtype=np.float32)[ISAAC_TO_MUJOCO]
        joint_pos_rel = joint_pos_isaac - self.default_q_isaac
        gravity = projected_gravity_b(self.state_cmd.pelvis_quat_w)
        ang_vel = np.asarray(self.state_cmd.root_ang_vel_b, dtype=np.float32)
        ball_b = self._ball_pos_b()
        target_b = self._target_pos_b()

        self._gravity_history.append(gravity)
        self._ang_vel_history.append(ang_vel)
        self._joint_pos_history.append(joint_pos_rel)
        self._joint_vel_history.append(joint_vel_isaac)
        self._ball_history.append(ball_b)
        self._target_history.append(target_b)

        policy_obs = np.concatenate(
            [
                *self._gravity_history,
                *self._ang_vel_history,
                *self._joint_pos_history,
                *self._joint_vel_history,
                *self._action_history,
                *self._ball_history,
                *self._target_history,
            ]
        ).astype(np.float32)
        self_obs = np.concatenate(
            [
                ang_vel,
                joint_pos_rel,
                joint_vel_isaac,
                self._last_decoded_action_isaac,
                gravity,
            ]
        ).astype(np.float32)
        self._debug_ball_pos_b = ball_b.copy()
        self._debug_target_pos_b = target_b.copy()
        return policy_obs, self_obs

    def _set_command(self, target_q: np.ndarray) -> None:
        target_q = np.asarray(target_q, dtype=np.float32)
        if not np.isfinite(target_q).all():
            raise RuntimeError("ShootLab policy produced a non-finite joint target")
        if self.clip_joint_targets:
            target_q = np.clip(target_q, self.joint_min, self.joint_max)
        self.policy_output.actions = target_q
        self.policy_output.kps = self.kps
        self.policy_output.kds = self.kds

    def _hold_current_pose(self, reason: str) -> None:
        self._exit_reason = reason
        self.state_cmd.skill_cmd = FSMCommand.POS_RESET
        self._set_command(np.asarray(self.state_cmd.q, dtype=np.float32))

    def run(self) -> None:
        if self.time_step < self.warmup_steps:
            alpha = (self.time_step + 1) / self.warmup_steps
            self._set_command((1.0 - alpha) * self._entry_q + alpha * self.default_q)
            self.time_step += 1
            return

        policy_step = self.time_step - self.warmup_steps
        if policy_step >= self.max_policy_steps:
            self._hold_current_pose("policy_duration_elapsed")
            return

        policy_obs, self_obs = self._build_observations()
        if self._ball_invalid_steps > self.max_ball_invalid_steps:
            self._hold_current_pose("ball_state_timeout")
            return

        outputs = self.ort_session.run(
            ["joint_targets", "raw_z", "decoded_action"],
            {
                "policy_obs": policy_obs[None, :],
                "self_obs_raw": self_obs[None, :],
            },
        )
        joint_targets_isaac = outputs[0][0].astype(np.float32)
        raw_z = outputs[1][0].astype(np.float32)
        self._last_decoded_action_isaac = outputs[2][0].astype(np.float32)
        self._last_raw_z = raw_z.copy()
        self._action_history.append(raw_z)
        target_q = joint_targets_isaac[MUJOCO_TO_ISAAC]
        self._set_command(target_q)

        self.policy_output.debug_target_pos_b = self._debug_target_pos_b.copy()
        self.policy_output.debug_target_source[:] = self.TARGET_SOURCE_TO_CODE[
            self._target_source_runtime
        ]
        if self.runtime_mode == "sim":
            target_marker = self.target_pos.copy()
            target_marker[2] = max(float(target_marker[2]), 0.12)
            self.policy_output.viz_spheres = [
                {
                    "pos": target_marker,
                    "radius": 0.07,
                    "rgba": np.array([1.0, 0.0, 1.0, 0.95], dtype=np.float32),
                },
                {
                    "from": np.asarray(self.state_cmd.pelvis_pos_w, dtype=np.float64),
                    "to": target_marker,
                    "radius": 0.006,
                    "rgba": np.array([1.0, 0.0, 1.0, 0.45], dtype=np.float32),
                },
            ]
        else:
            self.policy_output.viz_spheres = None
        self.time_step += 1

    def exit(self) -> None:
        self.policy_output.ghost_qpos = None
        self.policy_output.viz_spheres = None
        self.policy_output.debug_target_pos_b[:] = 0.0
        self.policy_output.debug_target_source[:] = 0.0
        self.time_step = 0
        if self._exit_reason:
            print(f"[ShootLab] exit: {self._exit_reason}", flush=True)

    def set_target_pos_w(self, target_pos_w) -> None:
        target = np.asarray(target_pos_w, dtype=np.float64)
        if target.shape != (3,) or not np.isfinite(target).all():
            raise ValueError("target_pos_w must be a finite 3-vector")
        self.target_pos = target.copy()

    def activation_ready(self) -> bool:
        return bool(self.state_cmd.ball_valid) and np.isfinite(
            self.state_cmd.ball_pos_b
        ).all()

    def checkChange(self) -> FSMStateName:
        command = self.state_cmd.skill_cmd
        transitions = {
            FSMCommand.PASSIVE: FSMStateName.PASSIVE,
            FSMCommand.POS_RESET: FSMStateName.FIXEDPOSE,
            FSMCommand.LOCO: FSMStateName.LOCOMODE,
            FSMCommand.CMD_AMP: FSMStateName.SKILL_AMP,
            FSMCommand.CMD_FREEKICK: FSMStateName.SKILL_FREEKICK,
            FSMCommand.CMD_BEYONDMIMIC_MJ: FSMStateName.SKILL_BEYONDMIMIC_MJ,
            FSMCommand.CMD_STANDUP_MJ: FSMStateName.SKILL_STANDUP_MJ,
            FSMCommand.CMD_PINOCCHIO_1_6_MJ: FSMStateName.SKILL_PINOCCHIO_1_6_MJ,
        }
        if command in transitions:
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return transitions[command]
        if command == FSMCommand.CMD_SHOOTLAB:
            self.state_cmd.skill_cmd = FSMCommand.INVALID
        return FSMStateName.SKILL_SHOOTLAB
