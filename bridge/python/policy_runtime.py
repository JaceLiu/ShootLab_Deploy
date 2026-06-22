import time
from dataclasses import dataclass

import numpy as np

from common.ctrlcomp import StateAndCmd, PolicyOutput
from common.utils import FSMCommand, FSMStateName
from FSM.FSM import FSM
from common.rotation_helper import (
    get_gravity_orientation_real,
    transform_pelvis_to_torso_complete,
)
from common.remote_controller import RemoteController, KeyMap
from common.ball_state_dds import BallStateSubscriber, BallStatePublisher
from common.target_state_dds import TargetStateSubscriber, TargetStatePublisher
from bridge.python.bridge_state_dds import BridgeStateSubscriber
from common.logger import Logger


@dataclass
class PolicyCommandFrame:
    seq: int
    q_des: np.ndarray
    kp: np.ndarray
    kd: np.ndarray
    request_damping: bool = False
    exit_requested: bool = False


class PolicyRuntime:
    """Policy/FSM runner that consumes bridge-state DDS instead of Unitree SDK."""

    def __init__(self, config):
        self.config = config
        self.num_joints = config.num_joints
        self.control_dt = config.control_dt
        self.remote_controller = RemoteController()

        self.state_cmd = StateAndCmd(self.num_joints)
        self.policy_output = PolicyOutput(self.num_joints)
        self.FSM_controller = FSM(self.state_cmd, self.policy_output)

        self.bridge_state_sub = BridgeStateSubscriber(
            domain_id=config.bridge_domain_id,
            topic_name=config.bridge_state_topic,
            stale_ms=config.bridge_state_stale_ms,
        )
        self.bridge_state_sub.start()

        self.ball_sub = BallStateSubscriber(domain_id=0)
        self.ball_sub.start()
        self.target_sub = TargetStateSubscriber(domain_id=0)
        self.target_sub.start()
        self._corrected_target_pub = TargetStatePublisher(
            topic_name="rt/target_state_corrected"
        )
        self._corrected_ball_pub = BallStatePublisher(
            topic_name="rt/ball_state_corrected"
        )
        self._bias_pub = TargetStatePublisher(topic_name="rt/bias_values")

        self._prev_right_pressed    = False
        self._prev_left_pressed     = False
        self._prev_l2_right_pressed = False
        self._prev_l2_left_pressed  = False

        self._log_step = 0
        self._log_start = time.time()
        self._log_states = {FSMStateName[s] for s in config.log_states} if config.log_enabled else set()
        self._logger = (
            Logger(
                config.log_dir,
                config.log_tag,
                extra_meta={"robot_type": "real_policy_bridge", "control_dt": config.control_dt},
            )
            if config.log_enabled
            else None
        )

        self._cmd_seq = 0
        self.exit_requested = False

        self.wait_for_bridge_state()

    def wait_for_bridge_state(self):
        while self.bridge_state_sub.latest().tick == 0:
            time.sleep(self.control_dt)
        print("Successfully connected to bridge state.")

    def close(self):
        self.bridge_state_sub.stop()
        self.ball_sub.stop()
        self.target_sub.stop()
        if self._logger is not None:
            self._logger.close()

    def _set_remote_from_bridge(self, bridge_state):
        raw = bytes(int(v) & 0xFF for v in bridge_state.remote_raw)
        if len(raw) >= 24:
            self.remote_controller.set(raw[:24])

    def _apply_remote_commands(self):
        if self.remote_controller.is_button_pressed(KeyMap.select):
            self.exit_requested = True
            return

        # Target Y-bias: L1 held + D-pad Right/Left (edge-triggered, ±5 cm)
        l1_held   = self.remote_controller.is_button_pressed(KeyMap.L1)
        right_now = self.remote_controller.is_button_pressed(KeyMap.right)
        left_now  = self.remote_controller.is_button_pressed(KeyMap.left)
        if l1_held:
            if right_now and not self._prev_right_pressed:
                self.state_cmd.target_y_bias = float(np.clip(
                    self.state_cmd.target_y_bias - 0.05, -1.50, 1.50))
                print(f"\n[BIAS] target_y_bias = {self.state_cmd.target_y_bias:+.2f} m", flush=True)
            elif left_now and not self._prev_left_pressed:
                self.state_cmd.target_y_bias = float(np.clip(
                    self.state_cmd.target_y_bias + 0.05, -1.50, 1.50))
                print(f"\n[BIAS] target_y_bias = {self.state_cmd.target_y_bias:+.2f} m", flush=True)
        self._prev_right_pressed = right_now
        self._prev_left_pressed  = left_now

        # Ball Y-bias: L2 held + D-pad Right/Left (edge-triggered, ±5 cm)
        l2_held    = self.remote_controller.is_button_pressed(KeyMap.L2)
        l2_right   = self.remote_controller.is_button_pressed(KeyMap.right)
        l2_left    = self.remote_controller.is_button_pressed(KeyMap.left)
        if l2_held:
            if l2_right and not self._prev_l2_right_pressed:
                self.state_cmd.ball_y_bias = float(np.clip(
                    self.state_cmd.ball_y_bias - 0.05, -1.50, 1.50))
                print(f"\n[BALL BIAS] ball_y_bias = {self.state_cmd.ball_y_bias:+.2f} m", flush=True)
            elif l2_left and not self._prev_l2_left_pressed:
                self.state_cmd.ball_y_bias = float(np.clip(
                    self.state_cmd.ball_y_bias + 0.05, -1.50, 1.50))
                print(f"\n[BALL BIAS] ball_y_bias = {self.state_cmd.ball_y_bias:+.2f} m", flush=True)
        self._prev_l2_right_pressed = l2_right
        self._prev_l2_left_pressed  = l2_left

        if self.remote_controller.is_button_pressed(KeyMap.F1):
            self.state_cmd.skill_cmd = FSMCommand.PASSIVE

        if self.remote_controller.is_button_pressed(KeyMap.start):
            self.state_cmd.skill_cmd = FSMCommand.POS_RESET
        elif self.remote_controller.is_button_pressed(KeyMap.B):
            self.state_cmd.skill_cmd = FSMCommand.LOCO
        elif self.remote_controller.is_button_pressed(KeyMap.A):
            self.state_cmd.skill_cmd = FSMCommand.CMD_AMP
        elif self.remote_controller.is_button_pressed(KeyMap.R1):
            self.state_cmd.skill_cmd = FSMCommand.CMD_FREEKICK
        elif self.remote_controller.is_button_pressed(KeyMap.down):
            self.state_cmd.skill_cmd = FSMCommand.CMD_BEYONDMIMIC_MJ
        elif self.remote_controller.is_button_pressed(KeyMap.up):
            self.state_cmd.skill_cmd = FSMCommand.CMD_STANDUP_MJ
        elif self.remote_controller.is_button_pressed(KeyMap.R2):
            self.state_cmd.skill_cmd = FSMCommand.CMD_PINOCCHIO_1_6_MJ

        self.state_cmd.vel_cmd[0] = self.remote_controller.ly
        self.state_cmd.vel_cmd[1] = -self.remote_controller.lx
        self.state_cmd.vel_cmd[2] = -self.remote_controller.rx

    def _apply_robot_state(self, bridge_state):
        q = np.asarray(bridge_state.q, dtype=np.float32)
        dq = np.asarray(bridge_state.dq, dtype=np.float32)
        quat = np.asarray(bridge_state.imu_quat_wxyz, dtype=np.float32)
        ang_vel = np.asarray(bridge_state.imu_gyro, dtype=np.float32)

        gravity_orientation = get_gravity_orientation_real(quat)
        torso_quat = transform_pelvis_to_torso_complete(
            q[12], q[13], q[14], quat
        )

        self.state_cmd.q = q
        self.state_cmd.dq = dq
        self.state_cmd.gravity_ori = gravity_orientation.astype(np.float32)
        self.state_cmd.ang_vel = ang_vel
        self.state_cmd.torso_quat_w = torso_quat.astype(np.float32)
        self.state_cmd.pelvis_quat_w = quat.astype(np.float32)
        self.state_cmd.root_ang_vel_b = ang_vel.astype(np.float32)

    def _apply_perception_state(self):
        ball = self.ball_sub.latest()
        self.state_cmd.ball_pos_b = np.array([ball.x, ball.y, ball.z], dtype=np.float32)
        self.state_cmd.ball_valid = bool(ball.valid)

        target = self.target_sub.latest()
        self.state_cmd.target_pos_b = np.array([target.x, target.y, target.z], dtype=np.float32)
        self.state_cmd.target_valid = bool(target.valid)
        self.state_cmd.target_class_id = int(target.class_id)
        self.state_cmd.target_confidence = float(target.confidence)

        # Publish corrected target = raw + Y-bias (pelvis frame)
        corrected_t = self.state_cmd.target_pos_b.copy()
        corrected_t[1] += self.state_cmd.target_y_bias
        self._corrected_target_pub.publish(
            float(corrected_t[0]), float(corrected_t[1]), float(corrected_t[2]),
            valid=self.state_cmd.target_valid,
            confidence=self.state_cmd.target_confidence,
        )

        # Publish corrected ball = raw + Y-bias (pelvis frame)
        corrected_b = self.state_cmd.ball_pos_b.copy()
        corrected_b[1] += self.state_cmd.ball_y_bias
        self._corrected_ball_pub.publish(
            float(corrected_b[0]), float(corrected_b[1]), float(corrected_b[2]),
            valid=self.state_cmd.ball_valid,
        )

        # Publish current bias values for dashboard display (x=target_y_bias, y=ball_y_bias)
        self._bias_pub.publish(
            float(self.state_cmd.target_y_bias),
            float(self.state_cmd.ball_y_bias),
            0.0,
            valid=True,
        )

    def step(self) -> PolicyCommandFrame:
        bridge_state = self.bridge_state_sub.latest()
        if bridge_state.tick == 0:
            return PolicyCommandFrame(
                seq=self._cmd_seq,
                q_des=np.zeros(self.num_joints, dtype=np.float32),
                kp=np.zeros(self.num_joints, dtype=np.float32),
                kd=np.full(self.num_joints, 8.0, dtype=np.float32),
                request_damping=True,
                exit_requested=self.exit_requested,
            )

        self._set_remote_from_bridge(bridge_state)
        self._apply_remote_commands()
        self._apply_robot_state(bridge_state)
        self._apply_perception_state()

        self.FSM_controller.run()

        if self._logger is not None and self.FSM_controller.cur_policy.name in self._log_states:
            t = time.time() - self._log_start
            self._logger.log(self._log_step, t, self.state_cmd, self.policy_output)
            self._log_step += 1

        frame = PolicyCommandFrame(
            seq=self._cmd_seq,
            q_des=self.policy_output.actions.copy(),
            kp=self.policy_output.kps.copy(),
            kd=self.policy_output.kds.copy(),
            request_damping=self.exit_requested,
            exit_requested=self.exit_requested,
        )
        self._cmd_seq += 1
        return frame
