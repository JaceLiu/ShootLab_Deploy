import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent.parent.absolute()))

from common.path_config import PROJECT_ROOT
from common.ctrlcomp import *
from FSM.FSM import *
from common.utils import FSMStateName
from typing import Union
import numpy as np
import time
import os
import yaml

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_, unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_, unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_ as LowCmdHG
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_ as LowCmdGo
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as LowStateHG
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_ as LowStateGo
from unitree_sdk2py.utils.crc import CRC

from common.command_helper import create_damping_cmd, create_zero_cmd, init_cmd_hg, init_cmd_go, MotorMode
from common.logger import Logger
from common.rotation_helper import get_gravity_orientation_real, transform_imu_data, transform_pelvis_to_torso_complete
from common.remote_controller import RemoteController, KeyMap
from common.ball_state_dds import BallStateSubscriber
from common.target_state_dds import TargetStateSubscriber, TargetStatePublisher
from config import Config


class Controller:
    def __init__(self, config: Config):
        self.config = config
        self.remote_controller = RemoteController()
        self.num_joints = config.num_joints
        self.control_dt = config.control_dt
        
        
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = unitree_hg_msg_dds__LowState_()
        self.mode_pr_ = MotorMode.PR
        self.mode_machine_ = 0
        self.lowcmd_publisher_ = ChannelPublisher(config.lowcmd_topic, LowCmdHG)
        self.lowcmd_publisher_.Init()
        
        # inital connection
        self.lowstate_subscriber = ChannelSubscriber(config.lowstate_topic, LowStateHG)
        self.lowstate_subscriber.Init(self.LowStateHgHandler, 10)
        
        self.wait_for_low_state()
        
        init_cmd_hg(self.low_cmd, self.mode_machine_, self.mode_pr_)
        
        self.policy_output_action = np.zeros(self.num_joints, dtype=np.float32)
        self.kps = np.zeros(self.num_joints, dtype=np.float32)
        self.kds = np.zeros(self.num_joints, dtype=np.float32)
        self.qj = np.zeros(self.num_joints, dtype=np.float32)
        self.dqj = np.zeros(self.num_joints, dtype=np.float32)
        self.quat = np.zeros(4, dtype=np.float32)
        self.ang_vel = np.zeros(3, dtype=np.float32)
        self.gravity_orientation = np.array([0,0,-1], dtype=np.float32)
        
        self.state_cmd = StateAndCmd(self.num_joints)                   # 定义了机器人的state
        self.policy_output = PolicyOutput(self.num_joints)              # 定义了action, kp, kd
        self.FSM_controller = FSM(self.state_cmd, self.policy_output)

        # Ball state subscriber (DDS, from on-robot ball_detector_service)
        self.ball_sub = BallStateSubscriber(domain_id=0)
        self.ball_sub.start()
        self.target_sub = TargetStateSubscriber(domain_id=0)
        self.target_sub.start()
        self._corrected_target_pub = TargetStatePublisher(
            topic_name="rt/target_state_corrected"
        )

        self._prev_right_pressed = False
        self._prev_left_pressed  = False

        self.running = True
        self.counter_over_time = 0

        self._log_step   = 0
        self._log_start  = time.time()
        self._log_states = {FSMStateName[s] for s in config.log_states} if config.log_enabled else set()
        self._logger = (
            Logger(config.log_dir, config.log_tag,
                   extra_meta={"robot_type": "real", "control_dt": config.control_dt})
            if config.log_enabled else None
        )
        
        
    def LowStateHgHandler(self, msg: LowStateHG):
        self.low_state = msg
        self.mode_machine_ = self.low_state.mode_machine
        self.remote_controller.set(self.low_state.wireless_remote)

    def LowStateGoHandler(self, msg: LowStateGo):
        self.low_state = msg
        self.remote_controller.set(self.low_state.wireless_remote)

    def send_cmd(self, cmd: Union[LowCmdGo, LowCmdHG]):
        cmd.crc = CRC().Crc(cmd)
        self.lowcmd_publisher_.Write(cmd)

    def wait_for_low_state(self):
        while self.low_state.tick == 0:
            time.sleep(self.config.control_dt)
        print("Successfully connected to the robot.")

    def zero_torque_state(self):
        print("Enter zero torque state.")
        print("Waiting for the start signal...")
        while self.remote_controller.button[KeyMap.start] != 1:
            create_zero_cmd(self.low_cmd)
            self.send_cmd(self.low_cmd)
            time.sleep(self.config.control_dt)
        
    def run(self):
        try:
            # if(self.counter_over_time >= config.error_over_time):
            #     raise ValueError("counter_over_time >= error_over_time")
            
            loop_start_time = time.time()
            
            ## 1. 检测遥控器按键
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

            # PASSIVE: safety command — always overrides any pending command
            if self.remote_controller.is_button_pressed(KeyMap.F1):             # F1阻尼保护模式
                self.state_cmd.skill_cmd = FSMCommand.PASSIVE

            # All other skill commands: latched — only accepted when FSM has cleared the previous one.
            # elif self.state_cmd.skill_cmd == FSMCommand.INVALID:
            if self.remote_controller.is_button_pressed(KeyMap.start):
                self.state_cmd.skill_cmd = FSMCommand.POS_RESET
            elif self.remote_controller.is_button_pressed(KeyMap.B):            # Loco, B
                self.state_cmd.skill_cmd = FSMCommand.LOCO
            elif self.remote_controller.is_button_pressed(KeyMap.A):            # AMP, A
                self.state_cmd.skill_cmd = FSMCommand.CMD_AMP
            elif self.remote_controller.is_button_pressed(KeyMap.R1):           # FreeKick, R1
                self.state_cmd.skill_cmd = FSMCommand.CMD_FREEKICK
            elif self.remote_controller.is_button_pressed(KeyMap.down):         # BeyondMimicMJ, Down
                self.state_cmd.skill_cmd = FSMCommand.CMD_BEYONDMIMIC_MJ
            elif self.remote_controller.is_button_pressed(KeyMap.up):           # StandUpMJ, Up
                self.state_cmd.skill_cmd = FSMCommand.CMD_STANDUP_MJ
            elif self.remote_controller.is_button_pressed(KeyMap.R2):           # Pinocchio1.6MJ, R2
                self.state_cmd.skill_cmd = FSMCommand.CMD_PINOCCHIO_1_6_MJ
            
            self.state_cmd.vel_cmd[0] =  self.remote_controller.ly              # 速度指令
            self.state_cmd.vel_cmd[1] =  self.remote_controller.lx * -1
            self.state_cmd.vel_cmd[2] =  self.remote_controller.rx * -1
            
            ## 2. 获取底层状态
            for i in range(self.num_joints):
                self.qj[i] = self.low_state.motor_state[i].q            # 关节位置
                self.dqj[i] = self.low_state.motor_state[i].dq          # 关节速度

            # imu_state quaternion: w, x, y, z
            quat = self.low_state.imu_state.quaternion
            ang_vel = np.array(self.low_state.imu_state.gyroscope, dtype=np.float32)
            
            gravity_orientation = get_gravity_orientation_real(quat)

            # torso_quat_w：G1 的 IMU 在 pelvis，需经 waist 三关节变换到 torso_link
            # qj 为 MuJoCo 顺序：waist_yaw=12, waist_roll=13, waist_pitch=14
            torso_quat = transform_pelvis_to_torso_complete(
                self.qj[12], self.qj[13], self.qj[14], quat
            )

            self.state_cmd.q = self.qj.copy()
            self.state_cmd.dq = self.dqj.copy()
            self.state_cmd.gravity_ori = gravity_orientation.copy()
            self.state_cmd.ang_vel = ang_vel.copy()
            self.state_cmd.torso_quat_w  = torso_quat
            self.state_cmd.pelvis_quat_w = np.array(quat, dtype=np.float32)  # raw IMU [w,x,y,z]
            self.state_cmd.root_ang_vel_b = ang_vel.flatten().astype(np.float32)

            # Ball state from DDS (pelvis body frame, ~10 Hz)
            ball = self.ball_sub.latest()
            self.state_cmd.ball_pos_b = np.array([ball.x, ball.y, ball.z], dtype=np.float32)
            self.state_cmd.ball_valid  = bool(ball.valid)

            # Target state from DDS (pelvis body frame, from AprilTag / target detector)
            target = self.target_sub.latest()
            self.state_cmd.target_pos_b = np.array([target.x, target.y, target.z], dtype=np.float32)
            self.state_cmd.target_valid = bool(target.valid)
            self.state_cmd.target_class_id = int(target.class_id)
            self.state_cmd.target_confidence = float(target.confidence)

            # Publish corrected target = raw + Y-bias (pelvis frame)
            corrected_b = self.state_cmd.target_pos_b.copy()
            corrected_b[1] += self.state_cmd.target_y_bias
            self._corrected_target_pub.publish(
                float(corrected_b[0]), float(corrected_b[1]), float(corrected_b[2]),
                valid=self.state_cmd.target_valid,
                confidence=self.state_cmd.target_confidence,
            )

            self.FSM_controller.run()
            policy_output_action = self.policy_output.actions.copy()
            kps = self.policy_output.kps.copy()
            kds = self.policy_output.kds.copy()

            if (self._logger is not None and
                    self.FSM_controller.cur_policy.name in self._log_states):
                t = time.time() - self._log_start
                self._logger.log(self._log_step, t, self.state_cmd, self.policy_output)
                self._log_step += 1
            
            # 设定电机指令
            for i in range(self.num_joints):
                self.low_cmd.motor_cmd[i].q = policy_output_action[i]
                self.low_cmd.motor_cmd[i].qd = 0
                self.low_cmd.motor_cmd[i].kp = kps[i]
                self.low_cmd.motor_cmd[i].kd = kds[i]
                self.low_cmd.motor_cmd[i].tau = 0
                
            # send the command
            # create_damping_cmd(controller.low_cmd) # only for debug
            self.send_cmd(self.low_cmd)         # 发送指令
            
            loop_end_time = time.time()
            delta_time = loop_end_time - loop_start_time
            if(delta_time < self.control_dt):
                time.sleep(self.control_dt - delta_time)
                self.counter_over_time = 0
            else:
                print("control loop over time.")
                self.counter_over_time += 1
            pass
        except ValueError as e:
            print(str(e))
            pass
        
        pass
        
        
if __name__ == "__main__":
    config = Config()
    # Initialize DDS communication
    ChannelFactoryInitialize(0, config.net)
    
    controller = Controller(config)
    
    while True:
        try:
            controller.run()
            # Press the select key to exit
            if controller.remote_controller.is_button_pressed(KeyMap.select):           # 按下select键退出
                break
        except KeyboardInterrupt:
            break
    
    controller.ball_sub.stop()
    controller.target_sub.stop()
    if controller._logger is not None:
        controller._logger.close()
    create_damping_cmd(controller.low_cmd)
    controller.send_cmd(controller.low_cmd)
    print("Exit")
    