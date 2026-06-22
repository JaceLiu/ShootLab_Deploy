from common.path_config import PROJECT_ROOT

from FSM.FSMState import FSMStateName, FSMState
from common.ctrlcomp import StateAndCmd, PolicyOutput
import numpy as np
import yaml
from common.utils import FSMCommand
import os

class FixedPose(FSMState):
    def __init__(self, state_cmd:StateAndCmd, policy_output:PolicyOutput):
        super().__init__()
        self.state_cmd = state_cmd
        self.policy_output = policy_output
        self.name = FSMStateName.FIXEDPOSE
        self.name_str = "fixed_pose"
        self.alpha = 0.
        self.cur_step = 0
        
        current_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(current_dir, "config", "FixedPose.yaml")
        with open(config_path, "r") as f:
            config = yaml.load(f, Loader=yaml.FullLoader)
            self.kds = np.array(config["kds"], dtype=np.float32)
            self.kps = np.array(config["kps"], dtype=np.float32)
            self.default_angles = np.array(config["default_angles"], dtype=np.float32)
            self.joint2motor_idx = np.array(config["joint2motor_idx"], dtype=np.int32)
            self.control_dt = config["control_dt"]

        # Pre-build reordered output arrays (depend only on config, built once).
        n = len(self.joint2motor_idx)
        self._kps_out     = np.zeros(n, dtype=np.float32)
        self._kds_out     = np.zeros(n, dtype=np.float32)
        self._default_out = np.zeros(n, dtype=np.float32)
        self._kps_out[self.joint2motor_idx]     = self.kps
        self._kds_out[self.joint2motor_idx]     = self.kds
        self._default_out[self.joint2motor_idx] = self.default_angles

    def enter(self):
        print("Moving to default pos(configuration A).")
        self.total_time = 2.0
        self.num_step = int(self.total_time / self.control_dt)
        self.alpha = 0.
        self.cur_step = 0
        # Capture current joint positions in joint order (depends on live state).
        self.init_dof_pos = self.state_cmd.q[self.joint2motor_idx].copy()

    def run(self):
        self.cur_step += 1
        self.alpha = min(self.cur_step / self.num_step, 1.0)
        lerped = self.init_dof_pos * (1 - self.alpha) + self.default_angles * self.alpha
        actions_out = np.zeros(len(self.joint2motor_idx), dtype=np.float32)
        actions_out[self.joint2motor_idx] = lerped
        self.policy_output.actions = actions_out
        self.policy_output.kps = self._kps_out
        self.policy_output.kds = self._kds_out

    def exit(self):
        self.policy_output.actions = self._default_out
        self.policy_output.kps = self._kps_out
        self.policy_output.kds = self._kds_out
    
    def checkChange(self):
        if(self.state_cmd.skill_cmd == FSMCommand.LOCO):
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.LOCOMODE
        elif(self.state_cmd.skill_cmd == FSMCommand.PASSIVE):
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.PASSIVE
        elif(self.state_cmd.skill_cmd == FSMCommand.CMD_BEYONDMIMIC_MJ):
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.SKILL_BEYONDMIMIC_MJ
        elif(self.state_cmd.skill_cmd == FSMCommand.CMD_STANDUP_MJ):
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.SKILL_STANDUP_MJ
        elif(self.state_cmd.skill_cmd == FSMCommand.CMD_PINOCCHIO_1_6_MJ):
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.SKILL_PINOCCHIO_1_6_MJ
        elif(self.state_cmd.skill_cmd == FSMCommand.CMD_AMP):
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.SKILL_AMP
        elif(self.state_cmd.skill_cmd == FSMCommand.CMD_FREEKICK):
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.SKILL_FREEKICK
        else:
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.FIXEDPOSE