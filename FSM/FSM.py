from common.path_config import PROJECT_ROOT

from policy.passive.PassiveMode import PassiveMode
from policy.fixedpose.FixedPose import FixedPose
from policy.loco_mode.LocoMode import LocoMode
from policy.beyondmimic.BeyondMimic import BeyondMimic
from policy.beyondmimic_mj.BeyondMimicMJ import BeyondMimicMJ
from policy.robonaldo.FreeKick import FreeKick
from policy.amp.Amp import Amp
from FSM.FSMState import *
import time
from common.ctrlcomp import *
from enum import Enum, unique

@unique
class FSMMode(Enum):
    CHANGE = 1
    NORMAL = 2

class FSM:
    def __init__(self, state_cmd:StateAndCmd, policy_output:PolicyOutput):
        self.state_cmd = state_cmd
        self.policy_output = policy_output
        self.cur_policy : FSMState
        self.next_policy : FSMState
        self.sim_counter = 0
        self.FSMmode = FSMMode.NORMAL
        
        self.passive_mode = PassiveMode(state_cmd, policy_output)       # 阻尼保护模式
        self.fixed_pose_1 = FixedPose(state_cmd, policy_output)         
        self.loco_policy = LocoMode(state_cmd, policy_output)
        self.beyondmimic_policy = BeyondMimic(state_cmd, policy_output)
        self.beyondmimic_mj_policy = BeyondMimicMJ(state_cmd, policy_output)
        self.standup_mj_policy = BeyondMimicMJ(state_cmd, policy_output,
                                               "standup_mj.yaml",
                                               FSMStateName.SKILL_STANDUP_MJ)
        self.pinocchio_1_6_mj_policy = BeyondMimicMJ(
            state_cmd,
            policy_output,
            "g1_result_pinocchio_1_6_mj.yaml",
            FSMStateName.SKILL_PINOCCHIO_1_6_MJ,
        )
        self.freekick_policy = FreeKick(state_cmd, policy_output)
        self.amp_policy = Amp(state_cmd, policy_output)

        print("initalized all policies!!!")
        
        self.cur_policy = self.passive_mode             # 当前policy
        print("current policy is ", self.cur_policy.name_str)
        
        
        
    def run(self):
        start_time = time.time()
        if(self.FSMmode == FSMMode.NORMAL): 
            self.cur_policy.run()
            nextPolicyName = self.cur_policy.checkChange()
            
            if(nextPolicyName != self.cur_policy.name):
                # change policy
                self.FSMmode = FSMMode.CHANGE
                self.cur_policy.exit()
                self.get_next_policy(nextPolicyName)
                print("Switched to ", self.cur_policy.name_str)
        
        elif(self.FSMmode == FSMMode.CHANGE):
            self.cur_policy.enter()
            self.sim_counter = 0
            self.FSMmode = FSMMode.NORMAL
            self.cur_policy.run()
            
        # self.absoluteWait(self.cur_policy.control_horzion,self.start_time)
        end_time = time.time()
        # print("time cusume: ", end_time - start_time)

    def absoluteWait(self, control_dt, start_time):
        end_time = time.time()
        delta_time = end_time - start_time
        if(delta_time < control_dt):
            time.sleep(control_dt - delta_time)
        else:
            print("inference time beyond control horzion!!!")
            
            
    def get_next_policy(self, policy_name:FSMStateName):
        if(policy_name == FSMStateName.PASSIVE):
            self.cur_policy = self.passive_mode
        elif((policy_name == FSMStateName.FIXEDPOSE)):
            self.cur_policy = self.fixed_pose_1
        elif((policy_name == FSMStateName.LOCOMODE)):
            self.cur_policy = self.loco_policy
        elif((policy_name == FSMStateName.SKILL_BEYONDMIMIC)):
            self.cur_policy = self.beyondmimic_policy
        elif((policy_name == FSMStateName.SKILL_BEYONDMIMIC_MJ)):
            self.cur_policy = self.beyondmimic_mj_policy
        elif((policy_name == FSMStateName.SKILL_FREEKICK)):
            self.cur_policy = self.freekick_policy
        elif((policy_name == FSMStateName.SKILL_STANDUP_MJ)):
            self.cur_policy = self.standup_mj_policy
        elif((policy_name == FSMStateName.SKILL_AMP)):
            self.cur_policy = self.amp_policy
        elif((policy_name == FSMStateName.SKILL_PINOCCHIO_1_6_MJ)):
            self.cur_policy = self.pinocchio_1_6_mj_policy
        else:
            pass
            
        
        