from common.path_config import PROJECT_ROOT

from common.utils import FSMStateName    

class FSMState:
    """Base class for all FSM states (policies).

    Contract for writing to policy_output
    --------------------------------------
    - policy_output is a *write-only* output channel shared across all states.
    - Always write full arrays:  self.policy_output.kps = self.kps
    - Never read back values you did not just write.
    - Never hold a reference to policy_output's arrays for use across steps;
      read robot state from state_cmd, not from policy_output.
    - PolicyOutput's setters copy incoming values into its own buffer, so
      there is no need for .copy() at the call site.
    """

    def __init__(self):
        self.name = FSMStateName.INVALID
        self.name_str = "invalid"
        self.control_dt = 0.02

    def enter(self):
        raise NotImplementedError("enter() function must be implement!")
    
    def run(self):
        raise NotImplementedError("run() function must be implement!")
    
    def exit(self):
        raise NotImplementedError("exit() function must be implement!")
    
    def checkChange(self):
        # joystick callback
        raise NotImplementedError("checkChange() function must be implement!")
        