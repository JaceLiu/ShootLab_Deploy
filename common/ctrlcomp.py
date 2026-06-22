from common.path_config import PROJECT_ROOT

import numpy as np
from common.utils import FSMCommand


class StateAndCmd:
    def __init__(self, num_joints):
        # robot state
        self.num_joints = num_joints
        self.q = np.zeros(num_joints, dtype=np.float32)
        self.dq = np.zeros(num_joints, dtype=np.float32)
        self.ddq = np.zeros(num_joints, dtype=np.float32)
        self.tau_est = np.zeros(num_joints, dtype=np.float32)
        self.gravity_ori = np.array([0., 0., 1.])
        self.ang_vel = np.zeros(3)
        # body-frame velocities for BeyondMimic (filled in deploy_mujoco)
        self.root_lin_vel_b = np.zeros(3, dtype=np.float32)
        self.root_ang_vel_b = np.zeros(3, dtype=np.float32)
        self.torso_pos_w  = np.zeros(3, dtype=np.float32)
        self.torso_quat_w = np.array([1., 0., 0., 0.], dtype=np.float32)  # [w,x,y,z]
        # pelvis (floating base) state for FreeKick controller (world frame, filled in deploy_mujoco)
        self.pelvis_pos_w  = np.zeros(3, dtype=np.float32)
        self.pelvis_quat_w = np.array([1., 0., 0., 0.], dtype=np.float32)  # [w,x,y,z]
        # ball state for FreeKick controller
        # Simulation: ball_pos_w (world frame, filled by deploy_mujoco from MuJoCo)
        # Real robot: ball_pos_b (pelvis body frame, filled by deploy_real from DDS)
        self.ball_pos_w    = np.zeros(3, dtype=np.float32)
        self.ball_vel_w    = np.zeros(3, dtype=np.float32)
        self.ball_pos_b    = np.zeros(3, dtype=np.float32)  # real robot only
        self.ball_valid    = False                           # real robot only
        # target state for FreeKick controller
        # Real robot: target_pos_b / target_valid come from rt/target_state DDS.
        self.target_pos_b  = np.zeros(3, dtype=np.float32)
        self.target_valid  = False
        self.target_class_id = -1
        self.target_confidence = 0.0
        # manual target Y-bias (metres, + = pelvis +Y = left)
        # Adjusted by L1+right/left; published to rt/target_y_bias when DDS is wired up.
        self.target_y_bias = 0.0
        # manual ball Y-bias (metres, + = pelvis +Y = left)
        # Adjusted by L2+right/left in policy_runtime / deploy_mujoco.
        self.ball_y_bias = 0.0
        # joy cmd
        self.vel_cmd = np.zeros(3)
        self.skill_cmd = FSMCommand.INVALID
        # skill change cmd

class PolicyOutput:
    """Shared output buffer written by the active FSM policy each control step.

    The three array fields (actions, kps, kds) use property setters that always
    copy the incoming value into an internally-owned buffer.  This prevents
    policies from accidentally aliasing their own arrays with the shared output
    object — a subtle bug where an in-place write by one policy would silently
    corrupt another policy's internal state.

    Usage from a policy is unchanged:
        self.policy_output.actions = target_q   # safe: copies into buffer
        self.policy_output.kps[i]  = value      # safe: writes into buffer
    """

    def __init__(self, num_joints):
        self._actions = np.zeros(num_joints, dtype=np.float32)
        self._kps     = np.zeros(num_joints, dtype=np.float32)
        self._kds     = np.zeros(num_joints, dtype=np.float32)
        self.ghost_qpos = None   # np.ndarray (7+n_joints,) or None
        self.debug_target_pos_b = np.zeros(3, dtype=np.float32)
        self.debug_target_source = np.array([0.0], dtype=np.float32)
        # Optional debug spheres drawn in MuJoCo viewer.
        # List of dicts: {"pos": (3,) world-frame, "radius": float, "rgba": (4,) float}
        # Set to None to skip rendering. Cleared to None by FSM between policy activations.
        self.viz_spheres = None

    # ------------------------------------------------------------------
    # Property accessors: getters return the internal buffer directly so
    # that in-place writes (kps[i] = x) still land on the owned buffer;
    # setters always copy so callers can never share memory with us.
    # ------------------------------------------------------------------

    @property
    def actions(self):
        return self._actions

    @actions.setter
    def actions(self, value):
        self._actions[:] = value

    @property
    def kps(self):
        return self._kps

    @kps.setter
    def kps(self, value):
        self._kps[:] = value

    @property
    def kds(self):
        return self._kds

    @kds.setter
    def kds(self, value):
        self._kds[:] = value
        