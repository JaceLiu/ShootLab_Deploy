from common.path_config import PROJECT_ROOT

import numpy as np
from enum import Enum, unique

@unique
class FSMStateName(Enum):
    INVALID = -1
    PASSIVE = 1
    FIXEDPOSE = 2
    LOCOMODE = 4
    SKILL_BEYONDMIMIC = 12
    SKILL_BEYONDMIMIC_MJ = 13
    SKILL_FREEKICK = 14
    SKILL_STANDUP_MJ = 15
    SKILL_AMP = 16
    SKILL_PINOCCHIO_1_6_MJ = 17

@unique
class FSMCommand(Enum):
    INVALID = -1
    POS_RESET = 1
    LOCO = 2
    PASSIVE = 4
    SKILL_6 = 12
    CMD_BEYONDMIMIC_MJ = 13
    CMD_FREEKICK = 14
    CMD_STANDUP_MJ = 15
    CMD_AMP = 16
    CMD_PINOCCHIO_1_6_MJ = 19




def get_gravity_orientation(quaternion):
    qw, qx, qy, qz = quaternion
    gravity_orientation = np.zeros(3)
    gravity_orientation[0] = 2 * (-qz * qx + qw * qy)
    gravity_orientation[1] = -2 * (qz * qy + qw * qx)
    gravity_orientation[2] = 1 - 2 * (qw * qw + qz * qz)
    return gravity_orientation

def progress_bar(current, total, length=50):
    percent = current / total
    filled = int(length * percent)
    bar = "█" * filled + "-" * (length - filled)
    return f"\r|{bar}| {percent:.1%} [{current:.3f}s/{total:.3f}s]"

def scale_values(values, target_ranges):
    scaled = []
    for val, (new_min, new_max) in zip(values, target_ranges):
        scaled_val = (val + 1) * (new_max - new_min) / 2 + new_min
        scaled.append(scaled_val)
    return np.array(scaled)


