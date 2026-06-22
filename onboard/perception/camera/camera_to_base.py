"""
camera_to_base.py

Transform a 3D point from head_camera_link (RealSense D435, body frame:
X-forward, Y-left, Z-up) to the pelvis (base) frame.

Kinematic chain (from URDF g1_sysid_23dof.urdf):
    pelvis
      └─ waist_yaw_joint   (Rz, origin=[0, 0, 0])
          └─ waist_roll_joint  (Rx, origin=[-0.0039635, 0, 0.044])
              └─ waist_pitch_joint (Ry, origin=[0, 0, 0])
                  └─ head_joint   (Ry, origin=[0.0039635, 0, 0.3159])
                      └─ head_camera_joint (fixed,
                             xyz=[0.0448353662, 0.01, 0.1219029938]
                             rpy=[0.0119142, 0.8377475, 0.0053045])

Note: rs2_deproject_pixel_to_point returns the optical frame (Z-forward,
X-right, Y-down). Call optical_to_body() before passing to
transform_point_camera_to_base().
"""

import numpy as np


def _Rx(th):
    c, s = np.cos(th), np.sin(th)
    return np.array([[1, 0,  0],
                     [0, c, -s],
                     [0, s,  c]])


def _Ry(th):
    c, s = np.cos(th), np.sin(th)
    return np.array([[ c, 0, s],
                     [ 0, 1, 0],
                     [-s, 0, c]])


def _Rz(th):
    c, s = np.cos(th), np.sin(th)
    return np.array([[c, -s, 0],
                     [s,  c, 0],
                     [0,  0, 1]])


def _rpy_to_R(roll, pitch, yaw):
    """URDF convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)"""
    return _Rz(yaw) @ _Ry(pitch) @ _Rx(roll)


def _T(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3]  = t
    return T


# head_camera_joint fixed transform (from URDF)
_T_HEAD_CAMERA = _T(
    _rpy_to_R(roll=0.0119142, pitch=0.8377475, yaw=0.0053045),
    [0.0448353662, 0.01, 0.1219029938],
)


def transform_point_camera_to_base(p_cam, q_wy, q_wr, q_wp, q_head):
    """
    Transform a point in head_camera_link (body frame) to the pelvis (base) frame.

    Args:
        p_cam  : array-like (3,), point in camera body frame (X-forward, metres)
        q_wy   : waist_yaw_joint angle (rad)
        q_wr   : waist_roll_joint angle (rad)
        q_wp   : waist_pitch_joint angle (rad)
        q_head : head_joint angle (rad)

    Returns:
        np.ndarray (3,), point in pelvis frame (metres)
    """
    T = (
        _T(_Rz(q_wy), [0.0, 0.0, 0.0])
        @ _T(_Rx(q_wr), [-0.0039635, 0.0, 0.044])
        @ _T(_Ry(q_wp), [0.0, 0.0, 0.0])
        @ _T(_Ry(q_head), [0.0039635, 0.0, 0.3159])
        @ _T_HEAD_CAMERA
    )
    return (T @ np.array([*p_cam, 1.0]))[:3]


# ── Chest camera placeholder extrinsics ──────────────────────────────────────
# Kinematic chain:
#   pelvis → waist_yaw(q_wy,Rz,[0,0,0])
#          → waist_roll(q_wr,Rx,[−0.004,0,0.044])
#          → waist_pitch(q_wp,Ry,[0,0,0])
#          → chest_camera_joint (fixed, values below are PLACEHOLDERS)
#
# TODO: replace _CHEST_XYZ and _CHEST_RPY with actual measured/calibrated values.
# Measure: x = forward offset from waist_pitch_link origin,
#          z = upward offset, y = lateral offset (+ = left).

# YAW_CHEST_XYZ = [0.125, 0.00, 0.11] # measured by hand (YC_
# YAW_PITCH_XYZ = [-0.0039635, 0.0, 0.044] #(YP)
# PITCH_CHEST_XYZ = [0.1289635, 0.00, 0.066] #(PC = YC - YP)
#_CHEST_XYZ = [0.1289635, 0.00, 0.066]   # TODO: measure (metres, in waist_pitch frame)
#_CHEST_RPY = (0.00, 0.523599, 0.00)   # TODO: calibrate (roll, pitch, yaw) in radians
_CHEST_XYZ = [0.13444, 0.00, 0.06228]   # TODO: measure (metres, in waist_pitch frame)
_CHEST_RPY = (0.00, 0.2902482546, 0.00)   # TODO: calibrate (roll, pitch, yaw) in radians

_T_CHEST_CAMERA = _T(
    _rpy_to_R(*_CHEST_RPY),
    _CHEST_XYZ,
)


def get_default_chest_extrinsics():
    """Return the baked-in chest camera extrinsics."""
    return tuple(_CHEST_XYZ), tuple(_CHEST_RPY)


def _build_chest_camera_transform(chest_xyz=None, chest_rpy=None):
    xyz = _CHEST_XYZ if chest_xyz is None else chest_xyz
    rpy = _CHEST_RPY if chest_rpy is None else chest_rpy
    return _T(_rpy_to_R(*rpy), xyz)


def transform_point_chest_camera_to_base(p_cam, q_wy, q_wr, q_wp):
    """
    Transform a point in chest_camera_link (body frame) to the pelvis (base) frame.
    Extrinsics (_CHEST_XYZ, _CHEST_RPY) are placeholders — update before using.

    Args:
        p_cam : array-like (3,), point in chest camera body frame (X-forward, metres)
        q_wy  : waist_yaw_joint angle (rad)
        q_wr  : waist_roll_joint angle (rad)
        q_wp  : waist_pitch_joint angle (rad)

    Returns:
        np.ndarray (3,), point in pelvis frame (metres)
    """
    T = (
        _T(_Rz(q_wy), [0.0, 0.0, 0.0])
        @ _T(_Rx(q_wr), [-0.0039635, 0.0, 0.044])
        @ _T(_Ry(q_wp), [0.0, 0.0, 0.0])
        @ _T_CHEST_CAMERA
    )
    return (T @ np.array([*p_cam, 1.0]))[:3]


def transform_point_chest_camera_to_base_with_extrinsics(
    p_cam,
    q_wy,
    q_wr,
    q_wp,
    chest_xyz=None,
    chest_rpy=None,
):
    """Chest-camera to pelvis transform with optional runtime extrinsics override."""
    T = (
        _T(_Rz(q_wy), [0.0, 0.0, 0.0])
        @ _T(_Rx(q_wr), [-0.0039635, 0.0, 0.044])
        @ _T(_Ry(q_wp), [0.0, 0.0, 0.0])
        @ _build_chest_camera_transform(chest_xyz=chest_xyz, chest_rpy=chest_rpy)
    )
    return (T @ np.array([*p_cam, 1.0]))[:3]


def optical_to_body(p_optical):
    """
    RealSense optical frame → body frame (REP-103).

    Optical: Z-forward, X-right, Y-down
    Body:    X-forward, Y-left,  Z-up

    Args:
        p_optical : array-like (3,)

    Returns:
        np.ndarray (3,)
    """
    x, y, z = p_optical
    return np.array([z, -x, -y])
