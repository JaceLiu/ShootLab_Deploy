import numpy as np
from scipy.spatial.transform import Rotation as R


def get_gravity_orientation_real(quaternion):
    qw = quaternion[0]
    qx = quaternion[1]
    qy = quaternion[2]
    qz = quaternion[3]

    gravity_orientation = np.zeros(3)

    gravity_orientation[0] = 2 * (-qz * qx + qw * qy)
    gravity_orientation[1] = -2 * (qz * qy + qw * qx)
    gravity_orientation[2] = 1 - 2 * (qw * qw + qz * qz)

    return gravity_orientation


def transform_imu_data(waist_yaw, waist_yaw_omega, imu_quat, imu_omega):
    RzWaist = R.from_euler("z", waist_yaw).as_matrix()
    R_torso = R.from_quat([imu_quat[1], imu_quat[2], imu_quat[3], imu_quat[0]]).as_matrix()
    R_pelvis = np.dot(R_torso, RzWaist.T)
    w = np.dot(RzWaist, imu_omega[0]) - np.array([0, 0, waist_yaw_omega])
    return R.from_matrix(R_pelvis).as_quat()[[3, 0, 1, 2]], w


def transform_pelvis_to_torso_complete(waist_yaw, waist_roll, waist_pitch, pelvis_quat):
    """G1 pelvis IMU 四元数 → torso_link 四元数。

    G1 的 IMU 装在 pelvis，但策略的 anchor 是 torso_link。
    运动链: pelvis -> waist_yaw_joint(Z) -> waist_roll_joint(X) -> waist_pitch_joint(Y) -> torso_link

    参数均为标量关节角（弧度），pelvis_quat 格式为 [w, x, y, z]。
    返回 torso_link 四元数 [w, x, y, z]。
    """
    R_waist_yaw   = R.from_euler("z", waist_yaw)
    R_waist_roll  = R.from_euler("x", waist_roll)
    R_waist_pitch = R.from_euler("y", waist_pitch)

    # scipy 使用 [x, y, z, w] 格式
    R_pelvis = R.from_quat([pelvis_quat[1], pelvis_quat[2], pelvis_quat[3], pelvis_quat[0]])

    R_torso = R_pelvis * R_waist_yaw * R_waist_roll * R_waist_pitch

    q_xyzw = R_torso.as_quat()  # [x, y, z, w]
    return np.array([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]], dtype=np.float32)  # [w, x, y, z]
