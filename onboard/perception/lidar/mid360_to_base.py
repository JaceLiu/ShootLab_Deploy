import numpy as np

# ----------------- 基础函数 -----------------

def Rx(th):
    c, s = np.cos(th), np.sin(th)
    return np.array([[1, 0,  0],
                     [0, c, -s],
                     [0, s,  c]])

def Ry(th):
    c, s = np.cos(th), np.sin(th)
    return np.array([[ c, 0, s],
                     [ 0, 1, 0],
                     [-s, 0, c]])

def Rz(th):
    c, s = np.cos(th), np.sin(th)
    return np.array([[c, -s, 0],
                     [s,  c, 0],
                     [0,  0, 1]])

# URDF 中 rpy 的含义：roll=x, pitch=y, yaw=z
# 变换顺序：R = Rz(yaw) * Ry(pitch) * Rx(roll)
def rpy_to_R(roll, pitch, yaw):
    return Rz(yaw) @ Ry(pitch) @ Rx(roll)

def T_from_R_t(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

# ----------------- 封装变换函数 -----------------

def compute_mid360_to_base_transform(q_wy=0.0, q_wr=0.0, q_wp=0.0, q_head=0.593412, q_mid=0.0):
    """
    计算从 mid360 坐标系到 base (pelvis) 坐标系的变换矩阵
    
    参数:
        q_wy: waist_yaw_joint 角度 (绕 z)
        q_wr: waist_roll_joint 角度 (绕 x)
        q_wp: waist_pitch_joint 角度 (绕 y)
        q_head: head_joint 角度 (绕 y)
        q_mid: mid360_joint 角度 (绕 y)
    
    返回:
        T_pelvis_mid360: 4x4 齐次变换矩阵，将 mid360 坐标系下的点变换到 pelvis 坐标系
    """
    # ----------------- 从 URDF 抽出来的具体数值 -----------------
    # 1) waist_yaw_joint
    # <joint name="waist_yaw_joint" type="revolute">
    #   <origin xyz="0 0 0" rpy="0 0 0"/>
    #   <axis xyz="0 0 1"/>
    # </joint>
    t_pelvis_waist_yaw = np.array([0.0, 0.0, 0.0])
    R_pelvis_waist_yaw = Rz(q_wy)
    T_pelvis_waist_yaw = T_from_R_t(R_pelvis_waist_yaw, t_pelvis_waist_yaw)

    # 2) waist_roll_joint
    # <joint name="waist_roll_joint" type="revolute">
    #   <origin xyz="-0.0039635 0 0.044" rpy="0 0 0"/>
    #   <axis xyz="1 0 0"/>
    # </joint>
    t_waist_yaw_roll = np.array([-0.0039635, 0.0, 0.044])
    R_waist_yaw_roll = Rx(q_wr)
    T_waist_yaw_roll = T_from_R_t(R_waist_yaw_roll, t_waist_yaw_roll)

    # 3) waist_pitch_joint
    # <joint name="waist_pitch_joint" type="revolute">
    #   <origin xyz="0 0 0" rpy="0 0 0"/>
    #   <axis xyz="0 1 0"/>
    # </joint>
    t_waist_roll_torso = np.array([0.0, 0.0, 0.0])
    R_waist_roll_torso = Ry(q_wp)
    T_waist_roll_torso = T_from_R_t(R_waist_roll_torso, t_waist_roll_torso)

    # 4) head_joint
    # <joint name="head_joint" type="revolute">
    #   <origin xyz="0.0039635 0 0.3159" rpy="0 0 0"/>
    #   <axis xyz="0 1 0"/>
    # </joint>
    t_torso_head = np.array([0.0039635, 0.0, 0.3159])
    R_torso_head = Ry(q_head)
    T_torso_head = T_from_R_t(R_torso_head, t_torso_head)

    # 5) mid360_joint
    # <joint name="mid360_joint" type="revolute">
    #   <origin xyz="0 0.00003 0.10028" rpy="0 3.101 3.1415"/>
    #   <axis xyz="0 1 0"/>
    # </joint>
    t_head_mid360 = np.array([0.0, 0.00003, 0.10028])
    R_head_mid360_fixed = rpy_to_R(roll=0.0, pitch=3.101, yaw=3.1415)
    R_head_mid360 = R_head_mid360_fixed @ Ry(q_mid)
    T_head_mid360 = T_from_R_t(R_head_mid360, t_head_mid360)

    # ----------------- 链式相乘：pelvis -> mid360 -----------------
    # 注意：这些 T 的含义都是 T_parent_child
    T_pelvis_mid360 = (
        T_pelvis_waist_yaw @
        T_waist_yaw_roll   @
        T_waist_roll_torso @
        T_torso_head       @
        T_head_mid360
    )
    
    return T_pelvis_mid360


def transform_point_mid360_to_base(point_mid360, q_wy=0.0, q_wr=0.0, q_wp=0.0, q_head=0.593412, q_mid=0.0):
    """
    将 mid360 坐标系下的点转换到 base (pelvis) 坐标系
    
    参数:
        point_mid360: (3,) 或 (N, 3) numpy array，mid360 坐标系下的点
        q_wy, q_wr, q_wp, q_head, q_mid: 关节角（可选，使用默认值）
    
    返回:
        point_base: (3,) 或 (N, 3) numpy array，base 坐标系下的点
    """
    T = compute_mid360_to_base_transform(q_wy, q_wr, q_wp, q_head, q_mid)
    
    # 处理单个点或点数组
    if point_mid360.ndim == 1:
        # 单个点 (3,)
        p_homogeneous = np.append(point_mid360, 1.0)
        p_base = T @ p_homogeneous
        return p_base[:3]
    else:
        # 点数组 (N, 3)
        N = point_mid360.shape[0]
        p_homogeneous = np.hstack([point_mid360, np.ones((N, 1))])
        p_base = (T @ p_homogeneous.T).T
        return p_base[:, :3]


# ----------------- 示例代码（用于测试） -----------------
if __name__ == "__main__":
    # ----------------- 关节角（示例） -----------------
    # 这里随便给一组关节角，你可以换成真实的机器人状态
    q_wy   = 0.0   # waist_yaw_joint (绕 z)
    q_wr   = 0.0  # waist_roll_joint (绕 x)
    q_wp   = 0.0   # waist_pitch_joint (绕 y)
    q_head = 0.593412   # head_joint (绕 y)
    q_mid  = 0.0  # mid360_joint (绕 y)

    T_pelvis_mid360 = compute_mid360_to_base_transform(q_wy, q_wr, q_wp, q_head, q_mid)
    print("T_pelvis_mid360 =\n", T_pelvis_mid360)

    # ----------------- 把 mid360 系下的一个点变换到 pelvis 系 -----------------
    # 假设 mid360_link 坐标系下，有一点 p_mid360 = [0.1, 0, 0]（比如在传感器前方 0.1m）
    p_mid360 = np.array([2.125, 0, 0])  # 或者使用齐次坐标: np.array([2.125, 0, 0, 1.0])
    # p_mid360 = np.array([2.27695476,  0.23446804, -0.1281498, 1.0])

    p_pelvis = transform_point_mid360_to_base(p_mid360, q_wy, q_wr, q_wp, q_head, q_mid)
    
    print("p in mid360 frame :", p_mid360)
    print("p in pelvis frame :", p_pelvis)