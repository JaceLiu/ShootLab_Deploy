# BeyondMimic Policy

BeyondMimic 是一个基于 **ONNX 神经网络 + NPZ 参考动作** 的全身动作追踪策略，训练环境为 Isaac Lab（`Tracking-Flat-G1-Wo-State-Estimation-v0`），部署到 Unitree G1（29 DOF）上。

---

## 目录结构

```
policy/beyondmimic/
├── BeyondMimic.py          # 策略主类（FSMState 子类）
├── config/
│   └── beyondmimic.yaml    # PD 增益、动作缩放、路径配置
└── model/
    ├── policy_zuiwu_48000.onnx   # 训练好的神经网络（154 维观测）
    └── dance_zui.npz             # 参考动作序列（11384 帧 @ 50 Hz）
```

---

## 触发方式

在 `deploy_mujoco` 模拟中，按下 **L1 + B** 触发 `FSMCommand.SKILL_6`，FSM 切换到 `FSMStateName.SKILL_BEYONDMIMIC`。

退出方式：
- **Start** → 回到 FixedPose（POS_RESET）
- **R1 + A** → 回到 LocoMode
- **L1 release（R1 按住）** → PassiveMode

---

## ONNX 模型 I/O

| 名称 | 形状 | 说明 |
|------|------|------|
| 输入 `obs` | `[1, 154]` | 154 维观测向量（见下节） |
| 输入 `time_step` | `[1, 1]` | 当前策略步数（从 0 开始，暖机结束后计数） |
| 输出 `actions` | `[1, 29]` | 归一化动作，Isaac Lab 关节顺序，范围约 ±3 |

> 参考动作轨迹已**烘焙进 ONNX 权重**（作为常量张量），`time_step` 告知网络当前处于动作的哪个时刻。

---

## 154 维观测向量

训练配置：`G1FlatWoStateEstimationEnvCfg`（无状态估计，无绝对位置/线速度）。

| 段 | 维度 | 内容 | 来源 |
|----|------|------|------|
| `ref_jpos` | 29 | 参考关节位置（弧度） | NPZ `joint_pos[t]`，Isaac Lab 顺序 |
| `ref_jvel` | 29 | 参考关节速度（rad/s） | NPZ `joint_vel[t]`，Isaac Lab 顺序 |
| `anchor_ori_b` | 6 | torso_link 相对姿态，6D 旋转表示 | 见下方"锚点方向"小节 |
| `base_ang_vel` | 3 | 躯干角速度（body frame，rad/s） | MuJoCo `qvel[3:6]` |
| `joint_pos_rel` | 29 | 关节位置 - 默认位置（弧度） | Isaac Lab 顺序 |
| `joint_vel` | 29 | 关节速度（rad/s） | Isaac Lab 顺序 |
| `last_action` | 29 | 上一步输出的动作 | Isaac Lab 顺序 |
| **合计** | **154** | | |

### 锚点方向（anchor_ori_b）计算

锚点关节为 **torso_link**（NPZ body 索引 9）。

```
aligned_quat = R_yaw_alignment @ ref_anchor_quat_world
rel_quat     = torso_quat_robot_world.conjugate() @ aligned_quat
anchor_ori_b = rot6d(rel_quat)   # 旋转矩阵前两列展平，(6,)
```

其中 `R_yaw_alignment`（`init_to_world`）在 `enter()` 时一次性计算：
```
init_to_world = Rz(robot_yaw_at_enter) @ Rz(motion_t0_yaw).T
```
目的：消除机器人触发 BeyondMimic 时朝向与动作初始朝向之间的偏差，避免策略出现永久的朝向 bias。

---

## 关节顺序映射

训练（Isaac Lab）和部署（MuJoCo XML）使用不同的关节排列顺序，需通过置换数组相互转换。

```
MuJoCo 顺序（XML 顺序，逐肢体）：
  左腿(0-5) → 右腿(6-11) → 躯干(12-14) → 左臂(15-21) → 右臂(22-28)

Isaac Lab 顺序（左右交错）：
  l/r_hip_p, waist_y, l/r_hip_r, waist_r, l/r_hip_y, waist_p,
  l/r_knee, l/r_sho_p, l/r_ank_p, l/r_sho_r, l/r_ank_r,
  l/r_sho_y, l/r_elbow, l/r_wrist_r, l/r_wrist_p, l/r_wrist_y
```

转换操作（numpy 花式索引）：

```python
isaac_array  = mujoco_array[ISAAC_TO_MUJOCO]   # MuJoCo → Isaac
mujoco_array = isaac_array[MUJOCO_TO_ISAAC]     # Isaac  → MuJoCo
```

规则：`MUJOCO_TO_ISAAC[mj_i] = il_i`（MuJoCo 位置 i 对应的 Isaac Lab 索引），`ISAAC_TO_MUJOCO = argsort(MUJOCO_TO_ISAAC)`。

---

## 控制流程

### 阶段一：暖机插值（30 步，0.6 s）

策略触发时机器人姿态与动作起始帧可能差异较大。暖机阶段线性插值，避免 PD 控制产生冲击：

```
alpha = (step + 1) / 30
target_q = (1 - alpha) * 当前关节位置 + alpha * 动作第0帧关节位置
```

### 阶段二：策略运行

```
obs = build_obs(t)              # 构建 154 维观测
actions_il = ONNX(obs, step)    # 推理，Isaac Lab 顺序，clip ±3
actions_mj = actions_il[MUJOCO_TO_ISAAC]
target_q   = default_q_mj + action_scale_mj * actions_mj
```

`target_q` 作为 PD 控制的目标关节位置，经 `deploy_mujoco.py` 中的 PD 控制计算力矩后驱动仿真。

---

## PD 控制参数（来自 ONNX 元数据，训练 ImplicitActuator 增益）

`kps` 和 `kds` 由策略通过 `policy_output.kps/kds` 传出，覆盖全局默认值。

| 关节组 | kp (Nm/rad) | kd (Nm·s/rad) | tau_limit (Nm) |
|--------|------------|---------------|---------------|
| hip_pitch / hip_yaw | 40.179 | 2.558 | 88 |
| hip_roll / knee | 99.098 | 6.309 | 139 |
| ankle | 28.501 | 1.814 | 50 |
| waist_yaw | 40.179 | 2.558 | 88 |
| waist_roll / waist_pitch | 28.501 | 1.814 | 50 |
| shoulder / elbow / wrist_roll | 14.251 | 0.907 | 25 |
| wrist_pitch / wrist_yaw | 16.778 | 1.068 | 5 |

`tau_limit` 在 `deploy_mujoco/config/mujoco.yaml` 中全局配置，须与 G1 XML 的 `actuatorfrcrange` 一致。

---

## NPZ 动作文件格式

| 键 | 形状 | 说明 |
|----|------|------|
| `fps` | scalar | 帧率，50 Hz |
| `joint_pos` | `[T, 29]` | 关节位置，Isaac Lab 顺序 |
| `joint_vel` | `[T, 29]` | 关节速度，Isaac Lab 顺序 |
| `body_pos_w` | `[T, 30, 3]` | 30 个 body 的世界坐标位置 |
| `body_quat_w` | `[T, 30, 4]` | 30 个 body 的世界坐标四元数，`[w,x,y,z]` |
| `body_lin_vel_w` | `[T, 30, 3]` | 线速度（世界系） |
| `body_ang_vel_w` | `[T, 30, 3]` | 角速度（世界系） |

`T = 11384`，持续约 227 秒。Body 排列顺序为 Isaac Lab / PhysX joint_names 顺序（非 MuJoCo XML 顺序）。torso_link 为 body 索引 9（waist_pitch_joint 是第 9 个 joint，其子 body 索引为 9）。

---

## 配置文件 `config/beyondmimic.yaml`

| 字段 | 说明 |
|------|------|
| `onnx_path` | ONNX 文件名（相对 `model/`） |
| `motion_path` | NPZ 文件名（相对 `model/`） |
| `control_dt` | 控制周期（0.02 s） |
| `warmup_steps` | 暖机步数（30） |
| `clip_actions` | 动作裁剪幅度（±3.0） |
| `default_joint_pos` | 默认关节位置，MuJoCo 顺序（来自 ONNX 元数据） |
| `action_scale` | 动作缩放系数，MuJoCo 顺序（来自 ONNX 元数据） |
| `kps` / `kds` | PD 增益，MuJoCo 顺序（来自 ONNX 元数据） |
| `tau_limit` | 力矩限制，MuJoCo 顺序（与 G1 XML `actuatorfrcrange` 一致） |

---

## 添加新动作

1. 将新的 `.onnx` 和 `.npz` 放入 `model/`
2. 修改 `config/beyondmimic.yaml` 中 `onnx_path` 和 `motion_path`
3. 如果 ONNX 元数据中的 `default_joint_pos` / `action_scale` / `joint_stiffness` / `joint_damping` 有变化，同步更新 yaml 中的对应字段（可运行比较脚本从 ONNX 元数据提取）
