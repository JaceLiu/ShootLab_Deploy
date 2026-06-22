# Yaw Alignment in BeyondMimicMJ / FreeKick: 原理与奇异修复

## 背景

BeyondMimicMJ 和 FreeKick 在 `enter()` 时计算旋转矩阵 `_init_to_world`，将 NPZ 参考轨迹在水平面内旋转，使其初始朝向与机器人当前朝向对齐。这样无论机器人面向何处，policy 收到的 anchor 参考始终以机器人当前朝向为基准，与训练时一致。

---

## 训练代码的对齐方式（commands.py）

训练中 `anchor_ori_b` 的计算：

```python
# observations.py
_, ori = subtract_frame_transforms(
    robot_anchor_quat_w,   # 机器人当前 torso 方向
    anchor_quat_w,         # NPZ 参考 torso 方向（原始，未对齐）
)
anchor_ori_b = rot6d(ori)  # = rot6d(q_robot⁻¹ ⊗ q_motion)
```

训练中每 episode 初始化时，机器人被重置到完全匹配动作的姿态（包括 yaw）。因此：

```
q_robot ≈ q_motion  →  q_robot⁻¹ ⊗ q_motion ≈ identity  →  anchor_ori_b ≈ [1,0,0,1,0,0]
```

训练中不需要 yaw 对齐，因为机器人与动作的 yaw 天然一致。

---

## 部署时为什么需要 yaw 对齐

部署时机器人可能朝向任意方向（不是动作录制时的方向）。若不对齐，`q_robot⁻¹ ⊗ q_motion` 就包含一个 yaw 误差项，policy 会感知到这个误差并尝试修正，导致奇怪的转向行为。

因此需要计算 `_init_to_world`（仅提取 yaw 部分的旋转），在每帧用它旋转参考 anchor：

```python
aligned_quat  = _quat_mul(init_world_quat, ref_anchor_quat)
rel_quat      = _quat_mul(_quat_conj(robot_torso_quat), aligned_quat)
anchor_ori_6d = _rot6d_from_quat(rel_quat)
```

正确的 `_init_to_world` 应使得：在 t=0 时 `rel_quat ≈ identity`，即参考 anchor 与机器人 torso 方向一致。

---

## 旧逻辑（有 Bug）

```python
yaw_motion_mat = _quat_to_matrix(_yaw_quat(motion_t0_quat))
yaw_robot_mat  = _quat_to_matrix(_yaw_quat(robot_quat))
self._init_to_world = yaw_robot_mat @ yaw_motion_mat.T
```

`_yaw_quat` 的公式（ZYX 欧拉角分解）：

```
yaw = arctan2(2*(w*z + x*y), 1 - 2*(y² + z²))
```

### 问题：对各自的四元数分别提取 yaw

**直立时（pitch ≈ 0°）** ✅ 正确：

```
yaw(q_robot) = θ_r,  yaw(q_motion) = θ_m
init_to_world = R_z(θ_r) @ R_z(-θ_m) = R_z(θ_r - θ_m)
```

**躺下时（pitch ≈ ±90°）** ❌ 奇异：

```
q = q_z(θ) ⊗ q_y(-90°) 时，ZYX 欧拉角分解在 pitch=±90° 出现万向锁：
分子 = 2*(w*z + x*y) ≈ 0
分母 = 1 - 2*(y² + z²) ≈ 0
→ arctan2(≈0, ≈0) = 未定义，受 IMU 噪声随机驱动
```

结果：`_init_to_world` 变成随机旋转矩阵 → `anchor_ori_6d` 包含巨大虚假误差 → policy 疯狂纠错 → yaw 方向奇异扭转。

### 曾经尝试的第一个修复（无效）

用 `_yaw_mat_robust`：当机器人躺下时退化为单位矩阵（不做 yaw 对齐）。

**为什么无效**：单位矩阵意味着不做任何对齐，动作以 NPZ 原始坐标系播放。如果机器人的 yaw 与动作 t=0 的 yaw 不同，`anchor_ori_6d` 仍然包含 yaw 误差，奇异扭转依然发生。

---

## 正确修复：对相对四元数提取 yaw

### 核心洞察（来自训练代码 commands.py:386）

训练中对齐方式实际上是：

```python
delta_ori_w = yaw_quat(quat_mul(q_robot, quat_inv(q_motion)))
```

即对**相对四元数** `q_rel = q_robot ⊗ q_motion⁻¹` 提取 yaw，而不是对各自的四元数分别提取。

### 为什么相对四元数更鲁棒

当机器人和动作都是躺下姿态（例如仰躺，pitch=-90°）：

```
q_robot   = q_z(θ_r) ⊗ q_y(-90°)
q_motion  = q_z(θ_m) ⊗ q_y(-90°)

q_rel = q_robot ⊗ q_motion⁻¹
      = q_z(θ_r) ⊗ q_y(-90°) ⊗ q_y(90°) ⊗ q_z(-θ_m)
      = q_z(θ_r) ⊗ identity ⊗ q_z(-θ_m)
      = q_z(θ_r - θ_m)   ← 纯 Z 轴旋转！
```

对纯 Z 轴旋转 `q_z(θ)` = `[cos(θ/2), 0, 0, sin(θ/2)]`，ZYX 公式精确：

```
分子 = 2*(cos(θ/2)*sin(θ/2) + 0*0) = sin(θ) ≠ 0
分母 = 1 - 2*(0² + sin²(θ/2))       = cos(θ) ≠ 0（一般情况）
→ yaw = θ   ✓ 无奇异
```

### 新代码

```python
# enter() 中
motion_t0_quat = self.motion_body_quat[0, NPZ_ANCHOR_IDX].astype(np.float64)
robot_quat     = self.state_cmd.torso_quat_w.astype(np.float64)

# 对相对四元数提取 yaw，避免 ZYX 在躺下时的奇异
q_rel = _quat_mul(robot_quat, _quat_conj(motion_t0_quat))
self._init_to_world = _quat_to_matrix(_yaw_quat(q_rel))
```

### 验证：各姿态下的正确性

| 姿态 | q_rel 形式 | `_yaw_quat(q_rel)` 结果 |
|------|-----------|------------------------|
| 两者均直立，yaw 差 Δθ | `q_z(Δθ)` | `R_z(Δθ)` ✅ |
| 两者均仰躺，yaw 差 Δθ | `q_z(Δθ)` | `R_z(Δθ)` ✅ |
| 两者均侧躺，yaw 差 Δθ | `q_z(Δθ)` | `R_z(Δθ)` ✅ |
| 姿态不同（需注意） | 含非 Z 分量 | 提取 Z 分量近似 ✅ |

### 残余奇异性

仅在 `q_rel = [0, x, y, 0]` 时，即机器人相对动作恰好旋转了 180° 且旋转轴在水平面内（机器人完全倒转方向）时，才出现奇异。这种情况在实际使用中几乎不会发生。

---

## 对比总结

| 方法 | 直立 | 躺下（同向） | 躺下（不同 yaw） |
|------|------|------------|----------------|
| 旧：分别提取 yaw | ✅ | ❌ 随机噪声 | ❌ 随机噪声 |
| 第一次修复：躺下退化 identity | ✅ | ⚠️ 无对齐，有误差 | ❌ yaw 误差未消除 |
| 正确修复：对相对 q 提取 yaw | ✅ | ✅ 正确 | ✅ 正确 |

---

## FreeKick 策略的情况

FreeKick 的 `enter()` 使用相同的旧逻辑，但 FreeKick 的动作从站立开始，通常进入时机器人也是站立的，所以没有触发奇异。如果 FreeKick 将来需要支持非直立进入，可以应用相同的修复。
